# teleop.cpp - Code Details

This document explains what each part of `teleop.cpp` does and why, including the math behind the one non-obvious calculation `fingercurl`. It's meant to be read alongside the source file.

The program is a single standalone C++ program, `senseglove_reader`, that talks to a SenseGlove NOVA 2 through the official SenseGlove-API C++ SDK (SGCore/SGConnect) and turns it into raw joint-position data into a simple Consumable signal. It has four run modes (`--udp`, `--diag`, `--stream`, and a
no-argument legacy mode), all sharing the same connection and data-extraction
logic.

NOTE: This brief explaination is made mostly using AI to keep it small and consise

# Layout

- [**1. Includes and Aliases**](#1-includes-and-aliases)
- [**2. Configuration constants**](#2-configuration-constants)
- [**3. Data types**](#3-data-types)
- [**4. Shutdown Handling**](#4-shutdown-handling)
- [**5. extracting joint position data and validating it**](#5-extractjointpositions--validating-and-copying-one-frame)
- [**6. fingercurl Conversion**](#6-fingercurl--the-math)
- [**7. Legacy outputs for debugging**](#7-legacy-output-helpers-writecsvheader-writecsvrow-writebinaryframe)
- [**8. Run packet rate diagnostics**](#8-rundiagmode--ground-truth-packet-rate-diagnostic)
- [**9. UDP package mode teleoperation**](#9-runudpmode--the-live-teleoperation-path)
- [**10. Main function and argument parsings**](#10-main--argument-parsing-and-startup-sequence)

---

## 1. Includes and aliases

```cpp
#include <SenseGlove/Core/HandLayer.hpp>
#include <SenseGlove/Core/HandPose.hpp>
#include <SenseGlove/Core/Vect3D.hpp>
#include <SenseGlove/Core/SenseCom.hpp>
#include <SenseGlove/Core/HapticGlove.hpp>
#include <SenseGlove/Core/DeviceList.hpp>
```

These are the SenseGlove SDK headers actually used:

- `HandLayer` — the high-level, synchronous API for asking "is a glove
  connected?" and "give me its current hand pose." This is the main thing the
  program talks to.
- `HandPose` — the data object `HandLayer::GetHandPose()` fills in; holds the
  3D position of every tracked hand joint.
- `Vect3D` — a simple `{x, y, z}` vector type used inside `HandPose`.
- `SenseCom` — lets the program check whether SenseCom (SenseGlove's
  background service that actually talks to the glove over Bluetooth) is
  running and scanning.
- `HapticGlove` — brought in for its `EHapticGloveCalibrationState` enum type
  (declared but not currently acted on in this version of the file).
- `DeviceList` — a lower-level accessor used only in `--diag` mode to read
  per-device Bluetooth packet-rate counters directly, bypassing `HandLayer`.

```cpp
using namespace SGCore;
using Clock = std::chrono::steady_clock;
```
`SGCore` is the SDK's namespace; `using namespace` saves prefixing every type.
`Clock` is just a short alias for `std::chrono::steady_clock`, the clock used
throughout for timing — `steady_clock` is used (rather than `system_clock`)
because it can't jump backward or be adjusted by the system, which matters for
anything computing fixed intervals in a loop.

---

## 2. Configuration constants

```cpp
static constexpr double UPDATE_FREQUENCY = Config::UPDATE_FREQUENCY_HZ; // 60.0
static constexpr bool RIGHT_HAND = Config::TRACK_RIGHT_HAND;            // true
static constexpr const char* CSV_FILENAME = Config::CSV_FILENAME;

static constexpr size_t FINGER_COUNT = 5;
static constexpr size_t JOINTS_PER_FINGER = 4;
static constexpr size_t COORDS_PER_JOINT = 3;
static constexpr size_t FLOAT_COUNT = FINGER_COUNT * JOINTS_PER_FINGER * COORDS_PER_JOINT; // 60
```

The first three are pulled from `config.hpp` (a separate, user-editable file)
so the target rate, tracked hand, and CSV filename aren't buried in the main
logic. `UPDATE_FREQUENCY` is 60 Hz, matching the Nova 2's own maximum update
rate — there is no benefit to polling faster than the hardware itself
updates.

The next four constants describe the shape of one hand's worth of joint data,
as returned by `HandPose::GetJointPositions()`: **5 fingers, 4 joints per
finger, 3 coordinates (x, y, z) per joint** — 60 floats total for one full
hand pose. This 5×4 structure (thumb → pinky, proximal → distal, no separate
wrist entry) is what the SDK's own documentation specifies, and what
`extractJointPositions()` (below) validates against on every single frame.

---

## 3. Data types

```cpp
struct HandJointData { float x; float y; float z; };
using FingerData = std::vector<HandJointData>;
using HandData = std::vector<FingerData>;
```

A small, SDK-independent mirror of a 3D point (`HandJointData`), a finger's
list of joints (`FingerData`, expected length 4), and a full hand's list of
fingers (`HandData`, expected length 5). Keeping this separate from the SDK's
own `Vect3D`/`HandPose` types means the rest of the file (CSV/binary writers,
curl math) doesn't need to know anything about the SDK at all.

---

## 4. Shutdown handling

```cpp
static std::atomic<bool> running{true};

void signalHandler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM)
    {
        running.store(false);
    }
}
```

`running` is the one piece of shared state every loop in the program checks.
`signalHandler` is registered (in `main`) for `SIGINT` (Ctrl+C) and `SIGTERM`
(a normal `kill`), and does nothing except flip `running` to `false`. Every
`while (running.load())` loop in the file notices this on its next iteration
and unwinds cleanly — closing sockets, flushing files, printing a final
summary — rather than being killed mid-write.

It's an `std::atomic<bool>` rather than a plain `bool` because it's written
from a signal handler (which can interrupt the main thread at any point) and
read from the main loop; `atomic` is what makes that safe without a mutex.

---

## 5. `extractJointPositions()` — validating and copying one frame

```cpp
bool extractJointPositions(const SGCore::HandPose& handPose, HandData& output)
```

Takes the SDK's `HandPose` object for one frame and converts it into the
program's own `HandData` type, while checking that what came back actually
has the shape expected: exactly `FINGER_COUNT` (5) finger groups, each with
exactly `JOINTS_PER_FINGER` (4) joints. If either check fails, the function
returns `false` and copies nothing — every caller treats that as "skip this
frame, try again shortly" rather than crashing or publishing garbage.

This is the single choke point where the program's assumption about the
SDK's data shape is enforced; every other piece of code downstream (curl
calculation, CSV/binary writers) trusts that a `HandData` it receives really
is 5×4.

---

## 6. `fingerCurl()` — the math

```cpp
float fingerCurl(const FingerData& joints)
```

This is the one actual piece of math in the file: reducing one finger's 4
joint positions (a short, bent or straight 3D polyline from the base of the
finger to the fingertip) down to a single number between 0 and 1 describing
how curled that finger is.

**The idea:** compare the *path length* along the finger (summing the
distance between each consecutive joint) to the *straight-line distance*
between the first joint and the last.

- `segmentLengths` = sum of `|Pᵢ − Pᵢ₋₁|` for every consecutive pair of the
  4 joints — i.e. the total length of the 3-segment polyline from the base
  joint to the fingertip, following the finger's own articulation.
- `straightDist` = `|P₃ − P₀|` — the direct, straight-line distance from the
  first joint to the last, ignoring how it bent to get there.

```
curl = 1 − (straightDist / segmentLengths)
```

When the finger is perfectly straight, every joint lies on one line, so the
straight-line distance *equals* the path length — the ratio is 1, and
`curl = 1 − 1 = 0`. As the finger curls, the fingertip folds back toward the
base: the path length barely changes (it's still 3 fixed-length bone
segments), but the straight-line distance between the endpoints shrinks,
so the ratio drops below 1 and `curl` rises toward 1 as the fingertip
approaches the base joint.

Two properties make this a good choice over something simpler (like just the
bend angle at one joint):

- **It's scale-invariant.** It's a ratio of two lengths in the same units, so
  it doesn't matter whether the glove's internal hand model is calibrated for
  a large or small hand — a fully-open finger always reads ~0 and a
  fully-closed fist always reads ~1, regardless of absolute finger length.
- **It captures the whole finger's shape in one number**, rather than only
  one joint's angle, which matters because a glove's joints don't
  necessarily curl in perfect unison.

Two guards handle degenerate input:

```cpp
if (segmentLengths < 1e-6 || joints.size() < 2) return 0.0f;
...
curl = std::max(0.0, std::min(1.0, curl));
```

The first avoids a division by (near) zero if the joints are somehow
coincident or there's fewer than 2 points. The second clamps the result to
`[0, 1]` — in principle the ratio can't exceed 1 for a real, physically
bending finger, but floating-point noise or a momentarily odd pose from the
glove could push it a hair outside that range, and every downstream consumer
(this program's UDP packets, the Python calibration pipeline) assumes the
value is always in `[0, 1]`.

This exact formula is mirrored in Python (`finger_curl()` in
`orca_motionTracker`) so the two implementations always agree on what a given
hand pose "means," independent of which one happens to compute it.

---

## 7. Legacy output helpers (`writeCsvHeader`, `writeCsvRow`, `writeBinaryFrame`)

These exist for the two older, non-UDP modes (the default no-argument mode
and `--stream`), kept for debugging/offline analysis rather than live
teleoperation:

- **`writeCsvHeader`/`writeCsvRow`** write one text row per frame to a CSV
  file: a nanosecond timestamp, then every joint's raw `x,y,z` (5 fingers × 4
  joints × 3 coordinates = 60 numbers), column-named `fingerF_jointJ_x` etc.
  This is the *raw joint positions*, not the derived curl values — useful for
  offline analysis or debugging the finger-mapping itself, as opposed to just
  trusting the curl math.
- **`writeBinaryFrame`** writes the same information as fixed-width binary
  instead of text: an 8-byte `uint64_t` timestamp followed by `FLOAT_COUNT`
  (60) 4-byte floats, straight to `stdout`. No header, no delimiters — a
  reader on the other end needs to know the exact layout up front. This mode
  is a holdover from an earlier pipe-based architecture and isn't used by the
  current UDP-based pipeline.

---

## 8. `runDiagMode()` — ground-truth packet-rate diagnostic

```cpp
int runDiagMode()
```

Doesn't use `HandLayer` at all. Instead it waits for `DeviceList` (a lower
level than `HandLayer`) to report at least one device, then once a second
prints each device's own `PacketsPerSecondReceived()`/`PacketsPerSecondSent()`
counters — figures the SDK/SGConnect computes directly from the Bluetooth
traffic, independent of anything this program's own read loop does.

This mode exists purely for troubleshooting: it answers "is the Bluetooth
link itself delivering data at a healthy rate?" without this program's own
polling loop, frame extraction, or curl math being able to introduce or hide
a bottleneck.

---

## 9. `runUdpMode()` — the live teleoperation path

```cpp
int runUdpMode(const std::string& host, uint16_t port)
```

This is the mode actually used for live teleoperation, and the most
involved function in the file. Walking through it:

**Socket setup.**
```cpp
int sock = socket(AF_INET, SOCK_DGRAM, 0);
...
inet_pton(AF_INET, host.c_str(), &dest.sin_addr)
```
Opens one UDP socket and resolves the destination address/port once, up
front. UDP is connectionless and "fire-and-forget" — there's no handshake,
no retry, no delivery guarantee, which is the right trade-off here: a dropped
packet at 60 Hz just means the receiver uses slightly stale data for 1/60th
of a second, which matters far less than a protocol that could block or
retry and introduce latency into a live teleoperation loop.

**The fixed-cadence loop.**
```cpp
const auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / UPDATE_FREQUENCY));
auto nextTick = Clock::now();
...
while (running.load())
{
    nextTick += period;
    ...
    std::this_thread::sleep_until(nextTick);
    if (Clock::now() > nextTick + period) nextTick = Clock::now();
}
```
`period` is `1/60` of a second, expressed as a `steady_clock` duration.
Rather than just `sleep_for(period)` every iteration (which would
accumulate drift, since the work done each iteration also takes some
non-zero time), the loop computes an absolute target time (`nextTick`) by
repeatedly adding one fixed period, and sleeps *until* that absolute instant.
This keeps the average rate locked to exactly 60 Hz regardless of how long
each iteration's actual work takes, as long as that work stays under 1/60s.

The catch-up guard (`if (Clock::now() > nextTick + period) nextTick =
Clock::now();`) handles the case where the loop *has* fallen behind by a
full extra period or more — e.g. the glove briefly disconnected. Without this
check, the loop would otherwise try to "catch up" by firing many iterations
back-to-back with no sleeping at all until it caught up to the schedule,
which would briefly flood the UDP receiver with a burst of stale-looking
packets. Resetting `nextTick` to "now" in that case means it only ever
catches up by one period at most, never bursts.

**Per-frame work, inside the loop:**

1. `HandLayer::DeviceConnected(RIGHT_HAND)` — if the glove isn't connected,
   sleep briefly, reset `nextTick` (so the fixed cadence resumes cleanly once
   it reconnects, rather than the loop immediately thinking it's "behind"),
   and retry.
2. `HandLayer::GetHandPose(RIGHT_HAND, handPose)` — the one call that
   actually asks the SDK for this frame's hand pose.
3. `extractJointPositions(handPose, hand)` — validate and copy into this
   program's own 5×4 `HandData`.
4. `fingerCurl(hand[f])` for each of the 5 fingers — reduce each finger's 4
   joints down to one curl value, giving 5 floats total per frame.
5. Format and send the UDP packet.

**Packet format:**
```cpp
std::snprintf(buffer, sizeof(buffer), "%llu,%.5f,%.5f,%.5f,%.5f,%.5f",
              sequence, curls[0], curls[1], curls[2], curls[3], curls[4]);
```
A plain ASCII, comma-separated string: `sequence,thumb,index,middle,ring,pinky`,
each curl value to 5 decimal places. `sequence` is a monotonically
increasing counter (starting at 0, incremented once per successfully-sent
packet) that lets the receiver detect dropped or out-of-order packets —
expected occasionally over UDP, and the receiver is written to tolerate it.

**Rate reporting.** Once a second, the loop prints how many packets it
actually sent in the last second (`sentThisSecond`) — a quick live sanity
check that the publisher is really running at ~60 Hz, separate from
`runDiagMode`'s Bluetooth-layer view.

---

## 10. `main()` — argument parsing and startup sequence

**Argument parsing** picks exactly one mode: `--stream` (binary), `--diag`
(ground-truth packet rate), `--udp [host] [port]` (defaults
`127.0.0.1`/`5010`), or no arguments at all (legacy CSV logging). Anything
else prints a usage message and exits with an error.

**Signal registration and startup banner** happen next, followed by the
connection sequence every mode shares:

1. **Start or wait for SenseCom.** In every mode *except* `--udp`, the
   program calls `SenseCom::StartupSenseCom()` to launch SenseCom itself. In
   `--udp` mode, this call is deliberately skipped — that mode assumes
   SenseCom is already running (typically started separately, headless, so
   it can be left running across multiple `--udp` invocations rather than
   relaunched and having to rescan Bluetooth every time).
2. **Wait for `SenseCom::ScanningActive()`** — SenseCom has started up and is
   actively scanning for devices.
3. **Wait for `HandLayer::DeviceConnected(RIGHT_HAND)`** — the Nova 2 has
   actually been found and connected over Bluetooth.

Both wait loops poll every 500ms and check `running.load()` on every
iteration, so Ctrl+C during startup (before a glove is even connected) exits
immediately rather than hanging.

Once connected, `main` dispatches to whichever mode was requested
(`runDiagMode()`, `runUdpMode()`, or falls through into the legacy
stream/CSV loop inline). The legacy loop additionally handles mid-session
disconnects: if the glove drops out, it waits up to 30 seconds for it to
reconnect (resetting `nextTick` on success so the fixed cadence doesn't think
it's instantly far behind) before giving up and shutting down.