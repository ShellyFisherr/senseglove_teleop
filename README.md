# **senseglove_teleop**

This program is a small standalone C++ program (senseglove_reader) for sending a senseglove NOVA 2 fingercurl package over UDP. it is to be used either with a custom new package you want, or to be used with the [orca_motionTracker](https://github.com/ShellyFisherr/orca_motionTracker) package.

## **Layout**
- [Requirements](#requirements)
- [Install Guide](#installation-guide)
- [How to use](#how-to-use)
- [Parameters](#parameters)
- [Code explaination](#code-explaination)
- [Complications](#complications)

---
### **Requirements**
- Linux (x84-64)
- CMake >= 3.16
- A C++17 compiler (g++ or clang)
- The SenseGlove-API SDK (install guide also provided)
- SenseCom (Provided withe the SenseGlove-API)
- A SenseGlove NOVA 2 glove
- Bluetooth (SenseGlove communicated with the glove in this way)

---
### **Installation Guide**


---
below is a step by step guide on how to install this repo on Linux.

make a directory using the command:
```bash
cd
mkdir -p ~/RobotHand/senseglove/
```

If you do not have the SenseGlove-API SDK yet, clone it 
```bash
sudo apt update
git clone https://github.com/Adjuvo/SenseGlove-API
```

it Should create a new folder with the structure
```text
─── RobotHand
    └── senseglove
        └── SesnseGlove-API
            ├── android
            ├── examples
            ├── include
            ├── SenseCom
            ├── LICENCE
            └── README.md
```

The SenseCom program for linux is found in
```text
├── ...
├── SenseCom
    ├── SenseCom_Linux (unzipped)
        ├── SenseCom.x86_64
        ├── ...
    ├── ...

```
Right-click `SenseCom.x86_64`, go to `properties` and enable `Executable as program` so linux knows this is a program to run. Opening it now Runs the SenseCom program. If you need more help in connecting the glove to the program, A more in detail guide from the official creators can be found [On the official page](https://unreal.docs.senseglove.com/2.9/getting-started/sensecom/bluetooth-low-energy/gnu-linux.html)

To open the file, you can either double-click it, but it is adviced to open it via the command window using
```
~/RobotHand/senseglove/SenseGlove-API/SenseCom/SenseCom_Linux_1_8_4/SenseCom.x86_64
```


with the API installed now, the added reader can be installed as well using
```bash
cd ~/RobotHand/senseglove
git clone https://github.com/ShellyFisherr/senseglove_teleop
```

from here the filestructure should be
```text

─── RobotHand
    └── senseglove
        ├── SesnseGlove-API
        └── senseglove_teleop
            ├── CMakeLists.txt
            ├── config.hpp
            ├── LICENCE
            ├── README.md
            ├── teleop.cpp
```

Before building check the parameters from [config.hpp](config.hpp) and make sure everyting is correct. Parameters are expained more in [Parameters](#parameters) section.

To build the system, do
```bash
cd ~/RobotHand/senseglove/senseglove_teleop
mkdir -p build && cd build
cmake ..
cmake --build .
```

This produces `build/senseglove_reader`, which is the executable.

---
### **How to Use**
`senseglove_reader` needs SenseCom running and the glove connected and calibrated to it before it will produce any data. How that happens depends on the mode. 

#### **`--udp [host] [port]` - live teleoperation:**

This mode **does not start SenseCom teleop**. it assumes it is already running and calibrated separately, and just waits for it. When it is running, start the senseglove_reader using:

```bash
cd ~/RobotHand/senseglove/senseglove_teleop/build
./senseglove_reader --udp                  # defaults to 127.0.0.1:5010
./senseglove_reader --udp 127.0.0.1 5010   # equivalent, explicit
./senseglove_reader --udp 192.168.1.50 6000  # a different host/port
```
Choose which of the three options you need depending on your setup. the first one is the default one used for a internal system UDP bridge. The second one does the same but sets the port specifically,
The third one sets the address to a different specified host IP and port.

#### **`--diag` - Bluetooth packet-rate diagnostic:**
```bash
.senseglove_reader --diag
```
Starts SenseCom itself if it isnt already running (still adviced to start it manually to prevent any possible errors regarding the SenseCom program), then once per second prints each connected device's own ground-truth `PacketsPerSecondReceived` / `Sent` counters. Usefull for checking weither the bluetooth link itself is healthy, independent of anything in this program's own polling loop.

#### **`--stream > frames.bin`**
Starts SenseCom itslef, then writes the raw binary frames (timesstamp + all 60 raw joint-position floats, no curl reduction) to stdout. A holdover from an earlier pipe-based architecture; not used by the current UDP pipeline.

#### **No arguments — CSV logging (legacy)**

```bash
./senseglove_reader
```

Starts SenseCom itself, then logs one CSV row per frame (timestamp + all 60 raw joint-position values) to `tracking_data.csv` (configurable in [config.hpp](config.hpp)). Useful for offline analysis of the raw joint positions themselves, separate from the derived curl values.

---
### **Parameters**

There are 3 parameters in total, located in [config.hpp](config.hpp)

- UPDATE_FREQUENCY_HZ
    - update frequency at which data is read and send over UDP. 60 Hz is the standard, as this is the frequency at which the sensors read the glove data at default, and it is ill adviced of the senseglove creators to exceed it.

- TRACK_RIGHT_HAND
    - Which hand the glove corresponds to. `true` means the right hand is tracked, `false` means the left hand is tracked. NOTE: The right hand is the only one tested as it was the only one available at the time. (the orca_motionTracker package this system is build for also only has the right hand available, hence no effort was made to test and make a left handed mode specifically.)

- CSV_FILENAME
    - Name of the csv file that saves all fingers and their corresponding joint positions.


Make sure to rebuild the package after editting the CSV file using
```bash
cd ~RobotHand/senseglove/senseglove_teleop/build
cmake --build .
```
Since the UDP bridge is on the internal system, it is not needed to set a specific target IP for the UDP bridge, since the communication happens all in a single device. If wanted otherwise, the UDP IP and port can be set specifically upon launching the program, stated in [How to use](#how-to-use).

---
### **Code explaination**
A Code explaination on what each section of the code does can be found in [CODE_DETAILS](CODE_DETAILS.md)

---
### **Complications**
- **"Waiting for SenseCom to be running"** never resolves in `--udp` mode:
  SenseCom isn't actually running yet, or died. Check for a stale
  `unity.lock` file or crash-dump files (`mono_crash.mem.*.blob`) in the
  SenseCom directory left over from a previous crashed instance, and remove
  them before relaunching.
- **"Waiting for Nova 2 connection..."** never resolves: SenseCom is running
  and scanning, but the glove either isn't powered on, isn't paired, or is
  out of Bluetooth range.
- **Packet rate well under 60/sec** in `--udp` mode: run `--diag` alongside
  it to check whether the Bluetooth link itself is the bottleneck (SGConnect's
  own packet-rate counters) rather than anything in this program.