
#include <SenseGlove/Core/HandLayer.hpp>
#include <SenseGlove/Core/HandPose.hpp>
#include <SenseGlove/Core/Vect3D.hpp>
#include <SenseGlove/Core/SenseCom.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <string>
#include <thread>
#include <vector>

#include <config.hpp>

using namespace SGCore;

using Clock = std::chrono::steady_clock;

static constexpr double UPDATE_FREQUENCY = Config::UPDATE_FREQUENCY_HZ;
static constexpr bool RIGHT_HAND = Config::TRACK_RIGHT_HAND;
static constexpr const char* CSV_FILENAME = Config::CSV_FILENAME;

static constexpr size_t FINGER_COUNT = 5;
static constexpr size_t JOINTS_PER_FINGER = 4;
static constexpr size_t COORDS_PER_JOINT = 3;
static constexpr size_t FLOAT_COUNT =
    FINGER_COUNT * JOINTS_PER_FINGER * COORDS_PER_JOINT;

struct HandJointData
{
    float x;
    float y;
    float z;
};

using FingerData = std::vector<HandJointData>;
using HandData = std::vector<FingerData>;

static std::atomic<bool> running{true};

void signalHandler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM)
    {
        running.store(false);
    }
}

bool extractJointPositions(
    const SGCore::HandPose& handPose,
    HandData& output)
{
    output.clear();

    const auto& positions = handPose.GetJointPositions();

    if (positions.size() != FINGER_COUNT)
    {
        return false;
    }

    output.reserve(FINGER_COUNT);

    for (const auto& finger : positions)
    {
        if (finger.size() != JOINTS_PER_FINGER)
        {
            return false;
        }

        FingerData fingerData;
        fingerData.reserve(JOINTS_PER_FINGER);

        for (const auto& joint : finger)
        {
            HandJointData p{
                joint.GetX(),
                joint.GetY(),
                joint.GetZ()
            };

            fingerData.push_back(p);
        }

        output.push_back(std::move(fingerData));
    }

    return true;
}

void writeCsvHeader(std::ofstream& csv)
{
    csv << "timestamp_ns";

    for (size_t f = 0; f < FINGER_COUNT; ++f)
    {
        for (size_t j = 0; j < JOINTS_PER_FINGER; ++j)
        {
            csv << ",finger" << f << "_joint" << j << "_x"
                << ",finger" << f << "_joint" << j << "_y"
                << ",finger" << f << "_joint" << j << "_z";
        }
    }

    csv << '\n';
}

void writeCsvRow(
    std::ofstream& csv,
    uint64_t timestampNs,
    const HandData& hand)
{
    csv << timestampNs << std::setprecision(7);

    for (const auto& finger : hand)
    {
        for (const auto& joint : finger)
        {
            csv << ',' << joint.x
                << ',' << joint.y
                << ',' << joint.z;
        }
    }

    csv << '\n';
}

bool writeBinaryFrame(
    uint64_t timestampNs,
    const HandData& hand)
{
    float values[FLOAT_COUNT];

    size_t index = 0;

    for (const auto& finger : hand)
    {
        for (const auto& joint : finger)
        {
            values[index++] = joint.x;
            values[index++] = joint.y;
            values[index++] = joint.z;
        }
    }

    if (index != FLOAT_COUNT)
    {
        return false;
    }

    std::cout.write(
        reinterpret_cast<const char*>(&timestampNs),
        sizeof(timestampNs));

    std::cout.write(
        reinterpret_cast<const char*>(values),
        sizeof(values));

    return static_cast<bool>(std::cout);
}

int main(int argc, char* argv[])
{
    bool streamMode = false;

    if (argc == 2 && std::string(argv[1]) == "--stream")
    {
        streamMode = true;
    }
    else if (argc != 1)
    {
        std::cerr << "Usage: senseglove_reader [--stream]\n";
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cerr
        << "=====================================\n"
        << "       SenseGlove Nova 2 Linux\n"
        << "=====================================\n"
        << "Mode: " << (streamMode ? "Binary stream" : "CSV logging") << '\n'
        << "Rate: " << UPDATE_FREQUENCY << " Hz\n"
        << "Hand: " << (RIGHT_HAND ? "Right" : "Left") << '\n';

    std::cerr << "\nStarting SenseCom...\n";

    SGCore::SenseCom::StartupSenseCom();

    std::cerr << "Waiting for SenseCom scanning...\n";

    while (running.load() && !SGCore::SenseCom::ScanningActive())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (!running.load())
    {
        return 0;
    }

    std::cerr << "SenseCom scanning active.\n";
    std::cerr << "Waiting for Nova 2 connection...\n";

    while (running.load() &&
           !SGCore::HandLayer::DeviceConnected(RIGHT_HAND))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (!running.load())
    {
        return 0;
    }

    std::cerr << "Glove connected.\n";

    std::ofstream csv;

    if (!streamMode)
    {
        csv.open(CSV_FILENAME);

        if (!csv.is_open())
        {
            std::cerr << "Could not open CSV file: "
                      << CSV_FILENAME << '\n';
            return 1;
        }

        csv.imbue(std::locale::classic());
        writeCsvHeader(csv);

        std::cerr << "CSV logging: " << CSV_FILENAME << '\n';
    }

    if (streamMode)
    {
        // stdout is reserved exclusively for binary measurement frames.
        std::cout.sync_with_stdio(false);
    }

    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / UPDATE_FREQUENCY));

    auto nextTick = Clock::now();

    uint64_t sequence = 0;

    std::cerr << "Starting data processing. Press CTRL+C to stop.\n";

    while (running.load())
    {
        nextTick += period;

        if (!SGCore::HandLayer::DeviceConnected(RIGHT_HAND))
        {
            std::cerr << "Glove disconnected. Waiting...\n";

            const auto disconnectTime = Clock::now();

            while (running.load() &&
                   !SGCore::HandLayer::DeviceConnected(RIGHT_HAND))
            {
                const double elapsed = std::chrono::duration<double>(
                    Clock::now() - disconnectTime).count();

                if (elapsed > 30.0)
                {
                    std::cerr << "Glove disconnected for 30 seconds. Stopping.\n";
                    running.store(false);
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            if (!running.load())
            {
                break;
            }

            std::cerr << "Glove reconnected.\n";
            nextTick = Clock::now();
        }

        SGCore::HandPose handPose;

        if (!SGCore::HandLayer::GetHandPose(RIGHT_HAND, handPose))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        HandData hand;

        if (!extractJointPositions(handPose, hand))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        const auto now = Clock::now();

        const uint64_t timestampNs =
            static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    now.time_since_epoch()).count());

        bool success = true;

        if (streamMode)
        {
            success = writeBinaryFrame(timestampNs, hand);
        }
        else
        {
            writeCsvRow(csv, timestampNs, hand);

            if (++sequence % 60 == 0)
            {
                csv.flush();
            }
        }

        if (!success)
        {
            std::cerr << "Failed to write measurement frame.\n";
            break;
        }

        ++sequence;

        if (sequence % 60 == 0)
        {
            std::cerr << "Frames: " << sequence << '\n';
        }

        std::this_thread::sleep_until(nextTick);

        if (Clock::now() > nextTick + period)
        {
            nextTick = Clock::now();
        }
    }

    std::cerr << "\nStopping SenseGlove reader...\n";

    if (csv.is_open())
    {
        csv.flush();
        csv.close();
    }

    std::cerr << "Samples collected: " << sequence << '\n';

    return 0;
}
