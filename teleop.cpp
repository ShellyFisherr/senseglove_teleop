
#include <SenseGlove/Core/HandLayer.hpp>
#include <SenseGlove/Core/HandPose.hpp>
#include <SenseGlove/Core/SenseCom.hpp>
#include <SenseGlove/Core/Nova2Glove.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <config.hpp>

using namespace SGCore;

static constexpr double UPDATE_FREQUENCY = Config::UPDATE_FREQUENCY_HZ;

static constexpr bool RIGHT_HAND = Config::TRACK_RIGHT_HAND;

static constexpr const char* CSV_FILENAME = Config::CSV_FILENAME;


struct HandJointData
{
    float x;
    float y;
    float z;
};

using FingerData = std::vector<HandJointData>;
using HandData = std::vector<FingerData>;
using Clock = std::chrono::steady_clock;


static std::atomic<bool> running{true};


void signalHandler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM)
    {
        running.store(false);
    }
}

bool extractJoinPositions(
    const SGCore::HandPose& handPose,
    HandData& output)

{
    output.clear();

    const auto& positions = handPose.GetJointPositions();

    if (positions.empty())
    {
        return false;
    }

    output.reserve(positions.size());

    for (const auto& finger : positions)
    {
        FingerData fingerData;

        fingerData.reserve(finger.size());

        for (const auto& joint : finger)
        {
            HandJointData p;

            p.x = joint.GetX();
            p.y = joint.GetY();
            p.z = joint.GetZ();

            fingerData.push_back(p);
        }

        output.push_back(std::move(fingerData));
    }
    return true;
}


void writeCsvHeader(
    std::ofstream& csv,
    const HandData& hand)
{
    csv << "time_s,sequence,finger_count";

    for (size_t f = 0; f < hand.size(); ++f)
    {
        csv << ",finger" << f << "_joint_count";

        for (size_t j = 0; j < hand[f].size(); ++j)
        {
            csv << ",finger" << f << "_joint" << j << "_x"
                << ",finger" << f << "_joint" << j << "_y"
                << ",finger" << f << "_joint" << j << "_z";
        }
    }

    csv << "\n";
}

void writeCsvRow(
    std::ofstream& csv,
    double elapsedSeconds,
    uint64_t sequence,
    const HandData& hand)
{
    csv << std::setprecision(6)
        << elapsedSeconds
        << "," << sequence
        << "," << hand.size();

    for (const auto& finger : hand)
    {
        csv << "," << finger.size();

        for (const auto& joint : finger)
        {
            csv << "," << joint.x
                << "," << joint.y
                << "," << joint.z; 
        }
    }
    csv << "\n";
}


void processHandData(
    const HandData& hand,
    uint64_t sequence)

{
    if (sequence % 60 == 0)
    {
        std::cout
        << "sequence: " << sequence << " | "
        << "Finger groups: " << hand.size() << "\n";
    }

}


int main()
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout
    << "=====================================\n"
    << "       SenseGlove Nova 2 Linux\n"
    << "=====================================\n";

    std::cout
    << "\nRate: " << UPDATE_FREQUENCY << " HZ\n"
    << "Hand: " << (RIGHT_HAND ? "Right" : "Left") << "\n";


    std::cout << "\nStarting SenseCom...\n";

    SGCore::SenseCom::StartupSenseCom();

    std::cout << "Waiting for SenseCom scanning...\n";

    while (running.load() && !SGCore::SenseCom::ScanningActive())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (!running.load())
    {
        return 0;
    }

    std::cout << "SenseCom scanning active.\n";

    std::cout << "\nWaiting for Nova 2 Connection...\n";

    while (running.load() && !SGCore::HandLayer::DeviceConnected(RIGHT_HAND))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (!running.load())
    {
        return 0;
    }

    std::cout << "Glove Connected.\n";


    std::ofstream csv;

    csv.open(CSV_FILENAME);

    if (!csv.is_open())
    {
        std::cerr << "Could not open CSV file: "
                  << CSV_FILENAME << "\n";

        return 1;
    }

    csv.imbue(std::locale::classic());

    std::cout
        << "CSV logging: " << CSV_FILENAME << "\n";


    const auto startTime = Clock::now();

    const auto period = std::chrono::duration_cast(Clock::duration)(
        std::chrono::duration<double>(1.0 / UPDATE_FREQUENCY));

    auto nextTick = Clock::now();

    uint64_t sequence = 0;

    bool csvHeaderWritten = false;

    HandData previousShape;

    std::cout  
        << "\nStating Data processing...\n"
        << "Press CTRL+C to stop.\n\n";

    while (running.load())
    {
        nextTick += period;

        if (!SGCore::HandLayer::DeviceConnected(RIGHT_HAND))
        {
            std::cerr << "\nGlove Disconnected. Waiting...\n";

            const auto disconnectTime = Clock::now();

            while (running.load() && !SGCore::HandLayer::DeviceConnected(RIGHT_HAND))
            {
                const double elapsedDisconnect = std::chrono::duration<double>(
                    Clock::now() - disconnectTime).count();

                if (elapsedDisconnect > 30.0)
                {
                    std::cerr << "\nGlove Disconnected for too long. Shutting Down...\n";
                    running.store(false);
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            if (!running.load())
            {
                break;
            }

            std::cout << "Glove reconnected\n";

            nextTick = Clock::now();
        }

        SGCore::HandPose handPose;

        const bool poseAvailable = SGCore::HandLayer::GetHandPose(RIGHT_HAND, handPose);

        if (!poseAvailable)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        HandData hand;

        if (!extractJoinPositions(handPose, hand))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        if (!csvHeaderWritten)
        {
            writeCsvHeader(csv, hand);

            previousShape = hand;

            csvHeaderWritten = true;

            std::cout
                << "\nDetected hand structure:\n"
                << "Finger groups: " << hand.size() << "\n";

            for (size_t f = 0; f < hand.size(); ++f)
            {
                std::cout
                    << "finger group " << f << ": "
                    << hand[f].size() << "positions\n";
            }

            std::cout << "\n";
        }
        else
        {
            bool shapeChanged = hand.size() != previousShape.size();

            if (!shapeChanged)
            {
                for (size_t f = 0; f < hand.size(); ++f)
                {
                    if (hand[f].size() != previousShape[f].size())
                    {
                        shapeChanged = true;
                        break;
                    }
                }
            }

            if (shapeChanged)
            {
                std::cerr << "\nError: Hand joint structure changed. Skipping sample\n";
                continue;
            }

            ++sequence;

            const auto now = Clock::now();

            const double elapsedSeconds = std::chrono::duration<double>(now - startTime).count();

            processHandData(hand, sequence);

            writeCsvRow(
                csv,
                elapsedSeconds,
                sequence,
                hand);

            if (sequence % 60 == 0)
            {
                csv.flush();
            }
        }

        std::this_thread::sleep_until(nextTick);

        if (Clock::now() > nextTick + period)
        {
            nextTick = Clock::now();
        }
    }

    
    std::cout << "\nStopping SenseGlove...\n";

    if (csv.open())
    {
        csv.flush();
        csv.close();
    }

    std::cout << "Input stoppen.\n"
              << "Samples collected: " << sequence << "\n";

    return 0;
}