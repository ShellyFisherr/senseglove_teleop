
#pragma once

namespace Config
{
    // Tracking configuration

    constexpr double UPDATE_FREQUENCY_HZ = 60.0;

    // true = right hand
    // false = left hand
    constexpr bool TRACK_RIGHT_HAND = true;

    // CSV logging
    constexpr const char* CSV_FILENAME = "tracking_data.csv";
}
