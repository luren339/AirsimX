// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#pragma once
#include "TestBase.hpp"
#include "sensors/lidar/LidarSimpleParams.hpp"

namespace msr { namespace airlib {
class LidarSettingsTest : public TestBase {
public:
    void run() override
    {
        AirSimSettings::LidarSetting defaults;
        LidarSimpleParams original;
        original.initializeFromSettings(defaults);
        testAssert(original.update_frequency == 10.0f, "Default LiDAR update rate must stay at 10 Hz");
        AirSimSettings::LidarSetting configured;
        configured.settings = Settings::loadJSonString(R"({"UpdateFrequency":20,"ScanPatternFile":"example.csv","ScanPatternMode":"full_fov_uniform"})");
        LidarSimpleParams params;
        params.initializeFromSettings(configured);
        testAssert(params.update_frequency == 20.0f, "Explicit UpdateFrequency was ignored");
        testAssert(original.scan_pattern_file.empty() && original.scan_pattern_mode == "sequential", "CSV sampling must be opt-in");
        testAssert(params.scan_pattern_file == "example.csv" && params.scan_pattern_mode == "full_fov_uniform", "Scan pattern settings were ignored");
        for (const auto& json : {R"({"UpdateFrequency":0})", R"({"UpdateFrequency":-1})", R"({"UpdateFrequency":1e39})"}) {
            AirSimSettings::LidarSetting invalid;
            invalid.settings = Settings::loadJSonString(json);
            bool rejected = false;
            try { LidarSimpleParams check; check.initializeFromSettings(invalid); }
            catch (const std::invalid_argument&) { rejected = true; }
            testAssert(rejected, "Non-positive LiDAR update frequency must be rejected");
        }
    }
};
}}
