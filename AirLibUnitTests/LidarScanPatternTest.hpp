// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#pragma once

#include "TestBase.hpp"
#include "../Unreal/Plugins/AirSim/Source/UnrealSensors/LivoxScanPattern.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>

namespace msr { namespace airlib {
class LidarScanPatternTest : public TestBase {
public:
    void run() override
    {
        testAssert(LivoxScanPattern::parseSamplingMode("sequential") == LivoxScanSamplingMode::Sequential, "Sequential mode");
        testAssert(LivoxScanPattern::parseSamplingMode("full_fov_uniform") == LivoxScanSamplingMode::FullFovUniform, "Full FOV mode");
        expectThrow<std::invalid_argument>([] { LivoxScanPattern::parseSamplingMode("unknown"); });
        for (std::size_t i = 0; i < 5; ++i)
            testAssert(LivoxScanPattern::uniformSampleIndex(i, 5, 9) == i * 2, "Uniform sampling must span the table");
        testAssert(LivoxScanPattern::uniformSampleIndex(0, 1, 9) == 0, "One sample starts at the first direction");
        testAssert(LivoxScanPattern::uniformSampleIndex(4, 5, 1) == 0, "One direction can repeat");
        expectThrow<std::invalid_argument>([] { LivoxScanPattern::uniformSampleIndex(0, 0, 1); });
        expectThrow<std::invalid_argument>([] { LivoxScanPattern::uniformSampleIndex(0, 1, 0); });
        expectThrow<std::out_of_range>([] { LivoxScanPattern::uniformSampleIndex(2, 2, 3); });
        expectThrow<std::overflow_error>([] { LivoxScanPattern::uniformSampleIndex(3, 4, std::numeric_limits<std::size_t>::max()); });
        testAssert(LivoxScanPattern::sampleCountForFullFov(200000, 10) == 20000, "10 Hz sample budget");
        testAssert(LivoxScanPattern::sampleCountForFullFov(200000, 20) == 10000, "20 Hz sample budget");
        testAssert(LivoxScanPattern::sampleCountForFullFov(1, 10) == 1, "Positive request has at least one ray");
        expectThrow<std::invalid_argument>([] { LivoxScanPattern::sampleCountForFullFov(0, 10); });
        for (double f : {0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
            expectThrow<std::invalid_argument>([f] { LivoxScanPattern::sampleCountForFullFov(100, f); });
        expectThrow<std::overflow_error>([] { LivoxScanPattern::sampleCountForFullFov(100, 1e-300); });

        Fixture fixture;
        LivoxScanPattern pattern;
        std::string error;
        expectThrow<std::out_of_range>([&] { pattern.at(0); });
        testAssert(!pattern.loadCsv((fixture.path / "missing.csv").string(), error), "Missing CSV must fail");
        fixture.write(" Time/s , Azimuth/deg , Zenith/deg \r\n0, -90, 60\r\n.01,450,90\r\n\r\n");
        testAssert(pattern.loadCsv(fixture.file.string(), error), "Windows CSV and whitespace must be accepted: " + error);
        testAssert(pattern.size() == 2 && error.empty(), "Loaded sample count");
        testAssert(pattern.at(0).azimuth_deg == 270 && pattern.at(0).elevation_deg == 30, "Angle normalization and zenith conversion");
        testAssert(pattern.at(1).azimuth_deg == 90 && pattern.at(1).elevation_deg == 0, "Azimuth wrapping");
        testAssert(pattern.at(2).azimuth_deg == pattern.at(0).azimuth_deg, "Sequential wrap");
        const char* invalid[] = {
            "Time,Zenith,Azimuth\n0,90,0\n", "Time,Azimuth,Zenith\n", "Time,Azimuth,Zenith\n0,0,90,\n",
            "Time,Azimuth,Zenith\n0,0,90,1\n", "Time,Azimuth,Zenith\n0,0\n",
            "Time,Azimuth,Zenith\nnan,0,90\n", "Time,Azimuth,Zenith\n0,inf,90\n",
            "Time,Azimuth,Zenith\n0,0,181\n", "Time,Azimuth,Zenith\n0,0,-1\n",
            "Time,Azimuth,Zenith\n0,0,90x\n", "Time,Azimuth,Zenith\n0,,90\n",
            "Time,Azimuth,Zenith\n0,0,90\n1,bad,90\n"
        };
        for (const char* text : invalid) {
            fixture.write(text);
            testAssert(!pattern.loadCsv(fixture.file.string(), error), "Malformed CSV was accepted");
            testAssert(pattern.empty() && !error.empty(), "A failed load must not expose partial or old samples");
        }
    }
private:
    struct Fixture {
        std::filesystem::path path = std::filesystem::temp_directory_path() /
            ("airsimx-lidar-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::path file = path / "pattern.csv";
        Fixture() { std::filesystem::create_directories(path); }
        ~Fixture() { std::error_code ec; std::filesystem::remove_all(path, ec); }
        void write(const char* text) { std::ofstream out(file, std::ios::binary); out << text; }
    };
    template<typename Exception, typename Action> void expectThrow(Action action)
    {
        bool threw = false;
        try { action(); } catch (const Exception&) { threw = true; }
        testAssert(threw, "Expected exception was not raised");
    }
};
}}
