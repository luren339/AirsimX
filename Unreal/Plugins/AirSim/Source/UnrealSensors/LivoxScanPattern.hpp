// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct LivoxScanAngle
{
    float azimuth_deg = 0.0f;
    float elevation_deg = 0.0f;
};

enum class LivoxScanSamplingMode
{
    Sequential,
    FullFovUniform
};

// Reads the ordered angular samples distributed by Livox-SDK/livox_laser_simulation.
// The CSV's third column is zenith measured from +Z; AirSim needs elevation from
// the XY plane, hence elevation = 90 deg - zenith.
class LivoxScanPattern
{
public:
    static LivoxScanSamplingMode parseSamplingMode(const std::string& mode)
    {
        if (mode == "sequential")
            return LivoxScanSamplingMode::Sequential;
        if (mode == "full_fov_uniform")
            return LivoxScanSamplingMode::FullFovUniform;
        throw std::invalid_argument("unsupported Livox ScanPatternMode: " + mode);
    }

    // The same directed interval convention used by AirSim VectorMath.
    static bool isInHorizontalFov(float angle, float start, float end)
    {
        angle = std::fmod(angle, 360.0f);
        start = std::fmod(start, 360.0f);
        end = std::fmod(end, 360.0f);
        if (angle < 0) angle += 360.0f;
        if (start < 0) start += 360.0f;
        if (end < 0) end += 360.0f;
        return start < end ? start <= angle && angle <= end : start <= angle || angle <= end;
    }

    // Evenly span a complete ordered pattern in one frame. Using both endpoints
    // avoids repeatedly sampling a narrow, sequential slice of the direction table.
    static std::size_t uniformSampleIndex(std::size_t slot, std::size_t sample_count,
                                          std::size_t pattern_size)
    {
        if (sample_count == 0 || pattern_size == 0)
            throw std::invalid_argument("sample_count and pattern_size must be non-zero");
        if (slot >= sample_count)
            throw std::out_of_range("sample slot must be less than sample_count");
        if (sample_count == 1 || pattern_size == 1)
            return 0;
        if (sample_count >= pattern_size)
            return slot % pattern_size;

        if (slot > 0 && pattern_size - 1 > std::numeric_limits<std::uint64_t>::max() / slot)
            throw std::overflow_error("scan pattern sampling index exceeds uint64 range");
        const auto numerator = static_cast<std::uint64_t>(slot) *
                               static_cast<std::uint64_t>(pattern_size - 1);
        return static_cast<std::size_t>(numerator / static_cast<std::uint64_t>(sample_count - 1));
    }

    static std::size_t sampleCountForFullFov(std::size_t points_per_second, double update_frequency)
    {
        if (points_per_second == 0 || !std::isfinite(update_frequency) || update_frequency <= 0.0) {
            throw std::invalid_argument("full_fov_uniform requires positive points_per_second and finite update_frequency");
        }

        const long double requested = static_cast<long double>(points_per_second) /
                                      static_cast<long double>(update_frequency);
        const long double rounded = std::round(requested);
        if (!std::isfinite(rounded) ||
            rounded >= static_cast<long double>(std::numeric_limits<std::size_t>::max())) {
            throw std::overflow_error("full_fov_uniform sample count exceeds size_t range");
        }

        return std::max<std::size_t>(1, static_cast<std::size_t>(rounded));
    }

    bool loadCsv(const std::string& path, std::string& error)
    {
        angles_.clear();
        std::ifstream input(path);
        if (!input) {
            error = "cannot open scan pattern CSV: " + path;
            return false;
        }

        std::string line;
        std::vector<std::string> columns;
        if (!std::getline(input, line) || !splitRow(line, columns) ||
            (columns[0] != "Time" && columns[0] != "Time/s") ||
            (columns[1] != "Azimuth" && columns[1] != "Azimuth/deg") ||
            (columns[2] != "Zenith" && columns[2] != "Zenith/deg")) {
            error = "unexpected scan pattern CSV header: " + path;
            return false;
        }

        std::size_t line_number = 1;
        while (std::getline(input, line)) {
            ++line_number;
            if (trim(line).empty())
                continue;

            if (!splitRow(line, columns)) {
                error = "invalid scan pattern row " + std::to_string(line_number);
                angles_.clear();
                return false;
            }
            const std::string& time_text = columns[0];
            const std::string& azimuth_text = columns[1];
            const std::string& zenith_text = columns[2];

            char* end = nullptr;
            const double time_value = std::strtod(time_text.c_str(), &end);
            if (end == time_text.c_str() || *end != '\0' || !std::isfinite(time_value)) {
                error = "invalid time value in scan pattern row " + std::to_string(line_number);
                angles_.clear();
                return false;
            }
            const double azimuth = std::strtod(azimuth_text.c_str(), &end);
            if (end == azimuth_text.c_str() || *end != '\0' || !std::isfinite(azimuth)) {
                error = "invalid azimuth in scan pattern row " + std::to_string(line_number);
                angles_.clear();
                return false;
            }
            const double zenith = std::strtod(zenith_text.c_str(), &end);
            if (end == zenith_text.c_str() || *end != '\0' || !std::isfinite(zenith) || zenith < 0.0 || zenith > 180.0) {
                error = "invalid zenith in scan pattern row " + std::to_string(line_number);
                angles_.clear();
                return false;
            }

            double normalized_azimuth = std::fmod(azimuth, 360.0);
            if (normalized_azimuth < 0.0)
                normalized_azimuth += 360.0;
            angles_.push_back({static_cast<float>(normalized_azimuth), static_cast<float>(90.0 - zenith)});
        }

        if (angles_.empty()) {
            error = "scan pattern CSV has no samples: " + path;
            return false;
        }
        error.clear();
        return true;
    }

    bool empty() const { return angles_.empty(); }
    std::size_t size() const { return angles_.size(); }

    const LivoxScanAngle& at(std::size_t index) const
    {
        if (angles_.empty())
            throw std::out_of_range("scan pattern has no samples");
        return angles_.at(index % angles_.size());
    }

private:
    static std::string trim(const std::string& text)
    {
        const auto first = text.find_first_not_of(" \t\r\n");
        return first == std::string::npos ? "" : text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
    }

    static bool splitRow(const std::string& line, std::vector<std::string>& columns)
    {
        columns.clear();
        if (std::count(line.begin(), line.end(), ',') != 2)
            return false;
        std::istringstream row(line);
        std::string value;
        while (std::getline(row, value, ','))
            columns.push_back(trim(value));
        return columns.size() == 3;
    }

    std::vector<LivoxScanAngle> angles_;
};

// Runtime state for ordered sampling. Kept independent of Unreal so ray-budget
// accumulation and reset behavior can be verified by the native test target.
class LivoxScanPatternState
{
public:
    void reset()
    {
        sequential_index_ = 0;
        fractional_ray_budget_ = 0.0L;
    }

    std::size_t nextSequentialRayCount(std::uint32_t points_per_second,
                                       double delta_time,
                                       std::size_t max_points_per_frame)
    {
        if (!std::isfinite(delta_time) || delta_time < 0.0)
            throw std::invalid_argument("scan delta_time must be finite and non-negative");

        const long double requested = static_cast<long double>(points_per_second) *
                                      static_cast<long double>(delta_time) +
                                      fractional_ray_budget_;
        if (!std::isfinite(requested))
            throw std::overflow_error("sequential scan ray budget exceeds long double range");

        // Tick durations such as 0.1 s are not exact in binary. A tiny
        // tolerance prevents ten 0.2-ray frames from yielding only one ray.
        constexpr long double integer_tolerance = 1e-12L;
        const long double whole_rays = std::floor(requested + integer_tolerance);
        fractional_ray_budget_ = requested - whole_rays;
        if (fractional_ray_budget_ < 0.0L &&
            fractional_ray_budget_ > -integer_tolerance)
            fractional_ray_budget_ = 0.0L;

        if (whole_rays <= 0.0L || max_points_per_frame == 0)
            return 0;
        if (whole_rays >= static_cast<long double>(max_points_per_frame))
            return max_points_per_frame;
        return static_cast<std::size_t>(whole_rays);
    }

    std::size_t nextSequentialIndex(std::size_t pattern_size)
    {
        if (pattern_size == 0)
            throw std::invalid_argument("sequential scan pattern must not be empty");
        const std::size_t index = sequential_index_ % pattern_size;
        sequential_index_ = index + 1 == pattern_size ? 0 : index + 1;
        return index;
    }

private:
    std::size_t sequential_index_ = 0;
    long double fractional_ray_budget_ = 0.0L;
};
