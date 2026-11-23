// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Gps_hpp
#define msr_airlib_Gps_hpp

#include <random>
#include "common/Common.hpp"
#include "GpsSimpleParams.hpp"
#include "GpsBase.hpp"
#include "common/FirstOrderFilter.hpp"
#include "common/FrequencyLimiter.hpp"
#include "common/DelayLine.hpp"

namespace msr
{
namespace airlib
{

    class GpsSimple : public GpsBase
    {
    public: //methods
        GpsSimple(const AirSimSettings::GpsSetting& setting = AirSimSettings::GpsSetting())
            : GpsBase(setting.sensor_name)
        {
            // initialize params
            params_.initializeFromSettings(setting);
            Utils::log(Utils::stringf(
                "GpsSimple '%s': satellite_model=%d uere=%.2f sats=%g..%g",
                setting.sensor_name.c_str(), params_.satellite_model ? 1 : 0,
                params_.uere, params_.sats_min, params_.sats_max));

            //initialize frequency limiter
            freq_limiter_.initialize(params_.update_frequency, params_.startup_delay);
            delay_line_.initialize(params_.update_latency);

            //initialize filters
            eph_filter.initialize(params_.eph_time_constant, params_.eph_final, params_.eph_initial); //starting dilution set to 100 which we will reduce over time to targeted 0.3f, with 45% accuracy within 100 updates, each update occurring at 0.2s interval
            epv_filter.initialize(params_.epv_time_constant, params_.epv_final, params_.epv_initial);

        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
            freq_limiter_.reset();
            delay_line_.reset();

            eph_filter.reset();
            epv_filter.reset();

            for (real_T& state : correlated_state_) {
                state = 0.0f;
            }
            last_noise_time_ = 0;
            sats_offset_ = 0.0f;
            sats_value_ = 0.5f * (params_.sats_min + params_.sats_max);
            obstruction_remaining_s_ = 0.0f;
            last_sat_time_ = 0;
            uniform_.reset();
            current_sats_ = 0;

            addOutputToDelayLine(eph_filter.getOutput(), epv_filter.getOutput());
        }

        virtual void update() override
        {
            GpsBase::update();

            freq_limiter_.update();
            eph_filter.update();
            epv_filter.update();

            if (freq_limiter_.isWaitComplete()) { //update output
                addOutputToDelayLine(eph_filter.getOutput(), epv_filter.getOutput());
            }

            delay_line_.update();

            if (freq_limiter_.isWaitComplete())
                setOutput(delay_line_.getOutput());
        }

        //*** End: UpdatableState implementation ***//

        virtual ~GpsSimple() = default;

    private:
        void addOutputToDelayLine(real_T eph, real_T epv)
        {
            Output output;
            const GroundTruth& ground_truth = getGroundTruth();

            //GNSS
            output.gnss.time_utc = static_cast<uint64_t>(clock()->nowNanos() / 1.0E3);
            output.gnss.geo_point = ground_truth.environment->getState().geo_point;
            output.gnss.eph = eph;
            output.gnss.epv = epv;
            output.gnss.velocity = ground_truth.kinematics->twist.linear;
            output.is_valid = true;

            if (params_.satellite_model) {
                updateSatellites();
                // With a satellite geometry model the reported accuracy is
                // the range error diluted by the geometry, not the smoothed
                // start-up value.
                output.gnss.eph = current_eph_;
                output.gnss.epv = current_epv_;
                output.gnss.satellites_visible = static_cast<uint8_t>(current_sats_);
            }

            addNoise(output.gnss);

            if (params_.satellite_model) {
                output.gnss.fix_type =
                    current_sats_ >= params_.sats_min_3d   ? GnssFixType::GNSS_FIX_3D_FIX
                    : current_sats_ >= params_.sats_min_2d ? GnssFixType::GNSS_FIX_2D_FIX
                                                           : GnssFixType::GNSS_FIX_NO_FIX;
            }
            else {
                output.gnss.fix_type =
                    output.gnss.eph <= params_.eph_min_3d   ? GnssFixType::GNSS_FIX_3D_FIX
                    : output.gnss.eph <= params_.eph_min_2d ? GnssFixType::GNSS_FIX_2D_FIX
                                                            : GnssFixType::GNSS_FIX_NO_FIX;
            }

            output.time_stamp = clock()->nowNanos();

            delay_line_.push_back(output);
        }

        void addNoise(GnssReport& gnss)
        {
            // With the satellite model the position/vertical sigmas come from
            // the geometry (UERE x DOP) instead of the fixed settings.
            const real_T horizontal_sigma = params_.satellite_model
                ? params_.uere * current_hdop_
                : params_.position_noise_sigma;
            const real_T vertical_sigma = params_.satellite_model
                ? params_.uere * current_hdop_ * params_.vdop_ratio
                : params_.vertical_noise_sigma;
            if (horizontal_sigma <= 0 && vertical_sigma <= 0 &&
                params_.velocity_noise_sigma <= 0) {
                return;
            }
            // Draw north/east/up offsets and convert to lat/lon/alt. The
            // conversion used at 1e-7 resolution (0.011 m) is far finer than
            // any realistic GNSS error, so a local flat-earth approximation
            // is enough. With NoiseTau > 0 the offsets come from the
            // Gauss-Markov processes instead of independent draws.
            real_T north, east, up, vn = 0, ve = 0, vd = 0;
            if (params_.noise_tau > 0) {
                const TTimePoint now = clock()->nowNanos();
                const real_T dt = last_noise_time_ == 0 ? 0.1f
                    : static_cast<real_T>((now - last_noise_time_) / 1.0e9);
                last_noise_time_ = now;
                // Exact discretisation of a first-order Gauss-Markov process:
                // with gain sqrt(1 - alpha^2) the stationary standard
                // deviation is sigma. (A gain of 1 - alpha would under-scale
                // it by sqrt(dt / (2 tau)).)
                const real_T alpha = static_cast<real_T>(std::exp(-dt / params_.noise_tau));
                const real_T gain = std::sqrt(std::max(0.0f, 1.0f - alpha * alpha));
                north = step_correlated(0, horizontal_sigma, alpha, gain);
                east = step_correlated(1, horizontal_sigma, alpha, gain);
                up = step_correlated(2, vertical_sigma, alpha, gain);
                vn = step_correlated(3, params_.velocity_noise_sigma, alpha, gain);
                ve = step_correlated(4, params_.velocity_noise_sigma, alpha, gain);
                vd = step_correlated(5, params_.velocity_noise_sigma, alpha, gain);
            }
            else {
                north = horizontal_sigma > 0 ? horizontal_sigma * gauss_.next() : 0.0f;
                east = horizontal_sigma > 0 ? horizontal_sigma * gauss_.next() : 0.0f;
                up = vertical_sigma > 0 ? vertical_sigma * gauss_.next() : 0.0f;
                const real_T sigma = params_.velocity_noise_sigma;
                if (sigma > 0) {
                    vn = sigma * gauss_.next();
                    ve = sigma * gauss_.next();
                    vd = sigma * gauss_.next();
                }
            }
            const real_T meters_per_degree = 111320.0f;
            const real_T cos_lat = static_cast<real_T>(
                std::cos(Utils::degreesToRadians(gnss.geo_point.latitude)));
            gnss.geo_point.latitude += north / meters_per_degree;
            gnss.geo_point.longitude += east / (meters_per_degree * std::max(cos_lat, 0.01f));
            gnss.geo_point.altitude += up;
            gnss.velocity += Vector3r(vn, ve, vd);
        }

    private:
        typedef std::normal_distribution<> NormalDistribution;

        void updateSatellites()
        {
            const TTimePoint now = clock()->nowNanos();
            const real_T dt = last_sat_time_ == 0
                ? 0.1f
                : static_cast<real_T>((now - last_sat_time_) / 1.0e9);
            last_sat_time_ = now;

            // The count drifts slowly around the middle of its range.
            const real_T mid = 0.5f * (params_.sats_min + params_.sats_max);
            const real_T half = 0.5f * (params_.sats_max - params_.sats_min);
            const real_T alpha = static_cast<real_T>(
                std::exp(-dt / std::max(params_.sats_tau, 1.0f)));
            const real_T gain = std::sqrt(std::max(0.0f, 1.0f - alpha * alpha));
            sats_offset_ = alpha * sats_offset_ + gain * 0.5f * half * gauss_.next();

            // Obstruction events (buildings, canopy) cut the visible sky.
            bool obstructed = false;
            if (obstruction_remaining_s_ > 0) {
                obstruction_remaining_s_ -= dt;
                obstructed = true;
            }
            else if (params_.obstruction_probability_per_second > 0 &&
                     params_.obstruction_seconds > 0 &&
                     uniform_.next() < params_.obstruction_probability_per_second * dt) {
                obstruction_remaining_s_ = params_.obstruction_seconds;
                obstructed = true;
            }

            real_T sats = Utils::clip(mid + sats_offset_, params_.sats_min, params_.sats_max);
            if (obstructed) {
                sats = std::max(params_.sats_min, sats * 0.55f);
            }
            sats_value_ = sats;
            current_sats_ = static_cast<int>(std::lround(sats));

            // DOP grows as the square root of the inverse satellite count.
            const real_T count = std::max(1.0f, sats);
            current_hdop_ = params_.hdop_at_reference *
                std::sqrt(params_.sats_reference / count);
            if (obstructed) {
                current_hdop_ *= params_.obstruction_hdop_factor;
            }
            current_eph_ = params_.uere * current_hdop_;
            current_epv_ = current_eph_ * params_.vdop_ratio;
        }

        real_T step_correlated(int axis, real_T sigma, real_T alpha, real_T gain)
        {
            real_T& state = correlated_state_[axis];
            state = alpha * state + gain * sigma * gauss_.next();
            return state;
        }

        GpsSimpleParams params_;
        RandomGeneratorGausianR gauss_ = RandomGeneratorGausianR(0.0f, 1.0f);
        real_T correlated_state_[6] = {};
        TTimePoint last_noise_time_ = 0;

        // satellite-geometry model state
        RandomGeneratorR uniform_ = RandomGeneratorR(0.0f, 1.0f);
        real_T sats_offset_ = 0.0f;
        real_T sats_value_ = 0.0f;
        real_T obstruction_remaining_s_ = 0.0f;
        TTimePoint last_sat_time_ = 0;
        int current_sats_ = 0;
        real_T current_hdop_ = 0.0f;
        real_T current_eph_ = 0.0f;
        real_T current_epv_ = 0.0f;

        FirstOrderFilter<real_T> eph_filter, epv_filter;
        FrequencyLimiter freq_limiter_;
        DelayLine<Output> delay_line_;
    };
}
} //namespace
#endif
