// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_GpsSimpleParams_hpp
#define msr_airlib_GpsSimpleParams_hpp

#include "common/Common.hpp"

namespace msr
{
namespace airlib
{

    struct GpsSimpleParams
    {
        real_T eph_time_constant = 0.9f, epv_time_constant = 0.9f;
        real_T eph_initial = 100.0f, epv_initial = 100.0f; //initially fully diluted positions
        real_T eph_final = 0.1f, epv_final = 0.1f; // PX4 won't converge GPS sensor fusion until we get to 10% accuracty.
        real_T eph_min_3d = 3.0f, eph_min_2d = 4.0f;

        real_T update_latency = 0.2f; //sec
        real_T update_frequency = 50; //Hz
        real_T startup_delay = 1; //sec

        // Per-sample position and velocity errors. Historically the position
        // was exact and only the reported accuracy (EPH/EPV) was modelled,
        // which leaves an estimator fused with it unrealistically clean.
        // Defaults are 0 (off) so existing setups are unchanged; a profile
        // that wants realistic GNSS behaviour sets them in the sensor's
        // settings. Values are 1-sigma: horizontal and vertical position in
        // metres, velocity per axis in m/s.
        real_T position_noise_sigma = 0.0f;
        real_T vertical_noise_sigma = 0.0f;
        real_T velocity_noise_sigma = 0.0f;
        // Correlation time of the errors in seconds. 0 draws an independent
        // error per sample (white noise); a positive value models the slowly
        // varying, time-correlated error of a real receiver (multipath,
        // ionosphere) as a first-order Gauss-Markov process with this tau.
        real_T noise_tau = 0.0f;

        // Satellite-geometry model. When enabled, the number of satellites
        // drifts slowly and the reported accuracy - and the injected error -
        // follow the dilution of precision that number implies, instead of
        // the fixed PositionNoiseSigma/VerticalNoiseSigma:
        //   hdop = hdop_at_reference_sats * sqrt(reference_sats / sats)
        //   sigma_h = uere * hdop,  sigma_v = uere * hdop * vdop_ratio
        // The fix degrades (2D, then none) as the count falls.
        bool satellite_model = false;
        real_T uere = 0.8f;              // user-equivalent range error, m (1 sigma)
        real_T sats_min = 6.0f, sats_max = 15.0f;
        real_T sats_reference = 10.0f;   // count at which hdop_at_reference holds
        real_T hdop_at_reference = 0.9f;
        real_T vdop_ratio = 1.6f;        // VDOP / HDOP
        real_T sats_tau = 120.0f;        // s, correlation time of the count drift
        // Below these counts the solution is no longer 3D / 2D.
        int sats_min_3d = 5;
        int sats_min_2d = 4;
        // Occasional obstruction events (buildings, canopy): while active,
        // the satellite count is cut and the error grows.
        real_T obstruction_probability_per_second = 0.0f;
        real_T obstruction_seconds = 0.0f;
        real_T obstruction_hdop_factor = 3.0f;

        void initializeFromSettings(const AirSimSettings::GpsSetting& settings)
        {
            const auto& json = settings.settings;
            eph_time_constant = json.getFloat("EPH_TimeConstant", eph_time_constant);
            epv_time_constant = json.getFloat("EPV_TimeConstant", epv_time_constant);
            eph_initial = json.getFloat("EphInitial", eph_initial);
            epv_initial = json.getFloat("EpvInitial", epv_initial);
            eph_final = json.getFloat("EphFinal", eph_final);
            epv_final = json.getFloat("EpvFinal", epv_final);
            eph_min_3d = json.getFloat("EphMin3d", eph_min_3d);
            eph_min_2d = json.getFloat("EphMin2d", eph_min_2d);
            update_latency = json.getFloat("UpdateLatency", update_latency);
            update_frequency = json.getFloat("UpdateFrequency", update_frequency);
            startup_delay = json.getFloat("StartupDelay", startup_delay);
            position_noise_sigma = json.getFloat("PositionNoiseSigma", position_noise_sigma);
            vertical_noise_sigma = json.getFloat("VerticalNoiseSigma", vertical_noise_sigma);
            velocity_noise_sigma = json.getFloat("VelocityNoiseSigma", velocity_noise_sigma);
            noise_tau = json.getFloat("NoiseTau", noise_tau);
            satellite_model = json.getBool("SatelliteModel", satellite_model);
            uere = json.getFloat("Uere", uere);
            sats_min = json.getFloat("SatsMin", sats_min);
            sats_max = json.getFloat("SatsMax", sats_max);
            sats_reference = json.getFloat("SatsReference", sats_reference);
            hdop_at_reference = json.getFloat("HdopAtReference", hdop_at_reference);
            vdop_ratio = json.getFloat("VdopRatio", vdop_ratio);
            sats_tau = json.getFloat("SatsTau", sats_tau);
            sats_min_3d = json.getInt("SatsMin3d", sats_min_3d);
            sats_min_2d = json.getInt("SatsMin2d", sats_min_2d);
            obstruction_probability_per_second =
                json.getFloat("ObstructionProbabilityPerSecond", obstruction_probability_per_second);
            obstruction_seconds = json.getFloat("ObstructionSeconds", obstruction_seconds);
            obstruction_hdop_factor = json.getFloat("ObstructionHdopFactor", obstruction_hdop_factor);
        }
    };
}
} //namespace
#endif
