// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_SimpleImuParams_hpp
#define msr_airlib_SimpleImuParams_hpp

#include "common/Common.hpp"
#include "common/EarthUtils.hpp"
#include "common/AirSimSettings.hpp"
#include <cmath>

namespace msr
{
namespace airlib
{

    // A description of the parameters:
    // https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model-and-Intrinsics
    struct ImuSimpleParams
    {
        /* ref: Parameter values are for MPU 6000 IMU from InvenSense 
    Design and Characterization of a Low Cost MEMS IMU Cluster for Precision Navigation
    Daniel R. Greenheck, 2009, sec 2.2, pp 17
    http://epublications.marquette.edu/cgi/viewcontent.cgi?article=1326&context=theses_open
    Datasheet:
    https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
    For Allan Variance/Deviation plots see http://www.invensense.com/wp-content/uploads/2015/02/MPU-3300-Datasheet.pdf
    */
        struct Gyroscope
        {
            //angular random walk (ARW)
            real_T arw = 0.30f / sqrt(3600.0f) * M_PIf / 180; //deg/sqrt(hour) converted to rad/sqrt(sec)
            //Bias Stability (tau = 500s)
            real_T tau = 500;
            real_T bias_stability = 4.6f / 3600 * M_PIf / 180; //deg/hr converted to rad/sec
            Vector3r turn_on_bias = Vector3r::Zero(); //assume calibration is done

            //Calibration-curve errors. scale_factor multiplies every axis
            //(1 = ideal), misalignment leaks each axis into the other two
            //(fraction), and turn_on_bias is a fixed offset.
            real_T scale_factor = 1.0f;
            real_T misalignment = 0.0f;
        } gyro;

        struct Accelerometer
        {
            //velocity random walk (ARW)
            real_T vrw = 0.24f * EarthUtils::Gravity / 1.0E3f; //mg converted to m/s^2
            //Bias Stability (tau = 800s)
            real_T tau = 800;
            real_T bias_stability = 36.0f * 1E-6f * EarthUtils::Gravity; //ug converted to m/s^2
            Vector3r turn_on_bias = Vector3r::Zero(); //assume calibration is done

            //Calibration-curve errors. Real MEMS accelerometers are not
            //linear over their range: the output curvature grows with the
            //applied specific force. nonlinearity is the fractional output
            //deviation at 1 g; the term applied is nonlinearity * a * (|a|/g),
            //so it vanishes at rest and at small accelerations and reaches
            //the stated fraction at 1 g. scale_factor and misalignment are
            //the same per-axis and cross-axis errors as on the gyroscope.
            real_T scale_factor = 1.0f;
            real_T nonlinearity = 0.0f;
            real_T misalignment = 0.0f;
        } accel;

        real_T min_sample_time = 1 / 1000.0f; //internal IMU frequency

        void initializeFromSettings(const AirSimSettings::ImuSetting& settings)
        {
            const auto& json = settings.settings;
            float arw = json.getFloat("AngularRandomWalk", Utils::nan<float>());
            if (!std::isnan(arw)) {
                gyro.arw = arw / sqrt(3600.0f) * M_PIf / 180; // //deg/sqrt(hour) converted to rad/sqrt(sec)
            }
            gyro.tau = json.getFloat("GyroBiasStabilityTau", gyro.tau);
            float bias_stability = json.getFloat("GyroBiasStability", Utils::nan<float>());
            if (!std::isnan(bias_stability)) {
                gyro.bias_stability = bias_stability / 3600 * M_PIf / 180; //deg/hr converted to rad/sec
            }
            auto vrw = json.getFloat("VelocityRandomWalk", Utils::nan<float>());
            if (!std::isnan(vrw)) {
                accel.vrw = vrw * EarthUtils::Gravity / 1.0E3f; //mg converted to m/s^2
            }
            accel.tau = json.getFloat("AccelBiasStabilityTau", accel.tau);
            bias_stability = json.getFloat("AccelBiasStability", Utils::nan<float>());
            if (!std::isnan(bias_stability)) {
                accel.bias_stability = bias_stability * 1E-6f * EarthUtils::Gravity; //ug converted to m/s^2
            }

            //calibration curve
            accel.scale_factor = json.getFloat("AccelScaleFactor", accel.scale_factor);
            accel.nonlinearity = json.getFloat("AccelNonlinearity", accel.nonlinearity);
            accel.misalignment = json.getFloat("AccelMisalignment", accel.misalignment);
            float turn_on_bias_mg = json.getFloat("AccelTurnOnBias", Utils::nan<float>());
            if (!std::isnan(turn_on_bias_mg)) {
                accel.turn_on_bias = Vector3r::Constant(
                    turn_on_bias_mg * 1E-3f * EarthUtils::Gravity);
            }
            gyro.scale_factor = json.getFloat("GyroScaleFactor", gyro.scale_factor);
            gyro.misalignment = json.getFloat("GyroMisalignment", gyro.misalignment);
            float gyro_turn_on_bias_dps = json.getFloat("GyroTurnOnBias", Utils::nan<float>());
            if (!std::isnan(gyro_turn_on_bias_dps)) {
                gyro.turn_on_bias = Vector3r::Constant(
                    gyro_turn_on_bias_dps * M_PIf / 180.0f);
            }
        }
    };
}
} //namespace
#endif
