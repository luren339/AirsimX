// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_ArduCopter_hpp
#define msr_airlib_vehicles_ArduCopter_hpp

#include "vehicles/multirotor/firmwares/arducopter/ArduCopterApi.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"

namespace msr
{
namespace airlib
{

    class ArduCopterParams : public MultiRotorParams
    {
    public:
        ArduCopterParams(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting, std::shared_ptr<const SensorFactory> sensor_factory)
            : sensor_factory_(sensor_factory)
        {
            connection_info_ = getConnectionInfo(vehicle_setting);
        }

        virtual ~ArduCopterParams() = default;

        virtual std::unique_ptr<MultirotorApiBase> createMultirotorApi() override
        {
            return std::unique_ptr<MultirotorApiBase>(new ArduCopterApi(this, connection_info_));
        }

    protected:
        virtual void setupParams() override
        {
            auto& params = getParams();

            // Use connection_info_.model for the model name, see Px4MultiRotorParams for example

            // Only Generic for now
            setupFrameGenericQuad(params);

            // AirLib's generic quad spins rotors 1 and 2 counter-clockwise and
            // 3 and 4 clockwise; ArduPilot's QUAD X is the opposite (motors 1
            // and 2 CW, 3 and 4 CCW). With the directions left as they are,
            // ArduPilot's yaw controller drives the diagonal motor pairs the
            // wrong way, saturates them against each other and the vehicle
            // never lifts off ("Potential Thrust Loss").
            params.rotor_poses[0].direction = RotorTurningDirection::RotorTurningDirectionCW;
            params.rotor_poses[1].direction = RotorTurningDirection::RotorTurningDirectionCW;
            params.rotor_poses[2].direction = RotorTurningDirection::RotorTurningDirectionCCW;
            params.rotor_poses[3].direction = RotorTurningDirection::RotorTurningDirectionCCW;

            // The generic frame's 1.0 kg with these rotor parameters tops out
            // barely above its own weight, so ArduPilot would have to sit near
            // full throttle just to hover. 0.8 kg gives roughly 1.5:1 thrust
            // to weight, enough to hover below full throttle while staying
            // gentle for the rate gains in airsim-quadX.parm.
            // The inertia matrix follows the new mass.
            const real_T motor_assembly_weight = 0.055f;
            params.mass = 0.8f;
            const real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;
            computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses,
                                 box_mass, motor_assembly_weight);
        }

        virtual const SensorFactory* getSensorFactory() const override
        {
            return sensor_factory_.get();
        }

        static const AirSimSettings::MavLinkConnectionInfo& getConnectionInfo(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting)
        {
            return vehicle_setting.connection_info;
        }

    private:
        AirSimSettings::MavLinkConnectionInfo connection_info_;
        std::shared_ptr<const SensorFactory> sensor_factory_;
    };
}
} //namespace
#endif
