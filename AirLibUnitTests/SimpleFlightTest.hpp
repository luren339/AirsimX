#ifndef msr_AirLibUnitTests_SimpleFlightTest_hpp
#define msr_AirLibUnitTests_SimpleFlightTest_hpp

#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "TestBase.hpp"
#include "physics/PhysicsWorld.hpp"
#include "physics/FastPhysicsEngine.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "common/SteppableClock.hpp"
#include "vehicles/multirotor/MultiRotorPhysicsBody.hpp"
#include "vehicles/multirotor/firmwares/simple_flight/SimpleFlightApi.hpp"

namespace msr
{
namespace airlib
{

    // Deterministic controller tests: inject contact and kinematics, without
    // starting a physics thread or requiring an Unreal collision surface.
    class SimpleFlightLandingTest : public TestBase
    {
        class TestApi : public SimpleFlightApi
        {
        public:
            using SimpleFlightApi::SimpleFlightApi;
            using SimpleFlightApi::commandMotorPWMs;
            using SimpleFlightApi::commandVelocity;
            using SimpleFlightApi::getLandedState;
        };

    public:
        virtual void run() override
        {
            AirSimSettings::initializeSettings(R"({
                "SettingsVersion": 1.2,
                "SimMode": "Multirotor",
                "Vehicles": {"SimpleFlight": {"VehicleType": "SimpleFlight"}}
            })");
            AirSimSettings::singleton().load([] { return "Multirotor"; });
            testAssert(AirSimSettings::singleton().error_messages.empty(), "Invalid SimpleFlight settings");

            const auto clock = std::make_shared<SteppableClock>(0.02);
            ClockFactory::get(clock);
            auto* setting = AirSimSettings::singleton().getVehicleSetting("SimpleFlight");
            auto params = MultiRotorParamsFactory::createConfig(setting, std::make_shared<SensorFactory>());
            TestApi api(params.get(), setting);
            Kinematics kinematics;
            Environment environment(Environment::State(Vector3r::Zero(), GeoPoint(47.641468, -122.140165, 122.0f)));
            kinematics.reset();
            environment.reset();
            MultiRotorPhysicsBody vehicle(params.get(), &api, &kinematics, &environment);
            api.setSimulatedGroundTruth(&kinematics.getState(), &environment);
            api.reset();
            vehicle.setGrounded(true);
            api.enableApiControl(true);
            testAssert(api.isApiControlEnabled(), "Landing test could not obtain API control");
            testAssert(api.armDisarm(true), "Landing test could not arm");

            auto tick = [&](int count, bool send_descent = true) {
                for (int i = 0; i < count; ++i) {
                    clock->step();
                    if (send_descent)
                        api.commandVelocity(0, 0, 0.5f, YawMode::Zero());
                    api.update();
                }
            };
            auto is_landed = [&] { return api.getLandedState() == LandedState::Landed; };
            tick(1);
            testAssert(is_landed(), "Initial ground contact must report landed");

            auto state = Kinematics::State::zero();
            state.pose.position.z() = -3;
            kinematics.setState(state);
            vehicle.setGrounded(false);
            tick(1);
            testAssert(!is_landed(), "Losing contact must clear landed state even at zero velocity");
            api.commandVelocity(0, 0, 0, YawMode::Zero());
            tick(300, false); // Includes the API timeout transition to position hold.
            testAssert(!is_landed(), "Stationary hover must remain Flying after six seconds");

            // Neither near-home altitude nor low/zero motor outputs prove contact.
            state.pose.position.z() = -0.01f;
            kinematics.setState(state);
            for (int i = 0; i < 300; ++i) {
                api.commandMotorPWMs(0, 0, 0, 0);
                tick(1, false);
            }
            testAssert(!is_landed(), "Airborne stationary vehicle near home must not be marked landed");

            // A landing surface need not be at the takeoff altitude.
            state.pose.position.z() = -8;
            kinematics.setState(state);
            vehicle.setGrounded(true);
            tick(5);
            testAssert(!is_landed(), "Brief ground contact must not complete landing");
            vehicle.setGrounded(false);
            tick(1);
            vehicle.setGrounded(true);
            tick(6);
            testAssert(!is_landed(), "Interrupted contact must restart landing confirmation");

            state.twist.angular.z() = 0.2f;
            kinematics.setState(state);
            tick(20);
            testAssert(!is_landed(), "Angular motion must prevent landing confirmation");
            state.twist.angular.setZero();
            state.twist.linear.x() = 0.2f;
            kinematics.setState(state);
            tick(20);
            testAssert(!is_landed(), "Horizontal motion must prevent landing confirmation");
            state.twist.linear.setZero();
            kinematics.setState(state);
            tick(12);
            testAssert(api.getActuation(0) > simple_flight::Params::min_armed_throttle(),
                       "Regression must exercise landing above the armed-idle motor output");
            testAssert(is_landed(), "Stationary ground contact must land despite velocity-controller minimum throttle");
            tick(300, false);
            testAssert(is_landed(), "Lack of new API commands must not undo confirmed landing");

            vehicle.setGrounded(false);
            tick(1);
            testAssert(!is_landed(), "A second takeoff must clear landed state");
            vehicle.setGrounded(true);
            tick(5);
            api.reset();
            vehicle.setGrounded(false);
            api.enableApiControl(true);
            tick(1);
            vehicle.setGrounded(true);
            tick(6);
            testAssert(!is_landed(), "Reset must clear an in-progress landing timer");
            tick(6, false); // Let the API goal time out while contact is being confirmed.
            testAssert(is_landed(), "Landing must work after reset and an API timeout to position hold");

            // Estimators without a physics parent keep the legacy throttle-based
            // fallback; stationary flight at powered throttle is not a landing.
            api.setParent(nullptr);
            state.twist.linear.z() = -0.5f;
            kinematics.setState(state);
            api.commandMotorPWMs(0.5f, 0.5f, 0.5f, 0.5f);
            tick(2, false);
            testAssert(!is_landed(), "Unknown contact must retain throttle-and-motion takeoff detection");
            state.twist.linear.setZero();
            kinematics.setState(state);
            tick(300, false);
            testAssert(!is_landed(), "Unknown contact must not turn stationary powered hover into landing");
            api.commandMotorPWMs(0, 0, 0, 0);
            tick(14, false);
            testAssert(is_landed(), "A single low-throttle stop must land without an API-timeout hover restarting motors");

            api.setParent(&vehicle);
            vehicle.reset();
            AirSimSimpleFlightEstimator estimator;
            estimator.setGroundTruthPhysicsBody(&vehicle);
            testAssert(estimator.getGroundContactState() == simple_flight::IStateEstimator::GroundContactState::Unknown,
                       "A physics parent without contact updates must not imply known airborne state");
            FastPhysicsEngine no_ground_lock(false);
            no_ground_lock.insert(&vehicle);
            no_ground_lock.reset();
            testAssert(estimator.getGroundContactState() == simple_flight::IStateEstimator::GroundContactState::Unknown,
                       "Disabling ground locking must preserve the unknown-contact fallback");
            vehicle.setGrounded(false);
            testAssert(estimator.getGroundContactState() == simple_flight::IStateEstimator::GroundContactState::Airborne,
                       "Explicit airborne contact state must remain distinguishable from unknown");
            for (bool was_grounded : { false, true }) {
                vehicle.setGrounded(was_grounded);
                vehicle.invalidateGroundContact();
                testAssert(!vehicle.isGrounded(), "Invalidating contact must release the previous ground lock");
                testAssert(estimator.getGroundContactState() == simple_flight::IStateEstimator::GroundContactState::Unknown,
                           "Pose or kinematics changes must invalidate contact, not assert airborne state");
                state.twist.linear.z() = -0.5f;
                kinematics.setState(state);
                api.commandMotorPWMs(0.5f, 0.5f, 0.5f, 0.5f);
                tick(2, false);
                testAssert(!is_landed(), "Invalidated contact must retain legacy takeoff detection");
                state.twist.linear.setZero();
                kinematics.setState(state);
                api.commandMotorPWMs(0, 0, 0, 0);
                tick(14, false);
                testAssert(is_landed(), "Invalidated contact must not disable low-throttle landing until reset");
            }
        }
    };

    class SimpleFlightTest : public TestBase
    {
    public:
        virtual void run() override
        {
            AirSimSettings::initializeSettings(R"({
                "SettingsVersion": 1.2,
                "SimMode": "Multirotor",
                "Vehicles": {"SimpleFlight": {"VehicleType": "SimpleFlight"}}
            })");
            AirSimSettings::singleton().load([] { return "Multirotor"; });
            testAssert(AirSimSettings::singleton().error_messages.empty(), "Invalid SimpleFlight settings");

            const auto clock = std::make_shared<SteppableClock>(3E-3);
            ClockFactory::get(clock);
            auto params = MultiRotorParamsFactory::createConfig(
                AirSimSettings::singleton().getVehicleSetting("SimpleFlight"),
                std::make_shared<SensorFactory>());
            auto api = params->createMultirotorApi();
            testAssert(api != nullptr, "SimpleFlight API was not created");

            Kinematics kinematics(Kinematics::State::zero());
            Environment::State initial_environment;
            initial_environment.position = Vector3r::Zero();
            initial_environment.geo_point = GeoPoint(47.641468, -122.140165, 122.0f);
            Environment environment(initial_environment);
            kinematics.reset();
            environment.reset();
            MultiRotorPhysicsBody vehicle(params.get(), api.get(), &kinematics, &environment);
            api->setSimulatedGroundTruth(&kinematics.getState(), &environment);
            api->reset();

            const std::vector<UpdatableObject*> vehicles = { &vehicle };
            PhysicsWorld physics_world(std::unique_ptr<PhysicsEngineBase>(new FastPhysicsEngine()),
                                       vehicles, 3000000LL, false, false);
            // This core-only test has no Unreal collision surface to establish ground contact.
            vehicle.setGrounded(true);
            std::string message;
            const bool ready = api->isReady(message);
            testAssert(ready, "SimpleFlight not ready: " + message);
            api->enableApiControl(true);
            testAssert(api->isApiControlEnabled(), "API control was not enabled");
            testAssert(api->armDisarm(true), "SimpleFlight failed to arm");

            physics_world.startAsyncUpdator();
            const bool took_off = api->takeoff(15.0f);
            physics_world.stopAsyncUpdator();
            if (!took_off) {
                std::vector<std::string> messages;
                api->getStatusMessages(messages);
                for (const auto& status : messages)
                    std::cerr << status << std::endl;
                std::cerr << "Takeoff position: " << kinematics.getState().pose.position.transpose()
                          << ", velocity: " << kinematics.getState().twist.linear.transpose()
                          << ", simulation seconds: " << clock->elapsedSince(clock->getStart()) << std::endl;
            }
            testAssert(took_off, "Takeoff did not finish within 15 simulation seconds");
            assertFinite(kinematics.getState());
            testAssert(kinematics.getState().pose.position.z() < -1.0f, "Takeoff did not gain altitude in NED coordinates");
            const Vector3r near_position = kinematics.getState().pose.position;
            testAssert(!api->moveToPosition(near_position.x(), near_position.y(), near_position.z(), 1.0f, 0.0f,
                                            DrivetrainType::MaxDegreeOfFreedom, YawMode::Zero(), -1, 0),
                       "An immediately timed-out command must not report success");

            const Vector3r target(-5, -5, -5);
            physics_world.startAsyncUpdator();
            const bool reached_target = api->moveToPosition(target.x(), target.y(), target.z(), 5.0f, 15.0f,
                                                           DrivetrainType::MaxDegreeOfFreedom, YawMode(true, 0), -1, 0);
            physics_world.stopAsyncUpdator();
            testAssert(reached_target, "Position command did not finish within 15 simulation seconds");
            assertFinite(kinematics.getState());
            testAssert((kinematics.getState().pose.position - target).norm() < 0.5f,
                       "Vehicle did not reach the target within 0.5 meters");
            testAssert(clock->elapsedSince(clock->getStart()) > 0, "Simulation clock did not advance");
            testAssert(api->armDisarm(false), "SimpleFlight failed to disarm");
            api->enableApiControl(false);
            testAssert(!api->isApiControlEnabled(), "API control was not released");

            kinematics.reset();
            environment.reset();
            api->reset();
            physics_world.reset();
            assertFinite(kinematics.getState());
            testAssert(kinematics.getState().pose.position.norm() < 1E-5f, "Reset did not restore the initial position");
            testAssert(kinematics.getState().twist.linear.norm() < 1E-5f, "Reset did not restore zero velocity");
        }

    private:
        void assertFinite(const Kinematics::State& state)
        {
            testAssert(state.pose.position.allFinite(), "Position contains non-finite values");
            testAssert(state.twist.linear.allFinite(), "Velocity contains non-finite values");
            testAssert(state.pose.orientation.coeffs().allFinite(), "Orientation contains non-finite values");
            testAssert(std::abs(state.pose.orientation.norm() - 1.0f) < 1E-3f, "Orientation is not normalized");
        }
    };
}
}
#endif
