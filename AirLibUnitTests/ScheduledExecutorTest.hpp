#ifndef msr_AirLibUnitTests_ScheduledExecutorTest_hpp
#define msr_AirLibUnitTests_ScheduledExecutorTest_hpp

#include "TestBase.hpp"
#include "common/common_utils/ScheduledExecutor.hpp"

#include <chrono>
#include <thread>

namespace msr
{
namespace airlib
{

    class ScheduledExecutorTest : public TestBase
    {
    public:
        virtual void run() override
        {
            using common_utils::ScheduledExecutor;
            const auto callback = [](uint64_t) { return true; };

            // A timed pause must resume afterwards; an external pause() call while the
            // timer is pending must neither be inverted nor leave a stale timer behind.
            // Regression: the expiry used to toggle whatever the current state was, so a
            // PX4 lockstep unpause landing between the timer check and the toggle left the
            // simulation paused forever.
            for (int i = 0; i < 100; ++i) {
                ScheduledExecutor executor(callback, 1000000ULL);
                executor.start();
                executor.pauseForTime(0.002);
                executor.pause(false);
                std::this_thread::sleep_for(std::chrono::milliseconds(i % 4));
                testAssert(!executor.isPaused(), "A stale pause timer must not pause a running executor");
                executor.stop();
            }

            ScheduledExecutor executor(callback, 1000000ULL);
            executor.start();
            executor.pauseForTime(0.05);
            testAssert(executor.isPaused(), "pauseForTime must pause immediately");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            testAssert(!executor.isPaused(), "pauseForTime must resume after its period");

            executor.continueForTime(0.05);
            testAssert(!executor.isPaused(), "continueForTime must resume immediately");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            testAssert(executor.isPaused(), "continueForTime must pause after its period");

            // stop() while the executor is paused must still join its thread.
            executor.stop();
        }
    };
}
}
#endif
