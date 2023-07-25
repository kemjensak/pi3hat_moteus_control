#include "pi3hat_moteus_control/pi3hat_moteus_control.hpp"
#include <sys/mman.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <future>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;

namespace
{

  void LockMemory()
  {
    {
      const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
      if (r < 0)
      {
        throw std::runtime_error("Error locking memory");
      }
    }
  }

  template <typename Controller>
  void Run(Controller *controller)
  {
    // Initialize Moteus
    moteus::ConfigureRealtime(1);
    MoteusInterface::Options moteus_options;
    moteus_options.cpu = 2;
    MoteusInterface moteus_interface{moteus_options};

    // Set servomap
    std::vector<MoteusInterface::ServoCommand> commands;
    for (const auto &pair : controller->servo_bus_map())
    {
      commands.push_back({});
      commands.back().id = pair.first;
      commands.back().bus = pair.second;
    }

    std::vector<MoteusInterface::ServoReply> replies{commands.size()};
    std::vector<MoteusInterface::ServoReply> saved_replies;

    controller->Initialize(&commands);

    MoteusInterface::Data moteus_data;
    moteus_data.commands = {commands.data(), commands.size()};
    moteus_data.replies = {replies.data(), replies.size()};

    std::future<MoteusInterface::Output> can_result;

    const auto period =
        std::chrono::microseconds(static_cast<int64_t>(0.01 * 1e6));
    auto next_cycle = std::chrono::steady_clock::now() + period;

    const auto status_period = std::chrono::milliseconds(100);
    auto next_status = next_cycle + status_period;
    uint64_t cycle_count = 0;
    double total_margin = 0.0;
    uint64_t margin_cycles = 0;

    // We will run at a fixed cycle time.
    while (true)
    {
      cycle_count++;
      margin_cycles++;
      {
        const auto now = std::chrono::steady_clock::now();
        int skip_count = 0;
        while (now > next_cycle)
        {
          skip_count++;
          next_cycle += period;
        }
        if (skip_count)
        {
          std::cout << "\nSkipped " << skip_count << " cycles\n";
        }
      }
      // Wait for the next control cycle to come up.
      {
        const auto pre_sleep = std::chrono::steady_clock::now();
        std::this_thread::sleep_until(next_cycle);
        const auto post_sleep = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = post_sleep - pre_sleep;
        total_margin += elapsed.count();
      }
      next_cycle += period;

      controller->Run(saved_replies, &commands);

      if (can_result.valid())
      {
        // Now we get the result of our last query and send off our new
        // one.
        const auto current_values = can_result.get();

        // We copy out the results we just got out.
        const auto rx_count = current_values.query_result_size;
        saved_replies.resize(rx_count);
        std::copy(replies.begin(), replies.begin() + rx_count,
                  saved_replies.begin());
      }

      // Then we can immediately ask them to be used again.
      auto promise = std::make_shared<std::promise<MoteusInterface::Output>>();
      moteus_interface.Cycle(
          moteus_data,
          [promise](const MoteusInterface::Output &output)
          {
            // This is called from an arbitrary thread, so we just set
            // the promise value here.
            promise->set_value(output);
          });
      can_result = promise->get_future();
    }
  }

}

int main(int argc, char **argv)
{
  LockMemory();
  rclcpp::init(argc, argv);
  SampleController sample_controller{};
  Run(&sample_controller);
  rclcpp::shutdown();
  return 0;
}
