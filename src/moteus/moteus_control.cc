// Copyright 2020 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file
///
/// This is a simple application that demonstrates how to efficiently
/// monitor and control multiple moteus servos at a high rate using
/// the pi3hat.read/write test
///
/// It is contained in a single file for the purposes of
/// demonstration.  A real application should likely be implemented in
/// multiple translation units or structured for longer term
/// maintenance.

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

#include "pi3hat_moteus_control/moteus_protocol.h"
#include "pi3hat_moteus_control/pi3hat_moteus_interface.h"

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;

namespace {
struct Arguments {
  Arguments(const std::vector<std::string>& args) {
    for (size_t i = 0; i < args.size(); i++) {
      const auto& arg = args[i];
      if (arg == "-h" || arg == "--help") {
        help = true;
      } else if (arg == "--main-cpu") {
        main_cpu = std::stoull(args.at(++i));
      } else if (arg == "--can-cpu") {
        can_cpu = std::stoull(args.at(++i));
      } else if (arg == "--period-s") {
        period_s = std::stod(args.at(++i));
      } else if (arg == "--primary-id") {
        primary_id = std::stoull(args.at(++i));
      } else if (arg == "--primary-bus") {
        primary_bus = std::stoull(args.at(++i));
      } else if (arg == "--secondary-id") {
        secondary_id = std::stoull(args.at(++i));
      } else if (arg == "--secondary-bus") {
        secondary_bus = std::stoull(args.at(++i));
      } else {
        throw std::runtime_error("Unknown argument: " + arg);
      }
    }
  }

  bool help = false;
  int main_cpu = 1;
  int can_cpu = 2;
  double period_s = 0.01;
  int primary_id = 3;
  int primary_bus = 1;
  int secondary_id = 6;
  int secondary_bus = 1;
};

void LockMemory() {
  {
    const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
    if (r < 0) {
      throw std::runtime_error("Error locking memory");
    }
  }
}

std::pair<double, double> MinMaxVoltage(
    const std::vector<MoteusInterface::ServoReply>& r) {
  double rmin = std::numeric_limits<double>::infinity();
  double rmax = -std::numeric_limits<double>::infinity();

  for (const auto& i : r) {
    if (i.result.voltage > rmax) { rmax = i.result.voltage; }
    if (i.result.voltage < rmin) { rmin = i.result.voltage; }
  }

  return std::make_pair(rmin, rmax);
}

/// This holds the user-defined control logic.
class SampleController {
 public:
  SampleController(const Arguments& arguments) : arguments_(arguments) {
  }

  std::vector<std::pair<int, int>> servo_bus_map() const {
    return {
      { arguments_.primary_id, arguments_.primary_bus },
      { arguments_.secondary_id, arguments_.secondary_bus },
    };
  }

  void Initialize(std::vector<MoteusInterface::ServoCommand>* commands) {
    moteus::PositionResolution res;
    res.position = moteus::Resolution::kInt16;
    res.velocity = moteus::Resolution::kInt16;
    res.feedforward_torque = moteus::Resolution::kInt16;
    res.kp_scale = moteus::Resolution::kInt16;
    res.kd_scale = moteus::Resolution::kInt16;
    res.maximum_torque = moteus::Resolution::kIgnore;
    res.stop_position = moteus::Resolution::kIgnore;
    res.watchdog_timeout = moteus::Resolution::kIgnore;
    for (auto& cmd : *commands) {
      cmd.resolution = res;
    }
  }

  moteus::QueryResult Get(const std::vector<MoteusInterface::ServoReply>& replies, int id, int bus) {
    for (const auto& item : replies) {
      if (item.id == id && item.bus == bus) { return item.result; }
    }
    return {};
  }

  void Run(const std::vector<MoteusInterface::ServoReply>& status,
           std::vector<MoteusInterface::ServoCommand>* output) {
    cycle_count_++;

    // This is where your control loop would go.

    if (cycle_count_ < 5) {
      for (auto& cmd : *output) {
        // We start everything with a stopped command to clear faults.
        cmd.mode = moteus::Mode::kStopped;
      }
    } else {
      // Then we make the secondary servo mirror the primary servo.
      if (1) {
        auto& secondary_out = output->at(0);  // We constructed this, so we know the order.
        secondary_out.mode = moteus::Mode::kPosition;
        secondary_out.position.position = std::numeric_limits<double>::quiet_NaN();        
        secondary_out.position.velocity = 0.1;
        secondary_out.position.maximum_torque = 0.5;

        auto& secondary_out_2 = output->at(1);
        secondary_out_2.mode = moteus::Mode::kPosition;
        secondary_out_2.position.position = std::numeric_limits<double>::quiet_NaN();
        secondary_out_2.position.velocity = 0.1;
        secondary_out_2.position.maximum_torque = 0.5;
      }
    }
  }

 private:
  const Arguments arguments_;
  uint64_t cycle_count_ = 0;
};

template <typename Controller>
void Run(const Arguments& args, Controller* controller) {

  moteus::ConfigureRealtime(args.main_cpu);
  MoteusInterface::Options moteus_options;
  moteus_options.cpu = args.can_cpu;
  MoteusInterface moteus_interface{moteus_options};

  std::vector<MoteusInterface::ServoCommand> commands;
  for (const auto& pair : controller->servo_bus_map()) {
    commands.push_back({});
    commands.back().id = pair.first;
    commands.back().bus = pair.second;
  }

  std::vector<MoteusInterface::ServoReply> replies{commands.size()};
  std::vector<MoteusInterface::ServoReply> saved_replies;

  controller->Initialize(&commands);

  MoteusInterface::Data moteus_data;
  moteus_data.commands = { commands.data(), commands.size() };
  moteus_data.replies = { replies.data(), replies.size() };

  std::future<MoteusInterface::Output> can_result;

  const auto period =
      std::chrono::microseconds(static_cast<int64_t>(args.period_s * 1e6));
  auto next_cycle = std::chrono::steady_clock::now() + period;

  const auto status_period = std::chrono::milliseconds(100);
  auto next_status = next_cycle + status_period;
  uint64_t cycle_count = 0;
  double total_margin = 0.0;
  uint64_t margin_cycles = 0;

  // We will run at a fixed cycle time.
  while (true) {
    cycle_count++;
    margin_cycles++;
    {
      const auto now = std::chrono::steady_clock::now();
      if (now > next_status) {
        // NOTE: iomanip is not a recommended pattern.  We use it here
        // simply to not require any external dependencies, like 'fmt'.
        const auto volts = MinMaxVoltage(saved_replies);
        const std::string modes = [&]() {
          std::ostringstream result;
          result.precision(4);
          result << std::fixed;
          for (const auto& item : saved_replies) {
            result << item.id << "/"
                   << item.bus << "/"
                   << static_cast<int>(item.result.mode) << "/"
                   << item.result.position << "/"
                   << item.result.velocity << "/"
                   << item.result.torque <<" ";
          }
          return result.str();
        }();
        std::cout << std::setprecision(6) << std::fixed
                  << "Cycles " << cycle_count
                  << "  margin: " << (total_margin / margin_cycles)
                  << std::setprecision(1)
                  << "  volts: " << volts.first << "/" << volts.second
                  << "  modes: " << modes
                  << "   \r";
        std::cout.flush();
        next_status += status_period;
        total_margin = 0;
        margin_cycles = 0;
      }

      int skip_count = 0;
      while (now > next_cycle) {
        skip_count++;
        next_cycle += period;
      }
      if (skip_count) {
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

  // 보내고 받는 부분 테스트 필요
    if (can_result.valid()) {
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
        [promise](const MoteusInterface::Output& output) {
          // This is called from an arbitrary thread, so we just set
          // the promise value here.
          promise->set_value(output);
        });
    can_result = promise->get_future();
  }
}
}

int main(int argc, char** argv) {
  Arguments args({argv + 1, argv + argc});

  // Lock memory for the whole process.
  LockMemory();

  SampleController sample_controller{args};
  Run(args, &sample_controller);

  return 0;
}
