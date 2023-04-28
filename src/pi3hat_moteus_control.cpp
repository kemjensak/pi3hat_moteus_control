#include "rclcpp/rclcpp.hpp"
#include "moteus_msgs/msg/moteus_state_array.hpp"
#include "moteus_msgs/msg/moteus_command_array.hpp"
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

namespace
{
  struct Arguments
  {
    Arguments(const std::vector<std::string> &args)
    {
    }
    bool help = false;
    int main_cpu = 1;
    int can_cpu = 2;
    double period_s = 0.01;
    int primary_id = 1;
    int primary_bus = 1;
    int secondary_id = 2;
    int secondary_bus = 1;
  };

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

  std::pair<double, double> MinMaxVoltage(
      const std::vector<MoteusInterface::ServoReply> &r)
  {
    double rmin = std::numeric_limits<double>::infinity();
    double rmax = -std::numeric_limits<double>::infinity();

    for (const auto &i : r)
    {
      if (i.result.voltage > rmax)
      {
        rmax = i.result.voltage;
      }
      if (i.result.voltage < rmin)
      {
        rmin = i.result.voltage;
      }
    }

    return std::make_pair(rmin, rmax);
  }

  void MoteusCommand_callback(const moteus_msgs::msg::MoteusCommandArray::SharedPtr msg)
  {
    std::cout << "MoteusCommand_callback" << std::endl;
  }

  /// This holds the user-defined control logic.
  class SampleController
  {
  public:
    SampleController(const Arguments &arguments) : arguments_(arguments)
    {
    }

    std::vector<std::pair<int, int>> servo_bus_map() const
    {
      return {
          {arguments_.primary_id, arguments_.primary_bus},
          {arguments_.secondary_id, arguments_.secondary_bus},
          {3, 1},
      };
    }

    void Initialize(std::vector<MoteusInterface::ServoCommand> *commands)
    {
      moteus::PositionResolution res;
      res.position = moteus::Resolution::kInt16;
      res.velocity = moteus::Resolution::kInt16;
      res.feedforward_torque = moteus::Resolution::kInt16;
      res.kp_scale = moteus::Resolution::kInt16;
      res.kd_scale = moteus::Resolution::kInt16;
      res.maximum_torque = moteus::Resolution::kInt16;
      res.stop_position = moteus::Resolution::kIgnore;
      res.watchdog_timeout = moteus::Resolution::kIgnore;
      for (auto &cmd : *commands)
      {
        cmd.resolution = res;
      }
    }

    moteus::QueryResult Get(const std::vector<MoteusInterface::ServoReply> &replies, int id, int bus)
    {
      for (const auto &item : replies)
      {
        if (item.id == id && item.bus == bus)
        {
          return item.result;
        }
      }
      return {};
    }

    void Run(const std::vector<MoteusInterface::ServoReply> &replies,
             std::vector<MoteusInterface::ServoCommand> *output,
             const moteus_msgs::msg::MoteusCommandArray &msg_input,
             moteus_msgs::msg::MoteusStateArray &msg_output)
    {
      cycle_count_++;

      // This is where your control loop would go.

      if (cycle_count_ < 5)
      {
        for (auto &cmd : *output)
        {
          // We start everything with a stopped command to clear faults.
          cmd.mode = moteus::Mode::kStopped;
        }
      }
      else
      {
        for (auto &m_cmd : msg_input.moteus_commands)
        {
          auto &out = output->at(m_cmd.id - 1);
          out.mode = moteus::Mode::kPosition;  
          out.position.position = m_cmd.position;
          out.position.velocity = m_cmd.velocity;
          out.position.maximum_torque = m_cmd.maximum_torque;
        }

        // auto &out = output->at(0);
        // out.mode = moteus::Mode::kPosition;
        // out.position.position = std::numeric_limits<double>::quiet_NaN();
        // out.position.velocity = 0.1;
        // out.position.maximum_torque = 3.0;

        // const auto primary = Get(status, arguments_.primary_id, arguments_.primary_bus);
        msg_output.moteus_states.clear();
        for (auto reply : replies)
        {
          moteus_msgs::msg::MoteusState moteus_state;
          moteus_state.id = reply.id;
          moteus_state.position = reply.result.position;
          moteus_state.velocity = reply.result.velocity;
          moteus_state.torque = reply.result.torque;
          moteus_state.q_current = reply.result.q_current;
          moteus_state.d_current = reply.result.d_current;
          moteus_state.rezero_state = reply.result.rezero_state;
          moteus_state.voltage = reply.result.voltage;
          moteus_state.temperature = reply.result.temperature;
          moteus_state.fault = reply.result.fault;

          msg_output.moteus_states.push_back(moteus_state);
          std::cout << "id: " << reply.id << std::endl;
          std::cout << "position: " << reply.result.position << std::endl;
          std::cout << "velocity: " << reply.result.velocity << std::endl;
        }
      }
    }

  private:
    const Arguments arguments_;
    uint64_t cycle_count_ = 0;
  };

  template <typename Controller>
  void Run(const Arguments &args, Controller *controller)
  {

    auto node = rclcpp::Node::make_shared("moteus");

    rclcpp::Subscription<moteus_msgs::msg::MoteusCommandArray>::SharedPtr submsg;
    rclcpp::Publisher<moteus_msgs::msg::MoteusStateArray>::SharedPtr pubmsg;
    submsg = node->create_subscription<moteus_msgs::msg::MoteusCommandArray>("/command", 10, MoteusCommand_callback);
    pubmsg = node->create_publisher<moteus_msgs::msg::MoteusStateArray>("/state", 10);

    auto moteus_state_array = moteus_msgs::msg::MoteusStateArray();
    auto moteus_command_array = moteus_msgs::msg::MoteusCommandArray();

    moteus::ConfigureRealtime(args.main_cpu);
    MoteusInterface::Options moteus_options;
    moteus_options.cpu = args.can_cpu;
    MoteusInterface moteus_interface{moteus_options};

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
        std::chrono::microseconds(static_cast<int64_t>(args.period_s * 1e6));
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

      rclcpp::spin_some(node);
      
      controller->Run(saved_replies, &commands, moteus_command_array, moteus_state_array);

      pubmsg->publish(moteus_state_array);
      // 보내고 받는 부분 테스트 필요
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
  rclcpp::init(argc, argv);
  Arguments args({argv + 1, argv + argc});

  // Lock memory for the whole process.
  LockMemory();

  SampleController sample_controller{args};
  Run(args, &sample_controller);
  rclcpp::shutdown();
  return 0;
}
