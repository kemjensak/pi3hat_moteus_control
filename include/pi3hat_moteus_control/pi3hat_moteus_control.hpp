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

using std::placeholders::_1;

using MoteusInterface = moteus::Pi3HatMoteusInterface;
namespace
{
    /// This holds the user-defined control logic.
    class SampleController : public rclcpp::Node
    {
    public:
        SampleController();
        ~SampleController() override = default;

        std::vector<std::pair<int, int>> servo_bus_map() const;
        void Initialize(std::vector<MoteusInterface::ServoCommand> *commands);
        moteus::QueryResult Get(const std::vector<MoteusInterface::ServoReply> &replies, int id, int bus);
        void Run(const std::vector<MoteusInterface::ServoReply> &replies,
                 std::vector<MoteusInterface::ServoCommand> *output);

        rclcpp::Subscription<moteus_msgs::msg::MoteusCommandArray>::SharedPtr submsg_;
        rclcpp::Publisher<moteus_msgs::msg::MoteusStateArray>::SharedPtr pubmsg_;

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr_;

    private:
        uint64_t cycle_count_ = 0;
        moteus_msgs::msg::MoteusStateArray msg_output_;
        moteus_msgs::msg::MoteusCommandArray msg_input_;
    };

    SampleController::SampleController() : Node("moteus")
    {
        submsg_ = this->create_subscription<moteus_msgs::msg::MoteusCommandArray>("command", rclcpp::SensorDataQoS(),
                                                                                  [this](const moteus_msgs::msg::MoteusCommandArray::SharedPtr msg)
                                                                                  { msg_input_ = *msg; });

        pubmsg_ = this->create_publisher<moteus_msgs::msg::MoteusStateArray>("state", 1);
    }

    void SampleController::Initialize(std::vector<MoteusInterface::ServoCommand> *commands)
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

    void SampleController::Run(const std::vector<MoteusInterface::ServoReply> &replies,
                               std::vector<MoteusInterface::ServoCommand> *output)
    {

        cycle_count_++;

        if (cycle_count_ < 5)
        {
            for (auto &cmd : *output)
            {
                cmd.mode = moteus::Mode::kStopped;
            }
        }
        else
        {
            rclcpp::spin_some(this->get_node_base_interface());
            for (auto &m_cmd : msg_input_.moteus_commands)
            {
                auto &out = output->at(m_cmd.id - 1);
                out.mode = moteus::Mode::kPosition;
                out.position.position = std::numeric_limits<double>::quiet_NaN();
                out.position.velocity = m_cmd.velocity;
                out.position.maximum_torque = m_cmd.maximum_torque;
                std::cout << "id: " << m_cmd.id << " velocity: " << m_cmd.velocity << " torque: " << m_cmd.maximum_torque << std::endl;
            }

            // auto &out = output->at(0);
            // out.mode = moteus::Mode::kPosition;
            // out.position.position = std::numeric_limits<double>::quiet_NaN();
            // out.position.velocity = 0.1;
            // out.position.maximum_torque = 3.0;

            // const auto primary = Get(status, arguments_.primary_id, arguments_.primary_bus);
            msg_output_.moteus_states.clear();
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

                msg_output_.moteus_states.push_back(moteus_state);
                // std::cout << "id: " << reply.id << std::endl;
                // std::cout << "position: " << reply.result.position << std::endl;
                // std::cout << "velocity: " << reply.result.velocity << std::endl;
            }
            pubmsg_->publish(msg_output_);
        }
    }

    moteus::QueryResult SampleController::Get(const std::vector<MoteusInterface::ServoReply> &replies, int id, int bus)
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

    std::vector<std::pair<int, int>> SampleController::servo_bus_map() const
    {
        return {
            {1, 1},
            {2, 1},
            {3, 1},
        };
    }

}