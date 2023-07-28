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
        double cmd_offset_m2_ = std::numeric_limits<double>::quiet_NaN();
        double cmd_offset_m3_ = std::numeric_limits<double>::quiet_NaN();

        };

    SampleController::SampleController() : Node("moteus")
    {
        submsg_ = this->create_subscription<moteus_msgs::msg::MoteusCommandArray>("command", rclcpp::SensorDataQoS(),
                                                                                [this](const moteus_msgs::msg::MoteusCommandArray::SharedPtr msg)
                                                                                { msg_input_ = *msg; });

        pubmsg_ = this->create_publisher<moteus_msgs::msg::MoteusStateArray>("state", rclcpp::QoS(1));
    }

    void SampleController::Initialize(std::vector<MoteusInterface::ServoCommand> *commands)
    {
        moteus::PositionResolution res;
        res.position = moteus::Resolution::kInt16;
        res.velocity = moteus::Resolution::kIgnore;
        res.feedforward_torque = moteus::Resolution::kIgnore;
        res.kp_scale = moteus::Resolution::kIgnore;
        res.kd_scale = moteus::Resolution::kIgnore;
        res.maximum_torque = moteus::Resolution::kInt16;
        res.stop_position = moteus::Resolution::kIgnore;
        res.watchdog_timeout = moteus::Resolution::kIgnore;
        res.velocity_limit = moteus::Resolution::kInt16;
        res.acceleration_limit = moteus::Resolution::kInt16;
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
            if ((std::isnan(cmd_offset_m2_) || std::isnan(cmd_offset_m3_)) &&
                (!std::isnan(replies[1].result.position) && !std::isnan(replies[2].result.position)) &&
                replies.size() >= 3)
            {
                cmd_offset_m2_ = -replies[2].result.i2c_position + replies[1].result.i2c_position + replies[1].result.position;
                if (replies[2].result.i2c_position > 0.8) cmd_offset_m2_ += 1.0;
                if (replies[1].result.i2c_position > 0.5) cmd_offset_m2_ -= 1.0;
                
                cmd_offset_m3_ = -replies[2].result.i2c_position - replies[1].result.i2c_position + replies[2].result.position;
                if (replies[2].result.i2c_position > 0.8) cmd_offset_m3_ += 1.0;
                if (replies[1].result.i2c_position > 0.5) cmd_offset_m3_ -= 1.0;

                // cmd_offset_m3_ = -replies[2].result.i2c_position + replies[2].result.position;
                // if (replies[2].result.i2c_position > 0.8) cmd_offset_m3_ += 1;

                
            }
            // current_theta_4_ = replies[1].result.position - replies[2].result.position;
            
            rclcpp::spin_some(this->get_node_base_interface());
            for (auto &m_cmd : msg_input_.moteus_commands)
            {

                
                if (m_cmd.id > 3) continue;
                auto &out = output->at(m_cmd.id - 1);
                out.mode = moteus::Mode::kPosition;
                out.position.position = std::numeric_limits<double>::quiet_NaN();

               
                // apply offset
                if(m_cmd.id == 2){
                    out.position.position = m_cmd.position - msg_input_.moteus_commands[2].position + cmd_offset_m2_;
                    
                }
                else if(m_cmd.id == 3){
                    out.position.position = m_cmd.position + msg_input_.moteus_commands[1].position + cmd_offset_m3_;
                }

                // else if(m_cmd.id == 3){
                //     out.position.stop_position = output->at(1).position.stop_position - (latest_i2c_positions_[2] + m_cmd.position);
                //     output->at(1).position.stop_position += (latest_i2c_positions_[2] + m_cmd.position);
                // }
                // if(m_cmd.id == 4){
                //     out.position.stop_position = latest_i2c_positions_[1] + m_cmd.position + 
                //                                  latest_i2c_positions_[2] + msg_input_.moteus_commands.;
                // }
                // if(m_cmd.id == 5){
                //     out.position.stop_position = latest_i2c_positions_[1] + m_cmd.position + 
                //                                  latest_i2c_positions_[2] + msg_input_.moteus_commands.;
                // }
                else out.position.position = m_cmd.position;
                out.position.acceleration_limit = m_cmd.acceleration;
                out.position.velocity_limit = m_cmd.velocity;
                out.position.maximum_torque = m_cmd.maximum_torque;

                out.position.position /= (2.0 * M_PI);
                out.position.velocity /= (2.0 * M_PI);
                // out.position.stop_position /= (2.0 * M_PI);

            //     std::cout << "id: " << m_cmd.id
            //               << " velocity: " << m_cmd.velocity
            //               << " torque: " << m_cmd.maximum_torque
            //               << " position: " << out.position.position
            //               << std::endl;
            }

            // std::cout << "m2 offset: " << cmd_offset_m2_ << std::endl
            //             << "m3 offset: " << cmd_offset_m3_
            //             << "diff bet motors: " << replies[2].result.position - replies[1].result.position 
            //             << std::endl;
            // cmd_offset_m2_ = std::numeric_limits<double>::quiet_NaN();

            msg_output_.moteus_states.clear();
            for (auto reply : replies)
            {
                moteus_msgs::msg::MoteusState moteus_state;
                moteus_state.id = reply.id;
                moteus_state.position = reply.result.position * (2.0 * M_PI);
                moteus_state.i2c_position = reply.result.i2c_position * (2.0 * M_PI);
                moteus_state.velocity = reply.result.velocity * (2.0 * M_PI);
                moteus_state.torque = reply.result.torque;
                moteus_state.q_current = reply.result.q_current;
                moteus_state.d_current = reply.result.d_current;
                moteus_state.rezero_state = reply.result.rezero_state;
                moteus_state.voltage = reply.result.voltage;
                moteus_state.temperature = reply.result.temperature;
                moteus_state.fault = reply.result.fault;

                msg_output_.moteus_states.emplace_back(moteus_state);
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
            // {2, 1},
            // {3, 1},
            // {4, 1},
            // {5, 1},
            // {6, 1},
            // {7, 2},
            // {8, 2},
            // {9, 2},
            // {10, 2},
            // {11, 2},
            // {12, 2},
        };
    }

}
