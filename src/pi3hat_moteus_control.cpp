#include "pi3hat_moteus_control/pi3hat_moteus_control.hpp"

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
        mjbots::pi3hat::ConfigureRealtime(2);
        MoteusInterface::Options moteus_options;
        moteus_options.cpu = 3;
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

    MoteusController::MoteusController() : Node("moteus_control")
    {
        submsg_ = this->create_subscription<moteus_msgs::msg::MoteusCommandArray>("command", rclcpp::SensorDataQoS(),
                                                                                  [this](const moteus_msgs::msg::MoteusCommandArray::SharedPtr msg)
                                                                                  { msg_input_ = *msg; });
        pubmsg_ = this->create_publisher<moteus_msgs::msg::MoteusStateArray>("state", rclcpp::QoS(1));
        // this->declare_parameter("servomap", 1);

        // rclcpp::Parameter int_array_param = this->get_parameter("servomap");
        // int_servomap_ = int_array_param.as_integer_array();
        // std::cout << "int_servomap_: " << int_servomap_.at(0) << std::endl;
    }

    void MoteusController::Initialize(std::vector<MoteusInterface::ServoCommand> *commands)
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

    void MoteusController::ClampJointCommand(const moteus_msgs::msg::MoteusCommandArray &msg_input)
    {
        auto commands = msg_input.moteus_commands;
      
       // clamping each input commands for safety
        commands[0].position = std::clamp(double(commands[0].position), -1.5, 1.5);
        commands[1].position = std::clamp(double(commands[1].position), -3.0, 3.0);
        //----------------- LEFT shoulder(2,3) -----------------
        double m2_pos_cmd = std::clamp(double(msg_input_.moteus_commands[1].position), 0.1, 3.0);
        double m3_pos_cmd = std::clamp(double(msg_input_.moteus_commands[2].position), -3.0, 3.0);

        //----------------- LEFT elbow(4,5) -----------------
        double m4_pos_cmd = std::clamp(double(msg_input_.moteus_commands[3].position), -1.4, 2.3);
        double m5_pos_cmd = std::clamp(double(msg_input_.moteus_commands[4].position), -1.4, 3.0);

        double m6_pos_cmd = std::clamp(double(msg_input_.moteus_commands[5].position), -1.5, 1.5);

        double m7_pos_cmd = std::clamp(double(msg_input_.moteus_commands[6].position), -1.5, 1.5);

        //----------------- RIGHT shoulder(8,9) -----------------
        double m8_pos_cmd = std::clamp(double(msg_input_.moteus_commands[7].position), 0.1, 3.0);
        double m9_pos_cmd = std::clamp(double(msg_input_.moteus_commands[8].position), -3.0, 3.0);

        //----------------- RIGHT elbow(10,11) -----------------
        double m10_pos_cmd = std::clamp(double(msg_input_.moteus_commands[9].position), -1.4, 2.3);
        double m11_pos_cmd = std::clamp(double(msg_input_.moteus_commands[10].position), -1.4, 3.0);

        double m12_pos_cmd = std::clamp(double(msg_input_.moteus_commands[11].position), -1.5, 1.5);

        //----------------- HEAD neck(13,14) -----------------
        double m13_pos_cmd = std::clamp(double(msg_input_.moteus_commands[12].position), -1.4, 1.4);
    }

    bool MoteusController::CheckAllCommandReceived(const std::vector<MoteusInterface::ServoReply> &replies)
    {
        if (replies.size() != 14)
            {
                std::cout << "only received " << replies.size() << " IDs, not received: ";
                std::vector<uint> ids = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
                for (auto reply : replies)
                {
                    ids.erase(remove(ids.begin(), ids.end(), uint(reply.id)), ids.end());
                    // std::cout << reply.id << " " ;
                }
                for (uint64_t id = 0; id < ids.size(); id++)
                {
                    std::cout << ids.at(id) << " " ;
                }
                std::cout << std::endl;
                return 0;
            }
            else return 1;
    }

    void MoteusController::Run(const std::vector<MoteusInterface::ServoReply> &replies,
                               std::vector<MoteusInterface::ServoCommand> *output)
    {
        rclcpp::spin_some(this->get_node_base_interface());

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
            if (!CheckAllCommandReceived(replies))
            {
                return;
            }
          
            // check all feedback, command is arrived(if not, cannot calculate differential joint commands)
            if (msg_input_.moteus_commands.size() == 16)
            {

                // clamping each input commands for safety
                double m1_pos_cmd = std::clamp(double(msg_input_.moteus_commands[0].position), -1.5, 1.5);
                //----------------- LEFT shoulder(2,3) -----------------
                double m2_pos_cmd = std::clamp(double(msg_input_.moteus_commands[1].position), 0.1, 3.0);
                double m3_pos_cmd = std::clamp(double(msg_input_.moteus_commands[2].position), -3.0, 3.0);

                //----------------- LEFT elbow(4,5) -----------------
                double m4_pos_cmd = std::clamp(double(msg_input_.moteus_commands[3].position), -1.4, 2.3);
                double m5_pos_cmd = std::clamp(double(msg_input_.moteus_commands[4].position), -1.4, 3.0);

                double m6_pos_cmd = std::clamp(double(msg_input_.moteus_commands[5].position), -1.5, 1.5);

                double m7_pos_cmd = std::clamp(double(msg_input_.moteus_commands[6].position), -1.5, 1.5);

                //----------------- RIGHT shoulder(8,9) -----------------
                double m8_pos_cmd = std::clamp(double(msg_input_.moteus_commands[7].position), 0.1, 3.0);
                double m9_pos_cmd = std::clamp(double(msg_input_.moteus_commands[8].position), -3.0, 3.0);

                //----------------- RIGHT elbow(10,11) -----------------
                double m10_pos_cmd = std::clamp(double(msg_input_.moteus_commands[9].position), -1.4, 2.3);
                double m11_pos_cmd = std::clamp(double(msg_input_.moteus_commands[10].position), -1.4, 3.0);

                double m12_pos_cmd = std::clamp(double(msg_input_.moteus_commands[11].position), -1.5, 1.5);

                //----------------- HEAD neck(13,14) -----------------
                double m13_pos_cmd = std::clamp(double(msg_input_.moteus_commands[12].position), -1.4, 1.4);
                double m14_pos_cmd = std::clamp(double(msg_input_.moteus_commands[13].position), -1.4, 1.4);

                //----------------- waist(15,16) -----------------
                double m15_pos_cmd = std::clamp(double(msg_input_.moteus_commands[14].position), -0.3, 0.3);
                double m16_pos_cmd = std::clamp(double(msg_input_.moteus_commands[15].position), -0.9, 0.9);

                //----------------- leg(17,18) -----------------
                // double m17_pos_cmd = std::clamp(double(msg_input_.moteus_commands[17].position), 0.0, 1.4);
                // double m18_pos_cmd = std::clamp(double(msg_input_.moteus_commands[18].position), -0.785, 0.785);

                for (auto &m_cmd : msg_input_.moteus_commands)
                {
                    if (m_cmd.id>14)
                    {
                        continue;
                    }
                    auto &out = output->at(m_cmd.id - 1);
                    out.mode = moteus::Mode::kPosition;

                    // apply offset and set position
                    if (m_cmd.id == 1)
                    {
                        double offset = (GetSPIposition(replies, m_cmd.id) - GetI2Cposition(replies, m_cmd.id));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        if (std::isnan(m1_initial_offset))
                        {
                            m1_initial_offset = offset;
                        }
                        
                        out.position.position = m1_pos_cmd / (2.0 * M_PI) + m1_initial_offset;
                    }
                    //--------------------- Left shoulder (2, 3) position control start ---------------------
                    else if (m_cmd.id == 2)
                    {
                        double offset = (m2_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 2)) + (m3_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 3));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        out.position.position = GetSPIposition(replies, m_cmd.id) + offset;
                        
                        // std::cout << "m2offset: " << offset << std::endl;
                        // std::cout <<"m2: " <<out.position.position << std::endl;
                    }
                    else if (m_cmd.id == 3)
                    {
                        double offset = (m2_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 2)) - (m3_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 3));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        out.position.position = GetSPIposition(replies, m_cmd.id) + offset;
                        // std::cout << "m3offset: " << offset << std::endl;
                    }

                    //--------------------- Left elbow (4, 5) position control start ---------------------
                    else if (m_cmd.id == 4)
                    {// i2c 순서 바뀜 (4, 5)
                        // auto offset = (m4_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 5)) ;
                        auto offset = (m4_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 5)) + (m5_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 4));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        out.position.position = GetSPIposition(replies, m_cmd.id) + offset;
                        // if (GetSPIposition(replies, m_cmd.id) - offset
                        // std::cout << "m4offset: " << offset << std::endl;
                    }
                    else if (m_cmd.id == 5)
                    {// i2c 순서 바뀜 (4, 5)
                        // auto offset = (m4_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 5));
                        auto offset = (m4_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 5)) - (m5_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 4));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        out.position.position = GetSPIposition(replies, m_cmd.id) + offset;
                        
                        // std::cout << "m5offset: " << offset << std::endl;
                    }

                    else if (m_cmd.id == 6)
                    {
                        double offset = (GetSPIposition(replies, m_cmd.id) - GetI2Cposition(replies, m_cmd.id));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        if (std::isnan(m6_initial_offset))
                        {
                            m6_initial_offset = offset;
                        }
                        
                        out.position.position = m6_pos_cmd / (2.0 * M_PI) + m6_initial_offset;
                    }
                    else if (m_cmd.id == 7)
                    {
                        double offset = (GetSPIposition(replies, m_cmd.id) - GetI2Cposition(replies, m_cmd.id));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        if (std::isnan(m7_initial_offset))
                        {
                            m7_initial_offset = offset;
                        }
                        
                        out.position.position = m7_pos_cmd / (2.0 * M_PI) + m7_initial_offset;
                    }

                    //--------------------- Right shoulder (8, 9) position control start ---------------------
                    else if (m_cmd.id == 8)
                    { // i2c 순서 바뀜 (8, 9)
                        auto offset = (m8_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 9)) + (m9_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 8));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        out.position.position = GetSPIposition(replies, m_cmd.id) + offset;
                    }
                    else if (m_cmd.id == 9)
                    { // i2c 순서 바뀜 (8, 9)
                        auto offset = (m8_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 9)) - (m9_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 8));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        out.position.position = GetSPIposition(replies, m_cmd.id) + offset;
                    }

                    //--------------------- Right elbow (10, 11) position control start ---------------------
                    else if (m_cmd.id == 10)
                    { // i2c 순서 바뀜 (10, 11)
                        auto offset = (m10_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 11)) + (m11_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 10));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        out.position.position = GetSPIposition(replies, m_cmd.id) + offset;
                    }
                    else if (m_cmd.id == 11)
                    { // i2c 순서 바뀜 (10, 11)
                        auto offset = (m10_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 11)) - (m11_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 10));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        out.position.position = GetSPIposition(replies, m_cmd.id) + offset;
                    }

                    else if (m_cmd.id == 12)
                    {
                        double offset = (GetSPIposition(replies, m_cmd.id) - GetI2Cposition(replies, m_cmd.id));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        if (std::isnan(m12_initial_offset))
                        {
                            m12_initial_offset = offset;
                        }
                        
                        out.position.position = m12_pos_cmd / (2.0 * M_PI) + m12_initial_offset;
                    }

                    //--------------------- Head-neck (13, 14) position control start ---------------------
                    else if (m_cmd.id == 13)
                    {
                        auto offset = (m13_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 13)) + (m14_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 14));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        out.position.position = GetSPIposition(replies, m_cmd.id) + offset;
                    }
                    else if (m_cmd.id == 14)
                    {
                        auto offset = (m13_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 13)) - (m14_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 14));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        out.position.position = GetSPIposition(replies, m_cmd.id) + offset;
                    }

                    //--------------------- Waist (15, 16) position control start ---------------------
                    else if (m_cmd.id == 15)
                    {
                        auto offset = (m15_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 16)) + (m16_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 15));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        out.position.position = GetSPIposition(replies, m_cmd.id) + offset;
                    }
                    else if (m_cmd.id == 16)
                    {
                        auto offset = (m15_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 16)) - (m16_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 15));
                        if (offset > 1.0)
                        {
                            offset -= 1.0;
                        }
                        else if (offset < -1.0)
                        {
                            offset += 1.0;
                        }
                        out.position.position = GetSPIposition(replies, m_cmd.id) + offset;
                    }

                    //--------------------- Leg (17, 18) position control start ---------------------
                    else if (m_cmd.id == 17)
                    {
                        auto offset = (m15_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 15)) - (m16_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 16));
                        out.position.position = GetSPIposition(replies, m_cmd.id) + offset;
                    }
                    else if (m_cmd.id == 18)
                    {
                        auto offset = (m15_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 15)) + (m16_pos_cmd / (2.0 * M_PI) - GetI2Cposition(replies, 16));
                        out.position.position = GetSPIposition(replies, m_cmd.id) + offset;
                    }

                    // shulder, wrist (1, 6, 7, 12)
                    else out.position.position = m_cmd.position / (2.0 * M_PI);
                    out.position.acceleration_limit = m_cmd.acceleration / (2.0 * M_PI);
                    out.position.velocity_limit = m_cmd.velocity / (2.0 * M_PI);
                    out.position.maximum_torque = m_cmd.maximum_torque;

                    if(m_cmd.id == 15 || m_cmd.id == 16)
                    {
                        out.position.maximum_torque *= 5.0;
                    }

                    // std::cout << "id: " << m_cmd.id
                    //           << " velocity: " << m_cmd.velocity
                    //           << " torque: " << m_cmd.maximum_torque
                    //           << " position: " << out.position.position
                    //           << std::endl;
                }
            }

            msg_output_.moteus_states.clear();
            for (uint64_t i = 1; i <= replies.size(); i++)
            {
                auto reply = Get(replies, i);
                moteus_msgs::msg::MoteusState moteus_state;
                moteus_state.id = i;
                moteus_state.position = reply.position;
                moteus_state.i2c_position = reply.i2c_position;
                moteus_state.velocity = reply.velocity * (2.0 * M_PI);
                moteus_state.torque = reply.torque;
                moteus_state.q_current = reply.q_current;
                moteus_state.d_current = reply.d_current;
                moteus_state.rezero_state = reply.rezero_state;
                moteus_state.voltage = reply.voltage;
                moteus_state.temperature = reply.temperature;
                moteus_state.fault = reply.fault;

                msg_output_.moteus_states.emplace_back(moteus_state);
            }
            pubmsg_->publish(msg_output_);
        }
    }

    moteus::QueryResult MoteusController::Get(const std::vector<MoteusInterface::ServoReply> &replies, int id)
    {
        for (const auto &item : replies)
        {
            if (item.id == id)
            {
                return item.result;
            }
        }
        return {};
    }

    double MoteusController::GetSPIposition(const std::vector<MoteusInterface::ServoReply> &replies, int id)
    {
        for (const auto &item : replies)
        {
            if (item.id == id)
            {
                double spi = item.result.position;
                // if (spi < -0.5)
                //     spi += 1.0;
                // if (spi > 0.5)
                //     spi -= 1.0;
                return spi;
            }
        }
        return {};
    }


    double MoteusController::GetI2Cposition(const std::vector<MoteusInterface::ServoReply> &replies, int id)
    {
        for (const auto &item : replies)
        {
            if (item.id == id)
            {
                double i2c = item.result.i2c_position;
                if (i2c < -0.5)
                    i2c += 1.0;
                if (i2c > 0.5)
                    i2c -= 1.0;
                return i2c;
            }
        }
        return {};
    }

    std::vector<std::pair<int, int>> MoteusController::servo_bus_map() const
    {
        return {
            {1, 1},
            {2, 1},
            {3, 1},
            {4, 1},
            {5, 1},
            {6, 1},
            {7, 2},
            {8, 2},
            {9, 2},
            {10, 2},
            {11, 2},
            {12, 2},
            {13, 3},
            {14, 3},
            // {15, 4},
            // {16, 4},
            // {17, 4},
            // {18, 4},
        };
    }
}

int main(int argc, char **argv)
{
    LockMemory();
    rclcpp::init(argc, argv);
    MoteusController controller{};
    Run(&controller);
    rclcpp::shutdown();
    return 0;
}
