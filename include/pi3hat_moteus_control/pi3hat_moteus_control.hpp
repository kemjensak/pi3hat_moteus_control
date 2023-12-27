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
#include <algorithm>
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
    class MoteusController : public rclcpp::Node
    {
    public:
        MoteusController();
        ~MoteusController() override = default;

        std::vector<std::pair<int, int>> servo_bus_map() const;
        void Initialize(std::vector<MoteusInterface::ServoCommand> *commands);
        moteus::QueryResult Get(const std::vector<MoteusInterface::ServoReply> &replies, int id);
        double GetSPIposition(const std::vector<MoteusInterface::ServoReply> &replies, int id);
        double GetI2Cposition(const std::vector<MoteusInterface::ServoReply> &replies, int id);
        void ClampJointCommand(const moteus_msgs::msg::MoteusCommandArray &msg_input);
        bool CheckAllCommandReceived(const std::vector<MoteusInterface::ServoReply> &replies);
        void Run(const std::vector<MoteusInterface::ServoReply> &replies,
                 std::vector<MoteusInterface::ServoCommand> *output);
        

        rclcpp::Subscription<moteus_msgs::msg::MoteusCommandArray>::SharedPtr submsg_;
        rclcpp::Publisher<moteus_msgs::msg::MoteusStateArray>::SharedPtr pubmsg_;

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr_;

    private:
        uint64_t cycle_count_ = 0;
        moteus_msgs::msg::MoteusStateArray msg_output_;
        moteus_msgs::msg::MoteusCommandArray msg_input_;

        std::vector<int64_t> int_servomap_;
        std::vector<std::pair<int, int>> servomap_;

        std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply> full_replies_;

        double m1_initial_offset = std::numeric_limits<double>::quiet_NaN();
        double m6_initial_offset = std::numeric_limits<double>::quiet_NaN();
        double m7_initial_offset = std::numeric_limits<double>::quiet_NaN();
        double m12_initial_offset = std::numeric_limits<double>::quiet_NaN();
    };
}