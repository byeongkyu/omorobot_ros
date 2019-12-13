#ifndef OMOROBOT_R1_HW_INTERFACE_H_
#define OMOROBOT_R1_HW_INTERFACE_H_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <SerialPort.h>

class OmoRobotR1HardwareInterface: public hardware_interface::RobotHW
{
    public:
        OmoRobotR1HardwareInterface();
        ~OmoRobotR1HardwareInterface();

    public:
        virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        virtual void read(const ros::Time& time, const ros::Duration& period);
        virtual void write(const ros::Time& time, const ros::Duration& period);
        virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);
        virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);

    private:
        hardware_interface::JointStateInterface jnt_state_interface_;
        hardware_interface::PositionJointInterface jnt_pos_interface_;
        hardware_interface::VelocityJointInterface jnt_vel_interface_;
        hardware_interface::EffortJointInterface jnt_eff_interface_;

        std::vector<double> joint_cmd_;
        std::vector<double> joint_pos_;
        std::vector<double> joint_vel_;
        std::vector<double> joint_eff_;

        double wheel_gear_ratio_;

        boost::shared_ptr<SerialPort> serial_port_;
};

#endif //OMOROBOT_R1_HW_INTERFACE_H_
