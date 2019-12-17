#include <omorobot_r1_robot_hw/omorobot_r1_hardware_interface.h>
#include <boost/algorithm/string.hpp>
#include <cmath>

OmoRobotR1HardwareInterface::OmoRobotR1HardwareInterface()
{

}

OmoRobotR1HardwareInterface::~OmoRobotR1HardwareInterface()
{
    if(serial_port_->IsOpen())
    {
        serial_port_->Close();
    }

}

bool OmoRobotR1HardwareInterface::init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    //get portname
    std::string portName;
    if(!pnh.getParam("port_name", portName))
    {
        ROS_ERROR("[%s] Failed to get port name. Please set the parameter ~port_name", ros::this_node::getName().c_str());
        return false;
    }

    // get baudrate
    int baudrate;
    if(!pnh.getParam("baudrate", baudrate)) {
        ROS_ERROR("Failed to get baudrate. Please set the parameter ~baudrate");
        return false;
    }

    // get gear_ratio
    if(!pnh.getParam("gear_ratio", wheel_gear_ratio_)) {
        ROS_ERROR("Failed to get gear_ratio. Please set the parameter ~gear_ratio");
        return false;
    }

    SerialPort::BaudRate spBaudRate;
    switch(baudrate)
    {
        case 115200:
            spBaudRate = SerialPort::BAUD_115200;
            break;
        default:
            return false;
    }

    // libserial
    serial_port_ = boost::make_shared<SerialPort>(portName);
    serial_port_->Open();
    serial_port_->SetBaudRate(SerialPort::BAUD_115200);

    ros::Duration(0.5).sleep();
    serial_port_->Write("$CINIT\r\n");
    ros::Duration(2.0).sleep();
    std::string buf = serial_port_->ReadLine(100, '\n');

    // Disabled the feedback for command
    serial_port_->Write("$SCBEN,0\r\n");
    ros::Duration(0.5).sleep();

    // Get initial encoder value;
    serial_port_->Write("$QENCOD\r\n");
    buf = serial_port_->ReadLine(100, '\n');
    std::vector<std::string> results;
    boost::algorithm::split(results, buf, boost::is_any_of(","));

    last_encoder_vel[0] = std::stod(results[1]);
    last_encoder_vel[1] = std::stod(results[2]);

    ROS_INFO("[%s] initital ecoder value %.1f %.1f", ros::this_node::getName().c_str(), last_encoder_vel[0], last_encoder_vel[1]);


    // hardware_interface
    joint_cmd_.resize(2);
    for(int i = 0; i < joint_cmd_.size(); i++) {
        joint_cmd_[i] = 0.0;
    }
    joint_pos_.resize(2);
    joint_vel_.resize(2);
    joint_eff_.resize(2);

    hardware_interface::JointStateHandle state_handle_l_wheel("l_wheel", &joint_pos_[0], &joint_vel_[0], &joint_eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_l_wheel);
    hardware_interface::JointStateHandle state_handle_r_wheel("r_wheel", &joint_pos_[1], &joint_vel_[1], &joint_eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_r_wheel);

    registerInterface(&jnt_state_interface_);

    // joint_velocity_interface
    hardware_interface::JointHandle vel_handle_l_wheel(jnt_state_interface_.getHandle("l_wheel"), &joint_cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_l_wheel);
    hardware_interface::JointHandle vel_handle_r_wheel(jnt_state_interface_.getHandle("r_wheel"), &joint_cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_r_wheel);

    registerInterface(&jnt_vel_interface_);

    ROS_INFO("[%s] hardware initialized...", ros::this_node::getName().c_str());
    return true;
}

void OmoRobotR1HardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
    std::string buf;
    std::vector<std::string> results;

    serial_port_->Write("$QENCOD\r\n");

    try
    {
        buf = serial_port_->ReadLine(1000, '\n');
        if(buf.substr(0, 7) == "#QENCOD")
        {
            boost::algorithm::split(results, buf, boost::is_any_of(","));
            joint_vel_[0] = (std::stod(results[1]) - last_encoder_vel[0]) / 60000.0 * period.toSec() * M_PI;
            joint_vel_[1] = (std::stod(results[2]) - last_encoder_vel[1]) / 60000.0 * period.toSec() * M_PI;

            joint_pos_[0] += joint_vel_[0] / period.toSec();
            joint_pos_[1] += joint_vel_[1] / period.toSec();

            ROS_INFO("%f %f, %f, %f", joint_vel_[0], joint_pos_[0], joint_vel_[1], joint_pos_[1]);
        }

        last_encoder_vel[0] = std::stod(results[1]);
        last_encoder_vel[1] = std::stod(results[2]);
    }
    catch(...)
    {
        return;
    }
}

void OmoRobotR1HardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
    serial_port_->Write("$CRPM,60,60\r\n");
}

bool OmoRobotR1HardwareInterface::prepareSwitch(
    const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    for (std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it)
    {
        if (it->claimed_resources.empty())
        {
            continue;
        }
        for (std::vector<hardware_interface::InterfaceResources>::const_iterator res_it = it->claimed_resources.begin(); res_it != it->claimed_resources.end(); ++res_it)
        {
            std::vector<std::string> r_hw_ifaces = this->getNames();

            std::vector<std::string>::iterator if_name = std::find(r_hw_ifaces.begin(), r_hw_ifaces.end(), res_it->hardware_interface);
            if (if_name == r_hw_ifaces.end()) // this hardware_interface is not registered on this RobotHW
            {
                ROS_ERROR_STREAM("Bad interface: " << res_it->hardware_interface);
                std::cout << res_it->hardware_interface;
                return false;
            }

            std::vector<std::string> r_hw_iface_resources = this->getInterfaceResources(res_it->hardware_interface);
            for (std::set<std::string>::const_iterator ctrl_res = res_it->resources.begin(); ctrl_res != res_it->resources.end(); ++ctrl_res)
            {
                std::vector<std::string>::iterator res_name = std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), *ctrl_res);
                if (res_name == r_hw_iface_resources.end()) // this resource is not registered on this RobotHW
                {
                    ROS_ERROR_STREAM("Bad resource: " << (*ctrl_res));
                    std::cout << (*ctrl_res);
                    return false;
                }
            }
        }
    }
    return true;
}

void OmoRobotR1HardwareInterface::doSwitch(
    const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    for (std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it)
    {
        if (it->claimed_resources.empty())
        {
            continue;
        }
        for (std::vector<hardware_interface::InterfaceResources>::const_iterator res_it = it->claimed_resources.begin(); res_it != it->claimed_resources.end(); ++res_it)
        {
            std::vector<std::string> r_hw_ifaces = this->getNames();

            std::vector<std::string>::iterator if_name = std::find(r_hw_ifaces.begin(), r_hw_ifaces.end(), res_it->hardware_interface);
            if (if_name == r_hw_ifaces.end()) // this hardware_interface is not registered on this RobotHW
            {
                throw hardware_interface::HardwareInterfaceException("Hardware_interface " + res_it->hardware_interface + " is not registered");
            }

            std::vector<std::string> r_hw_iface_resources = this->getInterfaceResources(res_it->hardware_interface);
            for (std::set<std::string>::const_iterator ctrl_res = res_it->resources.begin(); ctrl_res != res_it->resources.end(); ++ctrl_res)
            {
                std::vector<std::string>::iterator res_name = std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), *ctrl_res);
                if (res_name == r_hw_iface_resources.end()) // this resource is not registered on this RobotHW
                {
                    throw hardware_interface::HardwareInterfaceException("Resource " + *ctrl_res + " is not registered");
                }
            }
        }
    }
}