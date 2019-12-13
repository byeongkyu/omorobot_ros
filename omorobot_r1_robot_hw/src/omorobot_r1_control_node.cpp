#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <omorobot_r1_robot_hw/omorobot_r1_hardware_interface.h>

class OmoRobotR1ControlNode
{
    public:
        OmoRobotR1ControlNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
            double control_frequency = 0.0;
            pnh.param<double>("rate", control_frequency, 10.0);

            assert(omorobot_hw.init(nh, pnh));
            ROS_INFO("[%s] wait for ready omorobot motors...", ros::this_node::getName().c_str());

            ros::Duration(0.5).sleep();
            ROS_INFO("[%s] ready. start controller...", ros::this_node::getName().c_str());

            cm = boost::make_shared<controller_manager::ControllerManager>(&omorobot_hw, nh);
            period = ros::Duration(1.0/control_frequency);

            loop_timer = nh.createTimer(period, &OmoRobotR1ControlNode::callback, this);
            loop_timer.start();
        }

        ~OmoRobotR1ControlNode() {
            loop_timer.stop();
        }

    private:
        void callback(const ros::TimerEvent& event)
        {
            omorobot_hw.read(ros::Time::now(), period);
            cm->update(ros::Time::now(), period);
            omorobot_hw.write(ros::Time::now(), period);
        }

    private:
        OmoRobotR1HardwareInterface omorobot_hw;
        boost::shared_ptr<controller_manager::ControllerManager> cm;
        ros::Duration period;
        ros::Timer loop_timer;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "omorobot_r1_control_node");
    ros::AsyncSpinner spinner(3);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    OmoRobotR1ControlNode m(nh, pnh);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}