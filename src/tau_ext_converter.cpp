#include "ros/ros.h"
#include "iiwa_driver/AdditionalOutputs.h"
#include "std_msgs/Float64MultiArray.h"

class MessageConverter{
    private:
    ros::Subscriber sub_;
    ros::Publisher pub_;

    public:
    MessageConverter(ros::NodeHandle *nh_){
        sub_ = nh_->subscribe("/additional_outputs", 10, &MessageConverter::iiwa_output_callback, this);

        pub_ = nh_->advertise<std_msgs::Float64MultiArray>("tau_ext", 10);
    }

    void iiwa_output_callback(const iiwa_driver::AdditionalOutputs::ConstPtr& incoming_msg){
        std_msgs::Float64MultiArray tau_ext;
        tau_ext.data = incoming_msg->external_torques.data;
        // for (size_t i = 0; i < tau_ext.data.size(); ++i)
        // {
        //     ROS_INFO("Data[%zu]: %f", i, tau_ext.data[i]);
        // }
        pub_.publish(tau_ext);
}


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tau_ext_converter");
    ros::NodeHandle nh_;

    MessageConverter conv = MessageConverter(&nh_);
    ros::spin();
}