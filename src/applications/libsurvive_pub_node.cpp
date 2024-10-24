#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <survive.h>

static volatile int keepRunning = 1;

void button_process(SurviveObject *so, enum SurviveInputEvent eventType, enum SurviveButton buttonId,
                    const enum SurviveAxis *axisIds, const SurviveAxisVal_t *axisVals) {
    ROS_INFO("Button event on object: %s, Button ID: %d", so->serial_number, buttonId);
}

int main(int argc, char **argv) {
    // ROS initialization
    ros::init(argc, argv, "libsurvive_pub_node");
    ROS_INFO("Starting Libsurvive publisher node...");
    ros::NodeHandle nh;
    
    // ROS publishers
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("survive/pose", 10);
    ros::Publisher button_pub = nh.advertise<std_msgs::Bool>("survive/button", 10);
    ROS_INFO("Publishing pose and button events...");

    // Libsurvive initialization
    SurviveContext *ctx = survive_init(argc, argv);
    if (!ctx) {
        ROS_ERROR("Failed to initialize Libsurvive.");
        return 1;
    }

    survive_startup(ctx);
    survive_install_button_fn(ctx, button_process);

    // Main loop: poll for tracking data
    ros::Rate loop_rate(30);  // 30 Hz loop
    while (ros::ok() && survive_poll(ctx) == 0) {
        // In a real application, you would retrieve pose data from SurviveObject and publish it here
        geometry_msgs::PoseStamped pose_msg;
        // Fill in pose_msg from SurviveObject
        pose_pub.publish(pose_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    survive_close(ctx);
    return 0;
}
