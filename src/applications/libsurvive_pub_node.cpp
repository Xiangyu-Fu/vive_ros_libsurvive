#include <stdio.h>
#include <string.h>
#include <survive_api.h>
#include <os_generic.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

// Keep the node running
static volatile int keepRunning = 1;

#ifdef __linux__
#include <assert.h>
#include <signal.h>
#include <stdlib.h>

void intHandler(int dummy) {
    if (keepRunning == 0)
        exit(-1);
    keepRunning = 0;
}
#endif

// Log function for Survive API
static void log_fn(SurviveSimpleContext *actx, SurviveLogLevel logLevel, const char *msg) {
    fprintf(stderr, "(%7.3f) SimpleApi: %s\n", survive_simple_run_time(actx), msg);
}

// Main function for ROS node
int main(int argc, char **argv) {
#ifdef __linux__
    signal(SIGINT, intHandler);
    signal(SIGTERM, intHandler);
    signal(SIGKILL, intHandler);
#endif

    // Initialize ROS
    ros::init(argc, argv, "survive_pub_node");
    ros::NodeHandle nh;

    // Publishers for Pose and Button Events
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("survive/pose", 10);
    ros::Publisher button_pub = nh.advertise<std_msgs::String>("survive/button_event", 10);

    // Initialize libsurvive with logging
    SurviveSimpleContext *actx = survive_simple_init_with_logger(argc, argv, log_fn);
    if (actx == 0) {
        ROS_ERROR("Failed to initialize libsurvive.");
        return 1;
    }

    // Start Survive's tracking in a separate thread
    survive_simple_start_thread(actx);

    ROS_INFO("Starting libsurvive tracking...");

    struct SurviveSimpleEvent event = {};
    while (ros::ok() && keepRunning && survive_simple_wait_for_event(actx, &event) != SurviveSimpleEventType_Shutdown) {
        switch (event.event_type) {
            case SurviveSimpleEventType_PoseUpdateEvent: {
                const struct SurviveSimplePoseUpdatedEvent *pose_event = survive_simple_get_pose_updated_event(&event);
                SurvivePose pose = pose_event->pose;
                const char* object_name = survive_simple_object_name(pose_event->object);
                // to string
                std::string object_name_str = object_name;

                // ROS_INFO("(%f) %s Pos: %f %f %f Rot: %f %f %f %f", pose_event->time, object_name, pose.Pos[0], pose.Pos[1],
                //          pose.Pos[2], pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);

                // Publish Pose
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.header.stamp = ros::Time::now();
                // pose_msg.header.frame_id = object_name_str;
                pose_msg.pose.position.x = pose.Pos[0];
                pose_msg.pose.position.y = pose.Pos[1];
                pose_msg.pose.position.z = pose.Pos[2];
                pose_msg.pose.orientation.x = pose.Rot[0];
                pose_msg.pose.orientation.y = pose.Rot[1];
                pose_msg.pose.orientation.z = pose.Rot[2];
                pose_msg.pose.orientation.w = pose.Rot[3];
                pose_pub.publish(pose_msg);
                break;
            }

            

            case SurviveSimpleEventType_ButtonEvent: {
                const struct SurviveSimpleButtonEvent *button_event = survive_simple_get_button_event(&event);
                SurviveObjectSubtype subtype = survive_simple_object_get_subtype(button_event->object);

                // Publish Button Event as String
                std_msgs::String button_msg;
                button_msg.data = "Object " + std::string(survive_simple_object_name(button_event->object)) +
                                  " Button " + SurviveButtonsStr(subtype, button_event->button_id) +
                                  " Event " + SurviveInputEventStr(button_event->event_type);

                button_pub.publish(button_msg);
                break;
            }

            case SurviveSimpleEventType_ConfigEvent: {
                const struct SurviveSimpleConfigEvent *cfg_event = survive_simple_get_config_event(&event);
                ROS_INFO("(%f) %s received configuration of length %u", cfg_event->time,
                         survive_simple_object_name(cfg_event->object), (unsigned)strlen(cfg_event->cfg));
                break;
            }

            case SurviveSimpleEventType_DeviceAdded: {
                const struct SurviveSimpleObjectEvent *obj_event = survive_simple_get_object_event(&event);
                ROS_INFO("(%f) Found '%s'", obj_event->time, survive_simple_object_name(obj_event->object));
                break;
            }

            case SurviveSimpleEventType_None:
                break;
        }

        ros::spinOnce();
    }

    ROS_INFO("Shutting down libsurvive...");
    survive_simple_close(actx);
    return 0;
}
