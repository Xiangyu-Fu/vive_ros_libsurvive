#include <survive.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

// Flag to keep the loop running
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

// Button callback function
SURVIVE_EXPORT void button_process(SurviveObject *so, enum SurviveInputEvent eventType, enum SurviveButton buttonId,
                                   const enum SurviveAxis *axisIds, const SurviveAxisVal_t *axisVals) {
  ROS_INFO("Button event: Object %s, Button %d", so->serial_number, buttonId);  // Replaced survive_object_name with so->serial_number
}

int main(int argc, char **argv) {
#ifdef __linux__
  signal(SIGINT, intHandler);
  signal(SIGTERM, intHandler);
  signal(SIGKILL, intHandler);
#endif

  // Initialize ROS
  ros::init(argc, argv, "survive_pub_node");
  ros::NodeHandle nh;

  // Publishers
  ros::Publisher button_pub = nh.advertise<std_msgs::Bool>("survive/button_event", 10);

  // Initialize libsurvive
  SurviveContext *ctx = survive_init(argc, argv);
  if (ctx == 0) // implies -help or similar
    return 0;

  // Start tracking
  survive_startup(ctx);

  // Install button handler
  survive_install_button_fn(ctx, button_process);

  // Main loop: Poll libsurvive and publish data
  ros::Rate loop_rate(100); // 100 Hz
  while (ros::ok() && keepRunning && survive_poll(ctx) == 0) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Clean up and close libsurvive
  survive_close(ctx);
  return 0;
}
