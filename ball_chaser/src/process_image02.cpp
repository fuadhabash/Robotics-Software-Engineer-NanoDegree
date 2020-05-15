#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
using namespace std;
// Define a global client that can request services
ros::ServiceClient client;
// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Driving the robot to the target.");
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
     if (!client.call(srv)) {
	    ROS_ERROR("Failed to call service DriveToTarget.");
	}
}
// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int height = img.height;
    int step = img.step;
    float x = 0.0;
    float z = 0.0;
    float offset = 0;
    int count = 0;
    for (int i = 0; i < height ; i++) {
        for (int j = 0; j < step; j++) {
            if (img.data[i * step + j] == white_pixel) {
                // vec_cout[j]++;
                offset += j - step / 2.0;
                count++;
            }
        }
    }
    if (count == 0) {
        x = 0.0;
        z = 0.0;
    }
    else {
        x = 0.1;
        // z = 0.5 * (step / 2.0 - idx_center_ball);
        z = -4.0 * offset / count / (step /2.0);
    }

    // Send request to service
    drive_robot(x, z);
}
int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;
    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    // Handle ROS communication events
    ros::spin();

  return 0;
}
