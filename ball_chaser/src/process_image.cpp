#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;
//FH
// Define global vector of last velocities value, moving state of the robot
//std::vector<double> velocities_last_value{ 0, 0 };
bool moving_state = false;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    //FH
    ROS_INFO_STREAM("Moving the robot to the ball");
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    int height = img.height;
    int step = img.step;
    float x = 0.0;
    float z = 0.0;
    int count = 0;

    bool first_x = false;
    int center_x;
    int position = 0;

    for (int i = 0; i < height ; i++) {
        for (int j = 0; j < step; j++) {
            if (img.data[i * step + j] == white_pixel) {
                if (!first_x) {
                  first_x = true;
                  center_x = j;
                }
                // vec_cout[j]++;
                //offset += j - step / 2.0;
                count++;
            }
        }
    }

    if (count == 0) position = 0; // no ball
    if (center_x < step/3) position = 1; //left
    if (step/3 <= center_x <= step*2/3) position = 2; //middle
    if (step*2/3 < center_x) position = 3; //right
    if (height*step/3 < count) position = 4; //reached

    switch (position) {
      case 0:
        x = 0;
        z = 0;
        moving_state = false;
        break;
      case 1:
        x = 0;
        z = -3;
        moving_state = true;
        break;
      case 2:
        x = 0.1;
        z = 0;
        moving_state = true;
        break;
      case 3:
        x = 0;
        z = 3;
        moving_state = true;
        break;
      case 4:
        x = 0;
        z = 0;
        moving_state = false;
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
