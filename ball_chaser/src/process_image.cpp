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
    float offset = 0;

    bool first_x = false;
    int center_x = 0;
    int position = 0;
    bool reached = false;

    for (int i = 0; i < height ; i++) {
        for (int j = 0; j < step; j++) {
            if (img.data[i * step + j] == white_pixel and img.data[(i * step + j) + 1] == white_pixel and img.data[(i * step + j) + 2] == white_pixel) {
                if (!first_x) {
                  first_x = true;
                  center_x = j+1;
                }
                offset += j - img.step / 2.0;   // offset = 0 middle of Image
                count++;
            }
        }
    }

    if (count == 0) {
      position = 0; // searching
      x = 0;
      z = -1;
    }
    else if (50000 < count) {
      position = 4; //reached
      if(center_x > (step/2 +10)){
        x = 0;
        z = 0.2;
      }
      if(center_x < (step/2 -10)){
        x = 0;
        z = -0.2;
      }
      if((step/2 -10) <= center_x <= (step/2 +10)){
        x = 0;
        z = 0;
      }
      reached = true;
    }
    else if (!reached && (center_x < step*3/8)) {
      position = 1; //left
      x = 0.2;
      z = -0.4;
    }
    else if (!reached && (step*3/8 < center_x <= step*5/8)) {
      position = 2; //middle
      x = 0.3;
      z = 0;
    }
    else if (!reached && (step*5/8 < center_x)) {
      position = 3; //right
      x = 0.2;
      z = 0.4;
    }

    // if (count == 0) position = 0; // searching
    // if (center_x < step*3/8) position = 1; //left
    // if (step*3/8 < center_x <= step*5/8) position = 2; //middle
    // if (step*5/8 < center_x) position = 3; //right
    // if (height*step/3 < count) position = 4; //reached
    // ROS_INFO_STREAM(position);
    // ROS_INFO_STREAM(center_x);
    // ROS_INFO_STREAM(count);
    // ROS_INFO_STREAM(offset);

    // switch (position) {
    //   case 0:
    //     x = 0;
    //     z = 0.8;
    //     moving_state = true;
    //     break;
    //   case 1:
    //     x = 0.2;
    //     z = -1;
    //     moving_state = true;
    //     break;
    //   case 2:
    //     x = 0.1;
    //     z = 0;
    //     moving_state = true;
    //     break;
    //   case 3:
    //     x = 0.2;
    //     z = 1;
    //     moving_state = true;
    //     break;
    //   case 4:
    //     x = 0;
    //     z = 0;
    //     moving_state = false;
    //     break;
    //   default:
    //     x = 0;
    //     z = 0;
    // }


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
