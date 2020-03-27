#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
	ball_chaser::DriveToTarget service;
	service.request.linear_x = lin_x;
	service.request.angular_z = ang_z;
	// Call service and check if it was successful
	if (!client.call(service)) {
		ROS_ERROR("Error: can't call service command_robot");
	}
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    bool is_ball_present = false;
    int ball_x = 0;		//x coordinate of the ball
    float lin_x = 0, ang_z = 0;

    // Start searching from the middle of the image (ball isn't in the sky).
    for (int i=0; i < img.height * img.step; i+=3) {
    	if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel) {
    		is_ball_present = true;

    		ball_x = i % img.step;
    		if (ball_x < img.step / 3) {
    			// ball is in the left part of the image
    			ROS_INFO_STREAM("ball is on the left!");
    			// Turn left
    			lin_x = 0.0;
    			ang_z = 0.1;
    		} else if (ball_x > img.step * 2/3) {
    			// ball is in the right part of the image
    			ROS_INFO_STREAM("ball is on the right!");
    			// Turn right
    			lin_x = 0.0;
    			ang_z = -0.1;
    		} else {
    			// ball is in the center
    			ROS_INFO_STREAM("ball is in the center!");
    			// Move forward
    			lin_x = 0.1;
    			ang_z = 0.0;
    		}

    		break;  // stop searching for white pixels
    	}
    }

    if (is_ball_present) {
    	drive_robot(lin_x, ang_z);
    } else {
    	ROS_INFO_STREAM("ball is not found =(");
    	drive_robot(0.0, 0.0);
    }
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
