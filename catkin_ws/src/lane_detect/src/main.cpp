#include "main.h"
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "std_msgs/Float32.h"
#include <vector>
#include <math.h>
#include <algorithm>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>
#include<pthread.h>

using namespace cv;
using namespace std;

int skipFrame = 1;
ros::Publisher steer_publisher;
ros::Publisher speed_publisher;
volatile float steer=-2;
volatile float velocity = 10;

int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;
    
    return -1;
}

/**
 * Returns the number of axes on the controller or 0 if an error occurs.
 */
size_t get_axis_count(int fd)
{
    __u8 axes;

    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

/**
 * Returns the number of buttons on the controller or 0 if an error occurs.
 */
size_t get_button_count(int fd)
{
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}

/**
 * Current state of an axis.
 */
struct axis_state {
    short x, y;
};

/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;

    if (axis < 3)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::imshow("View", cv_ptr->image);
	    waitKey(1);

        std_msgs::Float32 angle;
        std_msgs::Float32 speed;
        
        angle.data = steer;    
        speed.data = velocity;

        steer_publisher.publish(angle);
        speed_publisher.publish(speed);    


        imwrite("result.jpg",cv_ptr->image);       
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

pthread_t tid;

void* doJoystick(void *arg)
{
    
    const char *device;
    int js;
    struct js_event event;
    struct axis_state axes[3] = {0};
    size_t axis;

    device = "/dev/input/js0";

    js = open(device, O_RDONLY);

    if (js == -1)
        perror("Could not open joystick");
    else
        perror("Joystick connected"); 

    while (read_event(js, &event) == 0)
    {
        switch (event.type)
        {
            case JS_EVENT_BUTTON:
                printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
                break;
            case JS_EVENT_AXIS:
                axis = get_axis_state(&event, axes);
                if (axis < 3)
                    printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
                
                if (axis == 0)
                {
                    steer =  (axes[axis].x/400);
                   // velocity =  -(axes[axis].y/640);
                    printf("X: %6d - Steer: %f \n", axes[axis].x, steer); 
                }
                if (axis == 2)
                {
                   // steer =  (axes[axis].x/640);
                    velocity =  -(axes[axis].x/640);
                    printf("Y: %6d - Speed: %f \n", axes[axis].x, velocity);
                }
            default:
                
                break;
        }
        
        fflush(stdout);
    }

    close(js);

    return NULL;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    cv::namedWindow("View");
    
    ros::NodeHandle node_obj1;
    ros::NodeHandle node_obj2;
    
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    steer_publisher = node_obj1.advertise<std_msgs::Float32>("team1/set_angle",10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>("team1/set_speed",10);
    image_transport::Subscriber sub = it.subscribe("team1/camera/rgb", 1, imageCallback);	

    int err = pthread_create(&tid, NULL, &doJoystick, NULL);

    ros::spin();    
    cv::destroyAllWindows();
}
