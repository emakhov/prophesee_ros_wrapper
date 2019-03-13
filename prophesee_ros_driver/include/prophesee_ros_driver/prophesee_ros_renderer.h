//
// Created by emakhov on 13/03/19.
//

#ifndef PROJECT_PROPHESEE_ROS_RENDERER_H
#define PROJECT_PROPHESEE_ROS_RENDERER_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <prophesee_event_msgs/PropheseeEvent.h>
#include <prophesee_event_msgs/PropheseeEventBuffer.h>

#include "cd_frame_generator.h"

/// \brief Main class ROS listener and viewer
///
/// Listens ROS topics publishing data from Prophesee cameras and visualizes them on a screen
class PropheseeWrapperRenderer {
public:
    /// \brief Constructor
    PropheseeWrapperRenderer();

    /// \brief Destructor
    ~PropheseeWrapperRenderer();

    /// \brief Publish currently available CD data frame
    void publishData();

    /// \brief Checks if the frame generator class is initialized or not
    ///
    /// @return true if initialized and false otherwise
    bool isInitialized();

private:
    /// \brief Callback triggered when data are recieved from the camera info topic
    ///
    /// It gets width and height of the sensor and calls init() function
    ///
    /// @param msg : ROS message with the camera info
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

    /// \brief Initializes the frame generators and the displayers
    ///
    /// @param sensor_width : Width of the sensor
    /// @param sensor_height : Height of the sensor
    ///
    /// It initializes CD frame generator with the sensor's width and height.
    /// It also creates the displayers.
    bool init(const unsigned int& sensor_width, const unsigned int& sensor_height);

    /// \brief Node handler - the access point to communication with ROS
    ros::NodeHandle nh_;

    /// \brief Subscriber to the camera info topic
    ros::Subscriber sub_cam_info_;

    /// \brief Subscriber for CD events topic
    ros::Subscriber sub_cd_events_;

    /// \brief Publisher for gray-level frame
    ros::Publisher pub_cd_image_;

    /// \brief Used to subscribe to the gray-level frame topic
    image_transport::ImageTransport it_;

    /// \brief Instance of CDFrameGenerator class that generates a frame from CD events
    CDFrameGenerator cd_frame_generator_;

    /// \brief Display accumulation time
    /// The time interval to display events up to the current time, in us
    int display_acc_time_;

    /// \brief If the frame generators are initialized with teh sensor width and height
    bool initialized_;

    /// \brief  If visualizing CD events
    bool publish_cd_ = true;

};


#endif //PROJECT_PROPHESEE_ROS_RENDERER_H
