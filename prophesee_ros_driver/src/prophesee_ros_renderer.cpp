//
// Created by Egor Makhov on 13/03/19.
//

#include "prophesee_ros_renderer.h"

typedef const boost::function< void(const prophesee_event_msgs::PropheseeEventBuffer::ConstPtr& msgs)> callback;

PropheseeWrapperRenderer::PropheseeWrapperRenderer():
        nh_("~"),
        it_(nh_),
        display_acc_time_(5000),
        initialized_(false)
{
    std::string camera_name("");

    // Load Parameters
    nh_.getParam("camera_name", camera_name);
    nh_.getParam("publish_cd", publish_cd_);
    nh_.getParam("display_accumulation_time", display_acc_time_);

    const std::string topic_cam_info = "/prophesee/" + camera_name + "/camera_info";
    const std::string topic_cd_event_buffer = "/prophesee/" + camera_name + "/cd_events_buffer";
    const std::string topic_cd_image = "/prophesee/" + camera_name + "/cd_image";

    // Subscribe to camera info topic
    sub_cam_info_ = nh_.subscribe(topic_cam_info, 1, &PropheseeWrapperRenderer::cameraInfoCallback, this);

    // Subscribe to CD buffer topic
    if (publish_cd_) {
        callback displayerCDCallback = boost::bind(&CDFrameGenerator::add_events, &cd_frame_generator_, _1);
        sub_cd_events_ = nh_.subscribe(topic_cd_event_buffer, 500, displayerCDCallback);
        pub_cd_image_ = nh_.advertise<cv_bridge::CvImage>(topic_cd_image, 1);
    }
}

PropheseeWrapperRenderer::~PropheseeWrapperRenderer() {
    if (!initialized_)
        return;

    // Stop the CD frame generator thread
    if (publish_cd_)
        cd_frame_generator_.stop();

    nh_.shutdown();
}

bool PropheseeWrapperRenderer::isInitialized() {
    return initialized_;
}

void PropheseeWrapperRenderer::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    if (initialized_)
        return;

    if ((msg->width != 0) && (msg->height != 0))
        init(msg->width, msg->height);
}

bool PropheseeWrapperRenderer::init(const unsigned int& sensor_width, const unsigned int& sensor_height) {
    if (publish_cd_) {
        // Initialize CD frame generator
        cd_frame_generator_.init(sensor_width, sensor_height);
        cd_frame_generator_.set_display_accumulation_time_us(display_acc_time_);
        // Start CD frame generator thread
        cd_frame_generator_.start();
    }

    initialized_ = true;

    return true;
}

void PropheseeWrapperRenderer::publishData() {
    if (!publish_cd_)
        return;

    if (cd_frame_generator_.get_last_ros_timestamp() < ros::Time::now() - ros::Duration(0.5)) {
        cd_frame_generator_.reset();
        initialized_ = false;
    }

    const auto &cd_frame = cd_frame_generator_.get_current_frame();
    if (!cd_frame.empty()) {
        cv_bridge::CvImage cd_frame_msg;
        cd_frame_msg.header.stamp = ros::Time::now();
        cd_frame_msg.header.frame_id = "PropheseeCamera_cd_frame";
        cd_frame_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
        cd_frame_msg.image = cd_frame;

        // Publish the message
        pub_cd_image_.publish(cd_frame_msg);
    }
}

int process_ui_for(const int& delay_ms) {
    auto then = std::chrono::high_resolution_clock::now();
    int key   = cv::waitKey(delay_ms);
    auto now  = std::chrono::high_resolution_clock::now();
    // cv::waitKey will not wait if no window is opened, so we wait for him, if needed
    std::this_thread::sleep_for(std::chrono::milliseconds(
            delay_ms - std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count()));

    return key;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "prophesee_ros_renderer");

    PropheseeWrapperRenderer wv;

    while(ros::ok() && !wv.isInitialized()) {
        ros::spinOnce();
    }

    while (ros::ok()) {
        ros::spinOnce();

        wv.publishData();

        process_ui_for(33);
    }

    ros::shutdown();

    return 0;
}
