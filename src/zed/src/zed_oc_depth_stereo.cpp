///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2021, STEREOLABS.
//
///////////////////////////////////////////////////////////////////////////

// ----> Includes
#include <iostream>
#include <sstream>
#include <string>
#include <csignal>

#include "videocapture.hpp"

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// Sample includes
#include "calibration.hpp"
#include "stopwatch.hpp"
#include "stereo.hpp"
#include "ocv_display.hpp"

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
// <---- Includes

#define USE_OCV_TAPI // Comment to use "normal" cv::Mat instead of CV::UMat
// #define USE_HALF_SIZE_DISP // Comment to compute depth matching on full image frames


void signalHandler(int signum)
{
    if (signum == SIGINT) 
        rclcpp::shutdown();
}

class VideoPublisher : public rclcpp::Node
{
public:
    VideoPublisher() : Node("zed_video_publisher")
    {
        stereo_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("stereo_stream", 10);
        disparity_publisher_ = this->create_publisher<stereo_msgs::msg::DisparityImage>("disparity_stream", 10);
    }

    void publish_stereo(const cv::UMat& umat_left, const cv::UMat& umat_right)
    {
        // Stack frames horizontally
        cv::UMat umat_frame;
        cv::hconcat(umat_left, umat_right, umat_frame);

        // Convert UMat to Mat
        cv::Mat frame = umat_frame.getMat(cv::ACCESS_READ);

        // Convert OpenCV image to ROS message
        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        // Publish the image message
        stereo_publisher_->publish(*image_msg);
    }

    void publish_disparity(const cv::UMat& umat_frame, double f, double t, double min_disparity, double max_disparity)
    {
        // Convert UMat to Mat
        cv::Mat frame = umat_frame.getMat(cv::ACCESS_READ);

        // Convert OpenCV image to ROS message
        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", frame).toImageMsg();

        // Create and fill a Disparity Image object
        stereo_msgs::msg::DisparityImage msg;
        msg.image = *image_msg;
        msg.f = f;
        msg.t = t;
        msg.min_disparity = min_disparity;
        msg.max_disparity = max_disparity;

        // Publish the disparity image message
        disparity_publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_publisher_;
    rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr disparity_publisher_;
};


int main(int argc, char *argv[])
{
    signal(SIGINT, signalHandler);
    
    rclcpp::init(argc, argv);
    auto video_publisher = std::make_shared<VideoPublisher>();

    sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::INFO;

    // ----> Set Video parameters
    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::VGA;
    params.fps = sl_oc::video::FPS::FPS_15;
    params.verbose = verbose;
    // <---- Set Video parameters

    // ----> Create Video Capture
    sl_oc::video::VideoCapture cap(params);
    if( !cap.initializeVideo(-1) )
    {
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "See verbosity level for more details." << std::endl;

        return EXIT_FAILURE;
    }
    int sn = cap.getSerialNumber();
    std::cout << "Connected to camera sn: " << sn << std::endl;
    // <---- Create Video Capture

    // ----> Retrieve calibration file from Stereolabs server
    std::string calibration_file;
    // ZED Calibration
    unsigned int serial_number = sn;
    // Download camera calibration file
    if( !sl_oc::tools::downloadCalibrationFile(serial_number, calibration_file) )
    {
        std::cerr << "Could not load calibration file from Stereolabs servers" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Calibration file found. Loading..." << std::endl;

    // ----> Frame size
    int w, h;
    cap.getFrameSize(w,h);
    // <---- Frame size

    // ----> Initialize calibration
    cv::Mat map_left_x, map_left_y;
    cv::Mat map_right_x, map_right_y;
    cv::Mat cameraMatrix_left, cameraMatrix_right;
    double baseline=0;
    sl_oc::tools::initCalibration(calibration_file, cv::Size(w/2,h), map_left_x, map_left_y, map_right_x, map_right_y,
                                  cameraMatrix_left, cameraMatrix_right, &baseline);

    double fx = cameraMatrix_left.at<double>(0,0);
    double fy = cameraMatrix_left.at<double>(1,1);
    double cx = cameraMatrix_left.at<double>(0,2);
    double cy = cameraMatrix_left.at<double>(1,2);

    std::cout << " Camera Matrix L: \n" << cameraMatrix_left << std::endl << std::endl;
    std::cout << " Camera Matrix R: \n" << cameraMatrix_right << std::endl << std::endl;

#ifdef USE_OCV_TAPI
    cv::UMat map_left_x_gpu = map_left_x.getUMat(cv::ACCESS_READ,cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    cv::UMat map_left_y_gpu = map_left_y.getUMat(cv::ACCESS_READ,cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    cv::UMat map_right_x_gpu = map_right_x.getUMat(cv::ACCESS_READ,cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    cv::UMat map_right_y_gpu = map_right_y.getUMat(cv::ACCESS_READ,cv::USAGE_ALLOCATE_DEVICE_MEMORY);
#endif
    // ----> Initialize calibration

    // ----> Declare OpenCV images
#ifdef USE_OCV_TAPI
    cv::UMat frameYUV;  // Full frame side-by-side in YUV 4:2:2 format
    cv::UMat frameBGR(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Full frame side-by-side in BGR format
    cv::UMat left_raw(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Left unrectified image
    cv::UMat right_raw(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Right unrectified image
    cv::UMat left_rect(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Left rectified image
    cv::UMat right_rect(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Right rectified image
    cv::UMat left_for_matcher(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Left image for the stereo matcher
    cv::UMat right_for_matcher(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Right image for the stereo matcher
    cv::UMat left_disp_half(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Half sized disparity map
    cv::UMat left_disp(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Full output disparity
    cv::UMat left_disp_float(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Final disparity map in float32
    cv::UMat left_disp_image(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Normalized and color remapped disparity map to be displayed
    cv::UMat left_depth_map(cv::USAGE_ALLOCATE_DEVICE_MEMORY); // Depth map in float32
#else
    cv::Mat frameBGR, left_raw, left_rect, right_raw, right_rect, frameYUV, left_for_matcher, right_for_matcher, left_disp_half,left_disp,left_disp_float, left_disp_vis;
#endif
    // <---- Declare OpenCV images

    // ----> Stereo matcher initialization
    sl_oc::tools::StereoSgbmPar stereoPar;

    //Note: you can use the tool 'zed_open_capture_depth_tune_stereo' to tune the parameters and save them to YAML
    if(!stereoPar.load())
    {
        stereoPar.save(); // Save default parameters.
    }

    cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(stereoPar.minDisparity,stereoPar.numDisparities,stereoPar.blockSize);
    left_matcher->setMinDisparity(stereoPar.minDisparity);
    left_matcher->setNumDisparities(stereoPar.numDisparities);
    left_matcher->setBlockSize(stereoPar.blockSize);
    left_matcher->setP1(stereoPar.P1);
    left_matcher->setP2(stereoPar.P2);
    left_matcher->setDisp12MaxDiff(stereoPar.disp12MaxDiff);
    left_matcher->setMode(stereoPar.mode);
    left_matcher->setPreFilterCap(stereoPar.preFilterCap);
    left_matcher->setUniquenessRatio(stereoPar.uniquenessRatio);
    left_matcher->setSpeckleWindowSize(stereoPar.speckleWindowSize);
    left_matcher->setSpeckleRange(stereoPar.speckleRange);

    stereoPar.print();
    // <---- Stereo matcher initialization

    uint64_t last_ts=0; // Used to check new frame arrival

    // Infinite video grabbing loop
    while (rclcpp::ok())
    {
        // Get a new frame from camera
        const sl_oc::video::Frame frame = cap.getLastFrame();

        // ----> If the frame is valid we can convert, rectify and display it
        if(frame.data!=nullptr && frame.timestamp!=last_ts)
        {
            last_ts = frame.timestamp;

            // ----> Conversion from YUV 4:2:2 to BGR for visualization
#ifdef USE_OCV_TAPI
            cv::Mat frameYUV_cpu = cv::Mat( frame.height, frame.width, CV_8UC2, frame.data );
            frameYUV = frameYUV_cpu.getUMat(cv::ACCESS_READ,cv::USAGE_ALLOCATE_HOST_MEMORY);
#else
            frameYUV = cv::Mat( frame.height, frame.width, CV_8UC2, frame.data );
#endif
            cv::cvtColor(frameYUV,frameBGR,cv::COLOR_YUV2BGR_YUYV);
            // <---- Conversion from YUV 4:2:2 to BGR for visualization

            // ----> Extract left and right images from side-by-side
            left_raw = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
            right_raw = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));
            // <---- Extract left and right images from side-by-side

            // ----> Apply rectification
#ifdef USE_OCV_TAPI
            cv::remap(left_raw, left_rect, map_left_x_gpu, map_left_y_gpu, cv::INTER_AREA );
            cv::remap(right_raw, right_rect, map_right_x_gpu, map_right_y_gpu, cv::INTER_AREA );
#else
            cv::remap(left_raw, left_rect, map_left_x, map_left_y, cv::INTER_AREA );
            cv::remap(right_raw, right_rect, map_right_x, map_right_y, cv::INTER_AREA );
#endif
            // <---- Apply rectification

            // ----> Stereo matching
            double resize_fact = 1.0;
#ifdef USE_HALF_SIZE_DISP
            resize_fact = 0.5;
            // Resize the original images to improve performances
            cv::resize(left_rect,  left_for_matcher,  cv::Size(), resize_fact, resize_fact, cv::INTER_AREA);
            cv::resize(right_rect, right_for_matcher, cv::Size(), resize_fact, resize_fact, cv::INTER_AREA);
#else
            left_for_matcher = left_rect; // No data copy
            right_for_matcher = right_rect; // No data copy
#endif
            // Apply stereo matching
            left_matcher->compute(left_for_matcher, right_for_matcher, left_disp_half);

            left_disp_half.convertTo(left_disp_float,CV_32FC1);
            cv::multiply(left_disp_float,1./16.,left_disp_float); // Last 4 bits of SGBM disparity are decimal

#ifdef USE_HALF_SIZE_DISP
            cv::multiply(left_disp_float,2.,left_disp_float); // Last 4 bits of SGBM disparity are decimal
            cv::UMat tmp = left_disp_float; // Required for OpenCV 3.2
            cv::resize(tmp, left_disp_float, cv::Size(), 1./resize_fact, 1./resize_fact, cv::INTER_AREA);
#else
            left_disp = left_disp_float;
#endif
            // <---- Stereo matching

            // ----> Publish stereo frames
            video_publisher->publish_stereo(left_rect, right_rect);
            // <---- Publish stereo frames

            // ----> Publish disparity image
            video_publisher->publish_disparity(left_disp_float, fx, baseline, stereoPar.minDisparity, stereoPar.numDisparities);
            // <---- Publish disparity image
        }
    }

    return EXIT_SUCCESS;
}
