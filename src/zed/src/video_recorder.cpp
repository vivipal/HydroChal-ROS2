////////////////////////////////////////////////////////////////////////////
////
//// Copyright (c) 2021, STEREOLABS.
////
/////////////////////////////////////////////////////////////////////////////

//// ----> Includes
#include "videocapture.hpp"
#include "ocv_display.hpp"

#include <iostream>
#include <iomanip>

#include <opencv2/opencv.hpp>
// <---- Includes

// #define TEST_FPS 1

// The main function
int main(int argc, char *argv[])
{
    // ----> Silence unused warning
    (void)argc;
    (void)argv;
    // <---- Silence unused warning

    // ----> Set recording duration and file_name
    int duration = 3;  // recording duration in seconds
    if (argc > 1)
        duration = std::stoi(argv[1]);
    
    std::string recording_name = "recording";
    if (argc > 2)
        recording_name = argv[2];
    // <---- Set recording duration and file_name


    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::VGA;
    params.fps = sl_oc::video::FPS::FPS_15;

    // ----> Create Video Capture
    sl_oc::video::VideoCapture cap(params);
    if( !cap.initializeVideo() )
    {
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "See verbosity level for more details." << std::endl;

        return EXIT_FAILURE;
    }

    std::cout << "Connected to camera sn: " << cap.getSerialNumber() << "[" << cap.getDeviceName() << "]" << std::endl;
    // <---- Create Video Capture

    // ----> Frame size
    int frame_width, frame_height;
    cap.getFrameSize(frame_width, frame_height);

    std::cout << "Capture size: " << frame_width << ' ' << frame_height << std::endl;
    // <---- Frame size

    // ----> Video Writer definition
    // Define the output video file name and codec
    std::string outputFilename = "video/" + recording_name + ".avi";
    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');  // Codec for gray avi format

    // Create a VideoWriter object to write the video to a file
    cv::VideoWriter videoWriter(outputFilename, fourcc, (double) params.fps, cv::Size(frame_width, frame_height), true);

    // Check if the VideoWriter was successfully opened
    if (!videoWriter.isOpened())
    {
        std::cerr << "Could not open the output video file for writing" << std::endl;
        return EXIT_FAILURE;
    }
    // <---- Video Writer definition


#ifdef TEST_FPS
    // Timestamp to check FPS
    double lastTime = static_cast<double>(getSteadyTimestamp())/1e9;
    // Frame timestamp to check FPS
    uint64_t lastFrameTs = 0;
#endif

    double initTime = static_cast<double>(getSteadyTimestamp())/1e9;

    // Infinite video grabbing loop
    while (1)
    {
        // Get last available frame
        const sl_oc::video::Frame frame = cap.getLastFrame();

        // ----> If the frame is valid we can display it
        if(frame.data!=nullptr)
        {
#ifdef TEST_FPS
            if(lastFrameTs!=0)
            {
                // ----> System time
                double now = static_cast<double>(getSteadyTimestamp())/1e9;
                double elapsed_sec = now - lastTime;
                lastTime = now;
                std::cout << "[System] Frame period: " << elapsed_sec << "sec - Freq: " << 1./elapsed_sec << " Hz" << std::endl;
                // <---- System time

                // ----> Frame time
                double frame_dT = static_cast<double>(frame.timestamp-lastFrameTs)/1e9;
                std::cout << "[Camera] Frame period: " << frame_dT << "sec - Freq: " << 1./frame_dT << " Hz" << std::endl;
                // <---- Frame time
            }
            lastFrameTs = frame.timestamp;
#endif

            // ----> Conversion from YUV 4:2:2 to BGR for visualization
            cv::Mat frameYUV = cv::Mat( frame.height, frame.width, CV_8UC2, frame.data );

            cv::Mat frameBGR;
            cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);

            cv::Mat frameGray;
            cv::cvtColor(frameBGR, frameGray, cv::COLOR_BGR2GRAY);
            // <---- Conversion from YUV 4:2:2 to BGR for visualization
            
            // Write the current frame
            videoWriter.write(frameBGR);
        }
        // <---- If the frame is valid we can display it

        if (static_cast<double>(getSteadyTimestamp())/1e9 - initTime > duration)
            break;
    }

    // Release VideoWriter object
    videoWriter.release();
    std::cout << "Video saved to " << outputFilename << std::endl;

    return EXIT_SUCCESS;
}
