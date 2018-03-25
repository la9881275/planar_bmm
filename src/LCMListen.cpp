/**
 * @file   LCMListen.cpp
 * @author garrett (ghemann@gmail.com)
 * @date   120215
 * @brief  Example of listening for RGBD from LCM.
 */

// STD includes
#include <stdio.h>

// LCM includes
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/xtion/rgbd_t.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>

// Zlib includes
#include <zlib.h>



// Complete lcm class (basically like ros::node).
lcm::LCM* lcm_;

// LCM type (like a ros type message).
// This example isn't thread safe, but you can figure that
// out.
xtion::rgbd_t rgbd_;


// This is a callback class, you can add multiple callbacks
// to it for different types of data.
class LCMHandler
{
 public:
  ~LCMHandler() {}
  
  void handleRGBD(const lcm::ReceiveBuffer *rbuf,
                  const std::string &chan,
                  const xtion::rgbd_t *msg)
  {
    // You should make this thread safe
    rgbd_ = xtion::rgbd_t(*msg);
  }
};



int main(int argc, char* argv[])
{
  printf("LCMListen\n");

  // Setting up LCM here.
  // ttl=0 means listen only on this machine
  // ttl=1 means listen to the entire subnetwork
  std::string lcm_url = "udpm://230.255.76.67:7667?ttl=0";
  lcm_ = new lcm::LCM(lcm_url);
  if (!lcm_->good()) {
    printf("Error setting up lcm with url \"%s\"\n", lcm_url.c_str());
    return 1;
  }

  // Add lcm subscribers here.
  LCMHandler handler;
  std::string xtion_channel = "XTION";
  lcm_->subscribe(xtion_channel, &LCMHandler::handleRGBD, &handler);
  
  // Begin the main loop.  Typically you would spin this in 
  // a separate thread.
  printf("Spinning now...\n\n");
  while (true) {

    // This blocks until we get a message.
    lcm_->handle();
    // Note that there is another version, 
    // lcm_->handleTimeout(n) which waits n milliseconds before
    // continuing.

    // Parse out the new RGBD image.
    // You can see xtion_rgbd_t.lcm for info on this struct.
    int height = rgbd_.height;
    int width  = rgbd_.width;
    
    // Uncompress color image. (using opencv)
    cv::Mat3b cv_image(height, width, CV_32FC3); // 3b is uchar, 3 channel
    cv::imdecode(rgbd_.rgb, CV_LOAD_IMAGE_COLOR, &cv_image);
    // Color rearrange.
    cvtColor(cv_image, cv_image, CV_RGB2BGR);

    // Uncompress depth image. (using zlib)
    cv::Mat1w cv_depth = cv::Mat::zeros(height, width, CV_8U); // 1w is ushort, single channel
    unsigned long depth_size = height*width*2;
    uncompress(cv_depth.data, &depth_size,
               &rgbd_.depth[0], rgbd_.depthlen);
    cv::Mat1b cv_depth_image; // 1b is uchar, single channel
    normalize(cv_depth, cv_depth_image, 0, 255, cv::NORM_MINMAX, 0);

    // Show some stuff.
    cv::imshow("COLOR_IMAGE", cv_image);
    cv::imshow("DEPTH_IMAGE", cv_depth_image);
    cvWaitKey(1);
  }

  delete lcm_;

  return 0;
}
