/*
 * Camera.h
 *
 *  Created on: Mar 21, 2014
 *      Author: kaess
 */

#pragma once

class CameraParameters {
public:
  // image size in pixels
  int width;
  int height;
  // field of view in radians
  double fovHorizontal;
  double fovVertical;
  // focal length in pixels
  double fx;
  double fy;
  // principal point in pixels
  double px;
  double py;

  void print() {
    std::cout << width << "x" << height << " " << fovHorizontal << "-" << fovVertical
        << " " << fx << "-" << fy << " " << px << "," << py << std::endl;
  }
};

// hiding OpenNI2 implementation because openni is incompatible with C++11...
class OpenNI2;

class Camera {

  class OpenNI2* pni;

public:

  Camera(int width, int height, int fps);

  ~Camera();

  CameraParameters getParameters();

  bool getFrame(cv::Mat& imageD, cv::Mat& imageC, uint64_t& timestamp);

};
