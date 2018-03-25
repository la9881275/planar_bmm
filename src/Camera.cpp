/*
 * Camera.cpp
 *
 *  Created on: Mar 21, 2014
 *      Author: kaess
 */

#include <OpenNI.h>

#include <opencv2/opencv.hpp>

#include <pcl/common/time.h>

#include "Camera.h"

using namespace std;

class OpenNI2 {
private:

  int _width, _height, _fps;

  CameraParameters _camParams;

  openni::Device _device;
  openni::VideoStream _depth, _color;

  void check(const string& context, openni::Status status) {
    if (status != openni::STATUS_OK) {
      cout << "Error in " << context << ": "
          << openni::OpenNI::getExtendedError() << endl;
      exit(1);
    }
  }

public:

  // open device
  OpenNI2(int width, int height, int fps) :
      _width(width), _height(height), _fps(fps) {
    cout << "Opening RGB-D device..." << endl;
    check("init", openni::OpenNI::initialize());
    check("device.open", _device.open(openni::ANY_DEVICE)); //BUG~~~

    check("depth.create", _depth.create(_device, openni::SENSOR_DEPTH));
    check("depth.start", _depth.start());
    check("color.create", _color.create(_device, openni::SENSOR_COLOR));
    check("color.start", _color.start());

//    color.setProperty(XN_STREAM_PROPERTY_INPUT_FORMAT,
//        XN_IO_IMAGE_FORMAT_UNCOMPRESSED_YUV422);

    openni::VideoMode depthMode;
    depthMode.setFps(fps);
    depthMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
    depthMode.setResolution(width, height);
    check("depth.setVideoMode", _depth.setVideoMode(depthMode));

    openni::VideoMode colorMode;
    colorMode.setFps(fps);
    colorMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
    colorMode.setResolution(width, height);
    check("color.setVideoMode", _color.setVideoMode(colorMode));

    check("device.setDepthColorSyncEnabled",
        _device.setDepthColorSyncEnabled(true));
    check("device.setImageRegistrationMode",
        _device.setImageRegistrationMode(
            openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR));

    check("depth.setMirroringEnabled", _depth.setMirroringEnabled(false));
    check("color.setMirroringEnabled", _color.setMirroringEnabled(false));

    _camParams.width = width;
    _camParams.height = height;
    _camParams.fovHorizontal = _color.getHorizontalFieldOfView();
    _camParams.fovVertical = _color.getVerticalFieldOfView();
    _camParams.fx = width / (2 * tan(_camParams.fovHorizontal / 2.));
    _camParams.fy = height / (2 * tan(_camParams.fovVertical / 2.));
    _camParams.px = ((double) width) / 2.;
    _camParams.py = ((double) height) / 2.;

    cout << "Opened RGB-D device." << endl;
//    printModes();
  }

  // shutdown device
  ~OpenNI2() {
    _depth.stop();
    _color.stop();
    _depth.destroy();
    _color.destroy();
    _device.close();
    openni::OpenNI::shutdown();
  }

  CameraParameters getParameters() {
    return _camParams;
  }

  void printModes() {
    std::map<int, std::string> formatMap;

    formatMap[openni::PIXEL_FORMAT_DEPTH_1_MM] = "1mm";
    formatMap[openni::PIXEL_FORMAT_DEPTH_100_UM] = "100um";
    formatMap[openni::PIXEL_FORMAT_SHIFT_9_2] = "Shift 9 2";
    formatMap[openni::PIXEL_FORMAT_SHIFT_9_3] = "Shift 9 3";

    formatMap[openni::PIXEL_FORMAT_RGB888] = "RGB888";
    formatMap[openni::PIXEL_FORMAT_YUV422] = "YUV422";
    formatMap[openni::PIXEL_FORMAT_GRAY8] = "GRAY8";
    formatMap[openni::PIXEL_FORMAT_GRAY16] = "GRAY16";
    formatMap[openni::PIXEL_FORMAT_JPEG] = "JPEG";

    const openni::Array<openni::VideoMode> & rgbModes =
        _color.getSensorInfo().getSupportedVideoModes();

    openni::VideoMode currentRGB = _color.getVideoMode();

    std::cout << "RGB Modes: (" << currentRGB.getResolutionX() << "x"
        << currentRGB.getResolutionY() << " @ " << currentRGB.getFps() << "fps "
        << formatMap[currentRGB.getPixelFormat()] << ")" << std::endl;

    for (int i = 0; i < rgbModes.getSize(); i++) {
      std::cout << rgbModes[i].getResolutionX() << "x"
          << rgbModes[i].getResolutionY() << " @ " << rgbModes[i].getFps()
          << "fps " << formatMap[rgbModes[i].getPixelFormat()] << std::endl;
    }

    const openni::Array<openni::VideoMode> & depthModes =
        _depth.getSensorInfo().getSupportedVideoModes();

    openni::VideoMode currentDepth = _depth.getVideoMode();

    cout << "Depth Modes: (" << currentDepth.getResolutionX() << "x"
        << currentDepth.getResolutionY() << " @ " << currentDepth.getFps()
        << "fps " << formatMap[currentDepth.getPixelFormat()] << ")"
        << std::endl;

    for (int i = 0; i < depthModes.getSize(); i++) {
      cout << depthModes[i].getResolutionX() << "x"
          << depthModes[i].getResolutionY() << " @ " << depthModes[i].getFps()
          << "fps " << formatMap[depthModes[i].getPixelFormat()] << std::endl;
    }

  }

  // recover depth and color image (blocks until frame available)
  bool getFrame(cv::Mat& imageD, cv::Mat& imageC, uint64_t& timestamp) {
    
    openni::VideoFrameRef depthFrame, colorFrame;
    // blocking call (no need for waitForAnyStream)

    _depth.readFrame(&depthFrame); //bug here !!
    _color.readFrame(&colorFrame); //bug maybe ??

    timestamp = pcl::getTime()*1e9;

    if (colorFrame.getWidth() != _width
        || colorFrame.getHeight() != _height) {
      cout << "skipping frame" << endl;
      return false;
    }

    memcpy(imageD.data, (unsigned char*) depthFrame.getData(),
        _width * _height * 2);
    depthFrame.release();
//    memcpy(imageC.data, (unsigned char*) colorFrame.getData(),
//        _width * _height * 3);
    // swap color channels
    const cv::Mat colorTmp(_height, _width, CV_8UC3, const_cast<void*>(colorFrame.getData()));
    cvtColor(colorTmp, imageC, CV_RGB2BGR);
    colorFrame.release();

    return true;
  }

};

Camera::Camera(int width, int height, int fps) {
  pni = new OpenNI2(width, height, fps);
}

Camera::~Camera() {
  delete pni;
}

CameraParameters Camera::getParameters() {
  return pni->getParameters();
}

bool Camera::getFrame(cv::Mat& depth, cv::Mat& color, uint64_t& timestamp) {
  return pni->getFrame(depth, color, timestamp);
}



