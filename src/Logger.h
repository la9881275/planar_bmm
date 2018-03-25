/*
 * Logger.h
 *
 *  Created on: Apr 22, 2014
 *      Author: kaess
 */

#pragma once

#include "opencv2/opencv.hpp"

class Logger {
  FILE* _file;
  bool _write;
  int32_t _numFrames;
  int32_t _numFramesAvailable;
  // temporary buffers, only allocated once for efficiency
  std::vector<uint8_t> _depthCompressed;
  std::vector<uint8_t> _colorCompressed;

public:

  Logger(const std::string& fname, bool doWrite = false);

  bool getFrame(cv::Mat& depth, cv::Mat& color, uint64_t& timestamp);

  void writeFrame(const cv::Mat& depth, const cv::Mat& color,
      uint64_t timestamp);

  void close();
};
