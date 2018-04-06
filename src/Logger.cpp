/*
 * Logger.cpp
 *
 *  Created on: Apr 22, 2014
 *      Author: kaess
 */

#include <zlib.h> // compress2

#include "opencv2/opencv.hpp"

#include "Logger.h"


using namespace std;

inline void check(int ret) {
  if (ret!=1) {
    cout << "check failed on fread" << endl;
    exit(1);
  }
}

Logger::Logger(const string& fname, bool doWrite) {
  _numFrames = 0;
  _numFramesAvailable = 0;
  _write = doWrite;
  _depthCompressed.resize(640*480*2);
  _colorCompressed.resize(640*480*3);
  if (_write) {
    _file = fopen(fname.c_str(), "wb+");
    // upon closing of log file, the number of frames will be written to this space holder
    int32_t zero = 0;
    fwrite(&zero, sizeof(int32_t), 1, _file);
  } else {
    _file = fopen(fname.c_str(), "rb");
    check(fread(&_numFramesAvailable, sizeof(int32_t), 1, _file));
    cout << "Log file contains " << _numFramesAvailable << " frames." << endl;
  }
}

bool Logger::getFrame(cv::Mat& depth, cv::Mat& color, uint64_t& timestamp) {
  _numFrames++;
  if (_numFrames>_numFramesAvailable) {

    // end of log
    return false;

  } else {

    uint32_t depthCompressedSize, colorCompressedSize;
    unsigned long depthSize = depth.rows*depth.cols*2;

    check(fread(&timestamp, sizeof(int64_t), 1, _file));
    check(fread(&depthCompressedSize, sizeof(int32_t), 1, _file));
    check(fread(&colorCompressedSize, sizeof(int32_t), 1, _file));
    check(fread(&_depthCompressed[0], depthCompressedSize, 1, _file));
    check(fread(&_colorCompressed[0], colorCompressedSize, 1, _file));

    // assumes color and depth have already been allocated
    cv::imdecode(_colorCompressed, CV_LOAD_IMAGE_COLOR, &color);
    // cout << color << std::endl;
    // cv::imshow("bla",  color);
    // cv::waitKey(0);
    uncompress(depth.data, &depthSize, &_depthCompressed[0], depthCompressedSize);

    return true;
  }
}

void Logger::writeFrame(const cv::Mat& depth, const cv::Mat& color, uint64_t timestamp) {
  unsigned long compressedSize = _depthCompressed.size();
  compress2(&_depthCompressed[0], &compressedSize, depth.data,
      depth.rows * depth.cols * sizeof(short), Z_BEST_SPEED);
  int32_t depthSize = compressedSize;

  vector<int> jpegParams = { CV_IMWRITE_JPEG_QUALITY, 90, 0 };
  // note: _colorCompress is resized here
  cv::imencode(".jpg", color, _colorCompressed, jpegParams);
  int32_t colorSize = _colorCompressed.size();

  fwrite(&timestamp, sizeof(int64_t), 1, _file);
  fwrite(&depthSize, sizeof(int32_t), 1, _file);
  fwrite(&colorSize, sizeof(int32_t), 1, _file);
  fwrite(&_depthCompressed[0], depthSize, 1, _file);
  fwrite(&_colorCompressed[0], colorSize, 1, _file);
  fflush(_file);
  _numFrames++;
}

void Logger::close() {
  if (_write) {
    fseek(_file, 0, SEEK_SET);
    fwrite(&_numFrames, sizeof(int32_t), 1, _file);
    fflush(_file);
  }
  fclose(_file);
  _file = NULL;
}
