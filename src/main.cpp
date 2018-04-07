/*
 * main.cpp
 *
 *  Created on: Feb 12, 2014
 *      Author: kaess
 */

#include <iostream>

const std::string usage = "\n"
    "Planar 3D Mapping\n"
    "Michael Kaess, 2014\n"
    "\n"
    "Usage:\n"
    "  planar [OPTION...] LOGNAME\n"
    "\n"
    "Options:\n"
    "  -h  -?       show help options\n"
    "  -c           color planes in 3D view\n"
    "  -f           flip RGB\n"
    "  -i           show images\n"
    "  -n           no processing (e.g. just log)\n"
    "  -w basename  save RGB-D pair\n"
    "  -r basename  read RGB-D pair\n"
    "  -L logfile   write to logfile\n"
    "\n";

#include <signal.h>
#include <thread>

#include <opencv2/opencv.hpp>

#include <pcl/common/time.h>

#include "Camera.h"
#include "planes.h"
#include "Logger.h"
#include "Mapper.h"
#include "bingham.h"

using namespace std;

const int fps = 30;
const int width = 640;
const int height = 480;

const int processEveryNth = 2;

bool stop = false;

// catch signals to allow clean shutdown of Camera (otherwise old model Xtion
// will not start up correctly on the next trial)
void terminationHandler(int signum) {
  cout << "Shutting down, please wait..." << endl;
  stop = true;
  // allow time for logger to shut down
  this_thread::sleep_for(chrono::milliseconds(1000));
  exit(1);
}

void setupSignalHandlers(void (*handler)(int)) {
  struct sigaction newAction, oldAction;
  memset(&newAction, 0, sizeof(newAction));
  newAction.sa_handler = handler;
  sigemptyset(&newAction.sa_mask);

  // Set termination handlers and preserve ignore flag.
  sigaction(SIGINT, NULL, &oldAction);
  if (oldAction.sa_handler != SIG_IGN)
    sigaction(SIGINT, &newAction, NULL);
  sigaction(SIGHUP, NULL, &oldAction);
  if (oldAction.sa_handler != SIG_IGN)
    sigaction(SIGHUP, &newAction, NULL);
  sigaction(SIGTERM, NULL, &oldAction);
  if (oldAction.sa_handler != SIG_IGN)
    sigaction(SIGTERM, &newAction, NULL);
}

class Main {
  cv::Mat _color, _depth;
  cv::Mat _colorHalf, _depthHalf;
  uint64_t _timestamp;
  CameraParameters _camParams;
  shared_ptr<Visualizer> _visualizer;
  shared_ptr<Mapper> _mapper;

  string _logNameRead, _logNameWrite, _baseNameRead, _baseNameWrite;
  bool _showImage, _processing, _colorPlanes, _flipRGB;

  void processImage() {
    // print frame number and timing stats
    static double t0 = pcl::getTime();
    static int c = -1;
    c++;
    cout << endl;
    cout << "Frame " << c;
    if ((c+1) % 20 == 0) {
      double t = pcl::getTime();
      cout << " (" << (20. / (t - t0)) << " fps)" << endl;
      t0 = t;
    }
    cout << endl;

    if (_flipRGB) {
      cv::Mat colorTmp(_color);
      cvtColor(colorTmp, _color, CV_RGB2BGR);
    }

    if (_showImage) {
      const double scale = 0.5;
      _visualizer->showImage(_depth, _color, scale);
    }

    if (_processing) {

      if (c % processEveryNth != 0) { // subsample

        cout << "Skipping frame" << endl;

      } else {

        // extract planes
        planes_t planes;
        clusters_t clusters;
        PCPoint::Ptr cloud;
        std::tie(planes, clusters, cloud) = extractPlanes(_depth, _color,
            _camParams);

        if (_showImage) {
          _visualizer->showSegmentation(_color, clusters);
        }

        // add planes of current view to SLAM
        _mapper->processFrame(_timestamp, planes, clusters, cloud);
      }
    }
  }

  void processStream() {

    if (!_logNameRead.empty()) {
      cout << "processStream :: !_logNameRead.empty() \n";
      // read logged stream
      Logger logger(_logNameRead);
      while (!stop && logger.getFrame(_depth, _color, _timestamp)) {
        cout << "_color \n" << _color.rows  << ", " << _color.cols << std::endl;
        processImage();
//        this_thread::sleep_for(chrono::milliseconds(5000));
      }
      if (!stop) {
        cout << "End of log file" << endl;
      }

    } else {
      // live RGB-D stream

      // signal handler will destroy camera object to safely shut down camera
      // (avoids errors with older Xtions)
      setupSignalHandlers(terminationHandler);

      Camera camera(width, height, fps);
      _camParams = camera.getParameters();

      _camParams.print();

      Logger* logger = NULL;
      if (!_logNameWrite.empty()) {
        logger = new Logger(_logNameWrite, true);
      }
      cout << "starting stream" << endl;
      while (!stop) {
        if (camera.getFrame(_depth, _color, _timestamp)) {
          cout << "processing one" << endl;
          processImage();
          if (logger) {
            // write to log file
            logger->writeFrame(_depth, _color, _timestamp);
          }
          if (!_baseNameWrite.empty()) {
            imwrite(_baseNameWrite + ".D.png", _depth);
            imwrite(_baseNameWrite + ".C.png", _color);
          }
        }
      }
      if (logger) {
        // write frame count
        logger->close();
        delete logger;
      }
    }

  }

  void parseOptions(int argc, char* argv[]) {
    // parse command line arguments
    _logNameRead = "";
    _logNameWrite = "";
    _baseNameRead = "";
    _baseNameWrite = "";
    _showImage = false;
    _processing = true;
    _colorPlanes = false;
    _flipRGB = false;
    int c;
    while ((c = getopt(argc, argv, ":h?cfinL:r:w:")) != -1) {
      switch (c) {
      case 'h':
      case '?':
        cout << usage;
        exit(0);
        break;
      case 'c':
        _colorPlanes = true;
        break;
      case 'f':
        _flipRGB = true;
        break;
      case 'i':
        _showImage = true;
        break;
      case 'n':
        _processing = false;
        break;
      case 'L':
        if (optarg) {
          _logNameWrite = optarg;
        }
        break;
      case 'r':
        if (optarg) {
          _baseNameRead = optarg;
        }
        break;
      case 'w':
        if (optarg) {
          _baseNameWrite = optarg;
        }
        break;
      case ':':
        // unknown option, from getopt
        cout << "Error: Unknown argument." << endl;
        cout << usage;
        exit(1);
        break;
      }
    }
    if (argc == optind + 1) {
      _logNameRead = string(argv[optind]);
    } else if (argc > optind + 1) {
      cout << "Error: Too many arguments." << endl;
      cout << usage;
      exit(1);
    }
  }

public:

  Main(int argc, char* argv[]) {
    parseOptions(argc, argv);

    _visualizer = make_shared<Visualizer>(_colorPlanes);
    _mapper = make_shared<Mapper>(_visualizer);
  }

  void run(string dataset) {
    _depth = cv::Mat(height, width, CV_16UC1);
    _color = cv::Mat(height, width, CV_8UC3);
    _depthHalf = cv::Mat(height/2, width/2, CV_16UC1);
    _colorHalf = cv::Mat(height/2, width/2, CV_8UC3);

    // Kaess 
    if (dataset.compare("kaess")) {
      _camParams.width = 640;
      _camParams.height = 480;
      _camParams.fovHorizontal = 1.0226; // 60 degrees
      _camParams.fovVertical = 0.796616; // 45 degrees
      _camParams.fx = 570.342;
      _camParams.fy = _camParams.fx;
      _camParams.px = 320;
      _camParams.py = 240;
    }

    // ICL-NUIM
    if (dataset.compare("icl")) {
    _camParams.width = 640;
    _camParams.height = 480;
    _camParams.fovHorizontal = 1.5708; // 60 degrees
    _camParams.fovVertical = 1.5708; // 45 degrees
    _camParams.fx = 481.20;
    _camParams.fy = -480.00;
    _camParams.px = 319.50;
    _camParams.py = 239.50;
    }

    if (!_baseNameRead.empty()) {
      // read image from file; process and exit

      _depth = cv::imread(_baseNameRead + ".D.png",
          CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH);
      if (_depth.data == NULL) {
        cout << "Error: Could not open depth image" << endl;
        exit(1);
      }
      _color = cv::imread(_baseNameRead + ".C.png", CV_LOAD_IMAGE_COLOR);
      if (_color.data == NULL) {
        cout << "Error: Could not open color image" << endl;
        exit(1);
      }
      processImage();
      exit(0);

    } else {
      // sensor stream (log or live)

      // allows interruption through signal and clean termination of logging
      thread t(&Main::processStream, this);
      t.join();

    }
  }
};

int main(int argc, char* argv[]) {
  Main m(argc, argv);
  string dataset = "kaess";
  // string dataset = "icl";
  m.run(dataset);

  return (0);
}
