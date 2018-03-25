/*
 * Visualizer.cpp
 *
 *  Created on: Apr 24, 2014
 *      Author: kaess
 */

#include <vector>
#include <thread>

#include <Eigen/Dense>

#include <pcl/visualization/cloud_viewer.h>

#include "isam/isam.h"
#include "planes.h"
#include "Plane3d.h"
#include "Visualizer.h"

const int downsampleFactor = 2;
const bool drawOnlyPlanes = true;

using namespace std;

class Colors {
  vector<tuple<int, int, int> > colors;
public:
  Colors() {
    random_device rd;
    default_random_engine e1(rd());
    uniform_int_distribution<int> uniform_dist(0, 255);
    for (int i = 0; i < 500; i++) {
      int r = uniform_dist(e1);
      int g = uniform_dist(e1);
      int b = uniform_dist(e1);
      colors.push_back(make_tuple(r, g, b));
    }
  }
  tuple<int, int, int> get(int id) {
    return colors[id + 1];
  }
};

Colors colors;

tuple<int, int, int> getColor255(int id) {
  return colors.get(id);
}

tuple<double, double, double> getColor01(int id) {
  int r,b,g;
  tie(r,g,b) = colors.get(id);
  return make_tuple(((double)r)/255., ((double)g)/255., ((double)b)/255.);
}

void Visualizer::draw(pcl::visualization::PCLVisualizer& viewer, const vis_data_t& data) {
  static int counter = 0;

  vector<int> assoc;
  planes_t planes;
  clusters_t clusters;
  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;
  Eigen::Affine3f poseAffine;
  isam::Pose3d pose;
  std::tie(poseAffine, pose, assoc, planes, clusters, cloud) = data;

  PCPoint::Ptr downsampled(new PCPoint());
  for (int i=0; i<planes.size(); i++) {

    if (false) { // draw abstract planes
      static int planeCounter = 0;
      Eigen::Vector4d abcd = planes[i].transform_from(pose).vector(); // extracted planes are relative to current pose
      pcl::ModelCoefficients coeff;
      coeff.values.resize(4);
      coeff.values[0] = abcd(0);
      coeff.values[1] = abcd(1);
      coeff.values[2] = abcd(2);
      coeff.values[3] = abcd(3);
      viewer.addPlane(coeff, "p"+to_string(planeCounter));
      planeCounter++;
    }

    // draw all points
    if (drawOnlyPlanes) {
      int r, g, b;
      tie(r, g, b) = getColor255(assoc[i]);
      uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
      for (int idx = 0; idx < clusters[i].size(); idx+=downsampleFactor) {
        auto pt = cloud->at(clusters[i][idx]);
        if (_colorPlanes) {
          pt.rgba = rgb;
        }
        downsampled->push_back(pt);
      }
//      auto colorHandler = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(subset, r,g,b);
//      viewer.addPointCloud(subset, colorHandler, to_string(counter));
    }
  }
  if (!drawOnlyPlanes) {
    if (downsampleFactor > 1) {
      vector<int> subset;
      if (true) { // randomly sample (with duplicates...)
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, cloud->size()-1);
        for (int idx=0; idx<cloud->size()/downsampleFactor; idx++) {
          subset.push_back(dis(gen));
        }
      } else { // every nth point (generates patterns)
        for (int idx = 0; idx<cloud->size(); idx+=downsampleFactor) {
          subset.push_back(idx);
        }
      }
      downsampled.reset(new PCPoint(*cloud, subset));
    } else { // use full original point cloud
      downsampled.reset(new PCPoint(*cloud));
    }
  }
  viewer.addPointCloud(downsampled, to_string(counter));
  viewer.updatePointCloudPose(to_string(counter), poseAffine);
  counter++;
}

void Visualizer::loop() {
  pcl::visualization::PCLVisualizer viewer("3D Model");
//  viewer.setBackgroundColor(0.0, 0.0, 0.5);
  viewer.setBackgroundColor(1.0, 1.0, 1.0);
  //  viewer.setCameraPosition(0,0,-1, 0,0,2, 0,-1,0);
  viewer.setCameraPosition(-1,0,0, 2,0,0, 0,0,-1);

  while (!viewer.wasStopped()) {
    viewer.spinOnce(2);
//    this_thread::sleep_for(chrono::milliseconds(50));

    _mutexState.lock();
    while (!_data.empty()) {
      // draw one point cloud
      vis_data_t data = _data.front();
      draw(viewer, data);
      _data.pop();
    }
    _mutexState.unlock();

    _mutexPoses.lock();
    if (_posesUpdated) {
      // show latest camera coordinate frame
      viewer.removeCoordinateSystem();
      viewer.addCoordinateSystem(0.3, _poses[_poses.size()-1]);
      _posesUpdated = false;

      // adjust poses to latest estimate
      for (int counter = 0; counter<_poses.size(); counter++) {
        viewer.updatePointCloudPose(to_string(counter), _poses[counter]);
      }
    }
    _mutexPoses.unlock();
  }
}

Visualizer::Visualizer(bool colorPlanes) {
  _colorPlanes = colorPlanes;
  _posesUpdated = false;
  _thread = thread(&Visualizer::loop, this);
}

Visualizer::~Visualizer() {
  _thread.join();
}

void Visualizer::updatePoses(const poses_t& poses) {
  _mutexPoses.lock();
  _poses = poses;
  _posesUpdated = true;
  _mutexPoses.unlock();
}

void Visualizer::addState(const vis_data_t& data) {
  _mutexState.lock();
  _data.push(data);
  _mutexState.unlock();
}

// todo: move into thread
void Visualizer::showImage(const cv::Mat& depthIn, const cv::Mat& colorIn, double scale) {
  // scale
  cv::Mat depth, color;
  cv::resize(depthIn, depth, cv::Size(), scale, scale);
  cv::resize(colorIn, color, cv::Size(), scale, scale);
  // scale depth
  double low, high;
  cv::minMaxLoc(depth, &low, &high);
  cv::Mat depthScaled = depth * (65535 / high);
#if 1
  // combine images
  cv::Mat combined(color.rows, color.cols*2, CV_8UC3);
  cv::Mat left(combined, cv::Rect(0, 0, color.cols, color.rows));
  cv::Mat right(combined, cv::Rect(color.cols, 0, color.cols, color.rows));
  color.copyTo(left);
  cv::Mat depthScaled8;
  depthScaled.convertTo(depthScaled8, CV_8U, 1./256.);
  cv::cvtColor(depthScaled8, right, CV_GRAY2BGR);
  imshow("Color and Depth", combined);
#else
  imshow("Depth", depthScaled);
  imshow("Color", _colorHalf);
#endif
  cv::waitKey(1);
}

void Visualizer::showSegmentation(const cv::Mat& color, const clusters_t& clusters) {
  // visualize segmentation
  cv::Mat image = color.clone();
  int i = 0;
  for (auto cluster : clusters) {
    int r, g, b;
    tie(r, g, b) = getColor255(i);
    for (int point : cluster) {
      image.data[point * 3] = r;
      image.data[point * 3 + 1] = g;
      image.data[point * 3 + 2] = b;
    }
    i++;
  }
  imshow("Segmented Planes", image);
}

