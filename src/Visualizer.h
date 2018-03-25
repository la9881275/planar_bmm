/*
 * Visualizer.h
 *
 *  Created on: Apr 24, 2014
 *      Author: kaess
 */

#pragma once

#include <thread>
#include <mutex>
#include <queue>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "planes.h"

// eigen/std::vector special treatment
#include<Eigen/StdVector>
typedef std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > poses_t;


std::tuple<int, int, int> getColor255(int id);
std::tuple<double, double, double> getColor01(int id);

typedef std::tuple<Eigen::Affine3f, isam::Pose3d, std::vector<int>, planes_t, clusters_t, pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr> vis_data_t;

class Visualizer {
  bool _colorPlanes;
  std::thread _thread;
  std::mutex _mutexPoses, _mutexState;
  bool _posesUpdated;
  poses_t _poses;
  std::queue<vis_data_t> _data;

  void draw(pcl::visualization::PCLVisualizer& viewer, const vis_data_t& data);

  void loop();

public:

  Visualizer(bool usePlaneColor);

  ~Visualizer();

  void updatePoses(const poses_t& poses);

  void addState(const vis_data_t& data);

  void showImage(const cv::Mat& depth, const cv::Mat& color, double scale);

  void showSegmentation(const cv::Mat& color, const clusters_t& clusters);

};
