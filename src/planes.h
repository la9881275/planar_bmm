/*
 * planes.h
 *
 *  Created on: Feb 12, 2014
 *      Author: kaess
 */

#pragma once

#include <list>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "opencv2/opencv.hpp"

#include "Plane3d.h"
#include "Camera.h"

void _tic();
void _toc(const std::string& s);

typedef pcl::PointCloud<pcl::PointXYZRGBA> PCPoint;
typedef pcl::PointCloud<pcl::Normal> PCNormal;

#if 0
class Plane {
  Eigen::Vector3d _normal;
  double _d;
public:
  Plane(const Eigen::Vector3d& normal, double d) : _normal(normal), _d(d) {}
  double a() {return _normal(0);}
  double b() {return _normal(1);}
  double c() {return _normal(2);}
  double d() {return _d;}
  Eigen::Vector3d normal() {return _normal;}
};
#endif

typedef std::vector<isam::Plane3d> planes_t;

typedef std::vector<std::vector<int> > clusters_t;

std::tuple<planes_t, clusters_t, PCPoint::Ptr> extractPlanes(const cv::Mat& depth, const cv::Mat& color, const CameraParameters camParams);
