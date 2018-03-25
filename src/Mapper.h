/*
 * Mapper.h
 *
 *  Created on: Apr 24, 2014
 *      Author: kaess
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "isam/isam.h"

#include "planes.h"
#include "Plane3d.h"
#include "Visualizer.h"

class Mapper {

  isam::Slam _slam;
  std::vector<isam::Pose3d_Node*> _nodes;
  std::vector<isam::Plane3d_Node*> _planeNodes;

  std::shared_ptr<Visualizer> _visualizer;

  int findClosestPlane(const isam::Pose3d& pose, const isam::Plane3d& plane);
  bool isFullyConstrained(const planes_t& planes);

public:

  Mapper(std::shared_ptr<Visualizer> visualizer);

  void processFrame(uint64_t timestamp, const planes_t& planes, const clusters_t& clusters,
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

};
