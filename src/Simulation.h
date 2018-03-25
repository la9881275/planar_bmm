/**
 * @file Simulation.h
 * @brief Generate simulated robot trajectory and planes.
 * @author Michael Kaess
 * @version $Id: $
 */

#pragma once

#include <vector>
#include <tuple>

#include <isam/Pose2d.h>

#include "Plane3d.h"

class Simulation {
  std::mt19937 gen;

public:

  std::vector<isam::Pose2d> poses;
  std::vector<isam::Pose2d> odometry;
  std::vector<isam::Plane3d> planes;
  std::vector<std::tuple<int, int, isam::Plane3d> > measurements;

  Simulation();

  void generate();

  void add_planes(const int pose_id, const isam::Pose2d& pose);
};
