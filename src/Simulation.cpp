/**
 * @file Simulate.cpp
 * @brief Generate simulated robot trajectory and planes.
 * @author Michael Kaess
 * @version $Id: $
 */

#include <isam/Pose2d.h>
#include <vector>
#include <Eigen/Dense>

#include "Simulation.h"

using namespace std;
using namespace isam;
using namespace Eigen;

const int HEIGHT = 20;
const int WIDTH = 20;
const int SIMULATION_STEPS = 15; //100; //100; // 15;
const int BLOCK_STEPS = 5;
const double PLANE_VISIBILITY = 5.;
const int MIN_PLANES = 6;

enum Directions {
  NORTH = 1, EAST = 2, SOUTH = 3, WEST = 4
};

// cumulative density (CDF) for directions (forward, left, right, backward)
//const double dir_prob[4] = {0.6, 0.82, 1.0, 1.0};  // Manhattan world for incremental experiment
const double dir_prob[4] = { 1., 1., 1., 1. };   // straight line for batch experiment

void Simulation::add_planes(const int pose_id, const Pose2d& pose) {
  Pose3d pose3d;
  pose3d.of_pose2d(pose);
  Point3d p(pose.x(), pose.y(), 0);

  // which planes are visible?
  vector<int> visible;
  for (int i = 0; i < planes.size(); i++) {
    if (planes[i].distance(p) <= PLANE_VISIBILITY) {
      visible.push_back(i);
    }
  }

  // add new planes if not enough visible
  uniform_real_distribution<> unif_n(-1, 1);
  uniform_real_distribution<> unif_d(0, PLANE_VISIBILITY);
  while (visible.size() < MIN_PLANES) {
    // todo: normal could be 0,0,0... (unlikely)
    Plane3d plane_x(Vector3d(unif_n(gen), unif_n(gen), unif_n(gen)),
        unif_d(gen));
    Plane3d plane = plane_x.transform_from(pose3d);
    planes.push_back(plane);
    visible.push_back(planes.size() - 1);
  }

  // add measurements to visible planes
  for (int i = 0; i < visible.size(); i++) {
    int plane_id = visible[i];
    Plane3d measure = planes[plane_id].transform_to(pose3d);
    measurements.push_back(make_tuple(pose_id, plane_id, measure));
  }
}

Simulation::Simulation() {
  random_device rd;
  //  gen = mt19937(rd());
  gen = mt19937(42.);
}

void Simulation::generate() {
  poses.clear();
  odometry.clear();
  planes.clear();
  measurements.clear();

  Pose2d pose;

  pose = Pose2d(0, 0, -M_PI / 2.0);

  poses.push_back(pose);

  uniform_real_distribution<> action(0., 1.);

  int lastDirection = 1; // default direction is north

  for (int i = 0; i < SIMULATION_STEPS; i++) {
    // pick direction
    int dir;

    double act = action(gen);
    if (act <= dir_prob[0]) {
      // forward
      dir = lastDirection;
    } else if (act <= dir_prob[1]) {
      // left
      dir = ((lastDirection - 1) - 1 + 4) % 4 + 1;
    } else if (act <= dir_prob[2]) {
      // right
      dir = ((lastDirection + 1) - 1 + 4) % 4 + 1;
    } else {
      // backward
      dir = ((lastDirection + 2) - 1 + 4) % 4 + 1;
    }

    // check boundary
    int u = (int) pose.x();
    int v = (int) pose.y();
    if (dir == NORTH && u >= HEIGHT)
      continue;
    if (dir == SOUTH && u <= -HEIGHT)
      continue;
    if (dir == EAST && v <= -WIDTH)
      continue;
    if (dir == WEST && v >= WIDTH)
      continue;

    // always traverse a whole block
    for (int k = 0; k < BLOCK_STEPS; k++) {
      // move to new location
      Pose2d d;
      if (lastDirection != dir) {
        switch (dir) {
        case NORTH:
          d.set(0.0, 0.0, 0);
          break;
        case SOUTH:
          d.set(0.0, 0.0, M_PI);
          break;
        case EAST:
          d.set(0.0, 0.0, -M_PI / 2.0);
          break;
        case WEST:
          d.set(0.0, 0.0, M_PI / 2.0);
          break;
        }
        k = k - 1; // add an extra iteration for the heading check
        Pose2d b(0, 0, pose.t());
        d = d.ominus(b); // represent direction in vehicle frame
      } else {
        d.set(1.0, 0.0, 0);
      }
      lastDirection = dir;

      pose = pose.oplus(d);

      // save pose
      poses.push_back(pose);

      // save constraint
      odometry.push_back(d);

      // add plane measurements, and planes if needed
      add_planes(poses.size() - 1, pose);

    }
  }

  cout << "generated " << poses.size() << " poses and " << planes.size()
      << " planes with " << measurements.size() << " plane measurements" << endl;
}
