/*
 * Mapper.cpp
 *
 *  Created on: Apr 24, 2014
 *      Author: kaess
 */

#include <Eigen/Dense>

#include "isam/isam.h"
#include "isam/slam3d.h"
#include "Plane3d.h"

#include "planes.h"
#include "Visualizer.h"
#include "Mapper.h"

using namespace std;
using namespace isam;

const bool useRelative = true;

// for plane matching (allows for camera motion)
const double maxDistance = 0.10; //0.1; //0.12; //0.1; //0.05; //0.12;
const double maxAngle = /*8.  3.  5. 8.*/ 8. / 180. * M_PI;

// skip frames
const int renderEveryNth = 10;

// sigma: x,y,z, yaw,pitch,roll (radians)
typedef Eigen::Matrix<double, 6, 1> Vector6d;
//double tmp[] = { 0.01, 0.01, 0.001, 0.001, 0.0005, 0.0005 };
//double tmp[] = { 0.05, 0.05, 0.05, 0.01, 0.01, 0.01 }; // weak prior
double tmp[] = { 0.0005, 0.0005, 0.0005, 0.1, 0.1, 0.1 }; // strong prior
const Eigen::Map<Vector6d> poseSigmas(tmp);
const Eigen::Vector3d planeSigmas(0.001, 0.001, 0.001);
Covariance poseCov(poseSigmas.cwiseProduct(poseSigmas).asDiagonal());
Covariance planeCov(planeSigmas.cwiseProduct(planeSigmas).asDiagonal());

Mapper::Mapper(shared_ptr<Visualizer> visualizer) {
  _visualizer = visualizer;

  Properties prop = _slam.properties();
//    prop.method = LEVENBERG_MARQUARDT;
//  prop.method = DOG_LEG;
  prop.mod_batch = 1;
  _slam.set_properties(prop);
}

// returns the id of the best matching plane, or -1 if no match found
int Mapper::findClosestPlane(const Pose3d& pose, const Plane3d& plane) {
  int numMatches = 0;
  int bestMatch = -1;
  double bestDist = -1;
  // plane is in local frame of pose, and we'll leave it there
//  Plane3d p = plane.transform_from(pose);
  for (int c = 0; c < _planeNodes.size(); c++) {
    Plane3d p2 = _planeNodes[c]->value();
    // p2 is potentially relative to some base pose
    if (useRelative) {
      p2 = p2.transform_from(_planeNodes[c]->base()->value());
    }
    p2 = p2.transform_to(pose);
    // check distance between planes: pick point on p2,
    // calculate distance to plane; this assumes that normals are similar
    double dist = plane.distance(p2.point0());
    if (dist < maxDistance) {
      // now check normals
      double alpha = fabs(acos(plane.normal().dot(p2.normal())));
      if (alpha < maxAngle) {
        numMatches++;
        cout << numMatches << " " << c << " " << dist << " " << alpha << endl;
        if (numMatches == 1 || dist < bestDist) {
          bestMatch = c;
          bestDist = dist;
        }
      }
    }
  }
  if (numMatches > 1) {
    cout << "Warning: multiple potential matches for one plane." << endl;
  }
  return bestMatch;
}

bool Mapper::isFullyConstrained(const planes_t& planes) {
  // Detect underconstrained situations, i.e. less than 3 general planes:
  // We take the normals of all planes, using them as points in addition
  // to the origin. We fit a plane to that set, and check the distance
  // of each point (i.e. normal) to the plane. If there is an unconstrained
  // direction (minus noise), all of the normals will lie (approximately)
  // in a plane, i.e. their distance to the plane will all be close to 0.

  bool fullyConstrained = false;

  // setup matrix of points given by plane normals, and include the origin
  int num = planes.size();
  Eigen::MatrixXd points(3, num+1);
  for (int i = 0; i < num; i++) {
    const Eigen::Vector3d p = planes[i].normal();
    points.col(i) = p;
  }
  points.col(num) = Eigen::Vector3d(0.0, 0.0, 0.0);

  // find direction of lowest variance (if full rank)
  // subtract centroid
  Eigen::Vector3d mean = points.rowwise().mean();
  Eigen::MatrixXd points0 = points - mean.replicate(1, num+1);
  // the normal is given by the singular vector corresponding to lowest
  // singular value, here in last/third column
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(points0,
      Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.matrixU().cols() == 3) { // if full rank (otherwise not fully constrained)
    Eigen::Vector3d normal = svd.matrixU().col(2);
    double d = -normal.dot(mean);
    isam::Plane3d plane(normal, d);
    // check if any point is outside the plane, minus noise (could probably just threshold on third singular value...)
    for (int i=0; i<num+1; i++) {
      double dist = plane.distance(Point3d(points.col(i)));
      cout << "distance " << dist << endl;
      if (dist > 0.05) {
        fullyConstrained = true;
      }
    }
  }

  if (!fullyConstrained) {
    cout << "WARNING: Planes do not provide full pose constraints." << endl;
  }

//  return fullyConstrained;
  return true; // todo
}

void Mapper::processFrame(uint64_t timestamp, const planes_t& planes,
    const clusters_t& clusters, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {

  _tic();

  // data association
  Pose3d estimate;
  if (_nodes.size() > 0) {
    estimate = _nodes[_nodes.size() - 1]->value();
  }
  int numPlanes = _planeNodes.size();
  int numNewPlanes = 0;
  vector<int> assoc(planes.size());
  planes_t planesToCheck;
  for (int i = 0; i < planes.size(); i++) {
    // (local) data association
    int id = findClosestPlane(estimate, planes[i]);
    if (id >= 0) {
      assoc[i] = id;
      planesToCheck.push_back(planes[i]);
    } else {
      // no match, create new plane
      assoc[i] = numPlanes + numNewPlanes;
      numNewPlanes++;
    }
  }

  _toc("data association");

  if (_nodes.size() > 0 && !isFullyConstrained(planesToCheck)) {

    cout << "Warning: insufficient constraints provided by planes, skipping frame" << endl;

  } else {

    // add new pose
    Pose3d_Node* poseNode = new Pose3d_Node();
    _slam.add_node(poseNode);
    _nodes.push_back(poseNode);
    if (_nodes.size() == 1) {
      // prior on first pose only
      Pose3d origin;
      Pose3d_Factor* prior = new Pose3d_Factor(poseNode, origin, poseCov);
      _slam.add_factor(prior);
    } else {
#if 0
      // initialize based on estimate of previous pose
      poseNode->init(_nodes[_nodes.size()-2]->value());
#else
      // add constraint to previous pose
      Pose3d_Pose3d_Factor* constraint = new Pose3d_Pose3d_Factor(
          _nodes[_nodes.size() - 2], poseNode, Pose3d(), poseCov);
      _slam.add_factor(constraint);
#endif
    }

    _tic();

    // add new planes
    for (int i = 0; i < numNewPlanes; i++) {
      Plane3d_Node* planeNode = new Plane3d_Node();
      _slam.add_node(planeNode);
      _planeNodes.push_back(planeNode);
    }

    // add plane constraints
    for (int i = 0; i < assoc.size(); i++) {
      auto planeNode = _planeNodes[assoc[i]];
      auto measure = planes[i];
      Pose3d_Plane3d_Factor* fac = new Pose3d_Plane3d_Factor(poseNode,
          planeNode, measure, planeCov, useRelative);
      _slam.add_factor(fac);
    }

    cout << "Number of poses: " << _nodes.size() << endl;
    cout << "Number of planes: " << _planeNodes.size() << endl;
    cout << "Number of factors: " << _slam.get_factors().size() << endl;

    // optimize
    _slam.update();
//    _slam.batch_optimization();

    cout << endl;
    _toc("SLAM");

    // visualize
    static int counter = -1;
    counter++;
    if (counter % renderEveryNth == 0) {
      poses_t poses;
      Eigen::Affine3f poseAffine;
      Pose3d pose;
      for (int i = 0; i < _nodes.size(); i+=renderEveryNth) {
        pose = _nodes[i]->value();
        Eigen::Matrix4f wTo = pose.wTo().cast<float>();
        poseAffine = Eigen::Affine3f(wTo);
        poses.push_back(poseAffine);
      }
      _visualizer->addState(make_tuple(poseAffine, pose, assoc, planes, clusters, cloud));
      _visualizer->updatePoses(poses);
    }
  }
}
