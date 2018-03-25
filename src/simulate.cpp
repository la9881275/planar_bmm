// testing plane optimization
// kaess 7/2013

#include <string>
#include <vector>

#include <Eigen/Dense>

#include <pcl/common/time.h>

#include <isam/isam.h>

#include "Plane3d.h"
#include "Simulation.h"

using namespace std;
using namespace isam;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

// sigma: x,y,z, yaw,pitch,roll (radians)
//double pose_sigmas_doubles[] = {0.01, 0.01, 0.001, 0.001,0.0005,0.0005};
double pose_sigmas_doubles[] = { 0.1, 0.1, 0.1, 0.01, 0.01, 0.01 };
const Map<Vector6d> pose_sigmas(pose_sigmas_doubles);
const Vector3d plane_sigmas(0.005, 0.005, 0.005);

const bool useRelative = true;

class Example {
  mt19937 _gen;

  Slam _slam;

  vector<Pose3d_Node *> _pose_nodes;
  vector<Plane3d_Node *> _plane_nodes;

  Simulation _sim;

  Covariance _pose_cov;
  Covariance _plane_cov;

public:

  inline double sampleNormal(double mean, double sigma) {
    normal_distribution<> dist(mean, sigma);
    return dist(_gen);
  }

  Example() :
      _pose_cov(Covariance(pose_sigmas.cwiseProduct(pose_sigmas).asDiagonal())), _plane_cov(
          Covariance(plane_sigmas.cwiseProduct(plane_sigmas).asDiagonal())) {
    random_device rd;
    //    _gen = mt19937(rd());
    _gen = mt19937(42); // repeatable "randomness"

    _sim.generate();
  }

  void setup_pose0() {
    // initial pose
    Pose3d_Node* p1 = new Pose3d_Node();
    _slam.add_node(p1);
    _pose_nodes.push_back(p1);
    Pose3d_Factor* prior = new Pose3d_Factor(p1, Pose3d(), _pose_cov);
    _slam.add_factor(prior);
  }

  void add_pose(int i) {
    Pose3d odo;
    odo.of_pose2d(_sim.odometry[i - 1]);

    // add noise
    Matrix4d delta = odo.wTo();
    Vector6d noise;
    for (int i = 0; i < 6; i++) {
      noise[i] = sampleNormal(0, pose_sigmas[i]);
    }
    delta = Pose3d(noise).wTo() * delta; // Eigen: no aliasing for matrix mult
    isam::Pose3d deltaPose(delta);

    // construct new node
    Pose3d_Node * newNode = new Pose3d_Node();
    _slam.add_node(newNode);

    Pose3d_Pose3d_Factor * odo_factor = new Pose3d_Pose3d_Factor(
        _pose_nodes.back(), newNode, deltaPose, _pose_cov);
    _slam.add_factor(odo_factor);

    _pose_nodes.push_back(newNode);
  }

  void setup_poses() {

    setup_pose0();

    // construct pairwise constraints (with noise)
    for (int i = 1; i < _sim.poses.size(); i++) {
      add_pose(i);
    }

  }

  void setup_planes() {

    for (int i = 0; i < _sim.planes.size(); i++) {
      Plane3d_Node* plane = new Plane3d_Node();
      _slam.add_node(plane);
      _plane_nodes.push_back(plane);
    }

    for (int i = 0; i < _sim.measurements.size(); i++) {

      int pose_id, plane_id;
      Plane3d measure;
      tie(pose_id, plane_id, measure) = _sim.measurements[i];

      // add noise in tangent space
      Vector3d noise;
      for (int k = 0; k < 3; k++) {
        noise(k) = sampleNormal(0, plane_sigmas[k]);
      }
      measure = measure.exmap_3dof(noise);
      Pose3d_Plane3d_Factor * newFactor = new Pose3d_Plane3d_Factor(
          _pose_nodes[pose_id], _plane_nodes[plane_id], measure, _plane_cov,
          useRelative);
      _slam.add_factor(newFactor);

    }

  }

  void batch() {

    Properties prop;
//    prop.method = LEVENBERG_MARQUARDT;
    prop.method = DOG_LEG;
    _slam.set_properties(prop);

    setup_poses();

    setup_planes();

//    _slam.print_graph();

    cout << "before" << endl;
    for (int i = 0; i < _pose_nodes.size(); i++) {
      Pose3d p = _pose_nodes[i]->value();
      cout << p.x() << " " << p.y() << " " << p.z() << endl;
    }

    double t0 = pcl::getTime();

    _slam.batch_optimization();

    cout << "Time for optimization: " << pcl::getTime() - t0 << endl;

    //    cout << "after optimization" << endl;
//    _slam.print_graph();

    cout << "after" << endl;
    for (int i = 0; i < _pose_nodes.size(); i++) {
      Pose3d p = _pose_nodes[i]->value();
      cout << p.x() << " " << p.y() << " " << p.z() << endl;
    }

  }

  void add_planes(int id) {

    for (int k = 0; k < _sim.measurements.size(); k++) {

      int pose_id, plane_id;
      Plane3d measure;
      tie(pose_id, plane_id, measure) = _sim.measurements[k];

      if (pose_id == id) {

        // add as many planes as needed
        for (int tmp = _plane_nodes.size(); tmp <= plane_id; tmp++) {

          Plane3d_Node* plane = new Plane3d_Node();
          _slam.add_node(plane);
          _plane_nodes.push_back(plane);
        }

        // add noise in tangent space
        Vector3d noise;
        for (int t = 0; t < 3; t++) {
          noise(t) = sampleNormal(0, plane_sigmas[t]);
        }
        measure = measure.exmap_3dof(noise);
        Pose3d_Plane3d_Factor * newFactor = new Pose3d_Plane3d_Factor(
            _pose_nodes[pose_id], _plane_nodes[plane_id], measure, _plane_cov,
            useRelative);
        _slam.add_factor(newFactor);

      }
    }

  }

  void incremental() {
    Properties prop;
 //   prop.method = LEVENBERG_MARQUARDT;
//       prop.method = DOG_LEG;
    prop.mod_batch = 1; //20;
    _slam.set_properties(prop);


    setup_pose0();
    add_planes(0);

    for (int i = 1; i < _sim.poses.size(); i++) {
      cout << "time step " << i << endl;
      add_pose(i);
      add_planes(i);

      double t0 = pcl::getTime();

      _slam.update();

      cout << endl << "Time for update: " << pcl::getTime() - t0 << endl;
    }

    cout << "chi2=" << _slam.chi2() << " normalized chi2=" << _slam.normalized_chi2() << endl;
  }

};

int main(int argc, char * argv[]) {
  Example e;

  e.batch();
//  e.incremental();

  return 0;
}
