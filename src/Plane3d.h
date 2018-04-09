/*
 * Plane3d.h
 *
 *  Created on: Feb 12, 2014
 *      Author: kaess
 */

#pragma once

#include <Eigen/Dense>

#include <boost/math/special_functions/sinc.hpp>
#include <boost/math/special_functions/sign.hpp>

#include <isam/isam.h>


//#define WRONG  // use 4-vector for optimization
//#define WRONG2 // use 4-vector for error evaluation (also change sigmas 3d/4d)

namespace isam {


class Plane3d {
private:
  friend std::ostream& operator<<(std::ostream& out, const Plane3d& p) {
    p.write(out);
    return out;
  }

  Eigen::Vector4d _abcd;

  // normalize the homogeneous vector
  void _normalize() {
    _abcd.normalize();
  }

public:
#ifdef WRONG
  static const int dim = 4; // wrong DoF
#else
  static const int dim = 3; // 3 DoF
#endif
  static const int size = 4; // 4 params
  static const char* name() {
    return "Plane3d";
  }
  Plane3d() : _abcd(1.,0.,0.,0.) {}
  Plane3d(const Eigen::Vector3d& normal, double dist) {
    Eigen::Vector3d nn = normal.normalized();
    _abcd.head(3) = nn;
    _abcd(3) = -dist;
    _normalize();
  }
  Plane3d(const Eigen::Vector4d& vec) : _abcd(vec) {_normalize();}

  Eigen::VectorXb is_angle() const {
    Eigen::VectorXb isang (dim);
    isang << false, false, false, false;
    return isang;
  }


  static Eigen::Quaterniond delta3_to_quat(const Eigen::Vector3d& delta) {
    double theta = delta.norm();
    if (theta > M_PI) {std::cout << "t=" << theta << " delta3_to_quat error\n"; exit(1);} // todo
#if 0
    double S;
    if (theta < 0.0001) { // sqrt4(machine precession)
      S = 0.5 + theta*theta/48.;
    } else {
      S = sin(0.5*theta)/theta;
    }
#else
    double S = 0.5 * boost::math::sinc_pi(0.5*theta); // 0.5 from sinc!
#endif
    double C = cos(0.5*theta);
    return Eigen::Quaterniond(C, S*delta(0), S*delta(1), S*delta(2));
  }

  /**
   * Updates the homogeneous plane representation (4 parameters) using
   * a minimal parametrization (3DoF).
   * @param delta Vector containing update.
   * @return Plane updated by increment delta.
   */
  Plane3d exmap_3dof(const Eigen::Vector3d& delta) const {
#if 1
#if 0
    // using isam::Rot3d.exmap
    Rot3d r(Eigen::Quaterniond(_abcd));
    Eigen::Quaterniond q = r.exmap(delta).quaternion();
#else
    Eigen::Quaterniond q = delta3_to_quat(delta) * Eigen::Quaterniond(_abcd);
#endif
#else
    double theta = delta.norm();
    Eigen::Quaterniond dquat = (theta<1e-10)?Eigen::Quaterniond():Eigen::Quaterniond(Eigen::AngleAxisd(theta, delta.normalized()));
    Eigen::Quaterniond q = dquat * Eigen::Quaterniond(_abcd);
#endif
    return Plane3d(q.coeffs());
  }

#ifdef WRONG
  Plane3d exmap(const Eigen::Vector4d& delta) const {
    return Plane3d(vector()+delta);
  }
#else
  Plane3d exmap(const Eigen::Vector3d& delta) const {
    return exmap_3dof(delta);
  }
#endif

  Eigen::Vector4d vector() const {
    return Eigen::Vector4d(_abcd);
  }

  void set(const Eigen::Vector4d& v) {
    _abcd = v;
    _normalize();
  }

  // normal of n*x=d parameterization
  Eigen::Vector3d normal() const {
    return _abcd.head(3).normalized();
  }

  // d of n*x=d parameterization
  double d() const {
    return - _abcd(3) / _abcd.head(3).norm();
  }

  // distance from origin
  double distance() const {
    return fabs(d());
  }

  // point on the plane that is closest to the origin
  Point3d point0() const {
    return Point3d(d() * normal());
  }

  // distance of a point from the plane
  double distance(Point3d point) const {
    return fabs(normal().dot(point.vector()-point0().vector()));
  }

  Plane3d transform_to(Pose3d pose) const {
    // for a plane: inverse T, transposed (assuming T transforms a point)
    return Plane3d(pose.wTo().transpose() * vector());
  }

  Plane3d transform_from(Pose3d pose) const {
    return Plane3d(pose.oTw().transpose() * vector());
  }

  void write(std::ostream &out) const {
    out << "(" << _abcd(0) << ", " << _abcd(1) << ", " << _abcd(2) << "; " << _abcd(3) << ")";
  }
};


// iSAM node for plane
class Plane3d_Node : public NodeT<Plane3d> {
private:
  Pose3d_Node* _base;

public:
  Plane3d_Node() : _base(NULL) {}
  Plane3d_Node(const char* name) : NodeT<Plane3d>(name), _base(NULL) {}

  ~Plane3d_Node() {}

  void set_base(Pose3d_Node* base) { _base = base; }
  Pose3d_Node* base() { return _base; }
};


#ifdef WRONG2
 const int MEASURE_SIZE = 4;
#else
 const int MEASURE_SIZE = 3;
#endif

#if 0 // not used
class Plane3d_Factor : public FactorT<Plane3d> {
  Plane3d_Node* _plane;
public:

  Plane3d_Factor(Plane3d_Node* plane, const Plane3d& prior, const Noise& noise)
    : FactorT<Plane3d>("Plane3d_Factor", 3, noise, prior), _plane(plane) {
    _nodes.resize(1);
    _nodes[0] = plane;
  }

  void initialize() {
    if (!_plane->initialized()) {
      Plane3d predict = _measure;
      _plane->init(predict);
    }
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    Eigen::VectorXd err = _nodes[0]->vector(s) - _measure.vector(); // todo
    return err;
  }
};
#endif

// The actual measurement of a plane from a camera/pose
class Pose3d_Plane3d_Factor : public FactorT<Plane3d> {
private:
  Pose3d_Node* _pose;
  Plane3d_Node* _plane;
  bool _relative;
  Pose3d_Node* _base;

public:
  Pose3d_Plane3d_Factor(Pose3d_Node* pose, Plane3d_Node* plane,
                        const Plane3d& measure, const Noise& noise,
                        bool relative = false)
    : FactorT<Plane3d>("Pose3d_Plane3d_Factor", MEASURE_SIZE, noise, measure), _pose(pose), _plane(plane), _relative(relative), _base(NULL) {
    _nodes.resize(2);
    _nodes[0] = pose;
    _nodes[1] = plane;
    // for relative parameterization recover base pose
    if (_relative) {
      if (!plane->factors().empty()) {
        _nodes.resize(3);
        _nodes[2] = plane->factors().front()->nodes()[0];
        // todo: first factor might refer to a prior or other type of node...
        _base = dynamic_cast<Pose3d_Node*>(_nodes[2]);
      } else {
        // first factor: base is this pose, also init base for plane
        _base = _pose;
      }
      plane->set_base(_base);
    }
  }

  void initialize() {
    require(_pose->initialized(), "Plane3d: Pose3d_Plane3d_Factor requires pose to be initialized");
    if (!_plane->initialized()) {
      const isam::Pose3d& p = _pose->value();
      Plane3d predict;
      if (_relative) {
        predict = _measure;
      } else {
        predict = _measure.transform_from(p);
      }
      _plane->init(predict);
    }
  }

  /**
   * Error between the predicted and measured planes (orientation and distance)
   * @param s Determines if error based on estimate or linearization point.
   * @return Error vector of size 3.
   */
  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    // std::cout << "ratnesh :: basic error called \n";
    // calculate error in tangent space
    // todo: transform actual measure covariance into tangent space
    Pose3d po = _pose->value(s);
    const Plane3d& pl = _plane->value(s);
    if (_base) {
      // pose of camera relative to base camera (which might be this one!)
      po = po.ominus(_base->value(s));
    }
    Plane3d p = pl.transform_to(po);

#ifdef WRONG2
    Eigen::Vector4d error = p.vector() - _measure.vector();
#else
    // logmap
    // std::cout << "ratnesh :: logmap \n";
    Eigen::Quaterniond q(p.vector());
    Eigen::Quaterniond q_measured(_measure.vector());
    Eigen::Quaterniond dq = q * q_measured.conjugate();
#if 1
    // std::cout << "ratnesh :: AA error \n";
    Eigen::AngleAxisd aa(dq);
    if (aa.angle() > M_PI) aa.angle() -= 2.*M_PI;
    if (aa.angle() < -M_PI) aa.angle() += 2.*M_PI;
    if (aa.angle() > M_PI || aa.angle() < -M_PI) std::cout << "error in basic_error" << std::endl;
    Eigen::Vector3d error = aa.axis() * aa.angle();
#else
    // own logmap
    Eigen::Vector3d qv(dq.x(), dq.y(), dq.z());
    Eigen::Vector3d error = (qv.norm()<1e-8) ? Eigen::Vector3d() : 2.*acos(dq.w())*qv/qv.norm();
    if (dq.w() < -M_PI || dq.w() > M_PI) std::cout << "logmap issue " << error << std::endl;
#endif
#endif

    return error;
  }
};


#if 0 // not used (and probably not useful either because a plane graph is not fully constrained
// direct constraint between two planes (e.g. from marginalization of poses)
class Plane3d_Plane3d_Factor : public FactorT<Plane3d> {
private:
  Plane3d_Node* _plane1;
  Plane3d_Node* _plane2;

public:
  Plane3d_Plane3d_Factor(Plane3d_Node* plane1, Plane3d_Node* plane2,
                        const Plane3d& measure, const Noise& noise)
    : FactorT<Plane3d>("Plane3d_Plane3d_Factor", MEASURE_SIZE, noise, measure), _plane1(plane1), _plane2(plane2) {
    _nodes.resize(2);
    _nodes[0] = plane1;
    _nodes[1] = plane2;
  }

  void initialize() {
    require(_plane1->initialized(), "Plane3d: Plane3d_Plane3d_Factor requires first plane to be initialized");
    if (!_plane2->initialized()) {
      // todo...
      std::cout << "error in plane3d_plane3d" << std::endl;
      exit(1);
    }
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    Eigen::VectorXd error;

    // todo...
    std::cout << "error in plane3d_plane3d" << std::endl;
    exit(1);

    return error;
  }
};
#endif


}
