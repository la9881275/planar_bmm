/*
 * test.cpp
 *
 *  Created on: Apr 27, 2014
 *      Author: kaess
 */

#define BOOST_TEST_MODULE Plane3dTests
#include <boost/test/unit_test.hpp>

#include <iostream>

#include "Plane3d.h"

using namespace std;
using namespace isam;
using namespace Eigen;

Plane3d plane(Eigen::Vector3d(1.,0.,0.), 2.);

BOOST_AUTO_TEST_CASE(Distance) {
  BOOST_CHECK_EQUAL(plane.distance(), 2.);
  BOOST_CHECK_EQUAL(plane.distance(Point3d(2.,0.,0.)), 0.);
}
