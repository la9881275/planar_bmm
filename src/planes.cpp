/*
 * planes.cpp
 *
 *  Created on: Feb 12, 2014
 *      Author: kaess
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/common/time.h>

#include <thread>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

#include "Visualizer.h"
#include "Plane3d.h"
#include "planes.h"

using namespace std;

struct voxel_t {
  vector<int> pixels;
  int cluster_id;
};

// cube of 100 voxels side length yields resolution of approximately 1.1 degrees
const int nVoxelsCluster = 20; //15;

// minimum number of normals in a voxel to be used
const int minNormals = 1500; //1000;

// for filtering points that are too far from a plane
const double maxDistance = 0.03; //0.02;  // todo: should be dependent on distance?

// reject clusters with less points
const int minClusterSize = 2000;

// for depth clustering, minimum number to accept as plane
const int minNumDepthCluster = 1000; //200;

double tictoc_t0;

void _tic() {
  tictoc_t0 = pcl::getTime();
}

void _toc(const string& s) {
  cout << "Time for " << s << ": " << pcl::getTime() - tictoc_t0 << endl;
}

class Cluster {
  int _nSide;
  int _nVoxels;
  int _count;
  vector<voxel_t> _voxels;
  Eigen::MatrixXf _avgNormals;
  vector<list<int> > _clusters;

  // map [-1,1] to index 0..n-1
  inline int discretize(float f) {
    int i = floor((f + 1) * 0.5 * _nSide);
    if (i == _nSide)
      return i - 1;
    else
      return i;
  }

  // (x,y,z) to linear index
  inline int index(int x, int y, int z) {
    return (x * _nSide + y) * _nSide + z;
  }

  // add next index into voxel determined by (x,y,z)
  void add(const Eigen::Vector3f& normal) {
    if (!isnan(normal(0))) {
      Eigen::Vector3f n = normal;
      n.normalize();
      int idx = index(discretize(n(0)), discretize(n(1)), discretize(n(2)));
      _voxels[idx].pixels.push_back(_count);
    }
  }

public:

  // use voxel grid with side length n_side for clustering
  Cluster(int n_side) :
      _nSide(n_side), _nVoxels(n_side * n_side * n_side), _count(0) {
    voxel_t empty;
    empty.pixels = vector<int>();
    empty.cluster_id = -1;
    _voxels = vector<voxel_t>(_nVoxels, empty);
    _avgNormals = Eigen::MatrixXf(_nVoxels, 3);
  }

  // retrieve clusters from voxel grid
  clusters_t getClusters() {
    clusters_t clusters;
    for (auto cluster : _clusters) {
      if (cluster.size() != 0) { // skip empty clusters that were created by merging clusters
        vector<int> pixels;
        for (int idx : cluster) {
          pixels.insert(pixels.end(), _voxels[idx].pixels.begin(),
              _voxels[idx].pixels.end());
        }
        clusters.push_back(pixels);
      }
    }
    return clusters;
  }

  Eigen::MatrixXf addNormalsToVoxelGrid(const PCNormal::ConstPtr& pcl_normals) {
    int i = 0;
    Eigen::MatrixXf normals(pcl_normals->height * pcl_normals->width, 3);
    int dis = 0;
    for (auto pcl_normal : *pcl_normals) {
      if (!isnan(pcl_normal.normal_x)) {
        Eigen::Vector3f normal(pcl_normal.normal_x, pcl_normal.normal_y,
            pcl_normal.normal_z);
//        cout << pcl_normal.normal_x << " " << pcl_normal.normal_y << " " << pcl_normal.normal_z << endl;
        normal.normalize();
        normals.row(i) = normal;
        add(normal);
        i++;
      } else {
        dis++;
      }
      _count++;
    }
    cout << "discarded " << dis << endl;
    return normals;
  }

  void mergeClusters(int cA, int cB) {
    for (auto idx : _clusters[cB]) {
      _voxels[idx].cluster_id = cA;
    }
    // move list of clusters from B to the end of A
    _clusters[cA].splice(_clusters[cA].end(), _clusters[cB]);
  }

  void findClusters() {
    // combine neighboring voxels into clusters,
    // consider 3x3x3 cube, but only the voxels that have higher index
    // very efficient O(n) algorithm, except that in some cases a merging
    // of clusters (and renumbering of cluster IDs) is needed
    vector<tuple<int, int, int> > neighbors;
    neighbors.push_back(make_tuple(0, 0, 1));
    neighbors.push_back(make_tuple(0, 1, -1));
    neighbors.push_back(make_tuple(0, 1, 0));
    neighbors.push_back(make_tuple(0, 1, 1));
    neighbors.push_back(make_tuple(1, -1, -1));
    neighbors.push_back(make_tuple(1, -1, 0));
    neighbors.push_back(make_tuple(1, -1, 1));
    neighbors.push_back(make_tuple(1, 0, -1));
    neighbors.push_back(make_tuple(1, 0, 0));
    neighbors.push_back(make_tuple(1, 0, 1));
    neighbors.push_back(make_tuple(1, 1, -1));
    neighbors.push_back(make_tuple(1, 1, 0));
    neighbors.push_back(make_tuple(1, 1, 1));
    int idx = 0;
    // traverse cube in increasing order
    for (int x = 0; x < _nSide; x++) {
      for (int y = 0; y < _nSide; y++) {
        for (int z = 0; z < _nSide; z++) {
          // only process voxels with many normals
          if (_voxels[idx].pixels.size() > minNormals) {
            int cid = _voxels[idx].cluster_id;
            // voxel not yet assigned: start new cluster
            if (cid < 0) {
              cid = _clusters.size();
              _voxels[idx].cluster_id = cid;
              _clusters.push_back(list<int>());
              _clusters[cid].push_back(idx);
            }
            // check forward neighbors
            for (auto triple : neighbors) {
              int dx, dy, dz;
              tie(dx, dy, dz) = triple;
              // inside cube?
              if (x + dx < _nSide && y + dy < _nSide && z + dz < _nSide
                  && x + dx >= 0 && y + dy >= 0 && z + dz >= 0) {
                int idx2 = index(x + dx, y + dy, z + dz);
                // sufficient number of normals?
                if (_voxels[idx2].pixels.size() > minNormals) {
                  int cid2 = _voxels[idx2].cluster_id;
                  if (cid2 < 0) {
                    // add to cluster
                    _clusters[cid].push_back(idx2);
                    _voxels[idx2].cluster_id = cid;
                  } else if (cid != cid2) {
                    // merge clusters (rare case)
                    mergeClusters(cid, cid2);
                  }
                }
              }
            }
          }
          idx++;
        }
      }
    }
  }

  void visualizeVoxels() {
    pcl::visualization::PCLVisualizer viewer("voxels");
    viewer.setBackgroundColor(1, 1, 1);
    Eigen::Quaternionf id;
    int c = 0;
    for (int x = 0; x < _nSide; x++) {
      for (int y = 0; y < _nSide; y++) {
        for (int z = 0; z < _nSide; z++) {
          int n = _voxels[c].pixels.size();
          if (n > 10) {
            double d = min(1.0, ((double) n / 5000.));
            double r, g, b;
            tie(r, g, b) = getColor01(_voxels[c].cluster_id);
            viewer.addCube(x - d / 2, x + d / 2, y - d / 2, y + d / 2,
                z - d / 2, z + d / 2, r, g, b, to_string(c));
          }
          c++;
        }
      }
    }
    while (!viewer.wasStopped()) {
      viewer.spinOnce();
      this_thread::sleep_for(chrono::milliseconds(30));
    }
  }

  clusters_t process(const PCNormal::ConstPtr& pcl_normals) {
    // add each normal to the voxel grid
    Eigen::MatrixXf normals = addNormalsToVoxelGrid(pcl_normals);

#if 0
    // todo: might not be needed
    // calculate average normal for each cluster
    for (int i = 0; i < _nVoxels; i++) {
      if (_voxels[i].pixels.size() > 0) {
        Eigen::Vector3f avg;
        for (auto idx : _voxels[i].pixels) {
          avg += normals.row(idx);
        }
        avg.normalize();
        _avgNormals.row(i) = avg;
      }
    }
#endif

    findClusters();

//    visualizeVoxels();   // have to disable visualizer thread

    clusters_t res = getClusters();

    return res;
  }

};

PCNormal::Ptr computeNormals(const PCPoint::ConstPtr& cloud) {
  PCNormal::Ptr normalsOut(new PCNormal);

  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> norm_est;
  // Specify method for normal estimation
  //norm_est.setNormalEstimationMethod(norm_est.AVERAGE_3D_GRADIENT);
  norm_est.setNormalEstimationMethod(norm_est.COVARIANCE_MATRIX);
  // Specify max depth change factor (0.02f)
  norm_est.setMaxDepthChangeFactor(0.1f); //0.01f);
  // Specify smoothing area size (10.0f);
  norm_est.setNormalSmoothingSize(20.0f); //20.0f);

  norm_est.setDepthDependentSmoothing(true);
  norm_est.useSensorOriginAsViewPoint(); // just in case
  // Set the input points
  norm_est.setInputCloud(cloud);
  // Estimate the surface normals and
  // store the result in "normals_out"
  norm_est.compute(*normalsOut);

#if 0
  // visualize normals
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor(0.0, 0.0, 0.5);
  int level = 10;//100
  double scale = 0.02;//0.02
  viewer.addPointCloud(cloud);
  viewer.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(cloud,
      normalsOut, level, scale, "normals");

  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }
#endif

  return normalsOut;
}

clusters_t clusterDepth(const clusters_t& clusters,
    const PCNormal::ConstPtr& normals,
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud) {
  clusters_t refinedClusters;
  for (auto cluster : clusters) {

    // calculate average normal
    Eigen::Vector3f avgNormal(0., 0., 0.);
    for (int pid : cluster) {
      const pcl::Normal& n = normals->at(pid);
      Eigen::Vector3f normal(n.normal_x, n.normal_y, n.normal_z);
      normal.normalize(); // todo - probably not needed
      avgNormal += normal;
    }
    avgNormal.normalize();

    // for each point in cluster, calculate distance
    vector<float> distances;
    for (int pid : cluster) {
      const pcl::PointXYZRGBA& p = cloud->at(pid);
      Eigen::Vector3f point(p.x, p.y, p.z);
      float distance = -point.dot(avgNormal);
      distances.push_back(distance);
    }

    // quantize distance
    const float min = 0.;
    const float max = 10.;
    const float size = 0.05;
    const int num = (max - min) / size + 1;
    vector<vector<int> > cells(num);
    for (int i = 0; i < distances.size(); i++) {
      int cell = floor((distances[i] - min) / size);
      if (cell >= 0 && cell < num) {
        cells[cell].push_back(cluster[i]);
      }
    }

    // identify and return clusters
    vector<int> accCluster;
    for (int cell = 0; cell < cells.size(); cell++) {
      if (cells[cell].size() >= minNumDepthCluster) {
        // add to cluster
        accCluster.insert(accCluster.end(), cells[cell].begin(),
            cells[cell].end());
      }
      if (accCluster.size() > 0
          && (cells[cell].size() < minNumDepthCluster || cell == cells.size() - 1)) {
        // cluster is complete (either end of a cluster, or last cell)
        refinedClusters.push_back(accCluster); // todo: could use swap for efficiency
        accCluster.clear();
      }
    }
  }
  return refinedClusters;
}

void visualizeSegmentation(const string& windowName, const clusters_t& clusters,
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud) {
  // visualization
  cv::Mat image(cloud->height, cloud->width, CV_8UC3);
  unsigned char* p = image.data;
  for (auto point : *cloud) {
    *p++ = point.r;
    *p++ = point.g;
    *p++ = point.b;
  }
  int i = 0;
  for (auto cluster : clusters) {
    cout << "Cluster of size: " << cluster.size() << endl;
    int r, g, b;
    tie(r, g, b) = getColor255(i);
    for (int point : cluster) {
      image.data[point * 3] = r;
      image.data[point * 3 + 1] = g;
      image.data[point * 3 + 2] = b;
    }
    i++;
  }

  imshow(windowName, image);
}

planes_t fitPlanes(const clusters_t& clusters, const PCPoint::ConstPtr& cloud) {
  planes_t planes;
  for (auto cluster : clusters) {
    int num = cluster.size();
    Eigen::MatrixXd points(3, num);
    for (int i = 0; i < num; i++) {
      const pcl::PointXYZRGBA& p = cloud->at(cluster[i]);
      points(0, i) = p.x;
      points(1, i) = p.y;
      points(2, i) = p.z;
    }
    // subtract centroid
    Eigen::Vector3d mean = points.rowwise().mean();
    Eigen::MatrixXd points0 = points - mean.replicate(1, num);
    // the normal is given by the singular vector corresponding to lowest
    // singular value, here in last/third column
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(points0,
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (svd.matrixU().cols() != 3) {
      cout << "fitPlanes: SVD input did not have full rank" << endl;
    }
    Eigen::Vector3d normal = svd.matrixU().col(2);
    double d = normal.dot(mean);
    if (d < 0) { // orient normals towards observer
      d = -d;
      normal = -normal;
    }
    planes.push_back(isam::Plane3d(normal, d));
  }
  return planes;
}

clusters_t filterClusters(const planes_t& planes, const clusters_t& clusters,
    const PCPoint::ConstPtr& cloud) {
  clusters_t clustersFiltered;
  for (int i = 0; i < planes.size(); i++) {
    const isam::Plane3d& plane = planes[i];
    vector<int> filtered;
    for (int idx : clusters[i]) {
      const pcl::PointXYZRGBA& p = cloud->at(idx);
      Eigen::Vector3d pt(p.x, p.y, p.z);
      if (plane.distance(pt) < maxDistance) {
        filtered.push_back(idx);
      }
    }
    clustersFiltered.push_back(filtered);
  }
  return clustersFiltered;
}

PCPoint::Ptr toPointCloud(const cv::Mat& depth, const cv::Mat& color,
    const CameraParameters camParams) {
  // convert to pointcloud
  PCPoint::Ptr cloud(new PCPoint());
  cloud->width = camParams.width;
  cloud->height = camParams.height;
  cloud->is_dense = false; // important!! (not sure why "dense" won't work...)
  cloud->points.resize(camParams.width * camParams.height);
  PCPoint::iterator it = cloud->begin();
  uint8_t* pc = color.data;
  uint16_t* pd = (uint16_t*) depth.data;
  float fx_inv = 1. / camParams.fx;
  float fy_inv = 1. / camParams.fy;
  pcl::PointXYZRGBA point;
  point.a = 0;
  double px, py, pz;
  for (int y = 0; y < camParams.height; y++) {
    for (int x = 0; x < camParams.width; x++) {
      pz = ((float) *pd) * 0.001;
      pd++;
      if (pz == 0) {
        px = numeric_limits<float>::quiet_NaN();
        py = numeric_limits<float>::quiet_NaN();
        pz = numeric_limits<float>::quiet_NaN();
      } else {
        px = (x - camParams.px) * pz * fx_inv;
        py = (y - camParams.py) * pz * fy_inv;
      }
      // camera to world frame
      point.x = pz;
      point.y = px;
      point.z = py;
      point.r = *pc;
      pc++;
      point.g = *pc;
      pc++;
      point.b = *pc;
      pc++;
      *it = point;
      it++;
    }
  }

  //    pcl::io::savePLYFileASCII("test2.ply", *cloud);

  return cloud;
}

tuple<planes_t, clusters_t, PCPoint::Ptr> extractPlanes(const cv::Mat& depth,
    const cv::Mat& color, const CameraParameters camParams) {

  _tic();

  // fix color channels
  const cv::Mat colorTmp(color.rows, color.cols, CV_8UC3);
  cvtColor(color, colorTmp, CV_RGB2BGR);

  PCPoint::Ptr cloud = toPointCloud(depth, colorTmp, camParams);
  _toc("toPointCloud");

  // following Dirk Holz 2011 for fast plane extraction

  // calculate per pixel normal
  _tic();
  PCNormal::Ptr normals = computeNormals(cloud);
  _toc("computeNormals");

  double t0 = pcl::getTime();  // plane fitting

  // cluster by normal direction
  _tic();
  Cluster cluster(nVoxelsCluster);
  clusters_t normalClusters = cluster.process(normals);
  _toc("cluster.process");

  // cluster each cluster by distance
  _tic();
  clusters_t planeClusters = clusterDepth(normalClusters, normals, cloud);
  _toc("clusterDepth");

  cout << "num segmented planes: " << planeClusters.size() << endl;

  // filter out unsuitable planes
  _tic();
  clusters_t filteredClusters;
  for (auto cluster : planeClusters) {
    if (cluster.size() > minClusterSize) {
      filteredClusters.push_back(cluster);
    }
  }
  _toc("filter");

  cout << "num filtered planes: " << filteredClusters.size() << endl;

  // fit planes to individual clusters, filter and refit
  _tic();
  planes_t planes = fitPlanes(filteredClusters, cloud);
  filteredClusters = filterClusters(planes, filteredClusters, cloud);
  clusters_t clusters;
  for (auto cluster : filteredClusters) {
    if (cluster.size() > minClusterSize) {
      clusters.push_back(cluster);
    }
  }
  planes = fitPlanes(clusters, cloud);
  _toc("fitPlanes");

  cout << "Time for plane extraction: " << pcl::getTime() - t0 << endl;

  cout << "num final planes: " << planes.size() << endl;

#if 0
  visualizeSegmentation("normalClusters", normalClusters, cloud);
  visualizeSegmentation("planeClusters", planeClusters, cloud);
  while (cv::waitKey(10) == -1) {
  }
  cv::waitKey(10);
#endif

  return make_tuple(planes, clusters, cloud);
}
