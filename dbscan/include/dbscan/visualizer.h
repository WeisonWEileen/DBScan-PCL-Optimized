#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <random>

#include "dbscan/cluster.h"

void addAxis(pcl::visualization::PCLVisualizer::Ptr &viewer);
void display_clusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::vector<dbScanSpace::cluster> &clusters);
// void display_clusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud2);
void display_clusters(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &pVoxelPcVec, std::vector<int> &labels);
// Colors to display the generated clusters
const float colors[] = {
    255, 0,
    0,  // red 		1
    0,   255,
    0,  // green		2
    0,   0,
    255,  // blue		3
    255, 255,
    0,  // yellow		4
    0,   255,
    255,  // light blue	5
    255, 0,
    255,  // magenta     6
    255, 255,
    255,  // white		7
    255, 128,
    0,  // orange		8
    255, 153,
    255,  // pink		9
    51,  153,
    255,  //			10
    153, 102,
    51,  //			11
    128, 51,
    153,  //			12
    153, 153,
    51,  //			13
    163, 38,
    51,  //			14
    204, 153,
    102,  //		15
    204, 224,
    255,  //		16
    128, 179,
    255,  //		17
    206, 255,
    0,  //			18
    255, 204,
    204,  //			19
    204, 255,
    153,  //			20

};  // 20x3=60 color elements

constexpr int colors_size = sizeof(colors) / sizeof(colors[0]) / 3;

int getRandomColorIndex() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, colors_size - 1);
  return dis(gen);
}