#include "dbscan/visualizer.h"

#include <cstdint>
#include <random>

#include "pcl/impl/point_types.hpp"
#include "pcl/point_cloud.h"

// for dis dbscan
void display_clusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::vector<dbScanSpace::cluster> &clusters) {
  // Disable vtk render warning
  vtkObject::GlobalWarningDisplayOff();

  // pcl visualizer object
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCL VISUALIZER"));

  int PORT1 = 0;
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, PORT1);
  viewer->setBackgroundColor(0, 0, 0, PORT1);
  viewer->addText("ORIGINAL", 10, 10, "PORT1", PORT1);

  int PORT2 = 0;
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, PORT2);
  viewer->setBackgroundColor(0, 0, 0, PORT2);
  viewer->addText("CLUSTERS", 10, 10, "PORT2", PORT2);

  viewer->setPosition(0, 0);
  viewer->setBackgroundColor(0.0, 0.0, 0.0, 0.0);  // Setting background to a dark

  int numClust = 0;
  std::random_device seeder;
  std::ranlux48 gen(seeder());
  std::uniform_int_distribution<int> uniform_0_255(0, 255);

  int j = 0;
  for (auto &cluster : clusters) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());

    uint8_t r{0};
    uint8_t g{0};
    uint8_t b{0};

    if (j < 60) {
      r = static_cast<uint8_t>(colors[j]);
      g = static_cast<uint8_t>(colors[j + 1]);
      b = static_cast<uint8_t>(colors[j + 2]);

    } else {
      r = static_cast<uint8_t>(uniform_0_255(gen));
      g = static_cast<uint8_t>(uniform_0_255(gen));
      b = static_cast<uint8_t>(uniform_0_255(gen));
    }

    // Adding different color to each cluster
    for (auto &pointCluster : cluster.clusterPoints) {
      pcl::PointXYZRGB point;
      point.x = pointCluster.x;
      point.y = pointCluster.y;
      point.z = pointCluster.z;

      uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float *>(&rgb);

      cluster_rgb->points.push_back(point);
    }

    j += 3;

    std::string nameId = "cluster_";
    nameId += std::to_string(numClust);

    viewer->addPointCloud(cluster_rgb, nameId.c_str(), PORT2);
    numClust += 1;
  }

  double scale = 1;

  viewer->addCoordinateSystem(scale);
  pcl::PointXYZ p11, p22, p33;
  p11.getArray3fMap() << 1, 0, 0;
  p22.getArray3fMap() << 0, 1, 0;
  p33.getArray3fMap() << 0, 0.1, 1;

  viewer->addText3D("x", p11, 0.2, 1, 0, 0, "x_");
  viewer->addText3D("y", p22, 0.2, 0, 1, 0, "y_");
  viewer->addText3D("z", p33, 0.2, 0, 0, 1, "z_");

  if (cloud->points[0].r <= 0 and cloud->points[0].g <= 0 and cloud->points[0].b <= 0) {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(cloud, 255, 255, 0);
    viewer->addPointCloud(cloud, color_handler, "Original_Cloud", PORT1);
  } else {
    viewer->addPointCloud(cloud, "Original_Cloud", PORT1);
  }

  viewer->initCameraParameters();
  viewer->resetCamera();

  pcl::console::print_info("\npress [q] to exit!\n");

  while (!viewer->wasStopped()) {
    viewer->spin();
  }
}

// for color dbscan clustering showing
void display_clusters(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &pVoxelPcVec, std::vector<int> &labels) {
  // Disable vtk render warning
  vtkObject::GlobalWarningDisplayOff();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusterd(new pcl::PointCloud<pcl::PointXYZRGB>);

  // pcl visualizer object
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCL VISUALIZER"));

  int PORT1 = 0;
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, PORT1);
  viewer->setBackgroundColor(0, 0, 0, PORT1);
  viewer->addText("ORIGINAL", 10, 10, "PORT1", PORT1);

  int PORT2 = 0;
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, PORT2);
  viewer->setBackgroundColor(0, 0, 0, PORT2);
  viewer->addText("CLUSTERS", 10, 10, "PORT2", PORT2);

  viewer->setPosition(0, 0);
  viewer->setBackgroundColor(0.0, 0.0, 0.0, 0.0);  // Setting background to a dark

  std::random_device seeder;
  std::ranlux48 gen(seeder());
  std::uniform_int_distribution<int> uniform_0_255(0, 255);

  //   add color for the cluster_label based on labels
  int last_label = labels[0];
  int current_color[3] = {255, 0, 0};

  for (int i = 0; i < labels.size(); i++) {
    *cloud += *pVoxelPcVec[i];
    if (labels[i] != last_label) {
      last_label = labels[i];
      int color_idx = getRandomColorIndex();
      std::copy(colors + 3 * color_idx, colors + 3 * color_idx + 3, current_color);

      uint32_t rgb = (static_cast<uint32_t>(current_color[0]) << 16 | static_cast<uint32_t>(current_color[1]) << 8 | static_cast<uint32_t>(current_color[2]));

      for (auto &point : pVoxelPcVec[i]->points) {
        point.rgb = *reinterpret_cast<float *>(&rgb);
      }
      *cloud_clusterd += *pVoxelPcVec[i];
    } else {
      uint32_t rgb = (static_cast<uint32_t>(current_color[0]) << 16 | static_cast<uint32_t>(current_color[1]) << 8 | static_cast<uint32_t>(current_color[2]));

      for (auto &point : pVoxelPcVec[i]->points) {
        point.rgb = *reinterpret_cast<float *>(&rgb);
      }
      *cloud_clusterd += *pVoxelPcVec[i];
    }
  }
  viewer->addPointCloud(cloud_clusterd, "clustered", PORT2);

  viewer->addCoordinateSystem(1);

  if (cloud->points[0].r <= 0 and cloud->points[0].g <= 0 and cloud->points[0].b <= 0) {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(cloud, 255, 255, 0);
    viewer->addPointCloud(cloud, color_handler, "Original_Cloud", PORT1);
  } else {
    viewer->addPointCloud(cloud, "Original_Cloud", PORT1);
  }

  viewer->initCameraParameters();
  viewer->resetCamera();
  pcl::console::print_info("\npress [q] to exit!\n");

  while (!viewer->wasStopped()) {
    viewer->spin();
  }
}