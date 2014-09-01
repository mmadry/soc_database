// Author: Marianna Madry (madry@csc.kth.se)
// Date: July 2013

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "SOC2PCLReader.h"

void usage(const char* exec_name) {
   cerr << "----------------------------------------\n";
   cerr << "Loads a XYZRGB pointcloud and correspoding xy image coordinates (left stereo image) from the CRD format to the PCL, and visualizes the loaded pointcloud. The CRD format is described in the README file.\n";
   cerr << "Usage: " << exec_name << " filePath \n";
   cerr << "<filePath>: path to a CRD pointcloud file \n";
   cerr << "\n";
   cerr << "Example: ./read_crd ../testdata/mammal01_filterSegment_000.crd \n";
   cerr << "----------------------------------------\n";
}

int main (int argc, char** argv)
{
  if (argc != 2)
  {
    usage(argv[0]);
    exit(0);
  }

  string input_file(argv[1]);

  //Load
  SOC2PCLReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXY>::Ptr imXY (new pcl::PointCloud<pcl::PointXY>());
  reader.readCrd(input_file,cloud,imXY);
  
  //Visualize
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "loaded pointcloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "loaded pointcloud");
  viewer->spin();

  return (0);
}
