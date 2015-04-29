/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Marianna Madry
*  All rights reserved.
*
*  Contact: marianna.madry@gmail.com
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * The name of contributors may not be used to endorse or promote products 
*     derived from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "SOC2PCLReader.h"

void usage(const char* exec_name) {
  cerr << "----------------------------------------\n";
  cerr << "Loads a XYZ pointcloud in the SYNCPC format to the PCL and "
          "visualizes it. The SYNCPC format is described in the README file.\n";
  cerr << "Usage: " << exec_name << " <file> \n";
  cerr << "<file>: input SYNCPC pointcloud file \n";
  cerr << "\n";
  cerr << "Example: ./read_syncpc "
          "../testdata/mammal01_filterSegment_000.syncpc \n";
  cerr << "----------------------------------------\n";
}

int main(int argc, char** argv) {
  if (argc != 2) {
    usage(argv[0]);
    exit(1);
  }

  string input_file(argv[1]);

  // Load
  SOC2PCLReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  reader.read_syncpc(input_file, cloud);

  // Visualize
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "loaded pointcloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "loaded pointcloud");
  viewer->spin();

  return (0);
}
