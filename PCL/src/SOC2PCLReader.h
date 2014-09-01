// Author: Marianna Madry (madry@csc.kth.se)
// Date: July 2013

// SOC2PCLReaders loads the RGB-D Stereo Object Category (SOC) Database files in the SYNCPC and CRD format to the PCL. The SYNCPC and CRD formats are described in the README file.
// Please find attached two example apps that:
// - 'read_syncpc' that loads a XYZ pointcloud in the SYNCPC format to the PCL and visualizes the loaded pointcloud
// - 'read_crd' that loads a XYZRGB pointcloud and correspoding xy image coordinates (left stereo image) from the CRD format to the PCL, and visualizes the loaded pointcloud.

#ifndef _SOC2PCLREADER_H_
#define _SOC2PCLREADER_H_

#define ASSERT(x) if (!(x)) __assert_exit_(__STRING(expr), __FILE__, __LINE__, __PRETTY_FUNCTION__)

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem/operations.hpp> 

using namespace pcl;
using namespace std;
using namespace boost::filesystem;

class SOC2PCLReader{
    private:
      void __assert_exit_(const char *assertion, const char *file, unsigned int line, const char *function);
      bool fileExists(string input_file_txt);
      bool fileExtensionCorrect(string input_file_txt, string fileFormat);
      bool fileFormatFirstLineCorrect(string line, string fileFormat);
      bool pointCloudSizeCorrect(uint32_t size);
      void ERRORTEXT_fileNotOpen(string input_file_txt);
      
    public: 
      
      void readSyncpc(string input_file_txt, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
      void readCrd(string input_file_txt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXY>::Ptr &imXY); 

};

#endif