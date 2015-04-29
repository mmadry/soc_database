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

// SOC2PCLReaders loads the RGB-D Stereo Object Category (SOC) Database files in
// the SYNCPC and CRD format to the PCL. The SYNCPC and CRD formats are
// described in the README file.

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

class SOC2PCLReader {
 public:
  void read_syncpc(string input_file,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
  void read_crd(string input_file,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                pcl::PointCloud<pcl::PointXY>::Ptr &imXY);

 private:
  void __assert_exit_(const char *assertion, const char *file,
                      unsigned int line, const char *function);
  bool file_exists(string input_file);
  bool file_ext_correct(string input_file, string fileFormat);
  bool first_line_correct(string line, string fileFormat);
  bool pc_size_correct(uint32_t size);
};

#endif