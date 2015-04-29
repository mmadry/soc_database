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
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
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

#include "SOC2PCLReader.h"

void SOC2PCLReader::__assert_exit_(const char *assertion, const char *file,
                                   unsigned int line, const char *function) {
  cerr << "ASSERT: " << file << ":" << line << " " << function;
  exit(1);
}

bool SOC2PCLReader::file_exists(string input_file) {
  path p = input_file;

  if (exists(p)) {
    if (is_regular_file(p)) return true;
  } else {
    cout << "File does not exist: " << input_file << endl;
    return false;
  }
}

bool SOC2PCLReader::file_ext_correct(string input_file, string fileFormat) {
  int tmp = input_file.length() - fileFormat.length();
  if (input_file.substr(tmp).compare(fileFormat) == 0)
    return true;
  else {
    cout << "Incorrect input file extension: " << input_file
         << ". The required file extension is '." << fileFormat << "'" << endl;
    return false;
  }
}

bool SOC2PCLReader::first_line_correct(string line, string fileFormat) {
  if (line.compare(fileFormat) == 0)
    return true;
  else {
    cout << "Incorrect file format (line 1)." << endl;
    return false;
  }
}

bool SOC2PCLReader::pc_size_correct(uint32_t size) {
  if (((size * size) >= numeric_limits<int>::max()) || ((size * size) < 0)) {
    cout << "Error: number of points^2 exceeds 'int' range." << endl;
    return false;
  } else
    return true;
}

void SOC2PCLReader::read_syncpc(string input_file,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  string fileFormat = "syncpc";
  string line;
  ifstream file;

  ASSERT(file_exists(input_file));
  ASSERT(file_ext_correct(input_file, fileFormat));

  file.open(input_file.c_str());
  if (file.is_open()) {
    // First line
    getline(file, line);

    ASSERT(first_line_correct(line, fileFormat));

    // Second line
    getline(file, line);
    stringstream s(line);
    uint32_t tmp;
    s >> tmp;

    ASSERT(pc_size_correct(tmp));

    cloud->width = tmp;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
      file >> cloud->points[i].x >> cloud->points[i].y >> cloud->points[i].z;
      // Check values
      // cout << cloud->points[i].x << " " << cloud->points[i].y << " " <<
      // cloud->points[i].z << endl;
    }

    file.close();
  } else {
    cout << "Error: Cannot open the input file: " << input_file << endl;
    exit(1);
  }
}

void SOC2PCLReader::read_crd(string input_file,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                             pcl::PointCloud<pcl::PointXY>::Ptr &imXY) {
  string fileFormat = "crd";
  string line;
  ifstream file;

  ASSERT(file_exists(input_file));
  ASSERT(file_ext_correct(input_file, fileFormat));

  file.open(input_file.c_str());
  if (file.is_open()) {
    // First line
    getline(file, line);
    stringstream s(line);
    uint32_t sizePointCloud;
    s >> sizePointCloud;

    cloud->width = sizePointCloud;
    cloud->height = 1;

    imXY->width = sizePointCloud;
    imXY->height = 1;
    imXY->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < sizePointCloud; ++i) {
      int R, G, B;
      uint8_t r, g, b;
      pcl::PointXYZRGB point;

      // Load line
      file >> point.x >> point.y >> point.z >> imXY->points[i].x >>
          imXY->points[i].y >> R >> G >> B;
      r = static_cast<uint8_t>(R);
      g = static_cast<uint8_t>(G);
      b = static_cast<uint8_t>(B);

      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                      static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float *>(&rgb);

      cloud->points.push_back(point);

      // Check values
      /* uint8_t rr = (rgb >> 16) & 0x0000ff;
      uint8_t gg = (rgb >> 8)  & 0x0000ff;
      uint8_t bb = (rgb)       & 0x0000ff;
      cout <<  cloud->points[i].x << " " << cloud->points[i].y << " " <<
      cloud->points[i].z << " " << imXY->points[i].x << " " << imXY->points[i].y
      << " " << static_cast<int>(rr) << " " << static_cast<int>(gg) << " " <<
      static_cast<int>(bb) << endl; */
    }
    ASSERT(cloud->points.size() == sizePointCloud);
    file.close();
  } else {
    cout << "Error: Cannot open the input file: " << input_file << endl;
    exit(1);
  }
}
