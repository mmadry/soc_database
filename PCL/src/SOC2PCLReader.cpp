// Author: Marianna Madry (madry@csc.kth.se)
// Date: July 2013

#include "SOC2PCLReader.h"

void SOC2PCLReader::__assert_exit_(const char *assertion, const char *file, unsigned int line, const char *function){
  cerr << "ASSERT: " << file << ":" << line << " " << function; 
  exit(-1);
}

bool SOC2PCLReader::fileExists(string input_file_txt){
  
  path p = input_file_txt; 
  
  if (exists(p))  
  {
    if (is_regular_file(p))   
      return true;
  }
  else
  {
    cout << "File does not exist: " << input_file_txt << endl;
    return false;
  }
}

bool SOC2PCLReader::fileExtensionCorrect(string input_file_txt, string fileFormat){
  
  int tmp = input_file_txt.length()-fileFormat.length();
  if (input_file_txt.substr(tmp).compare(fileFormat) == 0) 
    return true;
  else
  {
   cout << "Incorrect input file extension: " << input_file_txt << ". The required file extension is '." << fileFormat << "'" << endl;
   return false;
  }
}

bool SOC2PCLReader::fileFormatFirstLineCorrect(string line, string fileFormat){
  
  if(line.compare(fileFormat) == 0)
    return true;
  else
  {
    cout << "Incorrect file format (line 1)." << endl;
    return false;
  }
}

bool SOC2PCLReader::pointCloudSizeCorrect(uint32_t size){
  
  if ( ((size*size) >= numeric_limits<int>::max()) || ((size*size) < 0) )
  {
    cout << "Error: number of points^2 exceeds 'int' range." << endl;
    return false;
  }
  else
    return true;
}

void SOC2PCLReader::ERRORTEXT_fileNotOpen(string input_file_txt){
           cout << "Error: can not open the input file: " << input_file_txt << endl;
           exit(0); 
}


void SOC2PCLReader::readSyncpc(string input_file_txt, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
  
  string fileFormat="syncpc";
  string line;
  ifstream input_file;

  ASSERT(fileExists(input_file_txt));
  ASSERT(fileExtensionCorrect(input_file_txt,fileFormat));
   
  input_file.open(input_file_txt.c_str());
  if (input_file.is_open())
  {
    // First line
    getline(input_file, line);
	    
    ASSERT(fileFormatFirstLineCorrect(line,fileFormat));
	
    // Second line
    getline(input_file, line);
    stringstream s(line);	
    uint32_t tmp;
    s >> tmp;
	
    ASSERT(pointCloudSizeCorrect(tmp));
	
    cloud->width  = tmp; //uint32_t
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);
	    
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      input_file >> cloud->points[i].x >> cloud->points[i].y >> cloud->points[i].z;
//       // Check values
//       cout << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
    }
	
    input_file.close();
  }
  else
    ERRORTEXT_fileNotOpen(input_file_txt);
}

void SOC2PCLReader::readCrd(string input_file_txt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXY>::Ptr &imXY){
  
  string fileFormat="crd";
  string line;
  ifstream input_file;

  ASSERT(fileExists(input_file_txt));
  ASSERT(fileExtensionCorrect(input_file_txt,fileFormat));
  
  input_file.open(input_file_txt.c_str());
  if (input_file.is_open())
  {
	
    // First line
    getline(input_file, line);
    stringstream s(line);	
    uint32_t sizePointCloud;
    s >> sizePointCloud;
	
    cloud->width  = sizePointCloud; 
    cloud->height = 1;
    
    imXY->width  = sizePointCloud; 
    imXY->height = 1;
    imXY->points.resize (cloud->width * cloud->height);

    for (size_t i = 0; i < sizePointCloud; ++i)
    {
      int R, G, B;
      uint8_t r, g, b;
      pcl::PointXYZRGB point;
	
      //Load line
      input_file >> point.x >> point.y >> point.z >> imXY->points[i].x >> imXY->points[i].y >> R >> G >> B;
      r = static_cast<uint8_t>(R); 
      g = static_cast<uint8_t>(G); 
      b = static_cast<uint8_t>(B); 
     
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      
      cloud->points.push_back (point);
            
//       //Check values
//       uint8_t rr = (rgb >> 16) & 0x0000ff;
//       uint8_t gg = (rgb >> 8)  & 0x0000ff;
//       uint8_t bb = (rgb)       & 0x0000ff;
//       cout <<  cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " " << imXY->points[i].x << " " << imXY->points[i].y << " " << static_cast<int>(rr) << " " << static_cast<int>(gg) << " " << static_cast<int>(bb) << endl;
           
    }
    ASSERT(cloud->points.size ()== sizePointCloud);
    input_file.close();
  }
  else
    ERRORTEXT_fileNotOpen(input_file_txt);

}

