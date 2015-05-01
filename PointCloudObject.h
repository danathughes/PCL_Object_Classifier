/**
*
*
**/

#include <iostream>
#include <vector>
#include <pcl/point_types.h>

#ifndef __POINTCLOUDOBJECT_H__
#define __POINTCLOUDOBJECT_H__

enum PCL_Filetype {PCD, PLY};

class PointCloudObject
{
   private:
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> parts;      

      static pcl::PointCloud<pcl::PointXYZ>::Ptr load_PCD(const char*);
      static pcl::PointCloud<pcl::PointXYZ>::Ptr load_PLY(const char*);

      pcl::PointCloud<pcl::PointXYZ>::Ptr extractPartPointCloud(double);

   public:
      PointCloudObject(pcl::PointCloud<pcl::PointXYZ>::Ptr);
      PointCloudObject(const char*);
      PointCloudObject(const char*, PCL_Filetype);
      void show();
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getParts();
};

#endif
