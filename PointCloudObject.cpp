/**
*
*
**/

#include<iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "PointCloudObject.h"

/**
* PointCloudObject(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
*
* Constructor which creates an object from the provided point cloud.
**/
PointCloudObject::PointCloudObject(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
   this->cloud = point_cloud;
}


/**
* PointCloudObject(const char* filename)
*
* Creates a new object and loads the point clouds from a file.  Assumes the file
* is a PCD file.
**/

PointCloudObject::PointCloudObject(const char* filename)
{
   this->cloud = PointCloudObject::load_PCD(filename);
}


/**
* PointCloudObject(const char* filename, PCL_Filetype filetype)
*
* Creates a new object and loads the point clouds from a file.  
**/

PointCloudObject::PointCloudObject(const char* filename, PCL_Filetype filetype)
{

   switch(filetype)
   {
      case PCD:
         this->cloud = PointCloudObject::load_PCD(filename);
         break;
      case PLY:
         this->cloud = PointCloudObject::load_PLY(filename);
         break;
   }
}


/**
* pcl::PointCloud<pcl::PointXYZ>::Ptr load_PCD(const char* filename)
*
* Load a cloud from a PCD file.
**/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudObject::load_PCD(const char* filename)
{
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::io::loadPCDFile(filename, *cloud);

   return cloud;
}


/**
* pcl::PointCloud<pcl::PointXYZ>::Ptr load_PLY(const char* filename)
*
* Load a cloud from a PLY file.
**/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudObject::load_PLY(const char* filename)
{
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::io::loadPLYFile(filename, *cloud);

   return cloud;
}


/**
* void show()
*
* Display the point cloud of the object.
**/
void PointCloudObject::show()
{
   pcl::visualization::CloudViewer* viewer = new pcl::visualization::CloudViewer("Object Point Cloud");
   viewer->showCloud(this->cloud);
   while(!viewer->wasStopped()) { }
   delete viewer;
}


/**
* pcl::PointCloud<pcl::PointXYZ>::Ptr extractPartPointCloud()
*
* Pull out a part from the remaining object
**/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudObject::extractPartPointCloud(double distanceThreshold)
{
   pcl::PointCloud<pcl::PointXYZ>::Ptr partCloud(new pcl::PointCloud<pcl::PointXYZ>);

   // The model coefficients and inliers the segmentation object will write to
   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
   pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

   // Set up the SAC Segmentation object
   pcl::SACSegmentation<pcl::PointXYZ> seg;

   seg.setOptimizeCoefficients(true);
   seg.setModelType(pcl::SACMODEL_PLANE);
   seg.setMethodType(pcl::SAC_RANSAC);
   seg.setDistanceThreshold(distanceThreshold);
   seg.setInputCloud(this->cloud);
   seg.segment(*inliers, *coefficients);

   // Put the inlier points into the partCloud
   

   return partCloud;
}


/**
* std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr getPart()
*
* Give the vector of parts extracted so far.
**/
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PointCloudObject::getParts()
{
   return this->parts;
}
