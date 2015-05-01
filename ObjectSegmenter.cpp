/**
*
*/

#include<iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "PointCloudObject.h"


int main(int argc, char** argv)
{
   std::cout << "Loading table_3_5.ply...";
   PointCloudObject* pco = new PointCloudObject("table_3_5.ply", PLY);
   std::cout << "Done!" << std::endl;

   std::cout << "Displaying the point cloud...";
   pco->show();
   std::cout << "Done!" << std::endl;

   std::cout << "Displaying the point cloud again...";
   pco->show();
   std::cout << "Done!" << std::endl;


   return 0;
}
