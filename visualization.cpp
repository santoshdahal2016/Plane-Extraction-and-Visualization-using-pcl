
#include <iostream>
#include <thread>

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace std::literals::chrono_literals;



int main(int argc, char const *argv[])
{

    if (argc != 3)
    {
        cout << "Usage : visualization plane.pcd original.pcd" << endl;
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *original_cloud) == -1)
    {
        PCL_ERROR("Couldn't read second file \n");
        return 1;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read first file \n");
        return 1;
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    viewer->initCameraParameters();

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v1);
    viewer->addText("Plane Extracted", 10, 10, "v1 text", v1);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "Plane Extracted", v1);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
    viewer->addText("Original", 10, 10, "v2 text", v2);

    viewer->addPointCloud<pcl::PointXYZ>(original_cloud, "Original", v2);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
    return 0;
}
