#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

int main(int argc, char const* argv[])
{

   if (argc != 2)
    {
        std::cout << "Usage : Extract filename.pcd" << std::endl;
        return 1;
    }
  pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_xyzrgb_all(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Fill in the cloud data
  pcl::PCDReader reader;

   if (reader.read(argv[1], *cloud_blob) == -1) {
    PCL_ERROR ("Couldn't read file \n");
    return 1;
  }

  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud_blob);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>("downsampled.pcd", *cloud_filtered, false);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int)cloud_filtered->size();
  // While 30% of the original cloud is still there
  while (cloud_filtered->size() > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "plane_" << i << ".pcd";

    pcl::copyPointCloud(*cloud_p, *cloud_xyzrgb);

    int32_t rgb = (static_cast<uint32_t>(i *10) << 16 |
                   static_cast<uint32_t>(i *30) << 8 | static_cast<uint32_t>(i*40));
    for (auto &p : cloud_xyzrgb->points)
      p.rgb = rgb;

    *cloud_xyzrgb_all = (*cloud_xyzrgb_all) + (*cloud_xyzrgb);

    writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);

    // Create the filtering object
    extract.setNegative(true);
    extract.filter(*cloud_f);
    cloud_filtered.swap(cloud_f);
    i++;
  }

  writer.write<pcl::PointXYZRGB>("final.pcd", *cloud_xyzrgb_all, false);

  return (0);
}