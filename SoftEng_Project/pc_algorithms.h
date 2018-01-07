#ifndef PC_ALGORITHMS_H
#define PC_ALGORITHMS_H

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
//#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <cstddef>
#include <cstdint>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/mls.h>

#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>



#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kinect_rgbd_grabber.h"

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>



#include <string>
class PC_Algorithms
{
public:
    PC_Algorithms();
    void keypoints_to_pc(const cv::Mat &depth_query, const cv::Mat &depth_train,std::vector< cv::DMatch > *matches, std::vector< cv::DMatch > *matches_output, std::vector<cv::KeyPoint> &keyPts_query,std::vector<cv::KeyPoint> &keyPts_train,int n_matches, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_query, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_train);
    Eigen::Matrix4f ICPNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt, float MaxDistance=0.1, float RansacVar = 0.01, float Iterations = 100);

    void LLS(Eigen::Matrix4f& transformation, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);
    void Downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampled_cloud, float sz = 0.05);

    void color_mesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PolygonMesh& mesh);
};

#endif // PC_ALGORITHMS_H
