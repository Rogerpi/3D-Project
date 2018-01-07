#include "pc_algorithms.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"

#include "commonfunc.h"

PC_Algorithms::PC_Algorithms()
{

}

 void PC_Algorithms::keypoints_to_pc(const cv::Mat &depth_query, const cv::Mat &depth_train,std::vector< cv::DMatch > *matches,std::vector< cv::DMatch > *matches_output, std::vector<cv::KeyPoint> &keyPts_query,std::vector<cv::KeyPoint> &keyPts_train,int n_matches, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_query, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_train){
    commonFunc imgProc;

    cloud_query.reset (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_train.reset (new pcl::PointCloud<pcl::PointXYZ>);

    cloud_query->points.resize (int(matches->size()));
    cloud_train->points.resize (int(matches->size()));
    int p_count = 0;
    int i = -1;

    while(p_count <std::min(n_matches,int(matches->size())) && i < int(matches->size()) )
      {
        i++;

        cv::Point2f pt1 = keyPts_query[(*matches)[i].queryIdx].pt;
        cv::Point2f pt2 = keyPts_train[(*matches)[i].trainIdx].pt;

        int feat_query_y = (int) pt1.x;
        int feat_query_x = (int )pt1.y;
        int feat_train_y = (int) pt2.x;
        int feat_train_x = (int )pt2.y;

        //std::cout<<feat_query_x<<"  "<<feat_query_y<<std::endl;
            float X_q, Y_q, Z_q, X_t, Y_t, Z_t;

            unsigned short query_z = depth_query.at<unsigned short>(feat_query_x,feat_query_y); //change

            unsigned short train_z = depth_train.at<unsigned short>(feat_train_x,feat_train_y); //change
            // Render the 3D values

            std::cout<< i<<" " ;
            imgProc.myDepth2meter(feat_query_x,feat_query_y,query_z, X_q, Y_q, Z_q);

            imgProc.myDepth2meter(feat_train_x,feat_train_y,train_z, X_t, Y_t, Z_t);
            if(X_q>5 || Y_q > 5 || query_z == 0 || X_t>5 || Y_t > 5  ||train_z == 0){
                //n_matches++; //i continues but we didn't take this point
               //std::cout <<" -1"<<std::endl;
                continue;}

            // Write out the colored 3D point
            cloud_query->points[p_count].x = X_q;
            cloud_query->points[p_count].y = Y_q;
            cloud_query->points[p_count].z = Z_q;

            cloud_train->points[p_count].x = X_t;
            cloud_train->points[p_count].y = Y_t;
            cloud_train->points[p_count].z = Z_t;

            (*matches_output)[p_count] = (*matches)[i];
            p_count++;
            //std::cout<<" "<<p_count<<std::endl;


      }
    cloud_query->points.resize (p_count);
    cloud_train->points.resize (p_count);

}



Eigen::Matrix4f PC_Algorithms::ICPNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt, float MaxDistance, float RansacVar, float Iterations){

    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_icp (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_norm (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> norm_est;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (12); //12

    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*src, *points_with_normals_src);

    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> reg;
    reg.setTransformationEpsilon (1e-12);
    //reg.setEuclideanFitnessEpsilon(0.000000000001);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (MaxDistance);
    reg.setRANSACOutlierRejectionThreshold (RansacVar); // 0.05
    reg.setMaximumIterations (Iterations);



    reg.setInputSource (points_with_normals_src);
    reg.setInputTarget (points_with_normals_tgt);
    std::cerr << "PointCloud src has: " << points_with_normals_src->points.size () << " data points." << std::endl;
    std::cerr << "PointCloud tgt has: " << points_with_normals_tgt->points.size () << " data points." << std::endl;

    reg.align (*normals_icp);


    std::cout << "has converged:" << reg.hasConverged() << " score: " << reg.getFitnessScore() << std::endl;

    Eigen::Matrix4f transform_normals = reg.getFinalTransformation ();

    return transform_normals;

}



void PC_Algorithms::LLS(Eigen::Matrix4f& transformation, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out){

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> TESVD;
    if(cloud_in->points.size()<10){ //We set a minimum of 10 points to use it as a system of equations. If this is the case, Identity matrix will be returned
        Eigen::Affine3f transform(Eigen::Translation3f(0.0,0.0,0.0));
        transformation = transform.matrix();//identity
        std::cout<<"Not enough points"<<std::endl;
        return;
    }
    TESVD.estimateRigidTransformation (*cloud_in,*cloud_out,transformation);
    //std::cout<<"transf "<<std::endl<<transformation<<std::endl;
}





void PC_Algorithms::Downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampled_cloud, float sz){
   pcl::VoxelGrid<pcl::PointXYZRGB> grid;
   grid.setLeafSize (sz,sz,sz);
   grid.setInputCloud (input_cloud);
   grid.filter (*sampled_cloud);

}

void PC_Algorithms::color_mesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PolygonMesh& mesh)
{

    // Apply PassThrough filter
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PassThrough<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloud);
    filter.filter(*filtered);

    // Calculate normals
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree.reset(new pcl::search:: KdTree<pcl::PointXYZRGB>(false) ) ;
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);

    // Concatenate pointcloud and normals
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::concatenateFields(*filtered, *normals, *cloud_normals);

    // Poisson reconstruction
    pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
    poisson.setDegree(2);
    poisson.setDepth(10);
    poisson.setSolverDivide (6);
    poisson.setIsoDivide (6);

    poisson.setConfidence(true);
    poisson.setManifold(false); // perhaps, for our final pointcloud should be set to true
    poisson.setOutputPolygons(true);

    poisson.setInputCloud(cloud_normals);
    poisson.reconstruct(mesh);


    // Color the mesh
    pcl::PointCloud<pcl::PointXYZRGB> cloud_color_mesh;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud_color_mesh);

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud (cloud);

    // K nearest neighbor search
    int K = 3;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    for(int i=0;i<cloud_color_mesh.points.size();++i)
    {
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        float dist = 0.0; // probably redundant
        int red = 0;
        int green = 0;
        int blue = 0;

        if (kdtree.nearestKSearch (cloud_color_mesh.points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (int j = 0; j < pointIdxNKNSearch.size (); ++j)
            {
                r = cloud->points[ pointIdxNKNSearch[j] ].r;
                g = cloud->points[ pointIdxNKNSearch[j] ].g;
                b = cloud->points[ pointIdxNKNSearch[j] ].b;

                red += int(r);
                green += int(g);
                blue += int(b);
                dist += 1.0/pointNKNSquaredDistance[j];
            }
        }

        cloud_color_mesh.points[i].r = int(red/pointIdxNKNSearch.size ()+0.5); // maybe we don't need these 0.5
        cloud_color_mesh.points[i].g = int(green/pointIdxNKNSearch.size ()+0.5);
        cloud_color_mesh.points[i].b = int(blue/pointIdxNKNSearch.size ()+0.5);


    }
    pcl::toPCLPointCloud2(cloud_color_mesh, mesh.cloud);
}
