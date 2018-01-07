#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDialog>
#include <qfiledialog.h>
#include <qstring.h>
#include <QListWidget>
#include <QDir>

#include "kinect_rgbd_grabber.h"
#include "commonfunc.h"
#include "pc_algorithms.h"

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

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

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


#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    /****************** Spin boxes value changed **********************/
    void on_Outlier_spinBox_valueChanged(int arg1);

    void on_num_frames_spinBox_valueChanged(int arg1);

    void on_thresholdSpinBox_min_editingFinished();

    void on_thresholdSpinBox_max_editingFinished();

    void on_FrameRate_spinBox_valueChanged(int arg1);

    /******************************************************************/

    /******************** BUTTONS *************************************/

    void on_PbtnLoad_clicked();

    void on_PbtnSave_clicked();

    void on_PbtnNewScan_clicked();


    void on_PbtnSeePointCloud_clicked();

    void on_PbtnSeeFullPointCloud_clicked();

   /*******************************************************************/

    //Selecting  an element of the list will show the feature matching between
    //This image and the next one
    void on_listWidget_itemSelectionChanged();

    //Select if ICP will be used
    void on_icp_checkBox_clicked(bool checked);

    //Select if SVD will be used (feature match approach)
    void on_svd_checkBox_clicked(bool checked);



private:
    //Making code clearer. This functions use class variables and are must made to make code more clear

    //Load files from a QT file selector and save color and depth images on arrays that will be used later
    void loadFiles();

    //Threshold all depth images and remove borders (where most of the outlier appear)
    void threshold_and_outlier_removal();

    //Create PointClouds separately for all the depth+color images
    void createPC();

    //Compute feature matching for each consecutive pair of color+depth images
    void featureMatching();

    //Compute ICP to determine Transformation matrix between each consecutive pair of color+depth images. IF SVD checkbox is selected
    //Pointclouds will be transformed on the same coordinate system using feature matching approach before ICP (ICP would do a final adjust)
    //If SVD checkbox is not clicked, ICP will be used to transform the orignal pointclouds.
    void ICPAlign();

    //Downsample all pointclouds and store it in a array.
    void Downsample();

    //Merge all pointclouds using SVD + ICP transformations. Final Pointcloud will be downsampled to reduce data. Also, a radius outlier removal will be applyied.
    void MergePC();

private:
    Ui::MainWindow *ui;

    int  f_rate;

    int num_frames;
    QString dirname;

    /*********SYSTEM VARIABLES **************************/
    double min_depth_thr = 0.0, max_depth_thr = 1.45;
    bool robustMatch = false;
    std::string DirName = "DataSets_RGBD/Roger3/";
    int w_size = 4;

    std::vector<cv::Mat> depth_imgs, color_map_imgs;
    std::vector<cv::Mat> depth_thr_imgs, match_imgs;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointclouds;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointclouds_svd; // x
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_keyPts1, clouds_keyPts2;
    std::vector<Eigen::Matrix4f> RI_svd, RI_icp, RI_global ;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointclouds_d_sample;

    pcl::PointCloud<pcl::PointXYZRGB> global_cloud;
    /****************************************************/

   /********************* other variables ***************/
    PC_Algorithms pc_alg = PC_Algorithms();
    Kinect_RGBD_Grabber app;
    commonFunc imgProcessing;

     Eigen::Matrix4f identity_matrix;
    /*****************************************************/


};

#endif // MAINWINDOW_H
