#include "mainwindow.h"
#include "ui_mainwindow.h"

void MainWindow::loadFiles(){
    dirname = QFileDialog::getExistingDirectory(0, ("Select Output Folder"), QDir::currentPath());

    QDir directory(dirname);
    QStringList images = directory.entryList(QStringList() << "*.png" << "*.PNG",QDir::Files);

    ui->listWidget->clear();
    std::cout<<dirname.toStdString()<<std::endl;

    foreach(QString filename, images) {
        std::cout<<filename.toStdString()<<std::endl;

        if (filename.toStdString().find("depth") != std::string::npos) {
            cv::Mat depthMap = cv::imread(dirname.toStdString()+"/"+filename.toStdString(), CV_LOAD_IMAGE_UNCHANGED);
            depth_imgs.push_back(depthMap);

        }
        else if(filename.toStdString().find("color_map") != std::string::npos) {
            cv::Mat color_map_img = cv::imread(dirname.toStdString()+"/"+filename.toStdString(), CV_LOAD_IMAGE_UNCHANGED);
            color_map_imgs.push_back(color_map_img);
            ui->listWidget->addItem(filename);
        }
    }
    std::cout<<color_map_imgs.size()<<"  "<<depth_imgs.size()<<std::endl;
    //IMAGES LOADED
}

void MainWindow::threshold_and_outlier_removal(){

    cv::Mat depth_thr_img,color_map_thr_img;
    //Threshold + outlier
    for(int i=0; i<depth_imgs.size();i++){
        imgProcessing.threshold_depth(depth_imgs[i],depth_thr_img,min_depth_thr,max_depth_thr);
        imgProcessing.border_outlier_removal(depth_thr_img,depth_thr_img,w_size);
        //color_map_thr_img = imgProcessing.threshold_mapped_depth_to_color(depth_thr_img, color_map_imgs[i], min_depth_thr, max_depth_thr);
        depth_thr_imgs.push_back(depth_thr_img);
        //cv::imshow("Color mapped"+std::to_string(i), color_map_thr_img);
    }
}

void MainWindow::createPC(){

    for(int i=0; i<depth_imgs.size();i++){
        pointclouds[i] = imgProcessing.rgbd2pcl(color_map_imgs[i],depth_imgs[i], min_depth_thr, max_depth_thr);
    }
}

void MainWindow::featureMatching(){
    /**************Feature Matching     taking SVD transformations and Pointclouds**********************************/

    std::vector<cv::KeyPoint> keyPts_1, keyPts_2; keyPts_1.clear(); keyPts_2.clear();
    std::vector< cv::DMatch > *matches = new std::vector< cv::DMatch >;


    // Non-static function requires object creation
    commonFunc imgProcessing;

    //Feature Matching


    for(int i=0; i<depth_imgs.size()-1;i++){
        int next_i = (i+1)%depth_imgs.size();
        std::cout<<"next "<<next_i<<std::endl;
        keyPts_1.clear(); keyPts_2.clear();
        clouds_keyPts1[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        clouds_keyPts2[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        imgProcessing.featureMatching(color_map_imgs[i], keyPts_1, color_map_imgs[next_i], keyPts_2, matches, robustMatch);
        std::cout<<"matches number "<<matches->size()<<std::endl;
        //sort
        sort(matches->begin(), matches->end(), [](const cv::DMatch& a, const cv::DMatch& b){
            return a.distance < b.distance;
        });
         std::cout<<"before keypoints to pc"<<std::endl;
        std::vector< cv::DMatch > *best_matches = new std::vector< cv::DMatch >();
        best_matches->resize(50);
       pc_alg.keypoints_to_pc(depth_thr_imgs[i],depth_thr_imgs[next_i],matches,best_matches,keyPts_1,keyPts_2,50,clouds_keyPts1[i],clouds_keyPts2[i]);
       //cv::imshow(images[i].toStdString(),imgProcessing.showFeatureMatches(color_map_imgs[i],keyPts_1,color_map_imgs[i+1],keyPts_2,best_matches));

       //for displaying
       std::cout<<"before displaying"<<std::endl;
       cv::Mat img = imgProcessing.showFeatureMatches(color_map_imgs[i],keyPts_1,color_map_imgs[next_i],keyPts_2,best_matches);
       cvtColor(img, img, CV_BGR2RGB);
        std::cout<<"before push back"<<std::endl;
       match_imgs.push_back(img);

        std::cout<<"before LLS"<<std::endl;

       pc_alg.LLS(RI_svd[i],clouds_keyPts2[i],clouds_keyPts1[i]);

    }
}

void MainWindow::Downsample(){
    /**********************************DOWNSAMPLING*****************************************/
    std::cout<<std::endl<<"Downsampling"<<std::endl;
    std::cout<<"before downsampling"<<std::endl;
    for(int i= 0; i<pointclouds.size();i++){
        pointclouds_d_sample[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pc_alg.Downsampling(pointclouds[i],pointclouds_d_sample[i],0.005);
    }
    std::cout<<"Finished Downsampling"<<std::endl;

    /**************************************************************************/
}

void MainWindow::ICPAlign(){
    std::cout<<"starting icp"<<std::endl;
    for(int i=0 ;i< pointclouds.size()-1;i++){

        std::cout<<i<<std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr  rotated = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        if(ui->svd_checkBox->isChecked()){
            pcl::transformPointCloud (*(pointclouds_d_sample[i+1]),*rotated,RI_svd[i]);
            RI_icp[i] = pc_alg.ICPNormal(rotated,pointclouds_d_sample[i]);
        }
        else{
            RI_icp[i] = pc_alg.ICPNormal(pointclouds_d_sample[i+1],pointclouds_d_sample[i]);
        }
        std::cout<<"ICP"<<std::endl;
        std::cout<<RI_icp[i]<<std::endl;




    }
}

void MainWindow::MergePC(){
    for(int i=0; i<depth_imgs.size()-1;i++){
        std::cout<<"R I Matrix: "<<std::endl<<RI_svd[i]<<std::endl;

        Eigen::Matrix4f RIicp, RIsvd;
        if(ui->svd_checkBox->isChecked())
            RIsvd = RI_svd[i];
        else
            RIsvd = identity_matrix;

        if(ui->icp_checkBox->isChecked())
            RIicp = RI_icp[i];
        else
            RIicp = identity_matrix;

        if(i==0)
            RI_global[i]=RIicp*RIsvd;
        else
            RI_global[i]=RI_global[i-1]*RIicp*RIsvd;


    }
    global_cloud = *(pointclouds_d_sample[0]);
    for(int i=0; i<depth_imgs.size()-1;i++){
        pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
       pcl::transformPointCloud (*(pointclouds_d_sample[i+1]),transformed_cloud, RI_global[i]);
        global_cloud += transformed_cloud;
    }








}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    Eigen::Affine3f transform(Eigen::Translation3f(0.0,0.0,0.0));
    identity_matrix = transform.matrix();
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_PbtnLoad_clicked()
{
   std::cout<<"You pressed Load Data button"<<std::endl;

   depth_imgs.clear();
   color_map_imgs.clear();
   depth_thr_imgs.clear();
   match_imgs.clear();
   /************* Load Files *******************/
   loadFiles();
   /********************************************/

   RI_svd.resize(depth_imgs.size(), identity_matrix);
   RI_icp.resize(depth_imgs.size(), identity_matrix);
   RI_global.resize(depth_imgs.size());
   pointclouds_svd.resize(depth_imgs.size()); //x
   pointclouds.resize(depth_imgs.size());
   clouds_keyPts1.resize(depth_imgs.size());
   clouds_keyPts2.resize(depth_imgs.size());
   pointclouds_d_sample.resize(pointclouds.size());

    /************************* threshold + outlier removal **********************/
    threshold_and_outlier_removal();
    /***********************************************************/
    //Creating pcl
    createPC();

    featureMatching(); //Here is SVD

    Downsample();

        //ICPAlign();

    MergePC();

    /************************************************/

}


//This button is used to TEST different stuff. This should be used to save images, but we changed it(after getting all datasets) to make easier for us to test
void MainWindow::on_PbtnSave_clicked()
{
    std::cout<<"Testing ICP"<<std::endl;

    /**********************************DOWNSAMPLING*****************************************/
    std::cout<<std::endl<<"Downsampling"<<std::endl;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointclouds_d_sample;
    std::cout<<"before downsampling"<<std::endl;
    pointclouds_d_sample.resize(pointclouds.size());
    for(int i= 0; i<pointclouds.size();i++){
        pointclouds_d_sample[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pc_alg.Downsampling(pointclouds[i],pointclouds_d_sample[i],0.005);
    }
    std::cout<<"Finished Downsampling"<<std::endl;

    /**************************************************************************/
    Eigen::Affine3f transform(Eigen::Translation3f(0.1,0.0,0.0));
    Eigen::Matrix4f matrix = transform.matrix();



        // Datasets
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

        input_cloud = pointclouds_d_sample[0];

        /******************Outlier removal ***********************************/ //does almost nothing
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
           // build the filter
           outrem.setInputCloud(input_cloud);
           outrem.setRadiusSearch(0.01);
           outrem.setMinNeighborsInRadius (50);
           // apply filter
           outrem.filter (*cloud_filtered);


           std::cerr << "PointCloud before filtering has: " << input_cloud->points.size () << " data points." << std::endl;
           std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;


           /************************************************************/



        //Back to ICP



    std::cout<<"starting icp"<<std::endl;
    for(int i=0 ;i< pointclouds.size()-1;i++){
        std::cout<<i<<std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr  rotated = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud (*(pointclouds_d_sample[i]),*rotated,matrix);

        pcl::visualization::PCLVisualizer aux_viewer("Aux");


        RI_icp[i] = pc_alg.ICPNormal(rotated,pointclouds_d_sample[i]);
        aux_viewer.addPointCloud(rotated,"rotated2");
        pcl::transformPointCloud (*rotated,*rotated,RI_icp[i]);
        aux_viewer.addPointCloud(rotated,"rotated");
        aux_viewer.addPointCloud(pointclouds_d_sample[i],"sample");
        aux_viewer.spin();
        std::cout<<"ICP"<<std::endl;
        std::cout<<RI_icp[i]<<std::endl;

    }
    std::cout<<"icp done"<<std::endl;

}

void MainWindow::on_PbtnNewScan_clicked()
{

        int i=0;
        cv::Mat depth_img, color_img, color_map_img;

        while(i<num_frames){

            depth_img = app.GetDepthFrame();
            color_img = app.GetColorFrame();
            color_map_img = app.map_depth_to_color(depth_img,color_img);
            cv::cvtColor(color_map_img,color_map_img, cv::COLOR_RGBA2RGB);


            ui->listWidget->addItem(QString(std::to_string(i).c_str()));
            color_map_imgs.push_back(color_map_img);
            depth_imgs.push_back(depth_img);

            i++;

            ui->DisplayImagelabel->setPixmap(QPixmap::fromImage(QImage(color_map_img.data,
                            color_map_img.cols,color_map_img.rows, color_map_img.step, QImage::Format_RGB888)));

            ui->DisplayImagelabel->setScaledContents( true );
            ui->DisplayImagelabel->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );
            Sleep(f_rate);

        }

        RI_svd.resize(depth_imgs.size(), identity_matrix);
        RI_icp.resize(depth_imgs.size(), identity_matrix);
        RI_global.resize(depth_imgs.size());
        pointclouds_svd.resize(depth_imgs.size()); //x
        pointclouds.resize(depth_imgs.size());
        clouds_keyPts1.resize(depth_imgs.size());
        clouds_keyPts2.resize(depth_imgs.size());
        pointclouds_d_sample.resize(pointclouds.size());

         /************************* threshold + outlier removal **********************/
         threshold_and_outlier_removal();
         /***********************************************************/
         //Creating pcl
         createPC();

         featureMatching(); //Here is SVD

         Downsample();

             //ICPAlign();

         MergePC();





}

void MainWindow::on_PbtnSeePointCloud_clicked()
{

    int i=ui->listWidget->currentRow();
    pcl::visualization::PCLVisualizer match_viewer("Simple Match");

    /* color one of the keyPts*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_kPt2_color, pointcloud_transposed (new pcl::PointCloud<pcl::PointXYZRGB> );
    cloud_kPt2_color.reset (new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud_kPt2_color->points.resize(clouds_keyPts2[i]->points.size());
    for (size_t k = 0; k < clouds_keyPts2[i]->points.size(); k++) {
        cloud_kPt2_color->points[k].x = clouds_keyPts2[i]->points[k].x;
        cloud_kPt2_color->points[k].y = clouds_keyPts2[i]->points[k].y;
        cloud_kPt2_color->points[k].z = clouds_keyPts2[i]->points[k].z;

        cloud_kPt2_color->points[k].r = 255;
        cloud_kPt2_color->points[k].g = 255;
        cloud_kPt2_color->points[k].b = 0;
    }
    /******* - color keypoints ******/

    Eigen::Matrix4f RIicp, RIsvd;
    if(ui->svd_checkBox->isChecked())
        RIsvd = RI_svd[i];
    else
        RIsvd = identity_matrix;

    if(ui->icp_checkBox->isChecked())
        RIicp = RI_icp[i];
    else
        RIicp = identity_matrix;


    pcl::transformPointCloud (*pointclouds_d_sample[i+1],*pointcloud_transposed,RIicp*RIsvd);
    pcl::transformPointCloud (*cloud_kPt2_color,*cloud_kPt2_color,RIicp*RIsvd);

    match_viewer.addPointCloud(pointclouds_d_sample[i],"pc1");
    match_viewer.addPointCloud(pointcloud_transposed,"pc2");
    match_viewer.addPointCloud(clouds_keyPts1[i],"keyPts1");
    match_viewer.addPointCloud(cloud_kPt2_color,"keyPts2");
    match_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keyPts1");
    match_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keyPts2");

    match_viewer.spin();
}


// List selection
void MainWindow::on_listWidget_itemSelectionChanged()
{

    int i=ui->listWidget->currentRow();

    ui->DisplayImagelabel->setPixmap(QPixmap::fromImage(QImage(match_imgs[i].data,
                    match_imgs[i].cols,match_imgs[i].rows, match_imgs[i].step, QImage::Format_RGB888)));

    ui->DisplayImagelabel->setScaledContents( true );
    ui->DisplayImagelabel->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );

}


// SpinBoxes and stuff
void MainWindow::on_Outlier_spinBox_valueChanged(int arg1)
{
    w_size = arg1;
    std::cout<<"Outlier Window: "<<w_size<<std::endl;

}

void MainWindow::on_num_frames_spinBox_valueChanged(int arg1)
{
 num_frames = arg1;
}

void MainWindow::on_FrameRate_spinBox_valueChanged(int arg1)
{
    f_rate = arg1*1000;
    std::cout<<"Frame Rate: "<<f_rate<<std::endl;
}

void MainWindow::on_PbtnSeeFullPointCloud_clicked()
{
    pcl::visualization::PCLVisualizer global_viewer("global");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr d_global_cloud (new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(&global_cloud);
    if(ui->outrem_3d_checkBox->isChecked()){
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
       // build the filter
       outrem.setInputCloud(input);
       outrem.setRadiusSearch(0.01);
       outrem.setMinNeighborsInRadius (50);
       // apply filter
       outrem.filter (*d_global_cloud);


       std::cerr << "PointCloud before filtering has: " << global_cloud.points.size () << " data points." << std::endl;
       std::cerr << "PointCloud after filtering has: " << d_global_cloud->points.size () << " data points." << std::endl;

        pc_alg.Downsampling(d_global_cloud,d_global_cloud,0.005);
    }
    else{
         pc_alg.Downsampling(input,d_global_cloud,0.005);
    }
    global_viewer.addPointCloud(d_global_cloud,"original");
    global_viewer.spin();


}


void MainWindow::on_icp_checkBox_clicked(bool checked)
{
    if (checked)
        ICPAlign();
    MergePC();
}

void MainWindow::on_svd_checkBox_clicked(bool checked)
{
    if(ui->icp_checkBox->isChecked())
        ICPAlign();
    MergePC();
}

void MainWindow::on_thresholdSpinBox_min_editingFinished()
{
    min_depth_thr = ui->thresholdSpinBox_min->value();

}

void MainWindow::on_thresholdSpinBox_max_editingFinished()
{
    max_depth_thr = ui->thresholdSpinBox_max->value();


}
