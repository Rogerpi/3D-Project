#include "pclviewer.h"
#include "ui_pclviewer.h"

PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer)

{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");

  // Setup the cloud pointer
  cloud_.reset (new PointCloudT);
  // The number of points in the cloud
  cloud_->resize (61128);

  // Set up the QVTK window
  viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  viewer_->setBackgroundColor (0.1, 0.1, 0.1);
  viewer_->addCoordinateSystem(1.0);
  ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
  viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();



  // Connect "Load" and "Save" buttons and their functions
  connect (ui->pushButton_load, SIGNAL(clicked ()), this, SLOT(loadFileButtonPressed ()));
  connect (ui->pushButton_save, SIGNAL(clicked ()), this, SLOT(saveFileButtonPressed ()));

  viewer_->resetCamera ();
  ui->qvtkWidget->update ();


}

PCLViewer::~PCLViewer ()
{
  delete ui;
}

void
PCLViewer::loadFileButtonPressed ()
{
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());
  PointCloudT::Ptr cloud_tmp (new PointCloudT);

  if (filename.isEmpty ()){
     std::cout<<"FILENAME EMPTY"<<std::endl;
      return;
  }

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive)){
    return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
    std::cout<<cloud_tmp->points.size()<<"POIINTS"<<std::endl;
  }
  else
    return_status = pcl::io::loadPLYFile (filename.toStdString (), *cloud_tmp);

  if (return_status != 0)
  {
    std::cout<<"ERROR READING POINT CLOUD"<<std::endl;
    PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }

  // If point cloud contains NaN values, remove them before updating the visualizer point cloud
  if (cloud_tmp->is_dense)
    pcl::copyPointCloud (*cloud_tmp, *cloud_);
  else
  {
    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
    std::vector<int> vec;
    std::cout<<"CLOUD NOT DENSE"<<std::endl;
    pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
  }

  //viewer_->updatePointCloud (cloud_, "cloud");
  viewer_->addPointCloud(cloud_,"Cloud");
  std::cout<<"Added point cloud"<<std::endl;
  viewer_->resetCamera ();
  ui->qvtkWidget->update ();

  std::cout<<"DONE"<<std::endl;
}

void
PCLViewer::saveFileButtonPressed ()
{
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloud_);
  else if (filename.endsWith (".ply", Qt::CaseInsensitive))
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  else
  {
    filename.append(".ply");
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  }

  if (return_status != 0)
  {
    PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }
}


