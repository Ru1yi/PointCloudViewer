#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_Lidar_test.h"


#include <QMainWindow>
#include <QWindow>
#include <QHBoxLayout>

#include <QDir>
#include <QTimer>
#include <QDebug>
#include <QSplitter>

#include <vtkGenericOpenGLRenderWindow.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::PCLVisualizer PCLViewer;
typedef std::shared_ptr<PointCloudT> PointCloudPtr;



class Lidar_test : public QMainWindow
{
	Q_OBJECT

public:
	Lidar_test(QWidget* parent = nullptr);
	~Lidar_test();
	void Lidar_test::loadNextPCD();

private slots:	// slot func
	void on_PushButton_Clicked();
	//void on_PushButton_ChangeView_Clicked();
	void on_ComboBox_View_Changed();

private:
	Ui::Lidar_testClass ui;
	PointCloudPtr cloudptr;
	PCLViewer::Ptr cloud_viewer;
	QStringList files;
	int currentFileIndex;
	QTimer* timer;
	std::string folder_path;
	std::string axis;
};
