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

#include <vtkFileOutputWindow.h>
#include <vtkOutputWindow.h>


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
	/**
	* @brief Change color field when pushbutton clicked
	*/
	void on_PushButton_ChangeColor_Clicked();
	/**
	* @brief Change view when combobox changed
	*/
	void on_ComboBox_View_Changed();
	/**
	* @brief Add or remove grid when checkbox state changed
	*/
	void on_CheckBox_Grid_stateChanged();

private:
	Ui::Lidar_testClass ui;
	PointCloudPtr cloudptr;
	PCLViewer::Ptr cloud_viewer;
	QStringList files;
	int currentFileIndex;
	QTimer* timer;
	std::string folder_path;	//pcd folder path
	std::string axis;	// color field

	const float lidar_height = 2.0641;

	// grid params
	int grid_size = 10;
	int grid_spacing = 1;
};
