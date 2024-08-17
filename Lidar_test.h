/**************************************************************************

Copyright © 2024-, SuZhou Invent Precision Machining CO.,Ltd All Rights Reserved

Author: Ru1yi

Date:2024-08-17

Description: Point Cloud Visualization Software Headers

**************************************************************************/

#pragma once

//#include <QtWidgets/QMainWindow>
#include "ui_Lidar_test.h"

// Qt Headers
#include <QWindow>
#include <QDir>
#include <QTimer>	
#include <QDebug>
#include <QSettings>	// Read ini files

// 3rd Party Headers
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkFileOutputWindow.h>
#include <vtkOutputWindow.h>
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
	// Qt member variables
	Ui::Lidar_testClass ui;
	QStringList files;
	QTimer* timer;
	QSettings* settings;

	// PCL visualizer member variables
	PointCloudPtr cloudptr;
	PCLViewer::Ptr cloud_viewer;
	std::string axis;	// color field

	// variables
	int currentFileIndex;
	std::string folder_path;	//pcd folder path	
    float lidar_height;

	// Grid params
	int grid_size = 10;
	int grid_spacing = 1;
};
