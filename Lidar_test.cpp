#include "Lidar_test.h"

Lidar_test::Lidar_test(QWidget* parent)
	: QMainWindow(parent)
{
	//ui->setupUi(this);
	ui.setupUi(this);

	// Create PCLViewer Obj & set the title
	this->cloud_viewer.reset(new PCLViewer("3D Viewer"));
	this->cloud_viewer->setShowFPS(false);
	this->cloud_viewer->setCameraPosition(0, 0, 80, 0, 1, 0);	//Top View

	// Embed the rendering window of cloud_viewer into QWidget
	auto viewerWinId = QWindow::fromWinId((WId)cloud_viewer->getRenderWindow()->GetGenericWindowId());
	QWidget* widget = QWidget::createWindowContainer(viewerWinId, nullptr);

	// Create a QVBoxLayout object and add the QWidget 
	QHBoxLayout* mainLayout = new QHBoxLayout;
	mainLayout->addStretch();
	mainLayout->addWidget(widget);

	centralWidget()->setLayout(mainLayout);
	widget->setMinimumSize(1920, 1080);
	resize(mainLayout->sizeHint());

	// Initialize folder path
	//this->folder_path = "E:/Dataset/weilai/pcd/pcd/";
	this->folder_path = "E:/Dataset/weilai/pcd_selected/";
	QDir directory(QString::fromStdString(folder_path));
	QStringList files = directory.entryList(QStringList() << "*.pcd", QDir::Files);

	qDebug() << "The files are:" << files;

	// Initialize files list and index
	this->files = files;
	currentFileIndex = 0;

	// Initialize color field & display
	this->axis = "intensity";
	QFont font("Times New Roman", 14); // set Font & Size
	this->ui.lineEdit_color->setFont(font);
	this->ui.lineEdit_color->setText("intensity");

	// Initialize checkBox_loop
	this->ui.checkBox_loop->setChecked(true);

	// Initialize and start timer
	timer = new QTimer(this);
	connect(timer, &QTimer::timeout, this, &Lidar_test::loadNextPCD);
	timer->start(100);	//msec   10hz
}

void Lidar_test::loadNextPCD()
{
	qDebug() << "currentFileIndex: " << currentFileIndex << endl;
	if (currentFileIndex >= files.size()) {
		if (this->ui.checkBox_loop->isChecked()) {
			this->currentFileIndex = 0;
		}
		else {
			timer->stop();
			return;
		}
	}
	this->cloudptr.reset(new PointCloudT);	// Release old memory & create a new memory for cloudptr 
	//qDebug() << QString::fromStdString(folder_path) << endl;
	std::string file_path = folder_path + files[currentFileIndex].toStdString();
	//qDebug() << QString::fromStdString(file_path) << endl;
	pcl::io::loadPCDFile(file_path, *this->cloudptr);

	// Color PointCloud according to the field
	pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(this->cloudptr, this->axis);

	// Add initial PointCloud then update data
	if (!this->cloud_viewer->updatePointCloud(this->cloudptr, color_handler)) {
		qDebug() << "update failed!" << endl;
		this->cloud_viewer->addPointCloud(this->cloudptr, color_handler);
	}

	this->cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

	this->cloud_viewer->spinOnce();

	currentFileIndex++;
}

void Lidar_test::on_PushButton_Clicked()
{
	qDebug() << "Change color field!" << endl;
	if ("z" == this->axis) {
		this->axis = "intensity";
		this->ui.lineEdit_color->setText("intensity");
	}
	else {
		this->axis = "z";
		this->ui.lineEdit_color->setText("z");
	}
	// Get current camera parameters
	std::vector<pcl::visualization::Camera> cam;
	this->cloud_viewer->getCameras(cam);
	qDebug() << "Cam:" << endl;
	qDebug() << "- pos: " << cam[0].pos[0] << " " << cam[0].pos[1] << " " << cam[0].pos[2] << endl;
	qDebug() << "- view: " << cam[0].view[0] << " " << cam[0].view[1] << " " << cam[0].view[2] << endl;
}

//void Lidar_test::on_PushButton_ChangeView_Clicked()
//{
//	this->cloud_viewer->setCameraPosition(-17.3, 1.125, 1.51408, 0.0893246, 0.0345669, 0.995403);
//}

void Lidar_test::on_ComboBox_View_Changed()
{
	switch (this->ui.comboBox_View->currentIndex()) {
	case 0:	//	Top View
		this->cloud_viewer->setCameraPosition(0, 0, 80, 0, 1, 0);
		break;
	case 1:	// Special View
		this->cloud_viewer->setCameraPosition(-5.35, -0.220377, 1.14, 0.1823246, 0.1045669, 0.975403);
		break;
	}
	qDebug() << "comboBox current index: " << this->ui.comboBox_View->currentIndex() << endl;
}

Lidar_test::~Lidar_test()
{
	//delete ui;
}
