#define _CRT_SECURE_NO_WARNINGS；
#pragma warning(disable:4996);

//las
#include <liblas/liblas.hpp>
#include <liblas/point.hpp>
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>
#include <liblas/factory.hpp>

//System 
#include <string.h>
#include <string>  
#include <io.h>  
#include <fstream>				
#include <iostream>		
#include <math.h>  
#include <vector>
#include <ctime>
#include <time.h>
#include <vector>
#include <exception>

//pcl
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>        //点类型定义头文件
#include <pcl/kdtree/kdtree_flann.h> //kdtree类定义头文件
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/shared_ptr.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>//特征提取
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h> 
#include <pcl/surface/gp3.h>//重构
#include <pcl/surface/poisson.h>
#include <pcl/common/transforms.h> //转换矩阵
#include <pcl/console/parse.h>

//opengl
#include <gl/glew.h>
#include <gl/freeglut.h>

#include "lasIO.h"
#include "camera.h"

using namespace std;
Camera camera;

//坐标轴
GLfloat transx, transy;
GLfloat scale;
int primw = 300;
int primh = 300;
GLfloat rotatex = 0, rotatey = 0;
GLint mousepx, mousepy;

uint8_t r = 0, g = 0, b = 0;

int window_width = 640;
int window_height = 480;

float thetaX = 0.0, thetaY = 0.0, scaleFactor = 1.0;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

pcl::PointCloud<pcl::PointXYZRGB> pcloud;

std::string lasfile = "C:/Users/Y583/Desktop/激光雷达数据处理软件开发/mesh.LiData";
std::string pcdfile = "C:/Users/Y583/Desktop/激光雷达数据处理软件开发/milk.pcd";

void lasRead()
{
	std::ifstream ifs;
	ifs.open(lasfile, std::ios::in | std::ios::binary);

	if (ifs.fail())
	{
		std::cout << "File Error!" << std::endl;
		return;
	}
		
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();

	std::cout << "Number of point records     : " << header.GetPointRecordsCount() << std::endl;//记录的点数信息
	std::cout << "File Signature(“LASF”)   : " << header.GetFileSignature() << std::endl;

	std::cout << std::setiosflags(std::ios::fixed); //设置小数
	std::cout << std::setprecision(9) << "X scale factor              : " << header.GetScaleX() << std::endl;
	std::cout << "Y scale factor              : " << header.GetScaleY() << std::endl;
	std::cout << "Z scale factor              : " << header.GetScaleZ() << std::endl;
	std::cout << "X offset                    : " << header.GetOffsetX() << std::endl;
	std::cout << "Y offset                    : " << header.GetOffsetY() << std::endl;
	std::cout << "Z offset                    : " << header.GetOffsetZ() << std::endl;
	std::cout << "Max X                       : " << header.GetMaxX() << std::endl;
	std::cout << "Max Y                       : " << header.GetMaxY() << std::endl;
	std::cout << "Max Z                       : " << header.GetMaxZ() << std::endl;
	std::cout << "Min X                       : " << header.GetMinX() << std::endl;
	std::cout << "Min Y                       : " << header.GetMinY() << std::endl;
	std::cout << "Min Z                       : " << header.GetMinZ() << std::endl;
	
	reader.ReadPointAt(0);
	liblas::Point const& p = reader.GetPoint();
	liblas::Color const& rgb = p.GetColor();
	
	std::cout << "r                           :" << rgb.GetRed() << std::endl;
	std::cout << "g                           :" << rgb.GetGreen() << std::endl;
	std::cout << "b                           :" << rgb.GetBlue() << std::endl;
	
	std::cout << "X                           :" << p.GetX() << std::endl;
	std::cout << "Y                           :" << p.GetY() << std::endl;
	std::cout << "Z                           :" << p.GetZ() << std::endl;
	std::cout << "Intensity                   :" << p.GetIntensity() << std::endl;
	std::cout << "Return Number               :" << p.GetReturnNumber() << std::endl;
	std::cout << "Number of Returns           :" << p.GetNumberOfReturns() << std::endl;
	std::cout << "Classification              :" << p.GetClassification() << std::endl;
	std::cout << "Scan Direction Flag         :" << p.GetScanDirection() << std::endl;
	std::cout << "Edge of Flight Line         :" << p.GetFlightLineEdge() << std::endl;
	std::cout << "Scan Angle Rank             :" << p.GetScanAngleRank() << std::endl;
	std::cout << "Point Source ID             :" << p.GetPointSourceID() << std::endl;
	std::cout << "GPS Time                    :" << p.GetTime() << std::endl;
	

	ifs.close();

	//pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	//int count = header.GetPointRecordsCount(); //获取总的点数
	//pointCloudPtr->resize(count);             
	//pointCloudPtr->width = 1;
	//pointCloudPtr->height = count;
	//pointCloudPtr->is_dense = false;

	//double x = 0, y = 0, z = 0;
	//int i = 0;
	//while (reader.ReadNextPoint()) //循环读取las文件中的点
	//{
	//	liblas::Point const& laspoint = reader.GetPoint();
	//	pointCloudPtr->points[i].x = laspoint.GetX();
	//	pointCloudPtr->points[i].y = laspoint.GetY();
	//	pointCloudPtr->points[i].z = laspoint.GetZ();
	//	++i;
	//}
	//pcl::io::savePCDFileASCII(pcdfile, *pointCloudPtr);

	//ifs.close();
}

int txtPoints(char* fname)
{
	int n = 0;
	int c = 0;
	FILE *fp;
	fp = fopen(fname, "r");
	do{
		c = fgetc(fp);
		if (c == '\n')
		{
			++n;
		}
	}
	while (c != EOF);
	fclose(fp);
	return n;
}

void txtRead()
{
	int n = 0;
	FILE *fp_1;
	fp_1 = fopen("", "r");
	n = txtPoints("");
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.width = n;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	double x, y, z;
	int i = 0;

	while (3 == fscanf(fp_1, "%lf,%lf,%lf\n", &x, &y, &z))
	{
		cloud.points[i].x = x;
		cloud.points[i].y = y;
		cloud.points[i].z = z;
		++i;
	}
	fclose(fp_1);

	pcl::io::savePCDFile("", cloud);
	std::cerr << "Saved " << cloud.points.size() << " " << std::endl;
}

void motion(int x, int y)
{
	int w, h;
	w = glutGet(GLUT_WINDOW_WIDTH);
	h = glutGet(GLUT_WINDOW_HEIGHT);

	if (0 <= x && x <= w && 0 <= y && y <= h)
	{
		rotatex = (mousepy - y) / (GLfloat)h * 360;
		rotatey = (mousepx - x) / (GLfloat)w * 360;
		/*cout<<"rotatex:rotatey"<<rotatex<<" "<<rotatey<<endl;*/
		glutPostRedisplay();
	}
}

void reshape(int width, int height)
{
	glMatrixMode(GL_PROJECTION);//将视口矩阵与投影矩阵设置为当前矩阵,投影变换，确定显示空间的大小
	glLoadIdentity();
	glViewport(0, 0, (GLsizei)width, (GLsizei)height);
	
	if (width <= height)
		gluOrtho2D(-10, 10, -10.0 / width*height, 10.0 / width*height);
	else
		gluOrtho2D(-10.0 / height*width, 10.0 / height*width, -10, 10);
	
	if (width <= height)
	{
		/*scale=(GLfloat)primw/w;*/
		transx = (50 - width / 2.0)*20.0 / width;
		transy = (50 - height / 2.0)*20.0 / width;
	}
	else
	{
		/*scale=(GLfloat)primh/h;*/
		transx = (50 - width / 2.0)*20.0 / height;
		transy = (50 - height / 2.0)*20.0 / height;
	}

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glLoadIdentity();
	glOrtho(-1.5, 1.5, -1.5, 1.5, -5, 5);
}

void drawCloud()
{
	glPointSize(1.0f);
	glColor3f(0.0, 1.0, 0.0);
	glBegin(GL_POINTS);
	for (int i = 0; i < pcloud.size(); i++)
	{
		glVertex3f(pcloud[i].x, pcloud[i].y, pcloud[i].z);
		glColor3f(r, g, b);
	}
	glEnd();
}

void setProjection()
{ 
	if (0 == window_height) 
	{ 
		window_height = 1;
	} 
	double ratio = 1.0 * window_width / window_height; 
	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity(); 
	gluPerspective(45, ratio, 0.01, 100); 
	glViewport(0, 0, window_width, window_height);
	glMatrixMode(GL_MODELVIEW); 
	glLoadIdentity(); 
	gluLookAt(camera.camera_x, camera.camera_y, camera.camera_z, camera.lookat_x, camera.lookat_y, camera.lookat_z, 0.0f, 1.0f, 0.0f); 
}

void setLight()
{ 
	GLfloat ambient[] = { 1.0, 1.0, 1.0, 1.0 }; 
	GLfloat diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat position[] = { 10.0, 10.0, 0.0, 1.0 }; 
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient); 
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse); 
	glLightfv(GL_LIGHT0, GL_POSITION, position);
	glEnable(GL_LIGHT0); glEnable(GL_LIGHTING); 
}

void ReadPointCloud()
{
	////随机点云生成
	//pcloud.width = 10000;             //此处点云数量
	//pcloud.height = 1;                //表示点云为无序点云
	//pcloud.points.resize(pcloud.width * pcloud.height);
		 
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcdfile, pcloud) == -1)
	{
		PCL_ERROR("error");
		return;
	}
	pcloud.points.resize(pcloud.width * pcloud.height);  //点云总数大小	

	for (int i = 0; i < 100; ++i)
	{
		for (size_t i = 0; i < pcloud.size(); ++i)   //循环填充点云数据
		{			
			pcloud[i].x = 32.0f * rand() / (RAND_MAX + 1.0f); //1024.0f * rand() / (RAND_MAX + 1.0f);
			pcloud[i].y = 32.0f * rand() / (RAND_MAX + 1.0f); //1024.0f * rand() / (RAND_MAX + 1.0f);
			pcloud[i].z = 32.0f * rand() / (RAND_MAX + 1.0f); //1024.0f * rand() / (RAND_MAX + 1.0f);
			/*r = pcloud[i].r;
			g = pcloud[i].g;
			b = pcloud[i].b;*/
					
		/*	std::cout << " " << pcloud[i].x
					  << " " << pcloud[i].y
				      << " " << pcloud[i].z 
				      << std::endl;	*/	
		}
		std::cout << " " << pcloud.width * pcloud.height;
	}
}

void GLdisplay()
{
	try
	{
		glClear(GL_COLOR_BUFFER_BIT); //把窗口清除为当前颜色
		glPointSize(8);

		glPushMatrix();
		glTranslatef(transx, transy, 0);
		glRotatef(rotatex, 1, 0, 0);
		glRotatef(rotatey, 0, 1, 0);
		glBegin(GL_LINES);
			glVertex3f(0, 0, 0);
			glVertex3f(0, 2, 0);
			glVertex3f(0, 0, 0);
			glVertex3f(2, 0, 0);
			glVertex3f(0, 0, 0);
			glVertex3f(0, 0, 2);
		glEnd();
		glPopMatrix();

		glFlush();

		if (thetaY < 0)
		{
			thetaY = thetaY + 360;
		}
		if (thetaY > 360)
		{
			thetaY = thetaY - 360;
		}
		if (thetaX < 0)
		{
			thetaX = thetaX + 360;
		}
		if (thetaX > 360)
		{
			thetaX = thetaX - 360;
		}
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glRotatef(thetaX, 1, 0, 0);
		glRotatef(thetaY, 0, 1, 0);
		glScalef(scaleFactor, scaleFactor, scaleFactor);
		setProjection();
		setLight();
		drawCloud();
		glutSwapBuffers();
		Sleep(100);

			//for (int i = 0; i < 100; i++)
			//{
			//	pcloud.resize(5000);
			//	for (int j=0; j < 5000; j++)
			//	{                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
			//		pcloud[j].x = rand() % 50;
			//		pcloud[j].y = rand() % 50;
			//		pcloud[j].z = rand() % 50;
			//	}
			//}
	}
	
	catch (...)
	{ 
	
	}
}

void SpecialKeys(int key, int x, int y) 
{ 
	if (key == GLUT_KEY_UP)
		camera.PitchCamera(camera.angle); 
	if (key == GLUT_KEY_DOWN) 
		camera.PitchCamera(-camera.angle);
	if (key == GLUT_KEY_LEFT)
		camera.YawCamera(-camera.angle);
	if (key == GLUT_KEY_RIGHT)
		camera.YawCamera(camera.angle); 
	glutPostRedisplay(); 
}

void KeyboardKeys(unsigned char key, int x, int y)
{
	if (key == 'w')
		camera.WalkStraight(camera.speed);
	if (key == 's') 
		camera.WalkStraight(-camera.speed); 
	if (key == 'a') 
		camera.WalkTransverse(camera.speed); 
	if (key == 'd') 
		camera.WalkTransverse(-camera.speed); 
	glutPostRedisplay();
}

void mousedown(int mouse, int state, int x, int y)
{
	if (state == GLUT_DOWN)
	{
		mousepx = x;
		mousepy = y;
	}
	//cout<<"mousepx:mousepy"<<endl;
	//cout<<mousepx<<"  "<<mousepy<<endl;
}

void vtkDisplay()
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::visualization::PCLVisualizer view("VTKview");

	//pcl::io::loadPCDFile<pcl::PointXYZ>("C:/Users/Y583/Desktop/激光雷达数据处理软件开发/sac-ia_ndt_room_scan_transformed.pcd", cloud);
	//view.updatePointCloud(cloud.makeShared());



	//彩色点云  
	pcl::PolygonMesh plyfile;
	pcl::io::loadPolygonFile("C:/Users/Y583/Desktop/激光雷达数据处理软件开发/mesh.ply", plyfile);
	view.addPolygonMesh(plyfile);


	//pcl::PointCloud<pcl::PointXYZ> pp;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr temp = cloud.makeShared();
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(temp, 0, 255, 0);

	/*for (int i = 0; i < 1000; i++)
	{
		if (i == 0)
			pp.resize(60000);
		for (int ii = 0; ii < 60000; ii++)
		{
			pp[ii].x = 54.0f * rand() / (RAND_MAX + 1.0f);
			pp[ii].y = 54.0f * rand() / (RAND_MAX + 1.0f);
			pp[ii].z = 54.0f * rand() / (RAND_MAX + 1.0f);
		}
		if (i == 0)
			view.addPointCloud(pp.makeShared(), tgt_h, "view");
		else
			view.updatePointCloud(pp.makeShared(), "view");
		view.spinOnce(1000);
	}*/
}

//多幅点云连续显示
void manyOpenPointCloud()
{
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	pcl::visualization::PCLVisualizer view("VTKview");

	//pcl::io::loadPCDFile(, cloud);
	view.addPointCloud(cloud.makeShared());

}

void rgbaData()
{
	typedef pcl::PointXYZRGBA PointT;
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	std::string filename = "C:/Users/Y583/Desktop/激光雷达数据处理软件开发/Serpent Mound Model LAS Data.pcd";

	try
	{
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename, *cloud) == -1)
		{
			PCL_ERROR("Couldn't read PCD file \n");
			return;
		}
		printf("Loaded %d data points from PCD\n", cloud->width * cloud->height);

		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			printf("%8.3f %8.3f %8.3f %5d %5d %5d %5d\n",
				cloud->points[i].x,
				cloud->points[i].y,
				cloud->points[i].z,
				cloud->points[i].r,
				cloud->points[i].g,
				cloud->points[i].b,
				cloud->points[i].a
				);
		}
		pcl::visualization::PCLVisualizer viewer("viewer");
		viewer.setCameraPosition(0, 0, -3.0, 0, -1, 0);
		viewer.addCoordinateSystem(0.3);
		viewer.addPointCloud(cloud);
		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
		}
	}
	catch (exception e)
	{

	}
}

//遍历指定文件夹下的文件
int FindFile(string fileName, string filePath)
{

	assert(fileName != "" && filePath != "");
	string exeName = fileName.substr(fileName.find_last_of('.'));
	string strPath = filePath;
	string filiterName = "*.*";
	struct _finddata_t fileInfo;
	long handle = _findfirst((strPath + filiterName).c_str(), &fileInfo);
	if (strPath[strPath.length() - 1] != '\\')
	{
		strPath = strPath + "\\";
	}
	if (handle == -1L)
	{
		cout << "Cannot Open The Path!" << endl;
		return 0;
	}
	do
	{
		string path = fileInfo.name;
		if (fileInfo.attrib & _A_SUBDIR)
		{
			if (strcmp(fileInfo.name, ".") != 0 && strcmp(fileInfo.name, "..") != 0)
			{
				FindFile(fileName, strPath + path + "\\");
			}
		}
		else if (fileInfo.attrib & _A_ARCH && path.substr(path.find_last_of('.')) == exeName)
		{
			cout << strPath + fileInfo.name << endl;
		}
	} 
	while (_findnext(handle, &fileInfo) == 0);
		_findclose(handle);
	return 0;
}

//同一文件夹下同类文件
void getFiles(string path, vector<string>& files)
{
	//文件句柄  
	long   hFile = 0;
	//文件信息  
	struct _finddata_t fileinfo;  //文件信息读取结构
	string p;  //赋值函数:assign()
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib &  _A_SUBDIR))  //比较文件类型是否是文件夹
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					files.push_back(p.assign(path).append("\\").append(fileinfo.name));
					getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
				}
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} 
		while (_findnext(hFile, &fileinfo) == 0);  //寻找下一个，成功返回0，否则-1
		_findclose(hFile);
	}
}

//找文件
vector<std::string> findfileinfolder(string fileFolderPath, string fileExtension) //文件路径和后缀名
{
	std::string fileFolder = fileFolderPath + "\\*." + fileExtension;
	vector<std::string> file;

	char fileName[2000];

	struct _finddata_t fileInfo;

	long findResult = _findfirst(fileFolder.c_str(), &fileInfo);
	if (findResult == -1)
	{
		_findclose(findResult);
		return file;
	}
	bool flag = 0;

	do
	{
		sprintf(fileName, "%s\\%s", fileFolderPath.c_str(), fileInfo.name);

		if (fileInfo.attrib == _A_ARCH)
		{
			file.push_back(fileName);
			//Mat frame = imread(fileName, 1); //opencv
			//imshow("1", frame);
		}

	} 
	while (!_findnext(findResult, &fileInfo));
	_findclose(findResult);
	return file;
}

//将mesh点云染色
void poisson_Reconstruction()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object_cloud;
	typedef pcl::PointXYZRGB PointType;  //PointXYZRGB数据结构
	typedef pcl::Normal NormalType;       //法线结构
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud(*object_cloud, *cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PassThrough<pcl::PointXYZRGB> filter;
	filter.setInputCloud(cloud);
	filter.filter(*filtered);
	cout << "passthrough filter complete" << endl;

	cout << "begin normal estimation" << endl;
	pcl::NormalEstimationOMP<PointType, NormalType> ne;//计算点云法向
	ne.setNumberOfThreads(8);//设定临近点
	ne.setInputCloud(filtered);
	ne.setRadiusSearch(0.01);//设定搜索半径
	Eigen::Vector4f centroid;
	compute3DCentroid(*filtered, centroid);//计算点云中心
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);//将向量计算原点置于点云中心

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
	ne.compute(*cloud_normals);
	cout << "normal estimation complete" << endl;
	cout << "reverse normals' direction" << endl;

	//将法向量反向
	for (size_t i = 0; i < cloud_normals->size(); ++i)
	{
		cloud_normals->points[i].normal_x *= -1;
		cloud_normals->points[i].normal_y *= -1;
		cloud_normals->points[i].normal_z *= -1;
	}

	//融合RGB点云和法向
	cout << "combine points and normals" << endl;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

	//泊松重建
	cout << "begin poisson reconstruction" << endl;
	pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
	//poisson.setDegree(2);
	poisson.setDepth(8);
	poisson.setSolverDivide(6);
	poisson.setIsoDivide(6);

	poisson.setConfidence(false);
	poisson.setManifold(false);
	poisson.setOutputPolygons(false);

	poisson.setInputCloud(cloud_smoothed_normals);
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);

	cout << "finish poisson reconstruction" << endl;

	//给mesh染色
	pcl::PointCloud<pcl::PointXYZRGB> cloud_color_mesh;
	pcl::fromPCLPointCloud2(mesh.cloud, cloud_color_mesh);

	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud);

	// K nearest neighbor search
	int K = 5;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	for (int i = 0; i<cloud_color_mesh.points.size(); ++i)
	{
		uint8_t r = 0;
		uint8_t g = 0;
		uint8_t b = 0;
		float dist = 0.0;
		int red = 0;
		int green = 0;
		int blue = 0;
		uint32_t rgb;

		if (kdtree.nearestKSearch(cloud_color_mesh.points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			for (int j = 0; j < pointIdxNKNSearch.size(); ++j)
			{

				r = cloud->points[pointIdxNKNSearch[j]].r;
				g = cloud->points[pointIdxNKNSearch[j]].g;
				b = cloud->points[pointIdxNKNSearch[j]].b;

				red += int(r);
				green += int(g);
				blue += int(b);
				dist += 1.0 / pointNKNSquaredDistance[j];

				std::cout << "red: " << int(r) << std::endl;
				std::cout << "green: " << int(g) << std::endl;
				std::cout << "blue: " << int(b) << std::endl;
				cout << "dis:" << dist << endl;
			}
		}

		cloud_color_mesh.points[i].r = int(red / pointIdxNKNSearch.size() + 0.5);
		cloud_color_mesh.points[i].g = int(green / pointIdxNKNSearch.size() + 0.5);
		cloud_color_mesh.points[i].b = int(blue / pointIdxNKNSearch.size() + 0.5);
	}

	toPCLPointCloud2(cloud_color_mesh, mesh.cloud);
	pcl::io::savePLYFile("C:/Users/Y583/Desktop/激光雷达数据处理软件开发/mesh1.ply", mesh);
}

void main(int argc, char *argv[])
{
	//vector<std::string> pcd = findfileinfolder("", "");
	//rgbaData();
	poisson_Reconstruction();
	//lasRead();
	//vtkDisplay();
	//ReadPointCloud();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(640, 480);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("GLView");
	glutReshapeFunc(reshape);
	glutSpecialFunc(SpecialKeys);
	glutMotionFunc(motion);
	glutMouseFunc(mousedown);
	glutKeyboardFunc(KeyboardKeys);
	glutDisplayFunc(&GLdisplay);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glutMainLoop();

	//return 0;
}

void kdSearch()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Generate pointcloud data
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}
	
	//创建KdTreeFLANN对象，并把创建的点云设置为输入,创建一个searchPoint变量作为查询点
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	//设置搜索空间
	kdtree.setInputCloud(cloud);

	//设置查询点并赋随机值
	pcl::PointXYZ searchPoint;
	searchPoint.x = 1024.0f + rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f + rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f + rand() / (RAND_MAX + 1.0f);

	//K 临近搜索
	//创建一个整数（设置为10）和两个向量来存储搜索到的K近邻，两个向量中，一个存储搜索到查询点近邻的索引，另一个存储对应近邻的距离平方
	int k = 10;

	std::vector<int> pointIdxNKNSearch(k);
	std::vector<float> pointNKNSquaredDistance(k); //存储近邻点对应距离平方

	//打印相关信息
	std::cout << "K nearest neighbor search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with k=" << k << std::endl;

	if (kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)  //执行K近邻搜索
	{
		//打印所有近邻坐标
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
			<< " " << cloud->points[pointIdxNKNSearch[i]].y
			<< " " << cloud->points[pointIdxNKNSearch[i]].z
			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}
	
	/*下面的代码展示查找到给定的searchPoint的某一半径（随机产生）内所有近邻，重新定义两个向量
	pointIdxRadiusSearch  pointRadiusSquaredDistance来存储关于近邻的信息
	********************************************************************************/
	 
	//半径R内近邻搜索方法
	std::vector<int> pointIdxRadiusSearch;           //存储近邻索引
	std::vector<float> pointRadiusSquaredDistance;   //存储近邻对应距离的平方

	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);   //随机的生成某一半径
	//打印输出
	std::cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << std::endl;


	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)  //执行半径R内近邻搜索方法
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
		{
			std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
				<< " " << cloud->points[pointIdxRadiusSearch[i]].y
				<< " " << cloud->points[pointIdxRadiusSearch[i]].z
				<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
		}
	}
}
			

