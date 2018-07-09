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
#include <fstream>				
#include <iostream>		
#include <math.h>  
#include <vector>
#include <ctime>
#include <time.h>

//pcl
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>        //点类型定义头文件
#include <pcl/kdtree/kdtree_flann.h> //kdtree类定义头文件

//opengl
#include <gl/glew.h>
#include <gl/freeglut.h>

#include "lasIO.h"
#include "camera.h"
 
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


std::string lasfile = "C:/Users/Y583/Desktop/激光雷达数据处理软件开发/斯坦福兔子和德国某体育场的点云文件/lidar.las";
std::string pcdfile = "C:/Users/Y583/Desktop/激光雷达数据处理软件开发/mesh.ply";


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
	liblas::Reader  reader = f.CreateWithStream(ifs);
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
		 
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(pcdfile, pcloud) == -1)
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

int main(int argc, char *argv[])
{
	lasRead();
	ReadPointCloud();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_DOUBLE | GLUT_RGBA );
	glutInitWindowSize(640,480);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("View");
	glutReshapeFunc(reshape);
	glutSpecialFunc(SpecialKeys);
	glutMotionFunc(motion);
	glutMouseFunc(mousedown);
	glutKeyboardFunc(KeyboardKeys);
	glutDisplayFunc(&GLdisplay);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glutMainLoop();

	return 0;
	
	
	////创建KdTreeFLANN对象，并把创建的点云设置为输入,创建一个searchPoint变量作为查询点
	//pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	////设置搜索空间
	//kdtree.setInputCloud(cloud);

	////设置查询点并赋随机值
	//pcl::PointXYZ searchPoint;
	//searchPoint.x = 1024.0f + rand() / (RAND_MAX + 1.0f);
	//searchPoint.y = 1024.0f + rand() / (RAND_MAX + 1.0f);
	//searchPoint.z = 1024.0f + rand() / (RAND_MAX + 1.0f);

	////K 临近搜索
	////创建一个整数（设置为10）和两个向量来存储搜索到的K近邻，两个向量中，一个存储搜索到查询点近邻的索引，另一个存储对应近邻的距离平方
	//int k = 10;

	//std::vector<int> pointIdxNKNSearch(k);
	//std::vector<float> pointNKNSquaredDistance(k); //存储近邻点对应距离平方

	////打印相关信息
	//std::cout << "K nearest neighbor search at (" << searchPoint.x
	//	<< " " << searchPoint.y
	//	<< " " << searchPoint.z
	//	<< ") with k=" << k << std::endl;

	//if (kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)  //执行K近邻搜索
	//{
	//	//打印所有近邻坐标
	//	for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
	//		std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
	//		<< " " << cloud->points[pointIdxNKNSearch[i]].y
	//		<< " " << cloud->points[pointIdxNKNSearch[i]].z
	//		<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	//}
	
	//下面的代码展示查找到给定的searchPoint的某一半径（随机产生）内所有近邻，重新定义两个向量
	//pointIdxRadiusSearch  pointRadiusSquaredDistance来存储关于近邻的信息
	//********************************************************************************/
	// //半径 R内近邻搜索方法

	//std::vector<int> pointIdxRadiusSearch;           //存储近邻索引
	//std::vector<float> pointRadiusSquaredDistance;   //存储近邻对应距离的平方

	//float radius = 256.0f * rand() / (RAND_MAX + 1.0f);   //随机的生成某一半径
	////打印输出
	//std::cout << "Neighbors within radius search at (" << searchPoint.x
	//	<< " " << searchPoint.y
	//	<< " " << searchPoint.z
	//	<< ") with radius=" << radius << std::endl;


	//if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)  //执行半径R内近邻搜索方法
	//{
	//	for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
	//		std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
	//		<< " " << cloud->points[pointIdxRadiusSearch[i]].y
	//		<< " " << cloud->points[pointIdxRadiusSearch[i]].z
	//		<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	//}

}
			

