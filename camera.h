
class Camera 
{
public: 
	Camera();
	float camera_x, camera_y, camera_z; // �����λ������ 
	float lookat_x, lookat_y, lookat_z; // ������������� 
	void YawCamera(float fAngle); // yaw���� 
	void PitchCamera(float fAngle); // pitch���� 
	void WalkStraight(float fSpeed); // ǰ���ƶ����� 
	void WalkTransverse(float fSpeed); // �����ƶ����� 
	float angle; // ÿ����ת�Ƕ� 
	float speed; // ÿ���ƶ�����
	float sight; // ��Ұ 
	float rad_yz, rad_xz; // �����������yOzƽ�棬xOzƽ�����ɻ��� 
	float rotate_yz, rotate_xz; //�����������yOzƽ�棬xOzƽ�����ɽǶ� float PI;
	float PI;
};

Camera::Camera() // ������Ĺ��캯�� 
{ 
	PI = 3.1415; 
	angle = 3;
	speed = 0.3; 
	sight = 100; 
	rotate_yz = 0.0f; 
	rotate_xz = -90.0f;
	rad_yz = rotate_yz*PI / 180.0f; 
	rad_xz = rotate_xz*PI / 180.0f; 
	camera_x = 0.0f; 
	camera_y = 0.0f; 
	camera_z = 8.0f; 
	lookat_x = camera_x + sight*cos(rad_yz)*cos(rad_xz); 
	lookat_y = camera_y + sight*sin(rad_yz);
	lookat_z = camera_z + sight*cos(rad_yz)*sin(rad_xz);
}

void Camera::YawCamera(float fAngle) // yaw����ʵ�� 
{ 
	rotate_xz = (int)(rotate_xz + fAngle) % 360;
	rad_xz = rotate_xz*PI / 180.0f; 
	lookat_x = camera_x + sight*cos(rad_yz)*cos(rad_xz); 
	lookat_y = camera_y + sight*sin(rad_yz); 
	lookat_z = camera_z + sight*cos(rad_yz)*sin(rad_xz); 
}

void Camera::PitchCamera(float fAngle) // pitch����ʵ�� 
{ 
	rotate_yz = (int)(rotate_yz + fAngle) % 360; 
	rad_yz = rotate_yz*PI / 180.0f; 
	lookat_x = camera_x + sight*cos(rad_yz)*cos(rad_xz); 
	lookat_y = camera_y + sight*sin(rad_yz); 
	lookat_z = camera_z + sight*cos(rad_yz)*sin(rad_xz); 
}

void Camera::WalkStraight(float fSpeed) // ǰ���ƶ�����ʵ��
{ 
	camera_x += fSpeed*cos(rad_yz)*cos(rad_xz); 
	camera_y += fSpeed*sin(rad_yz); 
	camera_z += fSpeed*cos(rad_yz)*sin(rad_xz);
	lookat_x = camera_x + sight*cos(rad_yz)*cos(rad_xz);
	lookat_y = camera_y + sight*sin(rad_yz);
	lookat_z = camera_z + sight*cos(rad_yz)*sin(rad_xz);
}

void Camera::WalkTransverse(float fSpeed) // �����ƶ�����ʵ�� 
{ 
	camera_x += fSpeed*cos(rad_yz)*sin(rad_xz);
	camera_z -= fSpeed*cos(rad_yz)*cos(rad_xz);
	lookat_x = camera_x + sight*cos(rad_yz)*cos(rad_xz); 
	lookat_y = camera_y + sight*sin(rad_yz); 
	lookat_z = camera_z + sight*cos(rad_yz)*sin(rad_xz); 
}


//obj�ļ�
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/obj_io.h>
//#include <pcl/PolygonMesh.h>
////#include <pcl/ros/conversions.h>//formROSMsg����ͷ�ļ���
//#include <pcl/point_cloud.h>
//#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ����ͷ�ļ���
////#include <pcl/visualization/pcl_visualizer.h>
//
//using namespace std;
//using namespace pcl;
//int main()
//{
//	pcl::PolygonMesh mesh;
//	pcl::io::loadPolygonFile("K326-DD-REDUCE NOISE.obj", mesh);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
//	pcl::io::savePCDFileASCII("K326-DD-REDUCE NOISE.pcd", *cloud);
//
//	cout << cloud->size() << endl;
//
//	cout << "OK!";
//	cin.get();
//	return 0;
//}