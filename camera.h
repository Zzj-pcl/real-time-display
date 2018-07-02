
class Camera 
{
public: 
	Camera();
	float camera_x, camera_y, camera_z; // 摄像机位置坐标 
	float lookat_x, lookat_y, lookat_z; // 摄像机朝向坐标 
	void YawCamera(float fAngle); // yaw方法 
	void PitchCamera(float fAngle); // pitch方法 
	void WalkStraight(float fSpeed); // 前后移动方法 
	void WalkTransverse(float fSpeed); // 左右移动方法 
	float angle; // 每次旋转角度 
	float speed; // 每次移动距离
	float sight; // 视野 
	float rad_yz, rad_xz; // 摄像机朝向与yOz平面，xOz平面所成弧度 
	float rotate_yz, rotate_xz; //摄像机朝向与yOz平面，xOz平面所成角度 float PI;
	float PI;
};

Camera::Camera() // 摄像机的构造函数 
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

void Camera::YawCamera(float fAngle) // yaw方法实现 
{ 
	rotate_xz = (int)(rotate_xz + fAngle) % 360;
	rad_xz = rotate_xz*PI / 180.0f; 
	lookat_x = camera_x + sight*cos(rad_yz)*cos(rad_xz); 
	lookat_y = camera_y + sight*sin(rad_yz); 
	lookat_z = camera_z + sight*cos(rad_yz)*sin(rad_xz); 
}

void Camera::PitchCamera(float fAngle) // pitch方法实现 
{ 
	rotate_yz = (int)(rotate_yz + fAngle) % 360; 
	rad_yz = rotate_yz*PI / 180.0f; 
	lookat_x = camera_x + sight*cos(rad_yz)*cos(rad_xz); 
	lookat_y = camera_y + sight*sin(rad_yz); 
	lookat_z = camera_z + sight*cos(rad_yz)*sin(rad_xz); 
}

void Camera::WalkStraight(float fSpeed) // 前后移动方法实现
{ 
	camera_x += fSpeed*cos(rad_yz)*cos(rad_xz); 
	camera_y += fSpeed*sin(rad_yz); 
	camera_z += fSpeed*cos(rad_yz)*sin(rad_xz);
	lookat_x = camera_x + sight*cos(rad_yz)*cos(rad_xz);
	lookat_y = camera_y + sight*sin(rad_yz);
	lookat_z = camera_z + sight*cos(rad_yz)*sin(rad_xz);
}

void Camera::WalkTransverse(float fSpeed) // 左右移动方法实现 
{ 
	camera_x += fSpeed*cos(rad_yz)*sin(rad_xz);
	camera_z -= fSpeed*cos(rad_yz)*cos(rad_xz);
	lookat_x = camera_x + sight*cos(rad_yz)*cos(rad_xz); 
	lookat_y = camera_y + sight*sin(rad_yz); 
	lookat_z = camera_z + sight*cos(rad_yz)*sin(rad_xz); 
}



