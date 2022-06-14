//definition of the camera model and rays
#pragma once
#include "utils.hpp"
using namespace std;
using namespace Eigen;
#define PI 3.1415926535

class Ray
{
public:
	Vector3d start;
	Vector3d direction;
	double intensity = 1.0;
	double refraction_rate = 1.0;
	bool inside = 0;
	Ray() {}

	/*
	Init a ray
	Args:
		start [Vector3d]: [the start point of the ray]
		direction [Vector3d]: [the direction of the ray]
		intensity [double]: [the intensity of the ray]
		refraction [double]: [the refraction rate of the current ray]
		inside [bool]: [whether the ray is inside a mesh model or not]
	*/
	Ray(Vector3d start, Vector3d direction, double intensity, double refraction, bool inside)
	{
		this->start = start;
		this->direction = direction;
		this->intensity = intensity;
		this->refraction_rate = refraction;
		this->inside = inside;
	}
};

//TODO: rotation and translation
class Camera
{
public:
	//the definition of the unit ball of the camera
	double r = 0;
	double theta = 0;
	double phi = 0;
	double min_r = 2;
	double max_r = 20;
	double min_theta = 10 / 180 * PI;
	double max_theta = 0.5 * PI;
	
	//extrinsics
	Matrix3d rotation;
	Vector3d camera_position;
	Vector3d look_center;

	//intrinsics
	int width = 0;
	int height = 0;
	double cx = 0;
	double cy = 0;
	double fx = 0;
	double fy = 0;

	Camera() {}

	/*
	Init the camera
	Args:
		size [int]: [the size of the picture, which equals to width and height, defining the intrinsics]
		r [double]: [the r of the unit sphere, defining the extrinsics]
		theta [double]: [the theta of the unit sphere, defining the extrinsics]
		phi [double]: [the phi of the unit sphere, defining the extrinsics]
	*/
	Camera(int size, double r, double theta, double phi)
	{
		//load intrinsics
		this->width = size;
		this->height = size;
		this->cx = size * 0.5;
		this->cy = size * 0.5;
		this->fx = size * 0.5;
		this->fy = size * 0.5;

		//load extrinsics
		this->r = r;
		if (this->r < this->min_r)
		{
			this->r = this->min_r;
		}
		if (r > this->max_r)
		{
			this->r = this->max_r;
		}
		this->theta = theta;
		if (this->theta < this->min_theta)
		{
			this->theta = this->min_theta;
		}
		if (this->theta > this->max_theta)
		{
			this->theta = this->max_theta;
		}
		this->phi = phi;
		if (this->phi < 0 || this->phi >= 2 * PI)
		{
			this->phi = 0;
		}
		double x = r * cos(theta) * cos(phi);
		double y = r * sin(theta);
		double z = r * cos(theta) * sin(phi);
		this->camera_position << x, y, z;
		this->look_center << 0, 0, 0;

		double x1 = -sin(phi);
		double x2 = 0;
		double x3 = cos(phi);
		double y1 = -sin(theta) * cos(phi);
		double y2 = cos(theta);
		double y3 = -sin(theta) * cos(phi);
		double z1 = -cos(theta) * cos(phi);
		double z2 = -sin(theta);
		double z3 = -cos(theta) * sin(phi);
		this->rotation << x1, y1, z1,
						  x2, y2, z2,
						  x3, y3, z3; //不知道行列存储的对不对qaq
	}
};

/*
Get the ray of a pixel at (u, v) in ray tracing
Args:
	camera [Camera]: [the camera model]
	u [int]: [the u of the pixel]
	v [int]: [the v of the pixel]
Returns:
	new_ray [Ray]: [the ray of the pixel in ray tracing]
*/
Ray GetPixelRay(Camera& camera, int u, int v)
{
	Vector3d start;
	start = camera.camera_position;

	double x = (double(u) - camera.cx) / camera.fx;
	double y = (double(v) - camera.cy) / camera.fy;
	double z = 1;
	Vector3d direction;
	direction << x, y, z;
	direction = direction / direction.norm();
	direction = camera.rotation * direction;

	Ray new_ray = Ray(start, direction, 1.0, 1.0, 0);
	return new_ray;
}
