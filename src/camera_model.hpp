//definition of the camera model and rays
#pragma once
#include <math.h>
#define _USE_MATH_DEFINES
#include "utils.hpp"
using namespace std;
using namespace Eigen;

class Ray
{
public:
	Vector3d start;
	Vector3d direction;
	double intensity;
	double refraction_rate;
	Ray() {}

	/*
	Init a ray
	Args:
		start [Vector3d]: [the start point of the ray]
		direction [Vector3d]: [the direction of the ray]
		intensity [double]: [the intensity of the ray]
		refraction [double]: [the refraction rate of the current ray]
	*/
	Ray(Vector3d start, Vector3d direction, double intensity, double refraction)
	{
		this->start = start;
		this->direction = direction;
		this->intensity = intensity;
		this->refraction_rate = refraction;
	}
};

//TODO: rotation and translation
class Camera
{
public:
	//the definition of the unit ball of the camera
	double r;
	double theta;
	double phi;
	const double min_r = 2;
	const double max_r = 20;
	const double min_theta = 10 / 180 * M_PI;
	const double max_theta = 0.5 * M_PI;
	
	//extrinsics
	Matrix3d rotation;
	Vector3d camera_position;
	Vector3d look_center;

	//intrinsics
	int width;
	int height;
	double cx;
	double cy;
	double fx;
	double fy;
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
		this->theta = theta;
		this->phi = phi;
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
						  x3, y3, z3; 
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

	Ray new_ray = Ray(start, direction, 1.0, 1.0);
	return new_ray;
}
