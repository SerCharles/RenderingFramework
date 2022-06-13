//definition of the camera model and rays
#pragma once
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
	//extrinsics
	Matrix3d rotation;
	Vector3d translation;
	Matrix4d camera_to_world;
	Matrix4d world_to_camera;

	//position and look direction of camera
	Vector3d camera_position;
	Vector3d look_center;
	Vector3d x_axis;
	Vector3d y_axis;
	Vector3d z_axis;
	double cx;
	double cy;
	double fx;
	double fy;
	int width;
	int height;

	Camera()
	{

	}
	Camera(const char* intrinsic_name, const char* extrinsic_name)
	{
		Matrix4d extrinsic = LoadMatrix(extrinsic_name);
		this->rotation = extrinsic.block<3, 3>(0, 0);
		this->translation = extrinsic.block<3, 1>(0, 3);
		Matrix4d intrinsic = LoadMatrix(intrinsic_name);
		this->fx = intrinsic(0, 0);
		this->fy = intrinsic(1, 1);
		this->cx = intrinsic(0, 2);
		this->cy = intrinsic(1, 2);
	}
};
