//definition of the camera model and rays
#pragma once
#include "utils.hpp"
using namespace std;
using namespace Eigen;
#define PI 3.1415926535
#define TYPE_INIT 0 //init ray
#define TYPE_LOCAL 1 //ray used to generate shadow
#define TYPE_REFLECTION 2 //reflection ray
#define TYPE_REFRACTION 3 //refraction ray

class Ray
{
public:
	Vector3d start;
	Vector3d direction;
	double intensity = 1.0;
	int type = TYPE_INIT;
	int last_object_id = -1; //the id to not judge
	Ray() {}

	/*
	Init a ray
	Args:
		start [Vector3d]: [the start point of the ray]
		direction [Vector3d]: [the direction of the ray]
		intensity [double]: [the intensity of the ray]
		type [int]: [the type of the ray]
		last_object_id [int]: [the last object id to not be judged]
	*/
	Ray(Vector3d start, Vector3d direction, double intensity, int type, int last_object_id)
	{
		this->start = start;
		this->direction = direction;
		this->intensity = intensity;
		this->type = type;
		this->last_object_id = last_object_id;
	}
};

class Camera
{
public:
	//the definition of the unit ball of the camera
	double r = 0;
	double theta = 0;
	double phi = 0;
	double min_r = 5;
	double max_r = 30;
	double min_theta = 10.0 / 180.0 * PI;
	double max_theta = 170.0 / 180.0 * PI;

	//the definitions and variables used in changing views
	double speed_r = 1;
	double speed_theta = 0.002;
	double speed_phi = 0.002;
	double speed_translation = 1;
	int last_mouse_x = -1; //used in handling mousemove
	int last_mouse_y = -1; //used in handling mousemove
	bool mouse_down = 0; //used in handling mousemove
	
	
	//extrinsics
	Matrix3d rotation;
	Vector3d translation;
	Vector3d camera_position;

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
		this->theta = theta;
		this->phi = phi;
		this->translation << 0, 0, 0;
		this->ResetCameraPlace();


	}

	/*
	Reset the camera place and rotation matrix according to r, theta, phi
	*/
	void ResetCameraPlace()
	{
		if (this->r < this->min_r)
		{
			this->r = this->min_r;
		}
		else if (this->r > this->max_r)
		{
			this->r = this->max_r;
		}
		if (this->theta < this->min_theta)
		{
			this->theta = this->min_theta;
		}
		else if (this->theta > this->max_theta)
		{
			this->theta = this->max_theta;
		}
		while (this->phi < 0)
		{
			this->phi = this->phi + 2 * PI;
		}
		while (this->phi >= 2 * PI)
		{
			this->phi = this->phi - 2 * PI;
		}
		double x = r * cos(theta) * cos(phi);
		double y = r * sin(theta);
		double z = r * cos(theta) * sin(phi);
		this->camera_position << x, y, z;
		this->camera_position += this->translation;

		double x1 = -sin(phi);
		double x2 = 0;
		double x3 = cos(phi);
		double y1 = -sin(theta) * cos(phi);
		double y2 = cos(theta);
		double y3 = -sin(theta) * sin(phi);
		double z1 = -cos(theta) * cos(phi);
		double z2 = -sin(theta);
		double z3 = -cos(theta) * sin(phi);
		this->rotation << 
			x1, y1, z1,
			x2, y2, z2,
			x3, y3, z3;
	}

	/*
	Record the current mouse position when starting mouse move event
	Args:
		x [int]: [the current mouse x when beginning mouse move event]
		y [int]: [the current mouse y when beginning mouse move event]
	*/
	void MouseDown(int x, int y)
	{
		this->mouse_down = 1;
		this->last_mouse_x = x;
		this->last_mouse_y = y;
	}

	/*
	Set the mouse to be up when the mouse is up
	*/
	void MouseUp()
	{
		this->mouse_down = 0;
	}

	/*
	Handling mouse move event, used in rotating 
	Args:
		x [int]: [the current mouse x]
		y [int]: [the current mouse y]
	*/
	void MouseMove(int x, int y)
	{
		if (this->mouse_down)
		{
			int dx = x - this->last_mouse_x;
			int dy = y - this->last_mouse_y;
			this->phi = this->phi + dx * this->speed_phi;
			this->theta = this->theta - dy * this->speed_theta;
			this->ResetCameraPlace();
			this->last_mouse_x = x;
			this->last_mouse_y = y;
		}
	}

	/*
	Handling mouse wheel event, used in changing the radius
	Args:
		wheel_information [int]: [the moving place of the mouse, need to /120]
	*/
	void MouseWheel(int wheel_information)
	{
		double dr = -double(wheel_information / 120) * this->speed_r;
		this->r = this->r + dr;
		this->ResetCameraPlace();
	}

	/*
	Handling key event, used in translation
	Args:
		move_direction_camera [Vector3d]: [the moving direction of camera in the camera coordinate]
	*/
	void KeyUp(Vector3d move_direction_camera)
	{
		move_direction_camera = move_direction_camera * speed_translation;
		Vector3d move_direction_world = this->rotation * move_direction_camera;
		this->translation += move_direction_world;
		this->ResetCameraPlace();
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
	Ray new_ray = Ray(start, direction, 1.0, TYPE_INIT, -1);
	return new_ray;
}
