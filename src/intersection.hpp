//the computer geometry functions, mainly in getting the intersections between points
#pragma once
#include "utils.hpp"
#include "mesh_model.hpp"
#include "camera_model.hpp"

/*
Judge whether a 2D point is inside a 2D rectangle
Args:
	p_x [double]: [the x of the point]
	p_y [double]: [the y of the point]
	min_x [double]: [the min x of the rectangle]
	min_y [double]: [the min y of the rectangle]
	max_x [double]: [the max x of the rectangle]
	max_y [double]: [the max y of the rectangle]
Returns:
	result [bool] : [whether the 2D point is inside the 2D rectangle or not]
*/
bool JudgePointInsideRectangle(double p_x, double p_y, double min_x, double min_y, double max_x, double max_y)
{
	bool result = 1;
	if (p_x < min_x || p_x > max_x)
	{
		result = 0;
	}
	if (p_y < min_y || p_y > max_y)
	{
		result = 0;
	}
	return result;
}

/*
Get the intersection point between a triangle mesh and a ray, using the Moller-Trumbore Algorithm
Args:
	ray [Ray]: [the ray to be intersected]
	face [TriangleMesh]: [the mesh to be intersected]
	t [double]: [the intersecting t of the ray, -1 if empty]
	fraction [Vector3d]: [the fraction of the intersection point to the mesh, used in getting the final color]
*/
void GetIntersectionRayMesh(Ray& ray, TriangleMesh& face, double& t, Vector3d& fraction)
{
	Vector3d o = ray.start;
	Vector3d d = ray.direction;
	Vector3d p0 = face.vertexs[0].point;
	Vector3d p1 = face.vertexs[1].point;
	Vector3d p2 = face.vertexs[2].point;
	Vector3d e1 = p1 - p0;
	Vector3d e2 = p2 - p0;
	Vector3d s = o - p0;
	Vector3d s1 = d.cross(e2);
	Vector3d s2 = s.cross(e1);
	double down = s1.dot(e1);
	if (down == 0)
	{
		t = -1;
		return;
	}
	t = s2.dot(e2) / down;
	double b1 = s1.dot(s) / down;
	double b2 = s2.dot(d) / down;
	double b0 = 1 - b1 - b2;
	if (t <= 0 || b0 < 0 || b0 > 1 || b1 < 0 || b1 > 1 || b2 < 0 || b2 > 1)
	{
		t = -1;
		return;
	}
	fraction << b0, b1, b2;
}

/*
Judge whether a ray intersects with a bounding box
Args:
	ray [Ray]: [the ray to be intersected]
	bounding_box [BoundingBox]: [the bounding box to be intersected]
Returns:
	result [bool]: [whether intersect or not]
*/
bool JudgeIntersectionRayBoundingBox(Ray& ray, BoundingBox& bounding_box)
{
	double t_x_min = -1, t_x_max = -1, t_y_min = -1, t_y_max = -1, t_z_min = -1, t_z_max = -1;
	if (ray.direction(0) != 0)
	{
		t_x_min = (bounding_box.min_x - ray.start(0)) / ray.direction(0);
		t_x_max = (bounding_box.max_x - ray.start(0)) / ray.direction(0);
		Vector3d p_x_min = ray.start + ray.direction * t_x_min;
		Vector3d p_x_max = ray.start + ray.direction * t_x_max;
		if (t_x_min <= 0 ||
			JudgePointInsideRectangle(p_x_min(1), p_x_min(2), bounding_box.min_y, bounding_box.min_z, bounding_box.max_y, bounding_box.max_z) == 0)
		{
			t_x_min = -1;
		}
		if (t_x_max <= 0 ||
			JudgePointInsideRectangle(p_x_max(1), p_x_max(2), bounding_box.min_y, bounding_box.min_z, bounding_box.max_y, bounding_box.max_z) == 0)
		{
			t_x_max = -1;
		}
	}

	if (ray.direction(1) != 0)
	{
		t_y_min = (bounding_box.min_y - ray.start(1)) / ray.direction(1);
		t_y_max = (bounding_box.max_y - ray.start(1)) / ray.direction(1);
		Vector3d p_y_min = ray.start + ray.direction * t_y_min;
		Vector3d p_y_max = ray.start + ray.direction * t_y_max;
		if (t_y_min <= 0 ||
			JudgePointInsideRectangle(p_y_min(0), p_y_min(2), bounding_box.min_x, bounding_box.min_z, bounding_box.max_x,bounding_box.max_z) == 0)
		{
			t_y_min = -1;
		}
		if (t_y_max <= 0 ||
			JudgePointInsideRectangle(p_y_max(0), p_y_max(2), bounding_box.min_x, bounding_box.min_z, bounding_box.max_x, bounding_box.max_z) == 0)
		{
			t_y_max = -1;
		}
	}

	if (ray.direction(2) != 0)
	{
		t_z_min = (bounding_box.min_z - ray.start(2)) / ray.direction(2);
		t_z_max = (bounding_box.max_z - ray.start(2)) / ray.direction(2);
		Vector3d p_z_min = ray.start + ray.direction * t_z_min;
		Vector3d p_z_max = ray.start + ray.direction * t_z_max;
		if (t_z_min <= 0 ||
			JudgePointInsideRectangle(p_z_min(0), p_z_min(1), bounding_box.min_x, bounding_box.min_y, bounding_box.max_x, bounding_box.max_y) == 0)
		{
			t_z_min = -1;
		}
		if (t_z_max <= 0 ||
			JudgePointInsideRectangle(p_z_max(0), p_z_max(1), bounding_box.min_x, bounding_box.min_y, bounding_box.max_x, bounding_box.max_y) == 0)
		{
			t_z_max = -1;
		}
	}

	vector<double> t_list;
	t_list.clear();
	t_list.push_back(t_x_min);
	t_list.push_back(t_x_max);
	t_list.push_back(t_y_min);
	t_list.push_back(t_y_max);
	t_list.push_back(t_z_min);
	t_list.push_back(t_z_max);
	bool result = 0;
	for (int i = 0; i < t_list.size(); i++)
	{
		if (t_list[i] > 0)
		{
			result = 1;
			break;
		}
	}
	return result;
}

/*
Get all the possible intersection meshes of an octree node and a ray, recursive function
Args:
	ray [Ray]: [the ray to be intersected]
	oct_node [OctNode*]: [the octreee node to be intersected]
	intersection_mesh_list [vector<TriangleMesh>]: [the possible intersecting mesh to be judged]
*/
void GetAllIntersectionRayOctNode( Ray& ray, OctNode* oct_node, vector<TriangleMesh>& intersection_mesh_list)
{
	bool intersect = JudgeIntersectionRayBoundingBox(ray, oct_node->bounding_box);
	if (intersect == 0)
	{
		return;
	}
	if (oct_node->sons[0] != NULL)
	{
		for (int i = 0; i < 8; i++)
		{
			GetAllIntersectionRayOctNode(ray, oct_node->sons[i], intersection_mesh_list);
		}
	}
	else
	{
		for (int i = 0; i < oct_node->faces.size(); i++)
		{
			intersection_mesh_list.push_back(oct_node->faces[i]);
		}
	}
}


/*
Get the intersection point of a ray with a mesh model
Args:
	ray [Ray]: [the ray to be intersected]
	mesh_model [MeshModel]: [the mesh model to be intersected]
	id [int]: [the id of the first intersection mesh in the object model, -1 if nothing]
	t [double]: [the t of the ray to be traveled, -1 if nothing]
	fraction [Vector3d]: [the fraction of the intersection point to the mesh, used in getting the final color]
*/
void GetIntersectionRayMeshModel(Ray& ray, MeshModel& mesh_model, int& id, double& t, Vector3d& fraction)
{
	vector<TriangleMesh> all_intersection_mesh_list;
	all_intersection_mesh_list.clear();
	GetAllIntersectionRayOctNode(ray, mesh_model.root, all_intersection_mesh_list);
	t = DBL_MAX;
	id = -1;
	for (int i = 0; i < all_intersection_mesh_list.size(); i++)
	{
		double the_t = -1;
		Vector3d the_fraction;
		GetIntersectionRayMesh(ray, all_intersection_mesh_list[i], the_t, the_fraction);
		if (the_t > 0 && the_t < t)
		{
			t = the_t;
			id = all_intersection_mesh_list[i].id;
			fraction = the_fraction;
		}
	}
	if (id == -1)
	{
		t = -1;
	}
}

/*
Get the local ray after getting intersection
Args:
	ray [Ray]: [the ray to be intersected]
	light_direction [Vector3d]: [the light direction id]
	t [double]: [the t of the ray to be traveled]
	last_object_id [int]: [the last met object id]
Returns:
	new_ray [Ray]: [the new reflection ray]
*/
Ray GetLocalRay(Ray& ray, Vector3d light_direction, double t, int last_object_id)
{
	Vector3d intersection_point = ray.start + ray.direction * t;
	Vector3d new_direction = -light_direction;
	Ray new_ray(intersection_point, new_direction, ray.intensity, TYPE_LOCAL, last_object_id);
	return new_ray;
}

/*
Get the reflection ray after getting intersection
Args:
	ray [Ray]: [the ray to be intersected]
	face [TriangleMesh]: [the face to be intersected]
	t [double]: [the t of the ray to be traveled]
	last_object_id [int]: [the last met object id] 
Returns:
	new_ray [Ray]: [the new reflection ray]
*/
Ray GetReflectionRay(Ray& ray, TriangleMesh& face, double t, int last_object_id)
{
	Vector3d normal_direction = face.normal; //out normal direction
	//the normal direction must be opposite the ray direction

	Vector3d intersection_point = ray.start + ray.direction * t; //the start is the intersection point
	double normal_speed = ray.direction.dot(-normal_direction); //the original normal speed of the ray
	Vector3d normal_velocity = -normal_direction * normal_speed; //the original normal velocity of the ray, should reverse
	Vector3d tangent_velocity = ray.direction - normal_velocity; //the tangent velocity of the ray, should not change
	Vector3d new_direction = tangent_velocity - normal_velocity; //the new direction of the ray
	new_direction = new_direction / new_direction.norm();


	double new_intensity = ray.intensity * face.k_reflection; //change the intensity
	Ray new_ray(intersection_point, new_direction, new_intensity, TYPE_REFLECTION, last_object_id);
	return new_ray;
}

/*
Get the refraction ray after getting intersection
Args:
	ray [Ray]: [the ray to be intersected]
	face [TriangleMesh]: [the face to be intersected]
	t [double]: [the t of the ray to be traveled]
	last_object_id [int]: [the id of the last met object]
Returns:
	new_ray [Ray]: [the new refraction ray]
*/
Ray GetRefractionRay(Ray& ray, TriangleMesh& face, double t, int last_object_id)
{
	Vector3d intersection_point = ray.start + ray.direction * t;
	double new_intensity = ray.intensity * face.k_refraction; //change the intensity
	Ray new_ray(intersection_point, ray.direction, new_intensity, TYPE_REFRACTION, last_object_id);
	return new_ray;
}