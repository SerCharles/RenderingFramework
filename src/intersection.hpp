//the computer geometry functions, mainly in getting the intersections between points
#pragma once


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
	face [TriangleMesh]: [the mesh to be intersected]
	ray [Ray]: [the ray to be intersected]
	t [double]: [the intersecting t of the ray, -1 if empty]
	fraction [Vector3d]: [the fraction of the intersection point to the mesh, used in getting the final color]
*/
void GetIntersectionMesh(TriangleMesh& face, Ray& ray, double& t, Vector3d& fraction)
{
	Vector3d o = ray.start;
	Vector3d d = ray.direction;
	Vector3d p0 = face.vertexs[0];
	Vector3d p1 = face.vertexs[1];
	Vector3d p2 = face.vertexs[2];
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
Judge whether a ray intersects with the oct node
Args:
	ray [Ray]: [the ray]
Returns:
	result [bool]: [whether intersect or not]
*/
bool JudgeIntersection(Ray& ray)
{
	double t_x_min = -1, t_x_max = -1, t_y_min = -1, t_y_max = -1, t_z_min = -1, t_z_max = -1;
	if (ray.direction(0) != 0)
	{
		t_x_min = (this->bounding_box.min_x - ray.start(0)) / ray.direction(0);
		t_x_max = (this->bounding_box.max_x - ray.start(0)) / ray.direction(0);
		Vector3d p_x_min = ray.start + ray.direction * t_x_min;
		Vector3d p_x_max = ray.start + ray.direction * t_x_max;
		if (t_x_min <= 0 ||
			JudgePointInsideRectangle(p_x_min(1), p_x_min(2), this->bounding_box.min_y, this->bounding_box.min_z, this->bounding_box.max_y, this->bounding_box.max_z) == 0)
		{
			t_x_min = -1;
		}
		if (t_x_max <= 0 ||
			JudgePointInsideRectangle(p_x_max(1), p_x_max(2), this->bounding_box.min_y, this->bounding_box.min_z, this->bounding_box.max_y, this->bounding_box.max_z) == 0)
		{
			t_x_max = -1;
		}
	}

	if (ray.direction(1) != 0)
	{
		t_y_min = (this->bounding_box.min_y - ray.start(1)) / ray.direction(1);
		t_y_max = (this->bounding_box.max_y - ray.start(1)) / ray.direction(1);
		Vector3d p_y_min = ray.start + ray.direction * t_y_min;
		Vector3d p_y_max = ray.start + ray.direction * t_y_max;
		if (t_y_min <= 0 ||
			JudgePointInsideRectangle(p_y_min(0), p_y_min(2), this->bounding_box.min_x, this->bounding_box.min_z, this->bounding_box.max_x, this->bounding_box.max_z) == 0)
		{
			t_y_min = -1;
		}
		if (t_y_max <= 0 ||
			JudgePointInsideRectangle(p_y_max(0), p_y_max(2), this->bounding_box.min_x, this->bounding_box.min_z, this->bounding_box.max_x, this->bounding_box.max_z) == 0)
		{
			t_y_max = -1;
		}
	}

	if (ray.direction(2) != 0)
	{
		t_z_min = (this->bounding_box.min_z - ray.start(2)) / ray.direction(2);
		t_z_max = (this->bounding_box.max_z - ray.start(2)) / ray.direction(2);
		Vector3d p_z_min = ray.start + ray.direction * t_z_min;
		Vector3d p_z_max = ray.start + ray.direction * t_z_max;
		if (t_z_min <= 0 ||
			JudgePointInsideRectangle(p_z_min(0), p_z_min(1), this->bounding_box.min_x, this->bounding_box.min_y, this->bounding_box.max_x, this->bounding_box.max_y) == 0)
		{
			t_z_min = -1;
		}
		if (t_z_max <= 0 ||
			JudgePointInsideRectangle(p_z_max(0), p_z_max(1), this->bounding_box.min_x, this->bounding_box.min_y, this->bounding_box.max_x, this->bounding_box.max_y) == 0)
		{
			t_z_max = -1;
		}
	}

	vector<float> t_list;
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
Get all possible mesh that intersects with the ray recursively
Args:
	mesh_list [vector<TriangleMesh>]: [the possible intersecting mesh to be judged]
	ray [Ray]: [the ray to be judged]
*/
void GetPossibleMesh(vector<TriangleMesh>& mesh_list, Ray& ray)
{
	bool intersect = this->JudgeIntersection(ray);
	if (intersect == 0)
	{
		return;
	}
	if (this->sons[0] != NULL)
	{
		for (int i = 0; i < 8; i++)
		{
			this->sons[i]->GetPossibleMesh(mesh_list, ray);
		}
	}
	else
	{
		for (int i = 0; i < this->faces.size(); i++)
		{
			mesh_list.push_back(this->faces[i]);
		}
	}
}


/*
Get the intersection point of a ray with the mesh model
Args:
	ray [Ray]: [the ray to intersect]
	i [int]: [the id of the first intersection mesh in the object model, -1 if nothing]
	t [double]: [the t of the ray to be traveled, -1 if nothing]
	fraction [Vector3d]: [the fraction of the intersection point to the mesh, used in getting the final color]
*/
void GetIntersection(Ray& ray, int& i, double& t, Vector3d& fraction)
{
	vector<TriangleMesh> mesh_list;
	mesh_list.clear();
	this->root->GetPossibleMesh(mesh_list, ray);
	t = DBL_MAX;
	i = -1;
	for (int i = 0; i < mesh_list.size(); i++)
	{
		double the_t = -1;
		Vector3d the_fraction;
		GetIntersectionMesh(mesh_list[i], ray, the_t, the_fraction);
		if (the_t > 0 && the_t < t)
		{
			t = the_t;
			i = mesh_list[i].id;
			fraction = the_fraction;
		}
	}
	if (i == -1)
	{
		t = -1;
	}
}

/*
Get the reflection ray after getting intersection
Args:
	ray [Ray]: [the ray to intersect]
	i [int]: [the id of the first intersection mesh in the object model]
	t [double]: [the t of the ray to be traveled]
Returns:
	new_ray [Ray]: [the new reflection ray]
*/
Ray GetReflection(Ray& ray, int i, double t)
{
	Vector3d intersection_point = ray.start + ray.direction * t;
	double dist = sqrt(ray.direction.dot(this->faces[i].normal));
	Vector3d normal_speed = this->faces[i].normal * dist;
	Vector3d tangent_speed = ray.direction - normal_speed;
	Vector3d new_direction = tangent_speed - normal_speed;
	double k = this->k_reflection;
	double new_intensity = ray.intensity * k;
	Ray new_ray(intersection_point, new_direction, new_intensity, this->refraction_rate);
	return new_ray;
}

/*
Get the refraction ray after getting intersection
Args:
	ray [Ray]: [the ray to intersect]
	i [int]: [the id of the first intersection mesh in the object model]
	t [double]: [the t of the ray to be traveled]
Returns:
	new_ray [Ray]: [the new refraction ray]
*/
Ray GetRefraction(Ray& ray, int i, float t)
{
	Vector3d intersection_point = ray.start + ray.direction * t;

	double dist = sqrt(ray.direction.dot(this->faces[i].normal));
	Vector3d normal_speed = this->faces[i].normal * dist;
	Vector3d tangent_speed = ray.direction - normal_speed;

	double x = tangent_speed.norm();
	double y = normal_speed.norm();
	double n1 = ray.refraction_rate;
	double n2 = this->refraction_rate;
	double kx = n2 * y / sqrt(n1 * n1 * (x * x + y * y) - n2 * n2 * x * x);
	Vector3d new_speed = normal_speed + tangent_speed * kx;
	new_speed = new_speed / new_speed.norm();
	double k = this->k_refraction;
	double new_intensity = ray.intensity * k;

	Ray new_ray(intersection_point, new_speed, new_intensity, n2);
	return new_ray;
}