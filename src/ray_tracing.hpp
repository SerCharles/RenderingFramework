#pragma once
#include "basic.hpp"
//The node of one octtree of an object
class OctNode
{
public:
	const int min_faces = 5;
	const int max_depth = 4;
	int depth;
	vector<TriangleMesh> faces;
	BoundingBox bounding_box;
	OctNode* sons[8] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };

	OctNode(int depth, BoundingBox bounding_box, vector<TriangleMesh>& faces)
	{
		this->depth = depth;
		this->bounding_box = bounding_box;
		this->faces = faces;
		if (depth < max_depth && faces.size() > min_faces)
		{
			this->BuildSons();
		}
	}
	void BuildSons();
	void GetPossibleMesh(vector<TriangleMesh>& mesh_list, Ray& ray);
	bool JudgeIntersection(Ray& ray);
};

/*
Build the sons of an octnode
*/
void OctNode::BuildSons()
{
	double min_x = this->bounding_box.min_x;
	double min_y = this->bounding_box.min_y;
	double min_z = this->bounding_box.min_z;
	double max_x = this->bounding_box.max_x;
	double max_y = this->bounding_box.max_y;
	double max_z = this->bounding_box.max_z;
	double mid_x = (min_x + max_x) / 2;
	double mid_y = (min_y + max_y) / 2;
	double mid_z = (min_z + max_z) / 2;
	BoundingBox bounding_boxes[8];
	bounding_boxes[0].Set(min_x, min_y, min_z, mid_x, mid_y, mid_z);
	bounding_boxes[1].Set(mid_x, min_y, min_z, max_x, mid_y, mid_z);
	bounding_boxes[2].Set(min_x, mid_y, min_z, mid_x, max_y, mid_z);
	bounding_boxes[3].Set(min_x, min_y, mid_z, mid_x, mid_y, max_z);
	bounding_boxes[4].Set(mid_x, mid_y, min_z, max_x, max_y, mid_z);
	bounding_boxes[5].Set(mid_x, min_y, mid_z, max_x, mid_y, max_z);
	bounding_boxes[6].Set(min_x, mid_y, mid_z, mid_x, max_y, max_z);
	bounding_boxes[7].Set(mid_x, mid_y, mid_z, max_x, max_y, max_z);
	for (int i = 0; i < 8; i++)
	{
		vector<TriangleMesh> faces;
		faces.clear();
		for (int j = 0; j < this->faces.size(); j++)
		{
			if (JudgeFaceInsideBox(this->faces[j], bounding_boxes[i]))
			{
				faces.push_back(this->faces[j]);
			}
		}
		this->sons[i] = new OctNode(this->depth + 1, bounding_boxes[i], faces);
		faces.clear();
	}
}

//The bounding box and octtree of an object
class ObjectModel
{
public:
	vector<TriangleMesh> faces;
	OctNode* root;

	ObjectModel(vector<TriangleMesh>& faces)
	{
		this->faces = faces;
		BoundingBox bounding_box = this->BuildBoundingBox();
		this->root = new OctNode(1, bounding_box, this->faces);
	}
	BoundingBox BuildBoundingBox();
	void GetIntersection(Ray& ray, int& i, double& t, Vector3d& fraction);
	
};

/*
Build the bounding box of the object model
Returns:
	bounding_box [BoundingBox]: [the bounding box of the object model]
*/
BoundingBox ObjectModel::BuildBoundingBox()
{
	double min_x = DBL_MAX;
	double min_y = DBL_MAX;
	double min_z = DBL_MAX;
	double max_x = DBL_MIN;
	double max_y = DBL_MIN;
	double max_z = DBL_MIN;
	for (int i = 0; i < this->faces.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			double x = this->faces[i].vertexs[j](0);
			double y = this->faces[i].vertexs[j](1);
			double z = this->faces[i].vertexs[j](2);
			if (x < min_x)
			{
				min_x = x;
			}
			if (x > max_x)
			{
				max_x = x;
			}
			if (y < min_y)
			{
				min_y = y;
			}
			if (y > max_y)
			{
				max_y = y;
			}
			if (z < min_z)
			{
				min_z = z;
			}
			if (z > max_z)
			{
				max_z = z;
			}
		}
	}
	BoundingBox bounding_box(min_x, min_y, min_z, max_x, max_y, max_z);
	return bounding_box;
}

/*
Judge whether a ray intersects with the oct node
Args:
	ray [Ray]: [the ray]
Returns:
	result [bool]: [whether intersect or not]
*/
bool OctNode::JudgeIntersection(Ray& ray)
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
void OctNode::GetPossibleMesh(vector<TriangleMesh>& mesh_list, Ray& ray)
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
	double t = s2.dot(e2) / down;
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
Get the intersection point of a ray with the mesh model
Args:
	ray [Ray]: [the ray to intersect]
	i [int]: [the id of the first intersection mesh in the object model, -1 if nothing]
	t [double]: [the t of the ray to be traveled, -1 if nothing]
	fraction [Vector3d]: [the fraction of the intersection point to the mesh, used in getting the final color]
*/
void ObjectModel::GetIntersection(Ray& ray, int& i, double& t, Vector3d& fraction)
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

