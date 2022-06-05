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
	/*
	Build the sons of an octnode
	*/
	void BuildSons()
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
};



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
	
	/*
	Transform all the vertexs and normals to the camera coordinate
	Args:
		camera [Camera]: [the camera]
	*/
	void CoordinateTransform(Camera& camera)
	{
		for (int i = 0; i < this->faces.size(); i++)
		{
			this->faces[i].CoordinateTransform(camera);
		}
	}

	/*
	Build the bounding box of the object model
	Returns:
		bounding_box [BoundingBox]: [the bounding box of the object model]
	*/
	BoundingBox BuildBoundingBox()
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
};


/*
Definition of light and phong model
*/
class Light
{
public:
	Vector3d ambient;
	Vector3d diffuse;
	Vector3d specular;
	Vector3d direction;
	Light()
	{
		ambient << 1.0, 1.0, 1.0;
		diffuse << 1.0, 1.0, 1.0;
		specular << 1.0, 1.0, 1.0;
		direction << 0, 0, 1;
	}
	
	/*
	Transform the light to the camera coordinate
	Args:
		camera [Camera]: [the camera]
	*/
	void CoordinateTransform(Camera& camera)
	{
		this->direction = camera.rotation * this->direction;
	}

	/*
	Get the ambient light on a vertex of a triangle mesh
	Args:
		color [Vector3d]: [the color of the vertex to be lighted]
	Returns:
		ambient [Vector3d]: [the RGB result, between[0, 1)]
	*/
	Vector3d GetAmbient(Vector3d& color)
	{
		Vector3d ambient;
		double r = color(0) * this->ambient(0);
		double g = color(1) * this->ambient(1);
		double b = color(2) * this->ambient(2);
		ambient << r, g, b;
		return ambient;
	}

	/*
	Get the diffuse light on a vertex of a triangle mesh
	Args:
		color [Vector3d]: [the color of the vertex to be lighted]
		norm [Vector3d]: [the norm of the vertex to be lighted]
	Returns:
		diffuse [Vector3d]: [the RGB result, between[0, 1)]
	*/
	Vector3d GetDiffuse(Vector3d& color, Vector3d& norm)
	{
		Vector3d diffuse;
		Vector3d n = norm / norm.norm();
		Vector3d l = this->direction / this->direction.norm();
		double weight = n.dot(l);
		if (weight <= 0)
		{
			weight = 0;
		}
		double r = color(0) * this->diffuse(0) * weight;
		double g = color(1) * this->diffuse(1) * weight;
		double b = color(2) * this->diffuse(2) * weight;
		diffuse << r, g, b;
		return diffuse;
	}

	/*
	Get the specular light on a vertex of a triangle mesh
	Args:
		color [Vector3d]: [the color of the vertex to be lighted]
		norm [Vector3d]: [the norm of the vertex to be lighted]
		ray [Ray]: [the seeing direction]
	Returns:
		specular [Vector3d]: [the RGB result, between[0, 1)]
	*/
	Vector3d GetSpecular(Vector3d& color, Vector3d& norm, Ray& ray)
	{
		int p = 10;
		Vector3d n = norm / norm.norm();
		Vector3d l = this->direction / this->direction.norm();
		Vector3d v = ray.direction / ray.direction.norm();
		Vector3d r = n * n.dot(l) * 2 - l;
		double rv = r.dot(v);
		if (rv <= 0)
		{
			rv = 0;
		}
		double weight = 1;
		for (int i = 1; i <= p; i++)
		{
			weight = weight * rv;
		}
		Vector3d specular;
		double r = color(0) * this->specular(0) * weight;
		double g = color(1) * this->specular(1) * weight;
		double b = color(2) * this->specular(2) * weight;
		specular << r, g, b;
		return specular;
	}

	/*
	Get the color of the intersection point of a mesh using the phong model
	Args:
		ray [Ray]: [the seeing direction]
		face [TriangleMesh]: [the mesh to be lighted]
		fraction [Vector3d]: [the fraction of the seeing point on the mesh]
	Returns:
		color [Vector3d]: [the result RGB color, between[0, 1)]
	*/
	Vector3d PhongModel(Ray& ray, TriangleMesh& face, Vector3d& fraction)
	{
		Vector3d color;
		color << 0, 0, 0;
		for (int i = 0; i < 3; i++)
		{
			Vector3d ambient = this->GetAmbient(face.colors[i]);
			Vector3d diffuse = this->GetDiffuse(face.colors[i], face.normals[i]);
			Vector3d specular = this->GetSpecular(face.colors[i], face.normals[i], ray);
			Vector3d the_color = ambient + diffuse + specular;
			color = color + the_color * fraction(i);
		}
		return color;
	}
};


