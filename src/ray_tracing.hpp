#pragma once
#include <string>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>
#include <float.h>
#include <Eigen\Dense>
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp>  
using namespace std;
using namespace Eigen;
using namespace cv;

//util functions
/*
Load an matrix file
Returns:
	matrix [double matrix], [4 * 4]: [the loaded extrinsic]
*/
Matrix4d LoadMatrix(const char* filename)
{
	Matrix4d matrix;
	ifstream infile;
	infile.open(filename, ios::in);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			double x;
			infile >> x;
			matrix(i, j) = x;
		}
	}
	infile.close();
	return matrix;
}

/*
Switch a string to int
Args:
	str [string]: [the string to be switched]
Returns:
	sum [int]: [the result int]
*/
int StringToInt(string str)
{
	int length = str.size();
	int int_base = 1;
	int sum = 0;
	for (int i = length - 1; i >= 0; i--)
	{
		sum += (str[i] - '0') * int_base;
		int_base *= 10;
	}
	return sum;
}


/*
Use opencv to save the result picture
Args:
	data [array of Vector3d], [H * W]: [the result data]
	save_place [const char*]: [the full saving place]
	width [int]: [the width of the picture]
	height [int]: [the height of the picture]
*/
void SavePicture(Vector3d* data, const char* save_place, int width, int height)
{
	Mat image = Mat::zeros(Size(width, height), CV_8UC3);
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				int color = int(data[i * width + j](k) * 256);
				if (color >= 256)
				{
					color = 255;
				}
				else if (color < 0)
				{
					color = 0;
				}
				uchar ucolor = uchar(color);
				image.at<Vec3b>(i, j)[2 - k] = ucolor;
			}
		}
	}
	imwrite(save_place, image);
}


class TriangleMesh
{
public:
	Vector3d vertexs[3];
	Vector3d normals[3];
	Vector3d colors[3];
	Vector3d normal;
	int vertex_ids[3];
	int id;
	TriangleMesh() {}

	/*
	Init a triangle mesh
	Args:
		id [int]: [the id of the mesh, starting from 0]
		id_a [int]: [the id of the the first vertex, starting from 0]
		id_b [int]: [the id of the the second vertex, starting from 0]
		id_c [int]: [the id of the the third vertex, starting from 0]
		place_a [Vector3d]: [the place of the the first vertex]
		place_b [Vector3d]: [the place of the the second vertex]
		place_c [Vector3d]: [the place of the the third vertex]
		norm_a [Vector3d]: [the normal of the the first vertex, [-1, 1]]
		norm_b [Vector3d]: [the normal of the the second vertex, [-1, 1]]
		norm_c [Vector3d]: [the normal of the the third vertex, [-1, 1]]
		color_a [Vector3d]: [the color of the the first vertex, [0, 1)]
		color_b [Vector3d]: [the color of the the second vertex, [0, 1)]
		color_c [Vector3d]: [the color of the the third vertex, [0, 1)]
	*/
	TriangleMesh(int id, int id_a, int id_b, int id_c, Vector3d& place_a, Vector3d& place_b, Vector3d& place_c,
		Vector3d& norm_a, Vector3d& norm_b, Vector3d& norm_c, Vector3d& color_a, Vector3d& color_b, Vector3d& color_c)
	{
		this->id = id;
		this->vertex_ids[0] = id_a;
		this->vertex_ids[1] = id_b;
		this->vertex_ids[2] = id_c;
		this->vertexs[0] = place_a;
		this->vertexs[1] = place_b;
		this->vertexs[2] = place_c;
		this->normals[0] = norm_a;
		this->normals[1] = norm_b;
		this->normals[2] = norm_c;
		this->colors[0] = color_a;
		this->colors[1] = color_b;
		this->colors[2] = color_c;
		this->normal = (this->normals[0] + this->normals[1] + this->normals[2]) / 3;
		this->normal = this->normal / this->normal.norm();
	}

	/*
	Transform all the vertexs and normals to the camera coordinate
	Args:
		camera [Camera]: [the camera]
	*/
	void CoordinateTransform(Camera& camera)
	{
		for (int i = 0; i < 3; i++)
		{
			this->vertexs[i] = camera.rotation * this->vertexs[i] + camera.translation; //first rotation, then translation
			this->normals[i] = camera.rotation * this->normals[i];
		}
	}
};

//TODO:OBJMESH and texture
/*
Read a ply mesh
Args:
	filename [char*]: [the filename]
	size [double]: [the new size of the triangle mesh]
	center [Vector3d]: [the new center of the triangle mesh]
Returns:
	faces [vector<TriangleMesh>]: [the faces]
*/
vector<TriangleMesh> ReadPLYMesh(char* filename, double size, Vector3d center, Vector3d color)
{
	vector<Vector3d> vertexs;
	vector<Vector3d> normals;
	vector<Vector3d> colors;
	vector<TriangleMesh> faces;
	vertexs.clear();
	normals.clear();
	colors.clear();
	faces.clear();
	int vertex_num;
	int face_num;

	ifstream infile;
	infile.open(filename, ios::in);
	while (1)
	{
		string line;
		string vertex_line = "element vertex ";
		string face_line = "element face ";
		string end_line = "end_header";
		getline(infile, line);
		string substr_15 = line.substr(0, 15);
		string substr_13 = line.substr(0, 13);
		if (substr_15 == vertex_line)
		{
			string vertex_num_str = line.substr(15);
			vertex_num = StringToInt(vertex_num_str);
		}
		else if (substr_13 == face_line)
		{
			string face_num_str = line.substr(13);
			face_num = StringToInt(face_num_str);
		}
		else if (line == end_line)
		{
			break;
		}
	}

	//read the vertexs
	for (int i = 0; i < vertex_num; i++)
	{
		double x, y, z;
		double nx, ny, nz;
		double r, g, b, a;
		infile >> x >> y >> z;
		infile >> nx >> ny >> nz;
		//infile >> r >> g >> b >> a;
		//r = r / 256;
		//g = g / 256;
		//b = b / 256;
		Vector3d new_vertex;
		new_vertex << x, y, z;
		Vector3d new_norm;
		new_norm << nx, ny, nz;
		//Vector3d new_color;
		//new_color << r, g, b;
		vertexs.push_back(new_vertex);
		normals.push_back(new_norm);
		colors.push_back(color);
		//colors.push_back(new_color);
	}

	//normalize the vertexs to the center
	Vector3d mean;
	mean << 0, 0, 0;
	double square_error = 0;
	for (int i = 0; i < vertex_num; i++)
	{
		mean = mean + vertexs[i];
	}
	mean = mean / double(vertex_num);
	for (int i = 0; i < vertex_num; i++)
	{
		Vector3d dist = mean - vertexs[i];
		double error = dist.dot(dist);
		square_error += error;
	}
	double std_error = sqrt(square_error);
	for (int i = 0; i < vertex_num; i++)
	{
		Vector3d p = vertexs[i];
		p = p - mean;
		p = p / std_error;
		p = p * size;
		p = p + center;
		vertexs[i] = p;
	}

	//read and store the faces
	for (int i = 0; i < face_num; i++)
	{
		int n, id_a, id_b, id_c;
		infile >> n >> id_a >> id_b >> id_c;
		Vector3d place_a = vertexs[id_a];
		Vector3d place_b = vertexs[id_b];
		Vector3d place_c = vertexs[id_c];
		Vector3d norm_a = normals[id_a];
		Vector3d norm_b = normals[id_b];
		Vector3d norm_c = normals[id_c];
		Vector3d color_a = colors[id_a];
		Vector3d color_b = colors[id_b];
		Vector3d color_c = colors[id_c];
		TriangleMesh new_face = TriangleMesh(i, id_a, id_b, id_c, place_a, place_b, place_c,
			norm_a, norm_b, norm_c, color_a, color_b, color_c);
		faces.push_back(new_face);

	}
	vertexs.clear();
	normals.clear();
	colors.clear();
	infile.close();
	return faces;
}



//TODO: rotation and translation
class Camera
{
public:
	Matrix3d rotation;
	Vector3d translation;
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
		refraction [double]: [the refraction rate of the ray]
	*/
	Ray(Vector3d start, Vector3d direction, double intensity, double refraction)
	{
		this->start = start;
		this->direction = direction;
		this->intensity = intensity;
		this->refraction_rate = refraction;
	}
};

class BoundingBox
{
public:
	double min_x = 0;
	double min_y = 0;
	double min_z = 0;
	double max_x = 0;
	double max_y = 0;
	double max_z = 0;
	BoundingBox() {}
	/*
	Init a bounding box
	Args:
		min_x [double]: [the min x of the object]
		min_y [double]: [the min y of the object]
		min_z [double]: [the min z of the object]
		max_x [double]: [the max x of the object]
		max_y [double]: [the max y of the object]
		max_z [double]: [the max z of the object]
	*/
	BoundingBox(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z)
	{
		this->min_x = min_x;
		this->min_y = min_y;
		this->min_z = min_z;
		this->max_x = max_x;
		this->max_y = max_y;
		this->max_z = max_z;
	}

	void Set(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z)
	{
		this->min_x = min_x;
		this->min_y = min_y;
		this->min_z = min_z;
		this->max_x = max_x;
		this->max_y = max_y;
		this->max_z = max_z;
	}
};

//computer geometry functions
/*
Judge whether a face is inside a bounding box
Args:
	face [TriangleMesh]: [the face]
	box [BoundingBox]: [the bounding box]
Returns:
	result [bool]: [whether inside or not]
*/
bool JudgeFaceInsideBox(TriangleMesh& face, BoundingBox& box)
{
	bool result = 0;
	for (int i = 0; i < 3; i++)
	{
		double x = face.vertexs[i](0);
		double y = face.vertexs[i](1);
		double z = face.vertexs[i](2);
		if (x >= box.min_x && x <= box.max_x && y >= box.min_y && y <= box.max_y && z >= box.min_z && z <= box.max_z)
		{
			result = 1;
			break;
		}
	}
	return result;
}

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
	double k_reflection;
	double k_refraction;
	double refraction_rate;

	ObjectModel(vector<TriangleMesh>& faces, double reflection, double refraction, double refraction_rate)
	{
		this->faces = faces;
		BoundingBox bounding_box = this->BuildBoundingBox();
		this->root = new OctNode(1, bounding_box, this->faces);
		this->k_reflection = reflection;
		this->k_refraction = refraction;
		this->refraction_rate = refraction_rate;
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

//the main class of ray tracing
class RayTracing
{
public:
	vector<ObjectModel> objects;
	Camera camera;
	Light light;
	const double threshold = 0.01;
	const int max_depth = 6;
	
	RayTracing(){}
	RayTracing(vector<ObjectModel>& objects, Camera& camera, Light& light)
	{
		this->objects = objects;
		this->camera = camera;
		this->light = light;
	}

	/*
	Recursively trace one ray
	Args:
		ray [Ray]: [the ray to be traced]
		depth [int]: [current depth]
	Returns:
		color [Vector3d]: [result color of the ray]
	*/
	Vector3d TraceOneRay(Ray& ray, int depth)
	{
		Vector3d color;
		color << 0, 0, 0;
		if (depth > this->max_depth || ray.intensity <= this->threshold)
		{
			return color;
		}

		//get intersection results with all the models
		double best_t = DBL_MAX;
		int best_mesh_id = -1;
		Vector3d best_fraction;
		int best_i = -1;
		for (int i = 0; i < this->objects.size(); i++)
		{
			double t;
			int mesh_id;
			Vector3d fraction;
			this->objects[i].GetIntersection(ray, mesh_id, t, fraction);
			if (t > 0 || t < best_t)
			{
				best_t = t;
				best_mesh_id = mesh_id;
				best_fraction = fraction;
				best_i = i;
			}
		}
		if (best_i < 0)
		{
			return color;
		}

		//get the refraction and reflections, recursively get the results
		TriangleMesh final_mesh = this->objects[best_i].faces[best_mesh_id];
		color = this->light.PhongModel(ray, final_mesh, best_fraction);
		double k_reflection = this->objects[best_i].k_reflection;
		double k_refraction = this->objects[best_i].k_refraction;
		Ray reflection = this->objects[best_i].GetReflection(ray, best_mesh_id, best_t);
		Ray refraction = this->objects[best_i].GetRefraction(ray, best_mesh_id, best_t);
		Vector3d color_reflection = this->TraceOneRay(reflection, depth + 1);
		Vector3d color_refraction = this->TraceOneRay(refraction, depth + 1);
		color = color + color_reflection * k_reflection + color_refraction * k_refraction;
		return color;
	}

	/*
	The main function of ray tracing
	*/
	void Main()
	{

	}





};