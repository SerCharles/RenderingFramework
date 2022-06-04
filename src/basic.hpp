#pragma once
#include <Eigen\Dense>
#include <string>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>
#include <float.h>
using namespace std;
using namespace Eigen;


/*
Load an matrix file
Returns:
	matrix [double matrix], [4 * 4]: [the loaded extrinsic]
*/
Matrix<double, 4, 4> LoadMatrix(const char* filename)
{
	Matrix<double, 4, 4> matrix;
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

//TODO: rotation and translation
class Camera
{
public:
	Matrix<double, 4, 4> extrinsic;
	Matrix<double, 4, 4> intrinsic;
	Camera()
	{

	}
	Camera(const char* intrinsic_name, const char* extrinsic_name)
	{
		this->extrinsic = LoadMatrix(extrinsic_name);
		this->intrinsic = LoadMatrix(intrinsic_name);
	}
};

class TriangleMesh
{
public:
	Vector3d vertexs[3];
	Vector3d normals[3];
	Vector3d colors[3];
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
		color_a [Vector3d]: [the color of the the first vertex, [0, 256)]
		color_b [Vector3d]: [the color of the the second vertex, [0, 256)]
		color_c [Vector3d]: [the color of the the third vertex, [0, 256)]
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
	}
};

//TODO:OBJMESH and texture
/*
Read a ply mesh
Args:
	filename [char*]: [the filename]
Returns:
	faces [vector<TriangleMesh>]: [the faces]
*/
vector<TriangleMesh> ReadPLYMesh(char* filename)
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
			string vertex_num = line.substr(15);
			vertex_num = StringToInt(vertex_num);
		}
		else if (substr_13 == face_line)
		{
			string face_num = line.substr(13);
			face_num = StringToInt(face_num);
		}
		else if (line == end_line)
		{
			break;
		}
	}
	for (int i = 0; i < vertex_num; i++)
	{
		double x, y, z;
		double nx, ny, nz;
		double r, g, b, a;
		infile >> x >> y >> z;
		infile >> nx >> ny >> nz;
		infile >> r >> g >> b >> a;
		Vector3d new_vertex;
		new_vertex << x, y, z;
		Vector3d new_norm;
		new_norm << nx, ny, nz;
		Vector3d new_color;
		new_color << r, g, b;
		vertexs.push_back(new_vertex);
		colors.push_back(new_color);
	}
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

class Ray
{
public:
	Vector3d start;
	Vector3d direction;
	double intensity;
	double refraction_rate;
	Ray(){}
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