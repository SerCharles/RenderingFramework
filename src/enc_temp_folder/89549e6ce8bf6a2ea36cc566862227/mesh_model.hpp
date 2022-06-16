//definition of the vertexs, triangle face and mesh models
#pragma once
#include "utils.hpp"
using namespace std;
using namespace Eigen;

class Vertex
{
public:
	Vector3d point;
	Vector3d normal;
	Vector3d ambient;
	Vector3d diffuse;
	Vector3d specular;
	int id = -1;
	Vertex() {}
	Vertex(int id, Vector3d point, Vector3d normal, Vector3d ambient, Vector3d diffuse, Vector3d specular)
	{
		this->id = id;
		this->point = point;
		this->normal = normal / normal.norm();
		this->ambient = ambient;
		this->diffuse = diffuse;
		this->specular = specular;
	}
};

class TriangleMesh
{
public:
	Vertex vertexs[3];
	Vector3d normal;
	int id = -1;
	double k_reflection = 0;
	double k_refraction = 0;
	TriangleMesh() {}
	TriangleMesh(int id, Vertex& vertex_a, Vertex& vertex_b, Vertex& vertex_c, double k_reflection, double k_refraction)
	{
		this->id = id;
		this->vertexs[0] = vertex_a;
		this->vertexs[1] = vertex_b;
		this->vertexs[2] = vertex_c;
		this->normal = (this->vertexs[0].normal + this->vertexs[1].normal + this->vertexs[2].normal) / 3;
		this->normal = this->normal / this->normal.norm();
		this->k_reflection = k_reflection;
		this->k_refraction = k_refraction;
	}
};

/*
Read a ply mesh model
Args:
	filename [char*]: [the filename]
	size [double]: [the new size of the mesh model]
	center [Vector3d]: [the new center of the mesh model]
	ambient [Vector3d]: [the ambient weight of the mesh model]
	diffuse [Vector3d]: [the diffuse weight of the mesh model]
	specular [Vector3d]: [the specular weight of the mesh model]
	k_reflection [double]: [the reflection coefficient of the mesh model]
	k_refraction [double]: [the refraction coefficient of the mesh model]
Returns:
	faces [vector<TriangleMesh>]: [the faces]
*/
vector<TriangleMesh> ReadPLYMesh(char* filename, double size, Vector3d center, 
	Vector3d& ambient, Vector3d& diffuse, Vector3d& specular, double k_reflection, double k_refraction)
{
	
	vector<Vertex> vertexs;
	vector<TriangleMesh> faces;
	vertexs.clear();
	faces.clear();
	int vertex_num = 0;
	int face_num = 0;

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
	vector<Vector3d> points;
	vector<Vector3d> normals;
	points.clear();
	normals.clear();
	for (int i = 0; i < vertex_num; i++)
	{
		double x, y, z;
		double nx, ny, nz;
		infile >> x >> y >> z;
		infile >> nx >> ny >> nz;
		Vector3d new_point;
		new_point << x, y, z;
		Vector3d new_norm;
		new_norm << nx, ny, nz;
		points.push_back(new_point);
		normals.push_back(new_norm);
	}

	//normalize the vertexs to the center
	Vector3d mean;
	mean << 0, 0, 0;
	double square_error = 0;
	for (int i = 0; i < vertex_num; i++)
	{
		mean = mean + points[i];
	}
	mean = mean / double(vertex_num);
	for (int i = 0; i < vertex_num; i++)
	{
		Vector3d dist = mean - points[i];
		double error = dist.dot(dist);
		square_error += error;
	}
	square_error = square_error / double(vertex_num);
	double std_error = sqrt(square_error);
	for (int i = 0; i < vertex_num; i++)
	{
		Vector3d p = points[i];
		p = p - mean;
		p = p / std_error;
		p = p * size;
		p = p + center;
		points[i] = p;
	}

	//store the vertexs
	for (int i = 0; i < vertex_num; i++)
	{
		Vertex new_vertex = Vertex(i, points[i], normals[i], ambient, diffuse, specular);
		vertexs.push_back(new_vertex);
	}

	//read and store the faces
	for (int i = 0; i < face_num; i++)
	{
		int n, id_a, id_b, id_c;
		infile >> n >> id_a >> id_b >> id_c;
		
		TriangleMesh new_face = TriangleMesh(i, vertexs[id_a], vertexs[id_b], vertexs[id_c], k_reflection, k_refraction);
		faces.push_back(new_face);
	}
	vertexs.clear();
	normals.clear();
	points.clear();
	infile.close();
	return faces;
}

/*
Read a obj mesh model and texture mapping
Args:
	filename [char*]: [the filename]
	size [double]: [the new size of the mesh model]
	center [Vector3d]: [the new center of the mesh model]
	k_reflection [double]: [the reflection coefficient of the mesh model]
	k_refraction [double]: [the refraction coefficient of the mesh model]
Returns:
	faces [vector<TriangleMesh>]: [the faces]
*/
vector<TriangleMesh> ReadOBJMesh(char* filename, double size, Vector3d center, double k_reflection, double k_refraction)
{
	//store the information of faces
	struct FaceInfo
	{
	public:
		int av = 0, bv = 0, cv = 0; //vertex id
		int an = 0, bn = 0, cn = 0; //normal id
		int ap = 0, bp = 0, cp = 0; //pixel id
		string mtl_name;
		FaceInfo(int av, int ap, int an, int bv, int bp, int bn, int cv, int cp, int cn, string mtl_name)
		{
			this->av = av - 1;
			this->ap = ap - 1;
			this->an = an - 1;
			this->bv = bv - 1;
			this->bp = bp - 1;
			this->bn = bn - 1;
			this->cv = cv - 1;
			this->cp = cp - 1;
			this->cn = cn - 1;
			this->mtl_name = mtl_name;
		}
	};
	//store the information of mtl
	struct MTLInfo
	{
	public:
		string name;
		Vector3d ka;
		Vector3d kd;
		Vector3d ks;
		string ka_name;
		string kd_name;
		string ks_name;
		MTLInfo() 
		{
			name = "";
			ka << 0, 0, 0;
			kd << 0, 0, 0;
			ks << 0, 0, 0;
			ka_name = "";
			kd_name = "";
			ks_name = "";
		}
		void clear()
		{
			name = "";
			ka << 0, 0, 0;
			kd << 0, 0, 0;
			ks << 0, 0, 0;
			ka_name = "";
			kd_name = "";
			ks_name = "";
		}
	};

	//storage
	vector<Vector3d> points;
	vector<Vector3d> normals;
	vector<Vector2d> pixels;
	vector<FaceInfo> face_infos;
	vector<MTLInfo> mtl_infos;
	vector<Vertex> vertexs;
	vector<TriangleMesh> faces;
	map<string, vector<Vector3d>> textures;
	points.clear();
	normals.clear();
	pixels.clear();
	face_infos.clear();
	vertexs.clear();
	faces.clear();
	textures.clear();
	string mtl_path;
	string mtl_name;

	//headers
	string mtllib = "mtllib";
	string usemtl = "usemtl";
	string v = "v";
	string vn = "vn";
	string vt = "vt";
	string f = "f";
	string empty = "";
	string eof = " End of File";
	string newmtl = "newmtl";
	string Ka = "Ka";
	string Kd = "Kd";
	string Ks = "Ks";
	string map_Ka = "map_Ka";
	string map_Kd = "map_Kd";
	string map_Ks = "map_Ks";


    //read obj
	ifstream obj_file;
	obj_file.open(filename, ios::in);
	while (obj_file.peek() != EOF)
	{
		string head;
		obj_file >> head;
		if (head == mtllib)
		{
			obj_file >> mtl_path;
		}
		else if (head == usemtl)
		{
			obj_file >> mtl_name;
		}
		else if (head == v)
		{
			double x, y, z;
			obj_file >> x >> y >> z;
			Vector3d point;
			point << x, y, z;
			points.push_back(point);
		}
		else if (head == vn)
		{
			double nx, ny, nz;
			obj_file >> nx >> ny >> nz;
			Vector3d normal;
			normal << nx, ny, nz;
			normals.push_back(normal);
		}
		else if (head == vt)
		{
			double px, py;
			obj_file >> px >> py;
			Vector2d pixel;
			pixel << px, py;
			pixels.push_back(pixel);
		}
		else if (head == f)
		{
			char s;
			int av, ap, an, bv, bp, bn, cv, cp, cn;
			obj_file >> av >> s >> ap >> s >> an >> bv >> s >> bp >> s >> bn >> cv >> s >> cp >> s >> cn;
			FaceInfo face_info(av, ap, an, bv, bp, bn, cv, cp, cn, mtl_name);
			face_infos.push_back(face_info);
		}
		else
		{
			string line;
			getline(obj_file, line);
		}
	}
	obj_file.close();

	//read mtl
	//TODO: 文件系统转换
	ifstream mtl_file;
	mtl_file.open("C:\\Users\\SerCharles\\Desktop\\res\\shiba.mtl");
	MTLInfo current_mtl;
	while (mtl_file.peek() != EOF)
	{
		string head;
		mtl_file >> head;
		if (head == newmtl)
		{
			if (current_mtl.name != "")
			{
				mtl_infos.push_back(current_mtl);
			}
			current_mtl.clear();
			string name;
			mtl_file >> name;
			current_mtl.name = name;
		}
		else if (head == Ka)
		{
			double r, g, b;
			Vector3d ka;
			mtl_file >> r >> g >> b;
			ka << r, g, b;
			current_mtl.ka = ka;
		}
		else if (head == Kd)
		{
			double r, g, b;
			Vector3d kd;
			mtl_file >> r >> g >> b;
			kd << r, g, b;
			current_mtl.kd = kd;
		}
		else if (head == Ks)
		{
			double r, g, b;
			Vector3d ks;
			mtl_file >> r >> g >> b;
			ks << r, g, b;
			current_mtl.ks = ks;
		}
		else if (head == map_Ka)
		{
			string name;
			mtl_file >> name;
			current_mtl.ka_name = name;
			vector<Vector3d> v;
			v.clear();
			textures[name] = v;
		}
		else if (head == map_Kd)
		{
			string name;
			mtl_file >> name;
			current_mtl.kd_name = name;
			vector<Vector3d> v;
			v.clear();
			textures[name] = v;
		}
		else if (head == map_Ks)
		{
			string name;
			mtl_file >> name;
			current_mtl.ks_name = name;
			vector<Vector3d> v;
			v.clear();
			textures[name] = v;
		}
		else
		{
			string line;
			getline(obj_file, line);
		}
	}
	mtl_file.close();
	if (current_mtl.name != "")
	{
		mtl_infos.push_back(current_mtl);
	}

	

	//clear and return
	points.clear();
	normals.clear();
	pixels.clear();
	face_infos.clear();
	mtl_infos.clear();
	vertexs.clear();
	textures.clear();
	return faces;
}



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
		double x = face.vertexs[i].point(0);
		double y = face.vertexs[i].point(1);
		double z = face.vertexs[i].point(2);
		if (x >= box.min_x && x <= box.max_x && y >= box.min_y && y <= box.max_y && z >= box.min_z && z <= box.max_z)
		{
			result = 1;
			break;
		}
	}
	return result;
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
};


//The bounding box and octtree of an object
class MeshModel
{
public:
	vector<TriangleMesh> faces;
	OctNode* root = NULL;

	MeshModel() {}

	MeshModel(vector<TriangleMesh>& faces)
	{
		this->faces = faces;
		BoundingBox bounding_box = this->BuildBoundingBox();
		this->root = new OctNode(1, bounding_box, this->faces);
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
				double x = this->faces[i].vertexs[j].point(0);
				double y = this->faces[i].vertexs[j].point(1);
				double z = this->faces[i].vertexs[j].point(2);
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
		BoundingBox bounding_box = BoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
		return bounding_box;
	}
};