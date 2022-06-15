#pragma once
#include "utils.hpp"
#include "mesh_model.hpp"
#include "camera_model.hpp"
#include "intersection.hpp"


/*
Definition of light and phong model
*/
class Light
{
public:
	Vector3d color;
	Vector3d direction;
	Light() {}

	Light(Vector3d& color, Vector3d& direction)
	{
		this->color = color;
		this->direction = direction;
	}
};

/*
Get the ambient light on a vertex
Args:
	light [Light]: [the light source]
	ray [Ray]: [the looking ray]
	vertex [Vertex]: [the vertex to be lighted]
Returns:
	ambient [Vector3d]: [the RGB result, between[0, 1)]
*/
Vector3d GetAmbient(Light& light, Ray& ray, Vertex& vertex)
{
	Vector3d ambient;
	double r = vertex.color(0) * light.color(0);
	double g = vertex.color(1) * light.color(1);
	double b = vertex.color(2) * light.color(2);
	ambient << r, g, b;
	return ambient;
}

/*
Get the diffuse light on a vertex of a triangle mesh
Args:
	light [Light]: [the light source]
	ray [Ray]: [the looking ray]
	vertex [Vertex]: [the vertex to be lighted]
Returns:
	diffuse [Vector3d]: [the RGB result, between[0, 1)]
*/
Vector3d GetDiffuse(Light& light, Ray& ray, Vertex& vertex)
{
	Vector3d diffuse;
	Vector3d n = vertex.normal;
	Vector3d l = light.direction;
	double weight = -n.dot(l);
	if (weight <= 0)
	{
		weight = 0;
	}
	double r = light.color(0) * weight;
	double g = light.color(1) * weight;
	double b = light.color(2) * weight;
	diffuse << r, g, b;
	return diffuse;
}

/*
Get the specular light on a vertex of a triangle mesh
Args:
	light [Light]: [the light source]
	ray [Ray]: [the looking ray]
	vertex [Vertex]: [the vertex to be lighted]
Returns:
	specular [Vector3d]: [the RGB result, between[0, 1)]
*/
Vector3d GetSpecular(Light& light, Ray& ray, Vertex& vertex)
{
	int p = 10;
	Vector3d n = vertex.normal;
	Vector3d l = light.direction;
	Vector3d v = ray.direction;
	double normal_speed = l.dot(-n);
	Vector3d normal_velocity = -n * normal_speed;
	Vector3d tangent_velocity = l - normal_velocity;
	Vector3d r = tangent_velocity - normal_velocity;

	double rv = r.dot(-v);
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
	double red = light.color(0) * weight;
	double green = light.color(1) * weight;
	double blue = light.color(2) * weight;
	specular << red, green, blue;
	return specular;
}

/*
Get the color of the intersection point of a mesh using the phong model
Args:
	light [Light]: [the light source]
	ray [Ray]: [the seeing direction]
	face [TriangleMesh]: [the mesh to be lighted]
	fraction [Vector3d]: [the fraction of the seeing point on the mesh]
Returns:
	color [Vector3d]: [the result RGB color, between[0, 1)]
*/
Vector3d PhongModel(Light& light, Ray& ray, TriangleMesh& face, Vector3d& fraction)
{
	Vector3d color;
	color << 0, 0, 0;
	for (int i = 0; i < 3; i++)
	{
		Vector3d ambient = GetAmbient(light, ray, face.vertexs[0]) * face.ambient;
		Vector3d diffuse = GetDiffuse(light, ray, face.vertexs[0]) * face.diffuse;
		Vector3d specular = GetSpecular(light, ray, face.vertexs[0]) * face.specular;
		Vector3d the_color = ambient + diffuse + specular;
		color = color + the_color * fraction(i);
	}
	color = color * ray.intensity;
	return color;
}



//the main class of ray tracing
class RayTracing
{
public:
	vector<MeshModel> objects;
	Camera camera;
	Light light;
	const double threshold = 0.01;
	const int max_depth = 3;
	Vector3d* results;

	~RayTracing()
	{
		this->objects.clear();
	}

	RayTracing()
	{
		Vector3d light_color;
		Vector3d light_direction;
		light_color << 1.0, 1.0, 1.0;
		light_direction << 0, -1.0, 0;
		this->light = Light(light_color, light_direction);

		int picture_size = 300;
		double r = 10 * sqrt(2.0);
		double theta = 90.0 / 180.0 * PI;
		double phi = 0;
		this->camera = Camera(picture_size, r, theta, phi);

		this->objects.clear();
		Vector3d center;
		Vector3d color;
		double size = 1;
		double k_reflection = 0;
		double k_refraction = 0;
		double refraction_rate = 1;
		double ambient = 0;
		double diffuse = 0;
		double specular = 0;

		
		char name_board[100] = "C:\\Users\\SerCharles\\Desktop\\res\\board.ply";
		size = 10 * sqrt(2);
		center << 0, 0, 0;
		color << 0.4, 0.4, 0.4;
		k_reflection = 0.2;
		k_refraction = 0;
		ambient = 0.6;
		diffuse = 0.2;
		specular = 0.2;
		vector<TriangleMesh> board_mesh = ReadPLYMesh(name_board, size, center, color,
			k_reflection, k_refraction, ambient, diffuse, specular);
		MeshModel board = MeshModel(board_mesh);
		this->objects.push_back(board);
		
		
		char name_bunny[100] = "C:\\Users\\SerCharles\\Desktop\\res\\bunny.ply";
		size = 2;
		center << 0, 3, -4;
		color << 1, 1, 0.2;
		k_reflection = 0;
		k_refraction = 0;
		ambient = 0.6;
		diffuse = 0.2;
		specular = 0.2;
		vector<TriangleMesh> bunny_mesh = ReadPLYMesh(name_bunny, size, center, color,
			k_reflection, k_refraction, ambient, diffuse, specular);
		MeshModel bunny = MeshModel(bunny_mesh);
		this->objects.push_back(bunny);
		
		
		char name_cube[100] = "C:\\Users\\SerCharles\\Desktop\\res\\cube.ply";
		size = 2;
		center << 5, 4, 2;
		color << 0.2, 0.2, 0.2;
		k_reflection = 0;
		k_refraction = 0.6;
		vector<TriangleMesh> dragon_mesh = ReadPLYMesh(name_cube, size, center, color,
			k_reflection, k_refraction, ambient, diffuse, specular);
		MeshModel dragon = MeshModel(dragon_mesh);
		this->objects.push_back(dragon);
		
		int total_size = this->camera.height * this->camera.width;
		this->results = new Vector3d[total_size];
		
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
			if (i == ray.last_object_id)
			{
				continue;
			}
			double t;
			int mesh_id;
			Vector3d fraction;
			GetIntersectionRayMeshModel(ray, this->objects[i], mesh_id, t, fraction);
			if (t > 1e-3 && t < best_t)
			{
				best_t = t;
				best_mesh_id = mesh_id;
				best_fraction = fraction;
				best_i = i;
			}
		}

		//judge result according to the type of the ray
		if (ray.type == TYPE_LOCAL)
		{
			if (best_i < 0)
			{
				color << 1, 1, 1;
			}
			else
			{
				color << 0, 0, 0;
			}
			return color;
		}
		else
		{
			//generate ray tree, recursively get results
			if (best_i < 0)
			{
				return color;
			}
			TriangleMesh final_mesh = this->objects[best_i].faces[best_mesh_id];
			Vector3d color_phong = PhongModel(this->light, ray, final_mesh, best_fraction);
			double k_reflection = this->objects[best_i].k_reflection;
			double k_refraction = this->objects[best_i].k_refraction; 
			Ray local = GetLocalRay(ray, this->light.direction, best_t, best_i);
			Ray reflection = GetReflectionRay(ray, final_mesh, best_t, best_i);
			Ray refraction = GetRefractionRay(ray, final_mesh, best_t, best_i);
			Vector3d color_local = this->TraceOneRay(local, depth);
			Vector3d color_reflection = this->TraceOneRay(reflection, depth + 1);
			Vector3d color_refraction = this->TraceOneRay(refraction, depth + 1);
			color(0) = color_phong(0) * color_local(0);
			color(1) = color_phong(1) * color_local(1);
			color(2) = color_phong(2) * color_local(2);
			color = color + color_reflection + color_refraction;
			return color;
		}

		
	}

	/*
	The main function of ray tracing
	*/
	void Main()
	{
		for (int i = 0; i < this->camera.width; i++)
		{
			for (int j = 0; j < this->camera.height; j++)
			{
				Ray the_ray = GetPixelRay(this->camera, i, j);
				Vector3d the_color = this->TraceOneRay(the_ray, 1);
				this->results[j * this->camera.width + i] = the_color;
			}
		}
	}
};