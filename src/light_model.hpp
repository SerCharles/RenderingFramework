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
	Vector3d ambient;
	Vector3d diffuse;
	Vector3d specular;
	Vector3d direction;
	Light() {}
	Light()
	{
		ambient << 1.0, 1.0, 1.0;
		diffuse << 1.0, 1.0, 1.0;
		specular << 1.0, 1.0, 1.0;
		direction << 0, -1, 0;
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
	double r = vertex.color(0) * light.ambient(0);
	double g = vertex.color(1) * light.ambient(1);
	double b = vertex.color(2) * light.ambient(2);
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
	if (ray.inside == 1)
	{
		n = -vertex.normal;
	}

	Vector3d l = light.direction;
	double weight = -n.dot(l);
	if (weight <= 0)
	{
		weight = 0;
	}
	double r = vertex.color(0) * light.diffuse(0) * weight;
	double g = vertex.color(1) * light.diffuse(1) * weight;
	double b = vertex.color(2) * light.diffuse(2) * weight;
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
	if (ray.inside == 1)
	{
		n = -vertex.normal;
	}
	Vector3d l = light.direction;
	Vector3d v = ray.direction;
	double normal_speed = l.dot(-n);
	assert(normal_speed >= 0);
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
	double r = vertex.color(0) * light.specular(0) * weight;
	double g = vertex.color(1) * light.specular(1) * weight;
	double b = vertex.color(2) * light.specular(2) * weight;
	specular << r, g, b;
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
		Vector3d ambient = GetAmbient(light, ray, face.vertexs[0]);
		Vector3d diffuse = GetDiffuse(light, ray, face.vertexs[0]);
		Vector3d specular = GetSpecular(light, ray, face.vertexs[0]);
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
	const int max_depth = 6;

	RayTracing()
	{
		//TODO
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
			GetIntersectionRayMeshModel(ray, this->objects[i], mesh_id, t, fraction);
			if (t > 0 && t < best_t)
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
		color = PhongModel(this->light, ray, final_mesh, best_fraction);
		double k_reflection = this->objects[best_i].k_reflection;
		double k_refraction = this->objects[best_i].k_refraction;
		Ray reflection = GetReflectionRay(ray, final_mesh, best_t);
		Ray refraction = GetRefractionRay(ray, final_mesh, best_t);
		Vector3d color_reflection = this->TraceOneRay(reflection, depth + 1);
		Vector3d color_refraction = this->TraceOneRay(refraction, depth + 1);
		color = color + color_reflection + color_refraction;
		return color;
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
			}
		}
	}





};