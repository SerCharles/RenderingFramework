#pragma once
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

	RayTracing() {}
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