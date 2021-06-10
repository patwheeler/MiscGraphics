#pragma once
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

struct viewPoint {
	Eigen::Vector3d from;
	Eigen::Vector3d at;
	Eigen::Vector3d up;
	double angle, hither;
	int resx, resy;
};

struct Color {
	double r, g, b;

	struct Color& operator+=(const Color& rhs){
		r = r + rhs.r;
		g = g + rhs.g;
		b = b + rhs.b;
		return *this;
	}
	
	struct Color& operator*(const double comp){
		r *= comp;
		g *= comp;
		b *= comp;
		return *this;
	}
	
	bool operator==(const Color& rhs){
		if ((r != rhs.r) || (g != rhs.g) || (b != rhs.b))
			return false;
		
		return true;
	}		
};

struct Light {
	Eigen::Vector3d pos;
	//Color lightColor;
};

struct Fill {
	Color base;
	double Kd, Ks, e, Kt, ir;
};

struct Tri {
	Fill fill;
	Eigen::Vector4d v0, v1, v2;
	Eigen::Vector3d n0, n1, n2;
};

class Polygon{
public:
	Fill fill;
	std::vector<Eigen::Vector3d> vertices;
	std::vector<Eigen::Vector3d> normals;
	void setFill(Fill inVals) {fill = inVals;}
	void addVertex(Eigen::Vector3d vert) {vertices.push_back(vert);}
	void addNormal(Eigen::Vector3d norm) {normals.push_back(norm);}
	int numVerts() {return vertices.size();}
	int numNorms() {return normals.size();}
};

class Triangle{
public:
	std::vector<Tri> triangles;
	int numTriangles() {return triangles.size();}
	void getTriangles(Polygon &poly);
};

Eigen::Vector3d interpNorm(Tri tri, double alpha, double beta, double gamma);	
Eigen::Vector3d toWorld(Eigen::Vector3d coords, Eigen::Matrix4d M_inv);
Color shadeFragment(Fill initial, Eigen::Vector3d point, Eigen::Vector3d normal, std::vector<Light> bulbs, Eigen::Vector3d view);
Color clamp(Color col);
