#pragma once
#include <vector>
#include <memory>
#include <algorithm>	
#include <Eigen/Dense>
#include <cmath>

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

struct Ray {
	Eigen::Vector3d origin, direction;
	int depth;
	Ray(const Eigen::Vector3d &o, const Eigen::Vector3d &d) 
		: origin(o), direction(d), depth(0) {}
};

struct Hit {
	Eigen::Vector3d norm;
	Eigen::Vector3d hitLoc;
	Fill surfFill;
};

//Need to add trace fn for shadow/light interaction
class Surface {
public:
	virtual bool intersect(Ray r, double &t, Hit &hitInfo) = 0;
};

class Polygon : public Surface {
private:
	Fill fill;
	std::vector<Eigen::Vector3d> vertices;
public:
	void setFill(Fill inVals) {fill = inVals;}
	void addVertex(Eigen::Vector3d vert) {vertices.push_back(vert);}
	int numVerts() {return vertices.size();}
	bool intersect(Ray r, double &t, Hit &hitInfo) override;
};

class Sphere : public Surface {
private:
	double radius;
	Eigen::Vector3d center;
	Fill fill;
public:
	void setFill(Fill inVals) {fill = inVals;}
	void setVals(Eigen::Vector3d coordsIn, double radIn) {center = coordsIn; radius = radIn;}
	bool intersect(Ray r, double &t, Hit &hitInfo) override;
};
