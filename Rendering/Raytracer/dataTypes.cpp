#include <random>
#include "dataTypes.h"
#include "fops.h"

//Evaluate ray/poly intersection and update hit location
bool Polygon::intersect(Ray r, double &t, Hit &vals){
	//Get normal--guaranteed to be non-zero(nff requirement).
	Eigen::Vector3d surfNorm = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]).normalized();	

	float denom = surfNorm.dot(r.direction);

	//Vector describing origin to point on the plane
	Eigen::Vector3d diff = (vertices[0] - r.origin);
	float min_t = diff.dot(surfNorm)/denom;
	if ((min_t <= 0.00001) || (min_t > t)) 
		return false;

	//Calculate new point, P
	Eigen::Vector3d P = r.origin + min_t * (r.direction);

	//-----------------Crossing number algorithm: general polygon intersect-----------------------------
	
	//Find index of the component with the largest magnitude to avoid 'degenerate' projection
	//This results in the 3->2d projection of the greatest area
	Eigen::Vector3d::Index maxInd;
	surfNorm.cwiseAbs().maxCoeff(&maxInd);

	//Deciding basis of projection by 'ignoring' index calculated above	
	int first = 0;
	int second = 1;	
	switch((int)maxInd){
		case(0):
			first = 1;
			second = 2;
			break;
		case(1):
			second = 2;
			break;
		default:
			break;
	}
	//Setting basis for projection; p_0 and p_1 are the dims of P we care abt
	double p_0 = P[first];
	double p_1 = P[second];
	double ax_0_0, ax_0_1; 
	double ax_1_0, ax_1_1;	
	
	int crossingNum = 0;
	int maxNum = this->numVerts() - 1;
	int v0, v1;

	//Determines if an intersect occurs by looping over
	//each vertex of the polygon. Adds 1 to cn if true
	for(v0 = 0; v0 <= maxNum; v0++){
		v1 = v0 + 1;
		if(v0 == maxNum)
			v1 = 0;
		ax_0_0 = vertices[v0][first];
		ax_0_1 = vertices[v0][second];
		ax_1_0 = vertices[v1][first];
		ax_1_1 = vertices[v1][second];
		if (((ax_0_1 <= p_1) && (ax_1_1 > p_1)) || ((ax_0_1 > p_1) && (ax_1_1 <= p_1))){
			float vt = (float)(p_1 - ax_0_1)/(ax_1_1 - ax_0_1);
			if (p_0 < ax_0_0 + vt * (ax_1_0 - ax_0_0))
				++crossingNum;	
		}		
	}
	//cn must be odd to intersect
	if (crossingNum % 2 == 0)
		return false;
	
	t = min_t;
	vals.norm = surfNorm;
	vals.hitLoc = P;
	vals.surfFill = this->fill;
	return true;
}

bool Sphere::intersect(Ray r, double &t, Hit &vals){	
	Eigen::Vector3d dir = r.direction.normalized();
	Eigen::Vector3d oc = r.origin - center;
	float oc_d = dir.dot(oc);
	float disc = (oc_d * oc_d) - (oc.dot(oc) - radius * radius);
	if (disc < 0)
		return false;
	float sqrt_disc = std::sqrt(disc);
	float t_0 = -(oc_d) - sqrt_disc;
	float t_1 = -(oc_d) + sqrt_disc;
	float min_t = (t_0 > t_1) ? t_1 : t_0;
	
	if (min_t < 0.00001) return false;
		
	t = min_t;
	
	Eigen::Vector3d P = r.origin + (t * r.direction);
	Eigen::Vector3d surfNorm = (P - center).normalized();
	
	vals.norm = surfNorm;
	vals.hitLoc = P;
	vals.surfFill = this->fill;
	return true;
}

//Returns true if there is *any* intersection; false otherwise
bool isShadowed(Ray r, std::vector<std::shared_ptr<Surface>> surfaces){
	Hit record;
	double min_t = INFINITY;
	for (auto const& obj : surfaces)
		if (obj->intersect(r, min_t, record)) 
			return true;

	return false;
}

//Returns true + updates hit record to closest intersection; false otherwise
bool getRec(Ray r, Hit& record, std::vector<std::shared_ptr<Surface>> surfaces){
	double min_t = INFINITY;
	bool wasHit = false;
	for (auto const& obj : surfaces)
		if(obj->intersect(r, min_t, record)) wasHit = true;

	return wasHit;
}

//Returns reflection of ray--view independent
Ray reflRay(Ray incident, Hit record){
	Eigen::Vector3d refl_o, refl_d, incident_dir;
	incident_dir = incident.direction;
	double reflection = record.norm.dot(incident_dir);
	
	refl_o = record.hitLoc;
	refl_d = incident_dir - (2.0 * record.norm * reflection);
	Ray r(refl_o, refl_d);
	r.depth = incident.depth + 1;
	return r;	
}

//Prevent any rgb vals from exceeding 1
Color clamp(Color col){
	Color ret;
	ret.r = (col.r > 1.0) ? 1.0 : col.r;
	ret.g = (col.g > 1.0) ? 1.0 : col.g;
	ret.b = (col.b > 1.0) ? 1.0 : col.b;
	return ret;
} 

//Evaluate color of hit
Color trace(Color bg, Ray r, Eigen::Vector3d view, std::vector<Light> bulbs, std::vector<std::shared_ptr<Surface>> surfaces)
{
	//Check if ray hits surface--update record to closest if it does
	Hit record;	
	bool surfHit = getRec(r, record, surfaces);	
	Fill attributes = record.surfFill;	
	Color surf_col = bg;
	if (surfHit)
	{	
		surf_col = attributes.base;
		Color reflect_col;
		reflect_col.r = reflect_col.g = reflect_col.b = 0.;
		
		//Initialize some vars for diffuse and specular contributions
		int numLights = (int)bulbs.size();
		double intensity = 1.0/std::sqrt(numLights);
		double Kd, Ks;
		Kd = attributes.Kd;
		Ks = attributes.Ks;
		
		//Set reflection ray
		Ray refl = reflRay(r, record);

		double diffuse, specular;
		Eigen::Vector3d L;
		//------------SHADOW CALC--------------
		for (int i = 0; i < numLights; i++)
		{
			L = (bulbs[i].pos - record.hitLoc).normalized();
			Ray shadRay(record.hitLoc, L);
			bool inShadow = isShadowed(shadRay, surfaces);	
			if (inShadow) continue;
			
			//View dependent shading
			Eigen::Vector3d H = (L + view).normalized();
			specular = std::pow(std::max(0., record.norm.dot(H)), attributes.e);
			diffuse = std::max(0., record.norm.dot(L));
			
			reflect_col.r += ((Kd * surf_col.r * diffuse) + Ks * specular);
			reflect_col.g += ((Kd * surf_col.g * diffuse) + Ks * specular);
			reflect_col.b += ((Kd * surf_col.b * diffuse) + Ks * specular);
		}
		
		surf_col = reflect_col * intensity;
		if ((refl.depth) < 5 && Ks > 0)
			surf_col += trace(bg, refl, view, bulbs, surfaces) * Ks;	
	}
	
	return clamp(surf_col);
}

