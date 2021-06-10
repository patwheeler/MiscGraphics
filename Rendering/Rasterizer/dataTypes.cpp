#include "dataTypes.h"
#include "fops.h"

void Triangle::getTriangles(Polygon &poly){
	int numTris = poly.numVerts() - 2;
	//In the case the nff contains a pp(polygonal patch)
	bool hasNormals = (poly.numNorms()) > 0 ? true : false;
	Eigen::Vector3d firstN;
	Eigen::Vector3d firstV = poly.vertices[0];

	for (int i = 1; i <= numTris; i++){
		Tri instance;
		instance.fill = poly.fill;
		Eigen::Vector3d v1, v2, n1, n2;
		v1 = poly.vertices[i];
		v2 = poly.vertices[i + 1];
		if (hasNormals){
			firstN = poly.normals[0];
			n1 = poly.normals[i];
			n2 = poly.normals[i + 1];
		}else{
			firstN = (v1 - firstV).cross(v2 - firstV).normalized();
			n1 = n2 = firstN;
		}
		instance.v0 = Eigen::Vector4d(firstV[0], firstV[1], firstV[2], 1);
		instance.v1 = Eigen::Vector4d(v1[0], v1[1], v1[2], 1);
		instance.v2 = Eigen::Vector4d(v2[0], v2[1], v2[2], 1);
		instance.n0 = firstN;
		instance.n1 = n1;
		instance.n2 = n2;
		triangles.push_back(instance);
	}
}

//Prevent any rgb vals from exceeding 1
Color clamp(Color col){
	Color ret;
	ret.r = (col.r > 1.0) ? 1.0 : col.r;
	ret.g = (col.g > 1.0) ? 1.0 : col.g;
	ret.b = (col.b > 1.0) ? 1.0 : col.b;
	return ret;
} 

//Returns color of pixel based on contrib of each light source && fill
Color shadeFragment(Fill initial, Eigen::Vector3d point, Eigen::Vector3d normal, std::vector<Light> bulbs, Eigen::Vector3d view){
		Color surf_col = initial.base;
		int numLights = (int)bulbs.size();
		double intensity = 1.0/std::sqrt(numLights);
		double Kd = initial.Kd;
		double Ks = initial.Ks;
		double diffuse, specular;
		Color rc;
		rc.r = rc.g = rc.b = 0.;
		for (int j = 0; j < numLights; j++)
		{	
			Eigen::Vector3d L = (bulbs[j].pos - point).normalized();
			Eigen::Vector3d H = (L + view).normalized();
			specular = std::pow(std::max(0., normal.dot(H)), initial.e);
			diffuse = std::max(0., normal.dot(L));
			
			rc.r += (surf_col.r * Kd * diffuse) + (Ks * specular);
			rc.g += (surf_col.g * Kd * diffuse) + (Ks * specular);
			rc.b += (surf_col.b * Kd * diffuse) + (Ks * specular);
		}
		
		return clamp(rc*intensity);
}

Eigen::Vector3d toWorld(Eigen::Vector3d coords, Eigen::Matrix4d M_inv){
	Eigen::Vector4d pixelBase = Eigen::Vector4d(coords[0], coords[1], 1.0, coords[2]);
	pixelBase = M_inv * pixelBase;
	return (Eigen::Vector3d(pixelBase[0], pixelBase[1], pixelBase[2]));
}

Eigen::Vector3d interpNorm(Tri tri, double alpha, double beta, double gamma){
	double x = alpha*tri.n0[0] + beta*tri.n1[0] + gamma*tri.n2[0];
	double y = alpha*tri.n0[1] + beta*tri.n1[1] + gamma*tri.n2[1];
	double z = alpha*tri.n0[2] + beta*tri.n1[2] + gamma*tri.n2[2];
	return (Eigen::Vector3d(x, y, z).normalized());
}





