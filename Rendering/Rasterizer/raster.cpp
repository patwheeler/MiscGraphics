#include "dataTypes.h"
#include "fops.h"

using namespace std;

static viewPoint view;
static Color bgColor;
static std::vector<Light> lights;
static std::vector<Polygon> allGeometry;

void processVerts(Tri &currentTri, Eigen::Matrix4d M){		
	currentTri.v0 = M * currentTri.v0;
	currentTri.v1 = M * currentTri.v1;
	currentTri.v2 = M * currentTri.v2;
}

double edgeFn(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d c){
	return (((a[1] - b[1]) * c[0]) + ((b[0] - a[0]) * c[1]) + (a[0]*b[1]) - (b[0] * a[1]));
}

int main(int argc, char *argv[])
{
	//File names for input nff and output ppm
	string inName = argv[1];
	string outName = argv[2];
	
	parseFile(inName, bgColor, view, allGeometry, lights);
	
	int numPix = view.resx*view.resy;
	int numInd = (numPix* 3);
	//Initialize all pixies to bg color
	unsigned char *pixels;
	pixels = new unsigned char[numInd];
	for (int i = 0; i < numInd; i+=3){
		pixels[i] = bgColor.r * 255;
		pixels[i+1] = bgColor.g * 255;
		pixels[i+2] = bgColor.b * 255;
	}
	
	//Initialize all depth vals to INF
	float *zbuffer;
	zbuffer = new float[numPix];
	for (int i = 0; i < numPix; i++){
		zbuffer[i] = INFINITY;
	}
	
	//Setting up camera/origin space
	Eigen::Vector3d w, u, v;
	w = (view.from - view.at).normalized();
	u = view.up.cross(w).normalized(); 
	v = w.cross(u).normalized();
	//World-space descriptions
	double aspect = view.resx / view.resy;
	double radians = (view.angle*EIGEN_PI / 180.0);
	double halfWidth = tan(radians / 2.0);
	double right = halfWidth;
	double left = -halfWidth;
	double bottom = -halfWidth/aspect;
	double top = halfWidth/aspect;
	double near = view.hither;
	double far = view.hither*1000;

	//----------------------Projection Space----------------------	
	Eigen::Matrix4d M_vp, M_per, M_cam, M, M_inv;
	//Viewport transformation
	M_vp << (view.resx/2.0), 0, 0, ((view.resx-1)/2.0),
			0, (view.resy/2.0), 0, ((view.resy-1)/2.0),
			0, 0, 1, 0, 
			0, 0, 0, 1;
			
	//Perspective transformation
	M_per << (-2*near/(right-left)), 0, ((right+left)/(right-left)), 0,
			0, (2*near/(top-bottom)), ((top+bottom)/(top-bottom)), 0,
			0, 0, ((near + far)/(near-far)), ((2*far*near)/(near-far)),
			0, 0, 1, 0;
			
	//Camera transformation
	M_cam << u[0], v[0], w[0], view.from[0],
			u[1], v[1], w[1], view.from[1],
			u[2], v[2], w[2], view.from[2],
			0, 0, 0, 1;
	
	M_cam = M_cam.inverse().eval();
	M = M_vp*M_per*M_cam;
	M_inv = M.inverse().eval();
	
	//----------------------Prep Triangles-------------------------
	Triangle allTriangles;
	for (int i = 0; i < (int)allGeometry.size(); i++){
		allTriangles.getTriangles(allGeometry[i]);
	}	
	int numTriangles = allTriangles.numTriangles();
	//Transform each vertex
	for (int i = 0; i < numTriangles; i++){
		Tri &currentTri = allTriangles.triangles[i];
		processVerts(currentTri, M);
	}
	
	//------------------------Raster loop--------------------------
	for (int k = 0; k < numTriangles; k++){
		Tri currTri = allTriangles.triangles[k];
		
		//Divide
		currTri.v0 /= currTri.v0[3];
		currTri.v1 /= currTri.v1[3];
		currTri.v2 /= currTri.v2[3];

		//Bounding-box for this triangle
		int x_min = floor(min(min(currTri.v0[0], currTri.v1[0]), currTri.v2[0]));
		int x_max = ceil(max(max(currTri.v0[0], currTri.v1[0]), currTri.v2[0]));
		int y_min = floor(min(min(currTri.v0[1], currTri.v1[1]), currTri.v2[1]));
		int y_max = ceil(max(max(currTri.v0[1], currTri.v1[1]), currTri.v2[1]));
		
		//Check if triangle exists within viewing frustum
		if (x_min > view.resx || x_max < 0 || y_min > view.resy || y_max < 0) continue;
		
		//Clip
		if (x_min < 0) x_min = 0;
		if (x_max >= view.resx)
			x_max = view.resx - 1;
		if (y_min < 0) y_min = 0;
		if (y_max >= view.resy)
			y_max = view.resy - 1;
		
		//Only regard x and y coords
		Eigen::Vector2d v0 = Eigen::Vector2d(currTri.v0[0], currTri.v0[1]);
		Eigen::Vector2d v1 = Eigen::Vector2d(currTri.v1[0], currTri.v1[1]);
		Eigen::Vector2d v2 = Eigen::Vector2d(currTri.v2[0], currTri.v2[1]);
		
		//Get area of triangle
		double area = edgeFn(v0, v1, v2);
		
		//Loop over ROI
		for (int y = y_min; y <= y_max; y++) 
		{
			for (int x = x_min; x <= x_max; x++) 
			{
				Eigen::Vector2d targetPix = Eigen::Vector2d(x + 0.5, y + 0.5);
				//Barycentric coords; weights each vert contribute to triangle
				double alpha = edgeFn(v1, v2, targetPix)/area;
				double beta = edgeFn(v2, v0, targetPix)/area;
				double gamma = edgeFn(v0, v1, targetPix)/area;
				if (alpha >= 0.0 && beta >= 0.0 && gamma >= 0.0){
					int buffInd = y*view.resx + x;
					int index = ((y*view.resx*3) + x*3);
					float z = 1.0/(alpha*currTri.v0[2] + beta*currTri.v1[2] + gamma*currTri.v2[2]);
					
					//If curr z < curr z_buff(i.e. closer pixel), set new z_buff && eval
					//Else, go to next pixel
					if (z <= zbuffer[buffInd])
						zbuffer[buffInd] = z;		
					else
						continue;
					
					Eigen::Vector2d avg = alpha*v0 + beta*v1 + gamma*v2;
					Eigen::Vector3d weights(avg[0], avg[1], z);
					Eigen::Vector3d pixPos = toWorld(weights, M_inv);
					pixPos = pixPos.normalized();
					Eigen::Vector3d norm = interpNorm(currTri, alpha, beta, gamma);
					Color draw = shadeFragment(currTri.fill, pixPos, norm, lights, view.from);
					pixels[index + 0] = draw.r * 255.0;
					pixels[index + 1] = draw.g * 255.0;
					pixels[index + 2] = draw.b * 255.0;
				}
			}
		}
	}
	
	delete[] zbuffer;
	zbuffer = NULL;
	writeOut(outName, pixels, view);
	delete[] pixels;
	pixels = NULL;
	return 0;
}

