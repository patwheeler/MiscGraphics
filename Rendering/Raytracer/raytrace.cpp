#include <random>
#include "dataTypes.h"
#include "fops.h"

static viewPoint view;
static Color bgColor;
static std::vector<Light> lights;
static std::vector<std::shared_ptr<Surface>> allGeometry;

Color trace(Color, Ray, Eigen::Vector3d, std::vector<Light>, std::vector<std::shared_ptr<Surface>>);

int main(int argc, char *argv[])
{
	//File names for input nff and output ppm
	std::string inName = argv[1];
	std::string outName = argv[2];
	
	parseFile(inName, bgColor, view, allGeometry, lights);
	
	//Allocate array for pixel color writing
	//a single pixel (i.e. its color) described by 3 vals
	int numInd = (view.resx * view.resy * 3);
	unsigned char *pixels;
	pixels = new unsigned char[numInd];

	//Setting up camera/origin space
	Eigen::Vector3d w, u, v;
	w = (view.from - view.at).normalized();
	u = view.up.cross(w).normalized(); 
	v = w.cross(u).normalized();
		
	double aspect = view.resx / view.resy;
	double radians = (view.angle*EIGEN_PI / 180.0);
	double halfWidth = tan(radians / 2.0);
	double pixelWidth = 2.0*halfWidth / view.resx;
	double pixelHeight = pixelWidth / aspect;

	double baseu = -halfWidth;
	double basev = halfWidth/aspect;

	//---------------------DOES THIS DO AA?-----------------------
	//Answer is no...but...kinda? only barely
	//it's better than none and has almost no impact on render time
	//instead of tracing to center of pixel, has a random offset	
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.01, 0.49);
	std::vector<Color> firstPass;
	
	//--------------SCANLINE (starting from top left)--------------
	//For each pixel -> for each "surface" -> set pixcolor
	//pixcolor = intersect ? (eval reflection->surfcolor) : bgcolor 
	for (int j = 0; j < view.resy; j++) 
	{
		for (int i = 0; i < view.resx; i++) 
		{
			//Eigen::Vector3d curr_norm;		
			int index = (i*3 + j*(view.resx * 3));
			double rand_y = 0.5;
			double rand_x = 0.5;
			//Uncomment following two lines to reduce appearance of aliasing
			//double rand_y = distribution(generator);
			//double rand_x = distribution(generator);
			double v_c = basev - (j + rand_y) * pixelHeight;
			double u_c = baseu + (i + rand_x) * pixelWidth;
			Eigen::Vector3d direction = -w + (u * u_c) + (v * v_c);			
			Ray r(view.from, direction);
			//call to color pixel. recursion depth of 5. see dataTypes.cpp
			Color pixColor = trace(bgColor, r, view.from, lights, allGeometry);
			pixels[index] = pixColor.r * 255;
			pixels[index + 1] = pixColor.g * 255;
			pixels[index + 2] = pixColor.b * 255;	
			
		}
		
	}	
	writeOut(outName, pixels, view);
	delete[] pixels;
	pixels = NULL;
	return 0;
}
