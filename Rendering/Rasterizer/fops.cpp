#include "dataTypes.h"
#include "fops.h"

void getView(std::fstream& infile, viewPoint &view) 
{
	std::string line;
	std::string trash;
	std::stringstream s;
	getline(infile, line);
	s << line;
	s >> trash >> view.from[0] >> view.from[1] >> view.from[2];
	s.str("");
	s.clear();
	getline(infile, line);
	s << line;
	s >> trash >> view.at[0] >> view.at[1] >> view.at[2];
	s.str("");
	s.clear();
	getline(infile, line);
	s << line;
	s >> trash >> view.up[0] >> view.up[1] >> view.up[2];
	s.str("");
	s.clear();
	getline(infile, line);
	s << line;
	s >> trash >> view.angle;
	s.str("");
	s.clear();
	getline(infile, line);
	s << line;
	s >> trash >> view.hither;
	s.str("");
	s.clear();
	getline(infile, line);
	s << line;
	s >> trash >> view.resx >> view.resy;
}

void parseFile(std::string file, Color &bgColor, viewPoint &view, 
std::vector<Polygon> &geometry, std::vector<Light> &lights) 
{
	std::fstream inFile;
	inFile.open(file);
	std::string line;
	if (inFile) {
		while (std::getline(inFile, line)) {
			std::stringstream s(line);
			std::string word;
			s >> word;
			char letter = word[0];
			Fill currFill;

			switch (letter) 
			{
				case 'b':
				{
					s >> bgColor.r >> bgColor.g >> bgColor.b;
					break;
				}
				case 'v':
				{
					getView(inFile, view);
					break;
				}
				case 'l':
				{
					Light instance;
					s >> instance.pos[0] >> instance.pos[1] >> instance.pos[2];
					lights.push_back(instance);
					break;
				}
				case 'f':
				{	
					s >> currFill.base.r >> currFill.base.g >> currFill.base.b  
					>> currFill.Kd >> currFill.Ks >> currFill.e >> currFill.Kt >> currFill.ir;
					break;
				}
				case 'p':
				{
					bool isPatch = word.length() > 1 ? true : false;
					int numVerts;
					s >> numVerts;
					Polygon poly;
					poly.setFill(currFill);
					for (int i = 0; i < numVerts; i++) 
					{
						getline(inFile, line);
						std::stringstream vertInfo(line);
						Eigen::Vector3d vert;
						vertInfo >> vert[0] >> vert[1] >> vert[2];
						poly.addVertex(vert);
						if (isPatch){
							Eigen::Vector3d norm;
							vertInfo >> norm[0] >> norm[1] >> norm[2];
							poly.addNormal(norm);
						}
					}
					geometry.push_back(poly);	
					break;
				}
				//Didn't implement sphere for raster
				case 's':
				{
					break;
				}
				default:
					break;
			}
		}
	}
	inFile.close();
}

void writeOut(std::string filename, unsigned char *pixels, viewPoint view) 
{
	int len = view.resx*view.resy*3;
	std::fstream outFile;
	outFile.open(filename, std::fstream::binary | std::fstream::out | std::fstream::trunc);
	outFile << "P6\n";
	outFile << view.resx << " " << view.resy;
	outFile << "\n255\n";
	for (int i = 0; i < len; i += 3)
		outFile << pixels[i] << pixels[i + 1] << pixels[i + 2];
	outFile.close();
}
