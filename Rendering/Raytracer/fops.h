#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include "dataTypes.h"

void getView(std::fstream&, viewPoint&);
void parseFile(std::string, Color&, viewPoint&, std::vector<std::shared_ptr<Surface>>&, std::vector<Light>&);
void writeOut(std::string, unsigned char*, viewPoint);
