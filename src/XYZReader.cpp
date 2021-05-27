// ======================================================================================
// Copyright 2017 State Key Laboratory of Remote Sensing Science, 
// Institute of Remote Sensing Science and Engineering, Beijing Normal University

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ======================================================================================

#include "XYZReader.h"
#include "handlers.h"
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstdlib>


/*** CSF ***/
void read_xyz(std::string& fname, csf::PointCloud& pointcloud)
{
	unsigned numColumns(getNumberOfCols(fname));
	std::ifstream file(fname);

	if (!file)
	{
		std::cout << "Cannot open file "
		          << fname
				  << " for reading." << '\n';
		exit(1);
	}

	double x{}, y{}, z{}, realZ{}, I{}, xx{}, yy{}, zz{}, II{};
	unsigned int gId{}, classification{}, r{}, g{}, b{}, fuelType{};
	unsigned short nor{}, rn{}, dir{}, edge{};

	// ignóranse moitos datos porque a implementación do algoritmo non os soporta
	// non obstante, deixase a estrutura feita para futuras ampliacións
	switch (numColumns)
	{
	case 3:
		while(file >> x >> y >> z)
		{	
			//en CSF orixinal estaba así, non sei se se trata dun erro ou é así
			pointcloud.emplace_back(x, -z, y);
		}
		break;

	case 6:
		while (file >> x >> y >> z >> I >> gId >> classification)
		{
			pointcloud.emplace_back(x, -z, y);
		}
		break;

	case 7:
		while (file >> x >> y >> z >> realZ >> I >> gId >> classification)
		{
			pointcloud.emplace_back(x, -z, y);
		}
		break;

	case 8: 
		// Data from https://www.itc.nl/isprs/wgIII-3/filtertest/downloadsites/
		while (file >> x >> y >> z >> I >> xx >> yy >> zz >> II)
		{
			pointcloud.emplace_back(x, -z, y);
			pointcloud.emplace_back(xx, -zz, yy);
		}
		break;

	// Raw point cloud without RGB
	case 9:
		while (file >> x >> y >> z >> I >> rn >> nor >> dir >> edge >> classification)
		{
			pointcloud.emplace_back(x, -z, y);
		}
		break;

	// Over-segmented point cloud with group Ids
	case 10:
		while (file >> x >> y >> z >> I >> rn >> nor >> dir >> edge >>
			classification >> gId)
		{
			pointcloud.emplace_back(x, -z, y);
		}
		break;

	case 11:
		while (file >> x >> y >> z >> realZ >> I >> rn >> nor >> dir >> edge >>
			classification >> gId)
		{
			pointcloud.emplace_back(x, -z, y);
		}
		break;

	case 12:
		while (file >> x >> y >> z >> I >> rn >> nor >> dir >> edge >>
			classification >> r >> g >> b)
		{
			pointcloud.emplace_back(x, -z, y);
		}
		break;

	default:
		std::cout << "Unrecognized format." << '\n';
		exit(1);
	}
}
