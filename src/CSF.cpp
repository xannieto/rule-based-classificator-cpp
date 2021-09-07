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

#define DLL_IMPLEMENT

#include "AccumTime.h"
#include "c2cdist.h"
#include "Cloth.h"
#include "CSF.h"
#include "handlers.h"
#include "main_options.h"
#include "Rasterization.h"
#include "Vec3.h"
#include "XYZReader.h"

#include <fstream>

CSF::CSF(int index) {
    m_params.bSloopSmooth     = true;
    m_params.time_step        = 0.65;
    m_params.class_threshold  = 0.5;
    m_params.cloth_resolution = 1;
    m_params.rigidness        = 3;
    m_params.interations      = 500;

    m_index = index;
}

CSF::CSF() {
	m_params.bSloopSmooth = true;
	m_params.time_step = 0.65;
	m_params.class_threshold = 0.5;
	m_params.cloth_resolution = 1;
	m_params.rigidness = 3;
	m_params.interations = 500;
	m_index = 0;
}

CSF::~CSF()
{}

void CSF::setPointCloud(std::vector<Lpoint> points) {
    point_cloud.m_points.resize(points.size());

    int pointCount = static_cast<int>(points.size());
    #pragma omp parallel for
    for (int i = 0; i < pointCount; i++) {
        Lpoint las;
        las.setX(points[i].x());
        las.setY(-points[i].z());
        las.setZ(points[i].y());
        point_cloud.m_points[i] = las;
    }
}

void CSF::setPointCloud(double *points, int rows) {
	#define A(i, j) points[i + j * rows]

    for (int i = 0; i < rows; i++) {
        Lpoint p;
        p.setX(A(i, 0));
        p.setY(-A(i, 2));
        p.setZ(A(i, 1));
        point_cloud.m_points.push_back(p);
    }
}

void CSF::setPointCloud(csf::PointCloud& pc) {
    point_cloud.m_points.resize(pc.m_points.size());
    int pointCount = static_cast<int>(pc.m_points.size());
    #pragma omp parallel for
    for (int i = 0; i < pointCount; i++) {
        Lpoint las;
        las.setX(pc.m_points[i].x());
        las.setY(-pc.m_points[i].z());
        las.setZ(pc.m_points[i].y());
        point_cloud.m_points[i] = las;
    }
}

void CSF::setPointCloud(std::vector<std::vector<float> > points) {
    point_cloud.m_points.resize(points.size());
    int pointCount = static_cast<int>(points.size());
    #pragma omp parallel for
    for (int i = 0; i < pointCount; i++) {
        Lpoint las;
        las.setX(points[i][0]);
        las.setY(-points[i][2]);
        las.setZ(points[i][1]);
        point_cloud.m_points[i] = las;
    }
}

void CSF::readPointsFromFile(std::string filename) {
    this->point_cloud.m_points.resize(0);
    this->point_cloud.m_points = readPointCloud(filename);
    
    //invert
    for (Lpoint& point : this->point_cloud.m_points)
    {
        double y{point.y()}, z{point.z()};
        point.setY(-z); point.setZ(y);
    }
}

void CSF::do_filtering(std::vector<int>& groundIndexes,
                       std::vector<int>& offGroundIndexes,
                       bool              exportCloth) {
    // Terrain
    std::cout << "[" << this->m_index << "] Configuring terrain..." << std::endl;
    Lpoint bbMin, bbMax;

	AccumTime::instance().start();
    point_cloud.computeBoundingBox(bbMin, bbMax);
	AccumTime::instance().stop("Computing bounding box");

    double cloth_y_height = 0.05;

    int  clothbuffer_d = 2;
    Vec3 origin_pos(
        bbMin.x() - clothbuffer_d * m_params.cloth_resolution,
        bbMax.y() + cloth_y_height,
        bbMin.z() - clothbuffer_d * m_params.cloth_resolution
    );

    int width_num = static_cast<int>(
        std::floor((bbMax.x() - bbMin.x()) / m_params.cloth_resolution)
    ) + 2 * clothbuffer_d;

    int height_num = static_cast<int>(
        std::floor((bbMax.z() - bbMin.z()) / m_params.cloth_resolution)
    ) + 2 * clothbuffer_d;

    std::cout << "[" << this->m_index << "] Configuring cloth..." << std::endl;
    std::cout << "[" << this->m_index << "]  - width: " << width_num << " "
         << "height: " << height_num << '\n';

    Cloth cloth(
        origin_pos,
        width_num,
        height_num,
        m_params.cloth_resolution,
        m_params.cloth_resolution,
        0.3,
        9999,
        m_params.rigidness,
        m_params.time_step
    );

    std::cout << "[" << this->m_index << "] Rasterizing..." << '\n';
	AccumTime::instance().start();
    Rasterization::RasterTerrain(cloth, point_cloud, cloth.getHeightvals());
	AccumTime::instance().stop("Rasterize terrain");

    double time_step2 = m_params.time_step * m_params.time_step;
    double gravity    = 0.2;

    std::cout << "[" << this->m_index << "] Simulating..." << '\n';
	AccumTime::instance().start();
    cloth.addForce(Vec3(0, -gravity, 0) * time_step2);

    // boost::progress_display pd(params.interations);
    for (int i = 0; i < m_params.interations; i++) {
        double maxDiff = cloth.timeStep();
        cloth.terrCollision();
		//params.class_threshold / 100
        if ((maxDiff != 0) && (maxDiff < 0.005)) {
            // early stop
            break;
        }
        // pd++;
    }
	AccumTime::instance().stop("Cloth Simulation");
    if (m_params.bSloopSmooth) {
        std::cout << "[" << this->m_index << "]  - post handle..." << '\n';
		AccumTime::instance().start();
        cloth.movableFilter();
		AccumTime::instance().stop("Post handle");
    }

    //if (exportCloth)
    cloth.saveToFile(m_inputFile, mainOptions.outputDirName + "/cloth_nodes.txt");

    c2cdist c2c(m_params.class_threshold);
	AccumTime::instance().start();
    c2c.calCloud2CloudDist(cloth, point_cloud, groundIndexes, offGroundIndexes);
	AccumTime::instance().stop("Cloud to cloud distance");
}

void CSF::savePoints(std::vector<int> grp, std::string path) {
    if (path == "") {
        return;
    }

    std::ofstream f1(path.c_str(), std::ios::out);

    if (!f1)
        return;

    for (std::size_t i = 0; i < grp.size(); i++) {
        f1 << std::fixed << std::setprecision(8)
           << point_cloud.m_points[grp[i]].x()  << "	"
           << point_cloud.m_points[grp[i]].z()  << "	"
           << -point_cloud.m_points[grp[i]].y() << std::endl;
    }

    f1.close();
}
