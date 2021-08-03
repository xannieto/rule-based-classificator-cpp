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

#include "point_cloud.h"

csf::PointCloud::PointCloud(std::vector<Lpoint>& pc)
: m_points(pc)
{
}

void csf::PointCloud::computeBoundingBox(Lpoint& bbMin, Lpoint& bbMax) {
    if (m_points.empty()) {
        bbMin = bbMax = Lpoint();
        return;
    }

    bbMin = bbMax = m_points.at(0);

    for (std::size_t i = 1; i < m_points.size(); i++) { // zwm
        const Lpoint& P = m_points.at(i);
        
        if (P.x() < bbMin.x())
        {
            bbMin.setX(P.x());
        }
        else if (P.x() > bbMax.x())
        {
            bbMax.setX(P.x());
        }

        if (P.y() < bbMin.y())
        {
            bbMin.setY(P.y());
        }
        else if (P.y() > bbMax.y())
        {
            bbMax.setY(P.y());
        }

        if (P.z() < bbMin.z())
        {
            bbMin.setX(P.x());
        }
        else if (P.z() > bbMax.z())
        {
            bbMax.setZ(P.z());
        }
    }
}
