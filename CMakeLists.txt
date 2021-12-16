cmake_minimum_required(VERSION 3.14)
project(proxecto-tfg)

set(CMAKE_CXX_STANDARD 20)
if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG")
elseif (CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG")
endif ()

#-lstdc++fs : Experimental Filesystem (for some reason I cant link boost)
set(EXTERNAL_LIBS "-lstdc++fs")

#add_compile_options("-Wall")
add_compile_options("-O3")

# Includes
include_directories(inc)

find_package(OpenMP REQUIRED)
list(APPEND EXTERNAL_LIBS OpenMP::OpenMP_CXX)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
list(APPEND EXTERNAL_LIBS Eigen3::Eigen)

set(SRC_FILES
    src/main.cpp
	src/handlers.cpp
	src/point.cpp
	src/octree.cpp
	src/vector.cpp
    src/main_options.cpp
	src/SMRF.cpp
	src/TimeWatcher.cpp
	src/c2cdist.cpp
	src/Cloth.cpp
	src/Constraint.cpp
	src/CSF.cpp
	src/Particle.cpp
	src/point_cloud.cpp
	src/Rasterization.cpp
	src/AccumTime.cpp
	src/filter.cpp)

set(INC_FILES
    inc/handlers.h
	inc/point.h
	inc/octree.h
	inc/vector.h
	inc/laspec.h
    inc/main_options.h
	inc/convexHull.hpp
	inc/SMRF.h
	inc/TimeWatcher.h
	inc/pseudoInverseMoorePenrose.h
	inc/c2cdist.h
	inc/Cfg.h
	inc/Cloth.h
	inc/CSF.h
	inc/Particle.h
	inc/point_cloud.h
	inc/Vec3.h
	inc/AccumTime.h
	inc/filter.h)

add_executable(compute_dtm.out ${SRC_FILES} ${INC_FILES})

target_link_libraries(compute_dtm.out ${EXTERNAL_LIBS})
