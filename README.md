# rule-based-classificator-cpp

This project contains an implementation and integration of two LiDAR data filtering 
algorithms: Simple Morphological Filter and Cloth Simultation Filter (forked from https://github.com/jianboqi/CSF).


## CSF
Airborne LiDAR filtering method based on Cloth Simulation.

W. Zhang, J. Qi*, P. Wan, H. Wang, D. Xie, X. Wang, and G. Yan, “An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation,” Remote Sens., vol. 8, no. 6, p. 501, 2016.
(http://www.mdpi.com/2072-4292/8/6/501/htm)

## SMRF
Airborne LiDAR filtering method based on Simple Morphological Filter.

Thomas. J. Pingel et al. An improved simple morphological filter for the terrain classification of airbone LIDAR data. ISPRS  Journal of Photogrammetry and Remote Sensing. 2013, 77, pp. 21-30. ISSN: 0924-2716.

# Installation

To compile and use this project you must make sure you have the Eigen and OpenMP libraries installed. On Ubuntu 20.04, use the following commands:

```
# To install OpenMP
sudo apt install libomp-dev

# To install Eigen (download latest stable release)
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
tar xzvf eigen-3.3.9.tar.gz
cd eigen-3.3.9
mkdir build
cmake ..
make install # run it with root privileges
```
Once you have installed the dependencies you can compile and run the program:

```
mkdir build
cd build
cmake ..
cd ..
make -j $(nproc)
```
# Usage

If you run the program without arguments, it will output the help:

```
$ ./rule_based_classifier_cpp.out 
File not provided. Abort.
 -a: Choose the algorithm (SMRF, CSF)
 -f: Path to config file
 -h: Show this message
 -i: Path to input file
 -o: Path to output file
 -t: Set the number of threads for OpenMP
 -n <--num-points>     : Number of points to be read
```
An example in which SMRF is chosen as filtering algorithm and the input file and 
ouput directory is set: 

```
$ ./rule_based_classifier_cpp.out -a SMRF -f smrf.cfg -i data/input.txt -o out/
```
By default, SRMF is the filtering algorithm.

## Configuration files

Configuration files has the following form and all the parameters are expected
if you provide it. 

CSF: 
```
slop_smooth=true
cloth_resolution=1.0
rigidness=2
iterations=500
class_threshold=0.5
time_step=0.65
```

SMRF:
```
cell_size=1.0
max_slope=0.15
max_window_size=16.0
```