//
// Created by miguelyermo on 5/3/20.
//

/*
* FILENAME :  laspec.h
* PROJECT  :  rule-based-classifier-cpp
* DESCRIPTION :
*
*
*
*
*
* AUTHOR :    Miguel Yermo        START DATE : 20:23 5/3/20
*
*/
// LAS 1.4 Specification http://www.asprs.org/a/society/committees/standards/LAS_1_4_r13.pdf

#ifndef CPP_LASPEC_H
#define CPP_LASPEC_H

/*#define UNCLASSIFIED 0
#define UNKNOWN 1
#define GROUND  2
#define LOW_VEG 3
#define MED_VEG 4
#define HIGH_VEG 5
#define BUILDING 6
#define ROAD         11
#define ROAD_CANDIDATE 19
#define GRASS 101
#define TREE 105*/

inline constexpr unsigned char UNCLASSIFIED   { 0 };
inline constexpr unsigned char UNKNOWN        { 1 };
inline constexpr unsigned char GROUND         { 2 };
inline constexpr unsigned char LOW_VEG        { 3 };
inline constexpr unsigned char MED_VEG        { 4 };
inline constexpr unsigned char HIGH_VEG       { 5 };
inline constexpr unsigned char BUILDING       { 6 };
inline constexpr unsigned char ROAD           { 11 };
inline constexpr unsigned char ROAD_CANDIDATE { 19 };
inline constexpr unsigned char GRASS          { 101 };
inline constexpr unsigned char TREE           { 105 };


#endif //CPP_LASPEC_H
