//
// Created by miguel.yermo on 6/03/20.
//

/*
* FILENAME :  util.h  
* PROJECT  :  rule-based-classifier-cpp
* DESCRIPTION :
*  
*
*
*
*
* AUTHOR :    Miguel Yermo        START DATE : 14:17 6/03/20
*
*/

#ifndef CPP_UTIL_H
#define CPP_UTIL_H


// Memory Handling
namespace mem
{
template<class C>
void free(C & c)
{
	c.clear();
	c.shrink_to_fit();
}
} // namespace mem

#endif //CPP_UTIL_H
