#include <iostream>

#include "filter.h"

int main(int argc, char** argv)
{
	Filter filter{argc, argv};
	filter.doFilter();
	
	return EXIT_SUCCESS;
}
