#include <iostream>

#include "filter.h"
#include "handlers.h"
#include "main_options.h"

int main(int argc, char** argv)
{
	setDefaults();
	processArgs(argc, argv);

	Filter filter{};
	filter.doFilter();
	
	return EXIT_SUCCESS;
}
