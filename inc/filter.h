
#ifndef RULE_BASED_CLASSIFIER_FILTER_H
#define RULE_BASED_CLASSIFIER_FILTER_H

#include "Cfg.h"
class Filter
{

private:
	void doSMRF(Cfg& config);
	void doCSF(Cfg& config);

public:
	Filter(int argc, char** argv); 
	void doFilter();
};


#endif //RULE_BASED_CLASSIFIER_FILTER_H