#pragma once

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <libconfig.h++>

using namespace std;
using namespace libconfig;

class UWBConfigParser{
public:
	UWBConfigParser();
	bool set_boostar_config_file(const string cfg_file);
	void get_boostar_anchor_pos(string *anchor_pos){*anchor_pos=_boostar_param.anchor_pos;}
	void get_boostar_solve_param(string *solve_param){*solve_param=_boostar_param.solve_param;}
	void get_boostar_input_dev(string *input_dev){*input_dev=_boostar_param.input_dev;}
	void get_boostar_output_dev(string *output_dev){*output_dev=_boostar_param.output_dev;}
	
private:
	Config cfg;
	struct BOOSTAR_PARAM{
		string anchor_pos;
		string solve_param;
		string input_dev;
		string output_dev;
	}_boostar_param;
};