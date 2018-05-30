#include "config_parser.h"
UWBConfigParser::UWBConfigParser(){	
}
bool UWBConfigParser::set_boostar_config_file(const string cfg_file){
	try
	{
		cfg.readFile(cfg_file.c_str());
	}
	catch(const FileIOException &fioex)
	{
		std::cerr << "I/O error while reading file." << std::endl;
		return(EXIT_FAILURE);
	}
	catch(const ParseException &pex)
	{
		std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()<< " - " << pex.getError() << std::endl;
		return(EXIT_FAILURE);
	}
	Setting &root = cfg.getRoot();
	if(!root.exists("uwb")){
		return false;
	}
	Setting &uwb_param=root["uwb"];
	if(!uwb_param.exists("coordinates")){
		cerr<<"coordinates is empty"<<endl;
		return false;
	}
	//anchor pos
	Setting &anchor_pos=uwb_param["coordinates"];
	anchor_pos.lookupValue("anchor_pos",_boostar_param.anchor_pos);
	//solve param 
	if(uwb_param.exists("solve_param")){
		Setting &sovle_param=uwb_param["solve_param"];
	}
	if(!uwb_param.exists("dev")){
		cerr<<"dev param is empty"<<endl;
		return false;
	}
	//io dev 
	Setting &io_dev=uwb_param["dev"];
	io_dev.lookupValue("input", _boostar_param.input_dev);
    io_dev.lookupValue("output", _boostar_param.output_dev);
	if(_boostar_param.input_dev.empty()||_boostar_param.output_dev.empty()){
		return false;
	}
	return true;
}