#include "uwb.h"
#include "config_parser.h"
//class for parsing command line
class CmdLineParser
{
    int argc;
    char **argv;
public:
    CmdLineParser(int _argc, char **_argv): argc(_argc), argv(_argv) {}
    bool operator[] ( string param )
    {
        int idx = -1;
        for ( int i = 0; i < argc && idx == -1; i++ ) 
			if ( string ( argv[i] ) == param ) 
				idx = i;
        return ( idx != -1 ) ;
    }
    string operator()(string param, string defvalue = "-1")
    {
        int idx = -1;
        for ( int i = 0; i < argc && idx == -1; i++ ) if ( string ( argv[i] ) == param ) idx = i;
        if ( idx == -1 ) 
			return defvalue;
        else  
			return ( argv[  idx + 1] );
    }
};
void usge(){
	cout<<"use: "<<endl;
	cout<<"-in : input uwb tag uart dev"<<endl;
	cout<<"-out: output uart dev"<<endl;
	cout<<"-pos: archor init coordinate local.( e.g.[[id1,x1,y1,z1];[id2,x2,y2,z2];[];...])"<<endl;
	cout<<"-param: param to use solve the location.(e.g. accStd = 0.957, distStd = 0.33, distOffset = 0.11, trustLevel = 5) "<<endl;
	cout<<"-e:use ekf"<<endl;
	cout<<"-m: solve method TOF(default) or TDOA"<<endl;
	cout<<"-d: dbug message"<<endl;
	cout<<"-f: use config file "<<endl;
}
int main(int argc,char **argv){
	CmdLineParser option(argc,argv);
	string in_dev;
	string out_dev;
	string archor_pos;
	string uwb_param;
	string solve_method="TOF";
	bool dbug=false;
	int useekf=0;
	if(option["-m"])
		solve_method=option("-m");
	
	if(option["-e"])
		useekf=1;
	
	if(option["-d"])
		 dbug=true;
	 
	if(!option["-f"]){
		if(!option["-in"]) {
			cout<<"need set input dev "<<endl;
			usge();
			exit(1);
		}
		in_dev=option("-in");
		
		if(!option["-out"]){
			cout<<"need set output dev"<<endl;
			usge();
			exit(1);
		}
		out_dev=option("-out");
		
		if(!option["-pos"]){
			cout<<"need set anchor coordinate"<<endl;
			usge();
			exit(1);
		}
		archor_pos=option("-pos");
		
		if(!option["-param"]){
			cout<<"use default param "<<endl;
		}else{
			uwb_param=option("-param");
		}
	}else{
		UWBConfigParser uwb_cfg;
		if(option("-f").empty()){
			cout<<"config file is empty"<<endl;
			usge();
			exit(1);
		}
		if(!uwb_cfg.set_boostar_config_file(option("-f"))){
			cout<<"cfg file have err!"<<endl;
			exit(1);
		}
		uwb_cfg.get_boostar_input_dev(&in_dev);
		uwb_cfg.get_boostar_output_dev(&out_dev);
		uwb_cfg.get_boostar_anchor_pos(&archor_pos);
	}
	
	//open tag uart
	Interface_UART tag_uart(in_dev.c_str(),9600);
	//open fc uart
	Interface_UART fc_uart(out_dev.c_str(),9600);
	
	BooStar boostar_uwb(&tag_uart,&fc_uart,useekf,dbug);
	
	if(solve_method.compare("TOF") == 0){
		boostar_uwb.setMethod(BooStar::TOF);
	}else if(solve_method.compare("TDOA") == 0){
		boostar_uwb.setMethod(BooStar::TDOA);
	}else{
		cout<<"Do not support method"<<endl;
		exit(1);
	}
	
	boostar_uwb.setAnchorPos(archor_pos);
	
	if(!uwb_param.empty()) {
		cout<<"param:"<<uwb_param<<endl;
		boostar_uwb.setUWBparam(uwb_param);
	}
	boostar_uwb.run();
}