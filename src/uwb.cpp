#include <boost/algorithm/string.hpp>
#include <sys/time.h>
#include "uwb.h"
BooStar::BooStar(Interface_UART *tag_uart,Interface_UART *fc_uart, int useekf,bool dbug)
:_solve_method(BooStar::TOF)
,_anchor_pos("")
,_uwb_param("")
,_useKF(useekf)
,_distance_updated(false)
,_pos_updated(false)
,_tag_uart(tag_uart)
,_fc_uart(fc_uart)
,_dbug(dbug)
{
	
}
BooStar::~BooStar(){
	delete _tag_uart;
	delete _fc_uart;
	UWBX_destroyAll();
}
//"[[10,0,0,0];[11,1,0,0];[12,0.5,0.86,0];[13,0.5,0.5,0]]";
void BooStar::setAnchorPos(const string anchor_pos){
	string anchor_coor = anchor_pos;
	if(anchor_coor.front() != '[' || anchor_coor.back() != ']'){
		cout<<"err:anchor pos format err!!"<<endl;
		return ;
	}
	anchor_coor.erase(0,1); // delete first '['	
	anchor_coor.pop_back(); // delete last ']'
	
	vector<string> submsg;
	boost::split(submsg,anchor_coor,boost::is_any_of(";"));
	for(int i=0; i < NUM_ANCHORS; i++){
		submsg[i].erase(0,1); // delete first '['	
		submsg[i].pop_back(); // delete last ']'
		vector<string> subsubmsg;
		boost::split(subsubmsg,submsg[i],boost::is_any_of(","));
		if(subsubmsg.size() !=4 ){
			cout<<"err:: input anchor format err !"<<endl;
			return ;
		}
		if(_dbug)cout<<"id:"<<subsubmsg[0]<<" x:"<<subsubmsg[1]<<" y:"<<subsubmsg[2]<<" z:"<<subsubmsg[3]<<endl;
		
		_anchor_list.push_back(std::stoi(subsubmsg[0]));
		_config_msg[i].info.beacon_id=(uint8_t )i;//std::stoi(subsubmsg[0]);// id
		_config_msg[i].info.x = (uint32_t )(std::stof(subsubmsg[1])*1000);// x m-->mm 
		_config_msg[i].info.y = (uint32_t )(std::stof(subsubmsg[2])*1000);// y 
		_config_msg[i].info.z = (uint32_t )(std::stof(subsubmsg[3])*1000);// z 
		_config_msg[i].info.beacon_count=NUM_ANCHORS;
	}
	
	string temp_anchor_pos = anchor_pos;
	size_t pos = temp_anchor_pos.find(";");
	while(pos !=  string::npos){
		temp_anchor_pos.replace(pos,1,",");
		pos = temp_anchor_pos.find(";");
	}
	_anchor_pos=temp_anchor_pos;
	if(_dbug)cout<<"anchor pos:"<< _anchor_pos<<endl;
}
void BooStar::setMethod(enum METHOD solve_method){
	_solve_method=solve_method;
}
//read tag message
bool  BooStar::read_tag_message(string &msge){
	if(_tag_uart->read_data(msge)> 0){
		std::size_t head=msge.find("@");
		std::size_t end=msge.find("#");
		//Do not find the "@" OR "#" 
		if(head <0 || end <0 || end <= head)
			return false;
		msge=msge.substr(head,end-head+1);
		return true;
	}
	return false;
}
void  BooStar::send_message(uint8_t msg_id, uint8_t data_len, uint8_t data_buf[]){
    // sanity check
    if (data_len == 0) {
        return;
    }
    // message is buffer length + 1 (for checksum)
    uint8_t msg_len = data_len+1;

    // calculate checksum and place in last element of array
    uint8_t checksum = 0;
    checksum ^= msg_id;
    checksum ^= msg_len;
    for (uint8_t i=0; i< data_len; i++) {
        checksum = checksum ^ data_buf[i];
    }
	_fc_uart->write_data(MSG_HEADER);
	_fc_uart->write_data(msg_id);
	_fc_uart->write_data(msg_len);
	_fc_uart->write_data(data_buf, data_len);
	_fc_uart->write_data(&checksum, 1);
}
//@id,time,anchor id,anchor distance,...,#
//@1,102,30,654.3,49,1123.3,...#
void BooStar::handle_tag_message(const string msg){
	string message = msg;
	//check the msg valid 
	if(message.front() != '@' || message.back() != '#'){
		cout<<" cerr: message formation err "<<endl;
		return ;
	}
	//solve the tag pos 
	int x,y;
	if(_solve_method == BooStar::TDOA){
		// dot support now 
		/*int tagid=1;
		if(( UWBX_dataUpdate(message.c_str())>=0 )&&(UWBX_getPosition(1,&x, &y,&tagid) >= 0)){
				_tag_pos.info.x = x*10;  //cm-->mm
				_tag_pos.info.y = y*10;
				_tag_pos.info.z = 0;
				_pos_updated = true;
				if(_dbug) cout<<tagid<<"; x coordinate(cm): "<<x<<"; y coordinate(cm)"<< y<<endl;
		}else{
			cout<<"err: TDOA do not solve the location"<<endl;
			return ;
		}
		*/
	}else{
		if(UWBX_locate( message.c_str(), &x, &y, _useKF ) >= 0){
			_tag_pos.info.x = x*10;  //cm-->mm
			_tag_pos.info.y = y*10;
			_tag_pos.info.z = 0;
			_tag_pos.info.position_error = 0.3;
			_pos_updated = true;
			if(_dbug) cout<<"x coordinate(cm): "<<x<<"; y coordinate(cm)"<< y<<endl;
		}else{
			cout<<"err: TOF do not solve the locaation "<<endl;
		}
	}
	// delete first '@'	
	message.erase(0,1); 
	// delete last '#'
	message.pop_back(); 
	
	//split messge by ','
	vector<string> submsg;
	boost::split(submsg,message,boost::is_any_of(","));
	if(submsg.size()< 4)
		return;
	submsg.erase(submsg.begin(),submsg.begin()+2); //ersea the tag id and the time 
	//parse the distanc between tag and parse 
	for(int i =0; i<submsg.size()/2; i++ ){
		if(_dbug)cout<<"submsg anchor id:"<<submsg[2*i]<<" distance: "<<submsg[2*i+1]<<" cm"<<endl;
		if(!submsg[2*i].empty()){
			int anchor_id = anchor_is_valid(std::stoi(submsg[2*i]));
			if( anchor_id >= 0 && i< NUM_ANCHORS){
				memset(&_distance_msg[i],0,sizeof(union beacon_distance_msg));
				_distance_msg[i].info.beacon_id =(uint8_t)anchor_id ;
				_distance_msg[i].info.distance = (uint32_t)(std::stof(submsg[2*i+1])*10);  //cm-->mm 
				send_message(MSGID_BEACON_DIST, sizeof(_distance_msg[i].buf), _distance_msg[i].buf); // send distance  
			}else{
				cout<<"err:: get invalid anchor id"<<endl;
			}
		}
	}
}
void BooStar::send_anchor_config(){
	for(int i=0; i < NUM_ANCHORS; i++){
		send_message(MSGID_BEACON_CONFIG, sizeof(_config_msg[i].buf), _config_msg[i].buf);
	}
}
void BooStar::send_pos(){
	_pos_updated = false ;
	send_message(MSGID_POSITION, sizeof(_tag_pos.buf), _tag_pos.buf);
}
//ms
double BooStar::tic(){
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec*1000 + ((double)t.tv_usec)/1000);
}
//check the anchor id is valid or not 
int BooStar::anchor_is_valid(int anchor_id){
	if(_anchor_list.empty())
		return -1;
	for(int index=0; index < _anchor_list.size(); index++){
		if(_anchor_list[index] == anchor_id)
			return index;
	}
	return -1;
}
void BooStar::run(){
	if(UWBX_setAnchorPos(_anchor_pos.c_str()) < 0){
		cout<<"err: set anchor pos  err!"<<endl;
		exit(1);
	}
		
	if(!_uwb_param.empty()) {
		if(UWBX_setParam(_uwb_param.c_str()) < 0){
			cout<<"err: set uwb param  err!"<<endl;
			exit(1);
		}	
	}
	if(_solve_method == BooStar::TDOA)
		UWBX_startTDOA();
	
	double config_time = 0;
	double pos_time = 0;
	string tag_message;
	if(_dbug) cout<<"start run location "<<endl;
	
	while(1){
		tag_message.clear();
		//read tag uart message 
		if(!read_tag_message(tag_message)){
			if(_dbug) cout<<"err: read msg from tag err !"<<endl;
			continue;
		}
		//handle message 
		handle_tag_message(tag_message);
		
		//send message to fc_uart
		if(config_time ==0||(tic()-config_time > 1000/FREQ_CONFIG)){
			config_time=tic();  //ms
			send_anchor_config();
		}
		if((pos_time==0||(tic()-pos_time > 1000/FREQ_POS))&&_pos_updated){
			pos_time=tic();  //ms
			send_pos();
		}	
	}
}
