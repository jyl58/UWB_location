#pragma once

#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>

#include "libUWBX.h"
#include "Interface_UART.h"

#define NUM_ANCHORS 3

#define MSG_HEADER          0x01
#define MSGID_BEACON_CONFIG 0x02
#define MSGID_BEACON_DIST   0x03
#define MSGID_POSITION      0x04

#define FREQ_CONFIG 5
#define FREQ_POS 2

// structure for messages uploaded to ardupilot
#pragma pack(1)
union beacon_config_msg {
    struct {
        uint8_t beacon_id;
        uint8_t beacon_count;
        int32_t x;
        int32_t y;
        int32_t z;
    } info;
    uint8_t buf[14];
};
union beacon_distance_msg {
    struct {
        uint8_t beacon_id;
        uint32_t distance;
    } info;
    uint8_t buf[5];
};
union vehicle_position_msg {
    struct {
        int32_t x;
        int32_t y;
        int32_t z;
        int16_t position_error;
    } info;
    uint8_t buf[14];
};
#pragma pack()

using namespace std;
class BooStar{
public:
	enum METHOD{
		TOF=0,
		TDOA
	};
	BooStar(Interface_UART *tag_uart,Interface_UART *fc_uart,int useekf=0, bool dbug=false);
	~BooStar();
	void setAnchorPos(const std::string anchor_pos);
	void setUWBparam(const std::string uwb_param){_uwb_param = uwb_param;}
	void setMethod(enum METHOD solve_method);
	void run();
	
private:
	//system time clock 
	double tic();
	bool read_tag_message(std::string &mesge);
	void handle_tag_message(const std::string message);
	void send_message(uint8_t msg_id, uint8_t data_len, uint8_t data_buf[]);
	void send_anchor_config();
	void send_pos();
	int anchor_is_valid(int anchor_id);
	
	enum METHOD _solve_method;
	std::string _anchor_pos;
	std::string _uwb_param;
	int _useKF;
	bool _distance_updated;
	bool _pos_updated;
	//tag uart interface
	Interface_UART* _tag_uart;
	//fc uart interface 
	Interface_UART* _fc_uart;
	bool _dbug;
	
	union beacon_distance_msg _distance_msg[NUM_ANCHORS];
	union beacon_config_msg   _config_msg[NUM_ANCHORS];
	union vehicle_position_msg _tag_pos;
	
	std::vector<int> _anchor_list;
};