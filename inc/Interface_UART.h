/*
 * File:   Interface_UART.hpp
 * Author: Joe Churchwell
 *
 * Created on June 23, 2015, 8:46 PM
 */

#ifndef INTERFACE_UART_HPP
#define	INTERFACE_UART_HPP

#define TRUE 1
#define FALSE 0

#include <stdlib.h>
#include <inttypes.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <string>

typedef unsigned char uint8_t;

class Interface_UART {

public:
    Interface_UART(const char *portname, int speed);
    virtual ~Interface_UART();

    int SetupSerial(int fdes,int baud,int databits,int stopbits,int parity);
    void flush_buffer(void);
    int read_data(std::string &data_rtn);
    int write_data(uint8_t *data_out, uint8_t byte_count);
	int write_data(uint8_t data);

private:

    void set_blocking (int fd, int should_block);
    int set_interface_attribs (int fd, int speed, int parity);
    int fd;
};


#endif	/* INTERFACE_UART_HPP */
