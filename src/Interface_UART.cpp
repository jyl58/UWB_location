/*
 * File:   Interface_UART.cpp
 * Author: Joe Churchwell
 *
 * Created on June 23, 2015, 8:46 PM
 */

#include "Interface_UART.h"
#include <iostream>

Interface_UART::Interface_UART(const char* portname, int speed)
:fd(-1)
{
    fd = ::open(portname, (O_RDWR | O_NOCTTY | O_SYNC));
    if (fd < 0){
           std::cout<<"ubable open the port "<<portname<<std::endl;
            return;
    }

    SetupSerial(fd, speed, 8, 1, 'N');
}

Interface_UART::~Interface_UART() {

    /* Shutdown the file descriptor*/
    close(fd);
}

void Interface_UART::flush_buffer(){

    tcflush(fd, TCIOFLUSH);
}

int Interface_UART::read_data(std::string &data_rtn){

  char c;
  int rx_count = 0;
  while(1){

      if (read(fd, &c, 1) <= 0){
          //wait for messages
          usleep(10000);
      }else{
		  if (c == '\n'){
              break;
          }
		  data_rtn.push_back(c);
          rx_count++;
      }
  }
  return rx_count;
}

int Interface_UART::write_data(uint8_t *data_out, uint8_t len){

    if (fd != -1)
    {
        int count = ::write(fd, data_out, len);
		if (count < 0)
        {
			return 0;
        }
		return count;
    }
}
int Interface_UART::write_data(uint8_t data){
	return write_data(&data,1);
}

void Interface_UART::set_blocking(int fd, int should_block){

    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
            printf ("error %d from tggetattr", errno);
            return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
            printf ("error %d setting term attributes", errno);

}

/*
	Setup the communication options
	Hopefully POSIX compliant
*/
int Interface_UART::SetupSerial(int fdes,int baud,int databits,int stopbits,int parity)
{
	int n;
	struct termios options; //,options2;

	/* Get the current options */
	if (tcgetattr(fdes,&options) < 0) {
		perror("SetupSerial 1");
		return(FALSE);
	}

	/* Set the baud rate */
	switch (baud) {
            case 1200:
               n =  cfsetispeed(&options,B1200);
               n += cfsetospeed(&options,B1200);
               break;
            case 2400:
               n =  cfsetispeed(&options,B2400);
               n += cfsetospeed(&options,B2400);
               break;
            case 4800:
                n =  cfsetispeed(&options,B4800);
                n += cfsetospeed(&options,B4800);
                break;
            case 9600:
                n =  cfsetispeed(&options,B9600);
                n += cfsetospeed(&options,B9600);
               break;
            case 19200:
               n =  cfsetispeed(&options,B19200);
               n += cfsetospeed(&options,B19200);
               break;
            case 38400:
               n =  cfsetispeed(&options,B38400);
               n += cfsetospeed(&options,B38400);
               break;
            case 57600: // witilt
               n =  cfsetispeed(&options,B57600);
               n += cfsetospeed(&options,B57600);
               break;
            case 115200:
               n =  cfsetispeed(&options,B115200);
               n += cfsetospeed(&options,B115200);
               break;
            default:
		fprintf(stderr,"Unsupported baud rate\n");
		return(FALSE);
	}

	if (n != 0) {
		perror("SetupSerial 2");
		return(FALSE);
	}

   // Set the data size
   options.c_cflag &= ~CSIZE; // Character size mask
   switch (databits) {
   case 7:
      options.c_cflag |= CS7;
      break;
   case 8:
      options.c_cflag |= CS8;
      break;
   default:
      fprintf(stderr,"Unsupported data size\n");
      return(FALSE);
   }

	// Set parity
	switch (parity) {
	case 'n':
	case 'N':
		options.c_cflag &= ~PARENB; // Clear parity enable
		break;
	case 'o':
	case 'O':
		options.c_cflag |= PARENB; // Parity enable
		options.c_cflag |= PARODD; // Enable odd parity
		break;
	case 'e':
	case 'E':
		options.c_cflag |= PARENB; // Parity enable
		options.c_cflag &= ~PARODD; // Turn off odd parity = even
		break;
	default:
		fprintf(stderr,"Unsupported parity\n");
		return(FALSE);
	}

	// Set stop bits
	switch (stopbits) {
	case 1:
		options.c_cflag &= ~CSTOPB; // Not 2 stop bits = One stop bit
		break;
	case 2:
		options.c_cflag |= CSTOPB; // Two stop bits
		break;
	default:
		fprintf(stderr,"Unsupported stop bits\n");
		return(FALSE);
	}

   // Non blocking return immediately with data
   options.c_cc[VMIN] = 0;
   options.c_cc[VTIME] = 0;

   // Local flags
   options.c_lflag = 0;  // No local flags
   options.c_lflag &= ~ICANON; // Don't canonicalise
   options.c_lflag &= ~ECHO; // Don't echo
   options.c_lflag &= ~ECHOK; // Don't echo

   // Control flags
   options.c_cflag &= ~CRTSCTS; // Disable RTS/CTS
   options.c_cflag |= CLOCAL; // Ignore status lines
   options.c_cflag |= CREAD; // Enable receiver
   options.c_cflag |= HUPCL; // Drop DTR on close

   // oflag - output processing
   options.c_oflag &= ~OPOST; // No output processing
   options.c_oflag &= ~ONLCR; // Don't convert linefeeds

   // iflag - input 3
   options.c_iflag |= IGNPAR; // Ignore parity
   options.c_iflag &= ~ISTRIP; // Don't strip high order bit
   options.c_iflag |= IGNBRK; // Ignore break conditions
   options.c_iflag &= ~INLCR; // Don't Map NL to CR
   options.c_iflag &= ~ICRNL; // Don't Map CR to NL
   options.c_iflag |= (IXON | IXOFF | IXANY); // xon/xoff flow control

   // options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF);

	// Update the options and do it NOW
	if (tcsetattr(fdes,TCSANOW,&options) < 0) {
		perror("SetupSerial 3");
		return(FALSE);
	}

   // Clear the line
   tcflush(fdes,TCIFLUSH);

	return(TRUE);
}


int Interface_UART::set_interface_attribs(int fd, int speed, int parity){

    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
            printf ("error %d from tcgetattr", errno);
            return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing

    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);



    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
            printf ("error %d from tcsetattr", errno);
            return -1;
    }

    cfmakeraw(&tty);

    return 0;

}
