/*
 * Sample program for PiPS2 library.
 * Compile: g++ sample.cpp PiPS2.cpp -o sample -lwiringPi
 *
 * Sets up the PS2 remote for analog mode and to return all pressure values.
 * Reads controller every 10ms and prints the buttons that are pressed to the console.
 * BTN_START uses the functionality to detect a button push or release.
 * Holding R2 will print the pressure values of the right analog stick to console.
 * All other buttons just cause a message to be printed after every read if they are being pressed.
 *
 * You can just implement the same functionality from the Start button or from the R2 functionality
 * for any keys.
 *
 */

#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "ps2pi.h"

int arduino_fd;

#define READDELAYMS 10
char throttle_string[81];

char camera_orientation_string[81];
int old_x;
int old_y;


int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
                perror ("tcgetattr");
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
                perror("tcsetattr");
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                perror("tggetattr");
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                perror("attributes");
}


/**
* Send control strings to the Arduino, telling it
* to set Motor 1 and Motor 2 speeds accoring to the joystick settings.
*
 * */
void throttle(int x, int y, void *user_data){
	char *p= (char*) user_data;
	char buf [128];
	int m1, m2;

	if (x==old_x && y==old_y){
		return;
	}

	old_x = x;
	old_y = y;

	m1 = m2 = y
	 * 3;

	sprintf(p, "x:%d  y:%d ", x, y);
	if (x < 0){
		m1 += x*3;
	} else if (x>0){
		m2 -= x*3;
	}
	printf(" m1=%03d m2=%03d\n", m1, m2);
	sprintf(buf, "m2=%d,%d\n", m1,m2);
	write (arduino_fd, buf, strlen(buf));
	usleep ((strlen(buf) + 25) * 100);             // sleep enough to transmit plus                                     // receive 25:  approx 100 uS per char transmit
	if (read(arduino_fd, buf, 0) != -1){
		int n = read (arduino_fd, buf, sizeof buf);
	}

}

/*
 * Adjust the camera / ultrasound assembly according to the right joystick
 * */
void camera_gimbal_callback(int x, int y, void *user_data){
	char *p= (char*)user_data;
	sprintf(p, "  Cam x:%d y:%d ", x, y);
}

void xaction(int pressure, void* user_data)
{
	printf("Snapshot taken\n");
}

void startaction(int pressure, void* user_data)
{
		printf("START pressed\n");
}

void squareaction(int pressure, void* user_data)
{
		printf("SQUARE pressed\n");
}

void selectaction(int pressure, void* user_data)
{
		printf("SELECT pressed\n");
}

void terminate(int pressure, void *user_data)
{
	printf("STOP\n");
	write(arduino_fd, "m=0,0\n",5);

}

int main(int ac, char *av[])
{
	char* portname = "/dev/ttyUSB0";
	printf("Turnimator 4 Control Program\n");
	if (ac>1){
		portname = av[1];
	}
	if (wiringPiSetupPhys() == -1) {
		perror("Unable to start wiringPi");
		exit(-1);
	}

	arduino_fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (arduino_fd < 0)
	{
        perror(portname);
        exit(-1);
	}

	set_interface_attribs (arduino_fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (arduino_fd, 0);                // set no blocking

	ps2pi_t pips2;

	pips2.begin(11, 3, 5, 13);

//	int returnVal = pips2.reInitializeController(ANALOGMODE);
	delay(50);
	pips2.printData();

	pips2.setXAction(xaction, throttle_string);
	pips2.setStartAction(startaction, NULL);
	pips2.setSquareAction(squareaction, NULL);
	pips2.setSelectAction(selectaction, NULL);
	pips2.setCircleAction(terminate, NULL);

	pips2.setLeftJoyCallback(throttle, throttle_string);
	pips2.setRightJoyCallback(camera_gimbal_callback, camera_orientation_string);

	while (1) {
		pips2.readPS2();
		pips2.dispatch();
		printf("\r%s %s", throttle_string, camera_orientation_string);
	}
}
