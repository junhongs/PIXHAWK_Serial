/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.		 
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file serial_control_main.c
 *
 * Matlab CSV / ASCII format interface at 921600 baud, 8 data bits,
 * 1 stop bit, no parity
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <poll.h>

#include <drivers/drv_rc_input.h>
#include <uORB/topics/manual_control_setpoint.h>


#define INBUF_SIZE 64


__EXPORT int serial_control_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
static orb_advert_t	_manual_control_pub;	
static int mode_ch = 0;


static uint8_t inBuf[INBUF_SIZE];
static uint8_t checksum;
static uint8_t indRX;
static uint8_t cmdMSP;
static int tested_value = 0;
static uint64_t rcSerial_timestamp = 0;
 enum MSP_protocol_bytes {
	IDLE,
	HEADER_START,
	HEADER_M,
	HEADER_ARROW,
	HEADER_SIZE,
	HEADER_CMD
};

 enum Serial_Com{
	COM_RECEIVED,
	COM_UNRECEIVED
};


struct  packet_s{
	float x,y,z,r;
	uint16_t a[4];  
};
//__attribute__((packed)) 
static struct packet_s rcSerial;

static uint8_t read8(void);
static void serialCom(int serial_fd);
static int evaluateCommand(uint8_t c);


int serial_control_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: daemon {start|stop|status|test|mode} [-p <additional params>]\nstart [/dev/ttySx]\nmode\n\n");
	exit(1);
}
static uint8_t read8()  {
	return inBuf[indRX++]&0xff;
}
static void __attribute__ ((noinline)) s_struct_w(uint8_t *cb,uint8_t siz) {
  while(siz--) *cb++ = read8();
}
/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_spawn_cmd().
 */
int serial_control_main(int argc, char *argv[])
{


	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start"))
	{
		if (thread_running)
		{
			warnx("already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("serial_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 5,
					 2000,
					 serial_control_thread_main,
					 (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop"))
	{
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status"))
	{
		if (thread_running) {
			warnx("running");
		} else {
			warnx("stopped");
		}

		exit(0);
	}

	if(!strcmp(argv[1],"test"))
	{
		printf("tested value is %d\n",tested_value);

		exit(0);
	}
	if(!strcmp(argv[1],"mode"))
	{
		if(mode_ch == 1)
			mode_ch = 0;
		else
			mode_ch = 1;
		exit(0);
	}


	usage("unrecognized command");
	exit(1);
}

int serial_control_thread_main(int argc, char *argv[])
{
	const char* uart_name = "/dev/ttyS6";

	if (argc > 1) {
		uart_name = argv[1];
	}

	warnx("opening port %s", uart_name);

	int serial_fd = open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
	//int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

	//unsigned speed = 921600;

	if (serial_fd < 0) {
		err(1, "failed to open port: %s", uart_name);
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(serial_fd, &uart_config)) < 0) {
		warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
		close(serial_fd);
		return -1;
	}

	/* Clear ONLCR flag (which appends a CR for every LF) */
//	uart_config.c_oflag &= ~ONLCR;

	/* USB serial is indicated by /dev/ttyACM0*/
/*	if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {
*/
		/* Set baud rate */
/*		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			warnx("ERR SET BAUD %s: %d\n", uart_name, termios_state);
			close(serial_fd);
			return -1;
		}

	}
*/	
	memset(&rcSerial,0,sizeof(rcSerial));
	if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR SET CONF %s\n", uart_name);
		close(serial_fd);
		return -1;
	}

	/* subscribe to vehicle status, attitude, sensors and flow*/
	struct accel_report accel0;
	struct accel_report accel1;
	struct gyro_report gyro0;
	struct gyro_report gyro1;

	/* subscribe to parameter changes */
	int accel0_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	int accel1_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 1);
	int gyro0_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), 0);
	int gyro1_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), 1);
	int _rc_sub 		= orb_subscribe(ORB_ID(input_rc));

	thread_running = true;

	while (!thread_should_exit)
	{

		/*This runs at the rate of the sensors */
		struct pollfd fds[] = {
				{ .fd = accel0_sub, .events = POLLIN }
		};

		/* wait for a sensor update, check for exit condition every 500 ms */
		int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), 500);

		if (ret < 0)
		{
			/* poll error, ignore */

		}
		else if (ret == 0)
		{
			/* no return value, ignore */
			warnx("no sensor data");
		}
		else
		{

			/* accel0 update available? */
			if (fds[0].revents & POLLIN)
			{

//#define DEBUG
				if(mode_ch == 1)
				{
					char c = 0;
					 int rn=0;
					 rn = read(serial_fd,&c,1);
					 if(rn == 1) 
					 {
					 	write(serial_fd,&c,1);
					 	printf("serial_control::%d,%c\n",c,c);
					}
				}
//#else
				else
					{
					serialCom(serial_fd);


					static struct manual_control_setpoint_s manual;
					memset(&manual, 0 , sizeof(manual));
					

					manual.timestamp = hrt_absolute_time();

					if( manual.timestamp - rcSerial_timestamp > 5 * 1e5f)
					{
								manual.y = 0;
								manual.x = 0;
								manual.r = 0;
								manual.z = 0.5;

								manual.flaps = SWITCH_POS_MIDDLE;
								manual.aux1 = SWITCH_POS_MIDDLE;
								manual.aux2 = SWITCH_POS_MIDDLE;
								manual.aux3 = SWITCH_POS_MIDDLE;
								manual.aux4 = SWITCH_POS_MIDDLE;
								manual.aux5 = SWITCH_POS_MIDDLE;
								manual.mode_switch = SWITCH_POS_MIDDLE;
								manual.posctl_switch = SWITCH_POS_ON;
								manual.return_switch = SWITCH_POS_OFF;
								manual.loiter_switch = SWITCH_POS_OFF;
								manual.acro_switch = SWITCH_POS_OFF;
								manual.offboard_switch = SWITCH_POS_OFF;

					}
					else if( manual.timestamp - rcSerial_timestamp < 4 * 1e6f)
					{

								manual.y = rcSerial.y;
								manual.x = rcSerial.x;
								manual.r = rcSerial.r;
								manual.z = rcSerial.z;

								manual.flaps = SWITCH_POS_MIDDLE;
								manual.aux1 = SWITCH_POS_MIDDLE;
								manual.aux2 = SWITCH_POS_MIDDLE;
								manual.aux3 = SWITCH_POS_MIDDLE;
								manual.aux4 = SWITCH_POS_MIDDLE;
								manual.aux5 = SWITCH_POS_MIDDLE;
								manual.mode_switch = rcSerial.a[0];
								manual.posctl_switch = rcSerial.a[1];
								manual.return_switch = rcSerial.a[2];
								manual.loiter_switch = SWITCH_POS_OFF;
								manual.acro_switch = SWITCH_POS_OFF;
								manual.offboard_switch = SWITCH_POS_OFF;
					}


					
								/* mode switches */
					// if(c == 'c')
					// {
					// 	manual.mode_switch = SWITCH_POS_ON;
					// 	//warnx("User abort\n");
					// 	//break;
					// }else if(c == 'x')
					// {
					// 	manual.mode_switch = SWITCH_POS_MIDDLE;
					// 	//warnx("User abort\n");
					// 	//break;
					// }else if(c == 'z')
					// {
					// 	manual.mode_switch = SWITCH_POS_OFF;
					// 	//warnx("User abort\n");
					// 	//break;
					// }

					// if (_manual_control_pub != 0) {
					// 	orb_publish(ORB_ID(manual_control_setpoint), _manual_control_pub, &manual);

					// } else {
					// 	_manual_control_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual);
					// }



					bool rc_updated;
					orb_check(_rc_sub, &rc_updated);
					if (rc_updated) {
						struct rc_input_values rc_input;
						orb_copy(ORB_ID(input_rc), _rc_sub, &rc_input);
						if( ( tested_value = rc_input.values[4] )< 1500 )
						{
			 				if (_manual_control_pub  != 0) {
								orb_publish(ORB_ID(manual_control_setpoint), _manual_control_pub, &manual);

							} else {
								_manual_control_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual);
							}
						}
					}
				}
//#endif




				orb_copy(ORB_ID(sensor_accel), accel0_sub, &accel0);
				orb_copy(ORB_ID(sensor_accel), accel1_sub, &accel1);
				orb_copy(ORB_ID(sensor_gyro), gyro0_sub, &gyro0);
				orb_copy(ORB_ID(sensor_gyro), gyro1_sub, &gyro1);

				// write out on accel 0, but collect for all other sensors as they have updates
				//dprintf(serial_fd, "%llu,%d,%d,%d,%d,%d,%d\n", accel0.timestamp, (int)accel0.x_raw, (int)accel0.y_raw, (int)accel0.z_raw,(int)accel1.x_raw, (int)accel1.y_raw, (int)accel1.z_raw);


				
			}

		}
	}

	warnx("exiting");
	thread_running = false;

	fflush(stdout);
	return 0;
	// serialCom(serial_fd);


//TRASH CODE
//TRASH CODE
//TRASH CODE
//TRASH CODE
//TRASH CODE
//TRASH CODE


				serialCom(serial_fd);


				static struct manual_control_setpoint_s manual;
				memset(&manual, 0 , sizeof(manual));
				

				manual.timestamp = hrt_absolute_time();

				if( manual.timestamp - rcSerial_timestamp > 5 * 1e5f)
				{
							manual.y = 0;
							manual.x = 0;
							manual.r = 0;
							manual.z = 0.5;

							manual.flaps = SWITCH_POS_MIDDLE;
							manual.aux1 = SWITCH_POS_MIDDLE;
							manual.aux2 = SWITCH_POS_MIDDLE;
							manual.aux3 = SWITCH_POS_MIDDLE;
							manual.aux4 = SWITCH_POS_MIDDLE;
							manual.aux5 = SWITCH_POS_MIDDLE;
							manual.mode_switch = SWITCH_POS_MIDDLE;
							manual.posctl_switch = SWITCH_POS_ON;
							manual.return_switch = SWITCH_POS_OFF;
							manual.loiter_switch = SWITCH_POS_OFF;
							manual.acro_switch = SWITCH_POS_OFF;
							manual.offboard_switch = SWITCH_POS_OFF;

				}
				else if( manual.timestamp - rcSerial_timestamp < 4 * 1e6f)
				{

							manual.y = rcSerial.y;
							manual.x = rcSerial.x;
							manual.r = rcSerial.r;
							manual.z = rcSerial.z;

							manual.flaps = SWITCH_POS_MIDDLE;
							manual.aux1 = SWITCH_POS_MIDDLE;
							manual.aux2 = SWITCH_POS_MIDDLE;
							manual.aux3 = SWITCH_POS_MIDDLE;
							manual.aux4 = SWITCH_POS_MIDDLE;
							manual.aux5 = SWITCH_POS_MIDDLE;
							manual.mode_switch = rcSerial.a[0];
							manual.posctl_switch = rcSerial.a[1];
							manual.return_switch = rcSerial.a[2];
							manual.loiter_switch = SWITCH_POS_OFF;
							manual.acro_switch = SWITCH_POS_OFF;
							manual.offboard_switch = SWITCH_POS_OFF;
				}


				
							/* mode switches */
				// if(c == 'c')
				// {
				// 	manual.mode_switch = SWITCH_POS_ON;
				// 	//warnx("User abort\n");
				// 	//break;
				// }else if(c == 'x')
				// {
				// 	manual.mode_switch = SWITCH_POS_MIDDLE;
				// 	//warnx("User abort\n");
				// 	//break;
				// }else if(c == 'z')
				// {
				// 	manual.mode_switch = SWITCH_POS_OFF;
				// 	//warnx("User abort\n");
				// 	//break;
				// }

				// if (_manual_control_pub != 0) {
				// 	orb_publish(ORB_ID(manual_control_setpoint), _manual_control_pub, &manual);

				// } else {
				// 	_manual_control_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual);
				// }



				bool rc_updated;
				orb_check(_rc_sub, &rc_updated);
				if (rc_updated) {
					struct rc_input_values rc_input;
					orb_copy(ORB_ID(input_rc), _rc_sub, &rc_input);
					if( ( tested_value = rc_input.values[4] )> 1500 )
					{
		 				if (_manual_control_pub  != 0) {
							orb_publish(ORB_ID(manual_control_setpoint), _manual_control_pub, &manual);

						} else {
							_manual_control_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual);
						}
					}
				}


//TRASH CODE
//TRASH CODE				
//TRASH CODE
//TRASH CODE				
}




void serialCom(int serial_fd) {
	uint8_t c,cc,state;
	static uint8_t offset;
	static uint8_t dataSize;
	static uint8_t c_state;

	while( (cc = read(serial_fd,&c,1)) == 1 )
	{
		// write(serial_fd,&c,1);
		// printf("%c",c);
		state = c_state;
		// regular data handling to detect and handle MSP and other data
		if (state == IDLE) {
			if (c=='$') state = HEADER_START;
		} else if (state == HEADER_START) {
			state = (c=='M') ? HEADER_M : IDLE;
		} else if (state == HEADER_M) {
			state = (c=='<') ? HEADER_ARROW : IDLE;
		} else if (state == HEADER_ARROW) {
			if (c > INBUF_SIZE) {  // now we are expecting the payload size
				state = IDLE;
				continue;
			}
			dataSize = c;
			checksum = c;
			offset = 0;
			indRX = 0;
			state = HEADER_SIZE;  // the command is to follow
		} else if (state == HEADER_SIZE) {
			cmdMSP = c;
			checksum ^= c;
			state = HEADER_CMD;
		} else if (state == HEADER_CMD) {
			if (offset < dataSize) {
				checksum ^= c;
				inBuf[offset++] = c;
			} else {
				if (checksum == c) // compare calculated and transferred checksum
						evaluateCommand(cmdMSP); // we got a valid packet, evaluate it
				state = IDLE;
				cc = 0; // no more than one MSP per port and per cycle
			}
		}
		c_state = state;
	}
	 // while
return;
}


int evaluateCommand(uint8_t c) {
  //uint32_t tmp=0; 
  switch(c) {
    case 100:
      s_struct_w((uint8_t*)&rcSerial,sizeof(rcSerial));
      rcSerial_timestamp = hrt_absolute_time();
      return COM_RECEIVED;
      //printf("receive the packet\n");
      //rcSerialCount = 50; // 1s transition 
      break;
    // default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
    //   headSerialError();tailSerialReply();
    //   break;
  }
  return COM_UNRECEIVED;

}