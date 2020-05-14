/*spi_node spi communication between raspberry pi & stm32 nucleo*/

#include <ros/ros.h>
#include "origarm_ros/Command_Pre_Open.h"
#include "origarm_ros/Command_ABL.h"
#include "origarm_ros/Seg_ABL.h"
#include "origarm_ros/Seg_Pre.h"
#include "origarm_ros/Valve.h"
#include "origarm_ros/Cmd_ABL.h"

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <inttypes.h> //printf uint16_t
#include <time.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev0.0";
static uint32_t mode = 0x1; //default would be 0x0, not working though
static uint8_t bits = 8;    //change data size of stm32 from 16 bits to 8 bits
static char *input_file;
static char *output_file;
static uint32_t speed = 5000000;
static uint16_t delay;
static int verbose;

#define seg 9
#define act 6

struct QUATERNIONCOMPACT
{
	uint16_t imuData0 :15, maxLocHigh :1;
	uint16_t imuData1 :15, maxLocLow :1;
	uint16_t imuData2 :15, maxSign :1;
};

struct SENSORDATA
{
	uint16_t pressure:9, distance:7;
	struct QUATERNIONCOMPACT quaternionCom;
};

struct COMMANDDATA
{
	int16_t values[3];
	uint16_t commandType;
};

struct SENSORDATA sensorData[seg][act];
struct COMMANDDATA commandData[seg][act];

enum COMMAND_MODE{openingCommandType, pressureCommandType};

int16_t Cmd_pressure[seg][act];
int segNumber;

void pressureCallback(const origarm_ros::Command_Pre_Open& pressured)
{
	/*for(int i = 0; i < 6; i++)
	{
		Cmd_pressure[i] = pressured.segment[0].command[i].pressure;
	}*/

	for(int i = 0; i < seg; i++)
	{
		for (int j = 0; j < act; j++)
		{
			Cmd_pressure[i][j] = pressured.segment[i].command[j].pressure;
		}		
	} 	
}

void segNumberCallback(const origarm_ros::Cmd_ABL& msg)
{
	segNumber = msg.segmentNumber;
}

static void writeCommand()
{
	/*for (int i = 0; i < 9; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			commandData[i][j].commandType = openingCommandType;
			commandData[i][j].values[0] = (6*i+j+1)*0.015*32767;
		}
	}*/

	/*for (int i = 0; i < 6; i++)
	{
		commandData[0][i].commandType = pressureCommandType;
		commandData[0][i].values[0] = Cmd_pressure[i];//kPa or Pa?
	}*/

	for (int i = 0; i < seg; i++)
	{
		for (int j = 0; j < act; j++)
		{
			commandData[i][j].commandType = openingCommandType;
			commandData[i][j].values[0] = Cmd_pressure[i][j];
		}
	}

}

char *input_tx;

static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	int ret;
	int out_fd;

	struct spi_ioc_transfer tr;
	memset(&tr, 0, sizeof(struct spi_ioc_transfer));

	tr.tx_buf = (unsigned long)tx;
	tr.rx_buf = (unsigned long)rx;
	tr.len = len;
	tr.delay_usecs = delay;
	tr.speed_hz = speed;
	tr.bits_per_word = bits;
	
	if (mode & SPI_TX_QUAD)
		tr.tx_nbits = 4;
	else if (mode & SPI_TX_DUAL)
		tr.tx_nbits = 2;
	if (mode & SPI_RX_QUAD)
		tr.rx_nbits = 4;
	else if (mode & SPI_RX_DUAL)
		tr.rx_nbits = 2;
	if (!(mode & SPI_LOOP)) {
		if (mode & (SPI_TX_QUAD | SPI_TX_DUAL))
			tr.rx_buf = 0;
		else if (mode & (SPI_RX_QUAD | SPI_RX_DUAL))
			tr.tx_buf = 0;
	}

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	printf("ret:%d\r\n", ret);
	if (ret < 1)
		pabort("can't send spi message");
	
	printf("data transfer\r\n");
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev0.0)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word\n"
	     "  -i --input    input data from a file (e.g. \"test.bin\")\n"
	     "  -o --output   output data to a file (e.g. \"results.bin\")\n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n"
	     "  -v --verbose  Verbose (show tx buffer)\n"
	     "  -p            Send data (e.g. \"1234\\xde\\xad\")\n"
	     "  -N --no-cs    no chip select\n"
	     "  -R --ready    slave pulls low to pause\n"
	     "  -2 --dual     dual transfer\n"
	     "  -4 --quad     quad transfer\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "input",   1, 0, 'i' },
			{ "output",  1, 0, 'o' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ "no-cs",   0, 0, 'N' },
			{ "ready",   0, 0, 'R' },
			{ "dual",    0, 0, '2' },
			{ "verbose", 0, 0, 'v' },
			{ "quad",    0, 0, '4' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:d:b:i:o:lHOLC3NR24p:v",
				lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'i':
			input_file = optarg;
			break;
		case 'o':
			output_file = optarg;
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		case 'N':
			mode |= SPI_NO_CS;
			break;
		case 'v':
			verbose = 1;
			break;
		case 'R':
			mode |= SPI_READY;
			break;
		case 'p':
			input_tx = optarg;
			break;
		case '2':
			mode |= SPI_TX_DUAL;
			break;
		case '4':
			mode |= SPI_TX_QUAD;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
	if (mode & SPI_LOOP) {
		if (mode & SPI_TX_DUAL)
			mode |= SPI_RX_DUAL;
		if (mode & SPI_TX_QUAD)
			mode |= SPI_RX_QUAD;
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "spi_node");
	ros::NodeHandle nh;
	ROS_INFO("spi_node starts!");	
	
	int ret = 0;
	int fd;

	parse_opts(argc, argv);

	//fd = open(device, O_RDWR);
	fd = open(std::string("/dev/spidev0.0").c_str(), O_RDWR);	
	if (fd < 0)
		pabort("can't open device");

	//spi mode	
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	//bits per word
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	//max speed hz
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: 0x%x\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);


  int t = 0;
	
  ros::Subscriber sub1 = nh.subscribe("Command_Pre_Open", 1, pressureCallback);
  ros::Subscriber sub2 = nh.subscribe("Cmd_ABL", 1, segNumberCallback);		

  ros::Rate r(100); 

	while(ros::ok())
	{
		writeCommand();
		//transfer(fd, default_tx, default_rx, sizeof(default_tx));
		
		transfer(fd, (uint8_t *)(&commandData[0][0]), (uint8_t *)(&sensorData[0][0]), sizeof(sensorData));

		/*for (int i = 0; i < seg; i++)
		{
			for (int j = 0; j < act; j++)
			{
				printf("Command[%d][%d]: %hd\r\n", i, j, commandData[i][j].values[0]);
				//printf("Sensor[%d][%d] : %hu %hu\r\n", i, j, sensorData[i][j].pressure, sensorData[i][j].distance);
			}			
		}*/

		printf("time:%d, segN:%d\r\n", t, segNumber); 
		//printf("CommandPressure        | sensorData\r\n"); 
		for (int i = 0; i < seg; i++)
		{
			printf("Data[%d]: %hd %hd %hd %hd %hd %hd| %hu %hu %hu %hu %hu %hu| %hu %hu %hu %hu %hu %hu\r\n", i, 
				commandData[i][0].values[0], 
				commandData[i][1].values[0], 
				commandData[i][2].values[0], 
				commandData[i][3].values[0], 
				commandData[i][4].values[0], 
				commandData[i][5].values[0],
			    sensorData[i][0].pressure, 
				sensorData[i][1].pressure,
				sensorData[i][2].pressure,
				sensorData[i][3].pressure,
				sensorData[i][4].pressure,
				sensorData[i][5].pressure,
				sensorData[i][0].distance,
				sensorData[i][1].distance,
				sensorData[i][2].distance,
				sensorData[i][3].distance,
				sensorData[i][4].distance,
				sensorData[i][5].distance);			
		}
				
		//printf("time:%d\r\n", t);
		t = t+1;
		
		//sleep(1); //wait for 1 second
		//usleep(10000);//wait for 1000us = 1ms

		ros::spinOnce();
		r.sleep();    //sleep for 1/r sec
	}

	close(fd);

	return ret;

}