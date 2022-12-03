#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <time.h>

#include <string.h>
#include <sys/types.h>
#include <termios.h>

#include <libgen.h>

#include <pthread.h>

static int fd = 0;
#define MAX_BUF_SIZE 4096
#define DEV_AHRS_UART "/dev/ttyACM0"

typedef struct {
	uint8_t		sync1;
	uint8_t		sync2;
	uint8_t		descriptor;
	uint8_t		len;
} __attribute__((packed)) header;

typedef struct {
	header 		mip_header;
	uint8_t		field_len;
	uint8_t		field_descriptor;
	uint8_t		data; // referece for address
} __attribute__((packed)) packet;

typedef struct {
	float		roll;
	float		pitch;
	float		yaw;
	uint16_t	flags;
	uint16_t	checksum;
} __attribute__((packed)) msg;
	
void packet_dump(char *buffer, int count)
{
	int i=0;

	for(i=0;i<count;i++) {
		printf(" %02x",buffer[i]);
	}
	printf("\r\n");
}

uint16_t calc_checksum(char *frame, int length)
{
	int 		i;
	uint8_t		checksum_byte1, checksum_byte2;
	uint16_t	checksum;

	checksum_byte1 = 0;
	checksum_byte2 = 0;

	for(i=0;i<length;i++) {
		checksum_byte1 += frame[i];
		checksum_byte2 += checksum_byte1;
	}

	checksum = ((uint16_t)checksum_byte1 << 8) + (uint16_t)checksum_byte2;
	return checksum;
}

float rad2deg(float radian) 
{
	return (float)(radian * 180.0/3.141592);
}

int packet_process(char *frame, int readbytes)
{
	int 		index_of_packetdata = 0;
	packet 		*pframe = (packet *)frame;
	msg 		*sensor_data;
	uint16_t 	checksum, payload_checksum;
	uint8_t		payload_checksum_byte1, payload_checksum_byte2;

#if 0
	printf("%s: readbytes = %d\n",__func__,readbytes);
	printf("%s: sync1(0x%02x), sync2(0x%02x), desc(0x%02x), length(%d)\n",__func__,
		pframe->mip_header.sync1,
		pframe->mip_header.sync2,
		pframe->mip_header.descriptor,
		pframe->mip_header.len
	);
#endif

	do {
		pframe = (packet *)&frame[index_of_packetdata];

		if(pframe->mip_header.sync1 == 0x75 && pframe->mip_header.sync2 == 0x65) {
			if(pframe->mip_header.descriptor == 0x01) { // Orientation, Euler Angles (0x82, 0x05)
				if(pframe->field_descriptor == 0xF1) {
					// process sensor data
					sensor_data = (msg *)&(pframe->field_len);
					checksum = calc_checksum((char *)pframe, pframe->mip_header.len+4);
					//printf("%s: checksum [0x%04x]\n",__func__,(uint16_t)checksum);
					//packet_dump((char *)sensor_data,pframe->mip_header.len+2);
					//packet_dump((char *)pframe,pframe->mip_header.len+4+2);

					payload_checksum_byte1 = frame[pframe->mip_header.len+4];
					payload_checksum_byte2 = frame[pframe->mip_header.len+4+1];
					payload_checksum = ((uint16_t)payload_checksum_byte1 << 8) + (uint16_t)payload_checksum_byte2;
					//printf("%s: payload checksum [0x%04x]\n",__func__,payload_checksum);
					if(checksum == payload_checksum) {
#if 0
						printf("%10.2f %10.2f %10.2f\r\n",
							rad2deg(sensor_data->roll),
							rad2deg(sensor_data->pitch),
							rad2deg(sensor_data->yaw)
						);
#else
						//printf("%s: checksum ok\n",__func__);
#endif
					}
				}
			}
			index_of_packetdata += (pframe->mip_header.len)+4+2; // frame length + header_size(4) + checksum_size(2)
		} else {
			index_of_packetdata++;
		}
		printf("%s: index_of_packetdata = %d\n",__func__,index_of_packetdata);
	} while(index_of_packetdata < readbytes);
	//printf("%s: return = %d\n",__func__,readbytes-index_of_packetdata);
	return readbytes-index_of_packetdata;  // return unprocessed remain data;
}

int init_uart(char * dev, int baud, int *fd)
{
    struct termios newtio;
    *fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if ( *fd < 0) {
        printf("%s> uart dev '%s' open fail [%d]n", __func__, dev, * fd);
        return -1;
    }
    memset( &newtio, 0, sizeof(newtio));
    newtio.c_iflag = IGNPAR; // non-parity 
    newtio.c_oflag = OPOST | ONLCR;;
    newtio.c_cflag = CS8 | CLOCAL | CREAD; // NO-rts/cts 
    switch (baud)
    {
    case 115200:
      newtio.c_cflag |= B115200;
      break;
    case 57600:
      newtio.c_cflag |= B57600;
      break;
    case 38400:
      newtio.c_cflag |= B38400;
      break;
    case 19200:
      newtio.c_cflag |= B19200;
      break;
    case 9600:
      newtio.c_cflag |= B9600;
      break;
    case 4800:
      newtio.c_cflag |= B4800;
      break;
    case 2400:
      newtio.c_cflag |= B2400;
      break;
    default:
      newtio.c_cflag |= B115200;
      break;
    }

    newtio.c_lflag = 0;
    //newtio.c_cc[VTIME] = vtime; // timeout 0.1초 단위 
    //newtio.c_cc[VMIN] = vmin; // 최소 n 문자 받을 때까진 대기 
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush( * fd, TCIFLUSH);
    tcsetattr( * fd, TCSANOW, & newtio);
    return 0;
}

void* read_loop(void * arg)
{
    int result;
    char buffer[MAX_BUF_SIZE];
    fd_set reads, temps;
	int readbytes = 0;
	packet	*frame;
	int packet_processed = 0;
	

    FD_ZERO(&reads);
    FD_SET(fd, &reads);

    while(1) {
        temps = reads;
        result = select(FD_SETSIZE, & temps, NULL, NULL, NULL);

        if(result < 0) {
            exit(EXIT_FAILURE);
        }

        if(FD_ISSET(fd, &temps)) {
            memset(buffer, 0, sizeof(buffer));
			readbytes = read(fd, buffer, MAX_BUF_SIZE);

            if(readbytes == -1)
                continue;

            printf("%s: rx count[%d] :\r\n",__func__, readbytes);
			packet_processed = packet_process(buffer,readbytes);
			if(packet_processed) printf(".\r\n");

			//packet_dump(buffer, readbytes);
        }
    }
}

const char cmd_idle[]   = {0x75, 0x65, 0x01, 0x02, 0x02, 0x02, 0xE1, 0xC7};
const char cmd_ping[]   = {0x75, 0x65, 0x01, 0x02, 0x02, 0x01, 0xE0, 0xC6};
const char cmd_resume[] = {0x75, 0x65, 0x01, 0x02, 0x02, 0x06, 0xE5, 0xCB};
const char cmd_reset[]  = {0x75, 0x65, 0x01, 0x02, 0x02, 0x7E, 0x5D, 0x43};
const char cmd_init1[]  = {0x75, 0x65, 0x0D, 0x03, 0x03, 0x27, 0x01, 0x15, 0x36};
const char cmd_init2[]  = {0x75, 0x65, 0x0D, 0x0E, 0x0E, 0x02, 0xBA, 0xE3, 0xED, 0x9B, 0x3C, 0x7D, 0x6D, 0xDF, 0xBF, 0x85, 0x5C, 0xF5, 0xC4, 0x09};

void help(char *appname)
{
	printf("%s Usage:\n", basename(appname));
	printf("\t%s device\r\n", basename(appname));
}

int main(int argc, char *argv[])
{
    pthread_t p_thread;
    int ret = -1;

	if(argc < 2) {
		help(argv[0]);
		return -1;
	}

    ret = init_uart(argv[1], 19200, &fd);
    if(ret) printf("%s: init uart ret[%d], fd[%d] not ok!\r\n",__func__, ret, fd);

    sleep(1);

    pthread_create( &p_thread, NULL, read_loop, NULL);

	while(1) {
    	ret = write(fd, cmd_ping, sizeof(cmd_ping));
   		printf("%s: write [%d]\r\n",__func__, ret);
		usleep(100000);
	}
}
