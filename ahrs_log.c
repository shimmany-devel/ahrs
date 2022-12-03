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
#include <byteswap.h>

static int fd = 0;
#define MAX_BUF_SIZE 4096

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
	uint8_t		checksum_byte1;
	uint8_t		checksum_byte2;
} __attribute__((packed)) msg;

float big2little(float f)
{
    union
    {
        float f;
        char b[4];
    } src, dst;

    src.f = f;
    dst.b[3] = src.b[0];
    dst.b[2] = src.b[1];
    dst.b[1] = src.b[2];
    dst.b[0] = src.b[3];
    return dst.f;
}
	
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

int keepbytes = 0;
int packet_process(char *frame, int readbytes)
{
	int 		index_of_packetdata = 0;
	packet 		*pframe = (packet *)frame;
	msg 		*data; // payload
	uint16_t 	checksum, payload_checksum;

	while(index_of_packetdata < readbytes && readbytes >= 4) {
		pframe = (packet *)&frame[index_of_packetdata];

		if(pframe->mip_header.sync1 == 0x75 && pframe->mip_header.sync2 == 0x65) {
			switch(pframe->mip_header.descriptor) {
			  case 0x82: { // Orientation, Euler Angles (0x82, 0x05)
				// fragmentation check
				if(readbytes < 22) {
					keepbytes = readbytes;
					//packet_dump((char *)pframe,readbytes);
					//printf("#");
					return keepbytes;	
				} 
				if(pframe->field_descriptor == 0x05) { // process sensor data
					//printf("%s: sensor data\n",__func__);
					data = (msg *)&(pframe->data);
					checksum = calc_checksum((char *)pframe, pframe->mip_header.len+4);
					//printf("%s: calculate checksum [0x%04x]\n",__func__,checksum);
					//packet_dump((char *)data,pframe->mip_header.len);

					payload_checksum = ((uint16_t)(data->checksum_byte1) << 8) + 
									    (uint16_t)(data->checksum_byte2);

					//printf("%s: payload checksum [0x%04x]\n",__func__,payload_checksum);
					//if(checksum == payload_checksum && !data->flags) {
					if(checksum == payload_checksum) {
#if 1
						printf("%10.2f %10.2f %10.2f\r\n",
							rad2deg(big2little(data->roll)),
							rad2deg(big2little(data->pitch)),
							rad2deg(big2little(data->yaw))
						);
#endif
					} else {
						printf("%s: descriptor[0x%02x] checksum not ok\n",__func__,pframe->field_descriptor);
					}
				}
				index_of_packetdata += (pframe->mip_header.len)+4+2; // frame length + header_size(4) + checksum_size(2)
				break;
			  }
			  case 0x01: {
				uint8_t     payload_checksum_byte1, payload_checksum_byte2;

				if(pframe->field_descriptor == 0xF1) { // process resume ack
					data = (msg *)&(pframe->field_len);
					checksum = calc_checksum((char *)pframe, pframe->mip_header.len+4);

					payload_checksum_byte1 = frame[pframe->mip_header.len+4];
					payload_checksum_byte2 = frame[pframe->mip_header.len+4+1];
					payload_checksum = ((uint16_t)payload_checksum_byte1 << 8) + (uint16_t)payload_checksum_byte2;
					//packet_dump((char *)pframe,pframe->mip_header.len+4+2);
					//printf("%s: calculated checksum [0x%04x]\n",__func__,checksum);
					//printf("%s: payload    checksum [0x%04x]\n",__func__,payload_checksum);

					//if(checksum == payload_checksum && !data->flags) {
					if(checksum != payload_checksum) {
						printf("%s: descriptor[0x%02x] checksum not ok\n",__func__,pframe->field_descriptor);
					}
				}
				//printf("%s: packet processed %d\n",__func__,(pframe->mip_header.len)+4+2);
				index_of_packetdata += (pframe->mip_header.len)+4+2; // frame length + header_size(4) + checksum_size(2)
				break;
			  }

			  default: {
				printf("%s: Unknown Descriptor ==> 0x%02x\n",__func__,pframe->field_descriptor);
				index_of_packetdata += (pframe->mip_header.len)+4+2; // frame length + header_size(4) + checksum_size(2)
				break;
			  }
			}
		} else {
			index_of_packetdata++; // go next byte for finding sync1,sync2 
		}
		//printf("%s: index_of_packetdata = %d\n",__func__,index_of_packetdata);
		if(readbytes-index_of_packetdata < 0) {
			printf("%s: return = %d\n",__func__,readbytes-index_of_packetdata);
		} else {
			keepbytes = 0;
		}
	}
	
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
        result = select(FD_SETSIZE, &temps, NULL, NULL, NULL);
	
        if(result < 0) {
			printf("%s: select result = %d\n",__func__,result);
			sleep(2);
            exit(EXIT_FAILURE);
			//continue;
        }

        if(FD_ISSET(fd, &temps)) {
            memset(&buffer[packet_processed], 0, sizeof(buffer)-packet_processed);
			readbytes = read(fd, &buffer[packet_processed], MAX_BUF_SIZE);

			readbytes += packet_processed;
#if 0
			if(packet_processed) {
				printf("%s: *readbytes = %d\n",__func__,readbytes);
			}
#endif
            if(readbytes == -1) {
				printf("%s: skipped\n",__func__);
                continue;
			}
#if 0
			if(readbytes != 22) {
            	printf("%s: rx count[%d] :\r\n",__func__, readbytes);
			}
#endif
			packet_processed = packet_process(buffer,readbytes);
			//packet_dump(buffer, readbytes);
#if 0
			if(packet_processed==0 && readbytes < 22) {
				packet_dump(buffer, readbytes);
				remain_position = readbytes+packet_processed;
				remain_count = packet_processed * (-1);
				printf("%s: remain_position = %d, remain_count = %d\r\n",__func__,remain_position,remain_count);
				memcpy(buffer, &buffer[remain_position], remain_count);
			} else {
				remain_count = 0;
			}
#endif
        }
#if 0
		if(packet_processed) {
			printf("%s: packet_processed = %d\n",__func__,packet_processed);
		}
#endif
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

    ret = init_uart(argv[1], 115200, &fd);
    if(ret) printf("%s: init uart ret[%d], fd[%d] not ok!\r\n",__func__, ret, fd);

    sleep(1);

    pthread_create( &p_thread, NULL, read_loop, NULL);

   	ret = write(fd, cmd_idle, sizeof(cmd_idle));
	sleep(2);
   	ret = write(fd, cmd_init1, sizeof(cmd_init1));
	sleep(2);
   	ret = write(fd, cmd_init2, sizeof(cmd_init2));
	sleep(2);
   	ret = write(fd, cmd_resume, sizeof(cmd_resume));
	//printf("%s: write [%d]\r\n",__func__, ret);
	while(1) {

		sleep(5);
	}
}
