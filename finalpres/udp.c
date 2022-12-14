// #include <string.h>
// #include <stdio.h>
// #include <common/mavlink.h>
// #include <sys/socket.h>
// #include <arpa/inet.h>
// #include <stdlib.h>
// #include <fcntl.h>
// #include <errno.h>
// #include <unistd.h>
#include "udp.h"

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx
int sock;
struct sockaddr_in local_addr; 

void udp_init(){
    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    memset(&local_addr, 0, sizeof(local_addr));
	local_addr.sin_family = AF_INET;
	local_addr.sin_addr.s_addr = INADDR_ANY;
	local_addr.sin_port = htons(14551);

    if (-1 == bind(sock,(struct sockaddr *)&local_addr, sizeof(struct sockaddr)))
    {
		perror("error bind failed");
		close(sock);
		exit(EXIT_FAILURE);
    } 

    if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
	{
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(sock);
		exit(EXIT_FAILURE);
    }
}

struct sockaddr_in* target_init(const char *target_ip){
    struct sockaddr_in *target_addr = (struct sockaddr_in*)malloc(sizeof(struct sockaddr_in));
    memset(&target_addr, 0, sizeof(target_addr));
	target_addr->sin_family = AF_INET;
	target_addr->sin_addr.s_addr = inet_addr(target_ip);
	target_addr->sin_port = htons(14550);

    return target_addr;
}

void send_heartbeat(const char* target_ip) {
    //struct socketaddr_in* target_addr = target_init(target_ip);
    struct sockaddr_in target_addr;
    memset(&target_addr, 0, sizeof(target_addr));
	target_addr.sin_family = AF_INET;
	target_addr.sin_addr.s_addr = inet_addr(target_ip);
	target_addr.sin_port = htons(14551);

    uint8_t buf[BUFFER_LENGTH];
    socklen_t target_len = sizeof(&target_addr);
    int bytes_sent;
    mavlink_message_t msg;
	uint16_t len;
    printf("sending heartbeat\n");
    mavlink_msg_heartbeat_pack(42, 200, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_MODE_FLAG_TEST_ENABLED, 0, MAV_STATE_ACTIVE);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&target_addr, sizeof(struct sockaddr_in));
}

void encode_position(int pos_type, float x, float y, const char *target_ip) {
    struct sockaddr_in target_addr;
    memset(&target_addr, 0, sizeof(target_addr));
	target_addr.sin_family = AF_INET;
	target_addr.sin_addr.s_addr = inet_addr(target_ip);
	target_addr.sin_port = htons(14551);

    uint8_t buf[BUFFER_LENGTH];
    socklen_t target_len = sizeof(&target_addr);
    int bytes_sent;
    mavlink_message_t msg;
	uint16_t len;
    mavlink_msg_set_position_target_local_ned_pack(42, MESSAGE_POSITION, &msg, 101010, 1, 1, MAV_FRAME_LOCAL_NED, 0, x, y, pos_type, 2.5, 8.2, 0, 0, 0, 0, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&target_addr, sizeof(struct sockaddr_in));
}

void encode_vector(float vx, float vy, const char *target_ip) {
    struct sockaddr_in target_addr;
    memset(&target_addr, 0, sizeof(target_addr));
	target_addr.sin_family = AF_INET;
	target_addr.sin_addr.s_addr = inet_addr(target_ip);
	target_addr.sin_port = htons(14551);

    uint8_t buf[BUFFER_LENGTH];
    socklen_t target_len = sizeof(&target_addr);
    int bytes_sent;
    mavlink_message_t msg;
	uint16_t len;
    mavlink_msg_set_position_target_local_ned_pack(42, MESSAGE_VECTOR, &msg, 101010, 1, 1, MAV_FRAME_LOCAL_NED, 0, 0, 0, 0, vx, vy, 0, 0, 0, 0, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&target_addr, sizeof(struct sockaddr_in));
}

void encode_signal(int signal, const char *target_ip) {
    struct sockaddr_in target_addr;
    memset(&target_addr, 0, sizeof(target_addr));
	target_addr.sin_family = AF_INET;
	target_addr.sin_addr.s_addr = inet_addr(target_ip);
	target_addr.sin_port = htons(14551);

    uint8_t buf[BUFFER_LENGTH];
    socklen_t target_len = sizeof(&target_addr);
    int bytes_sent;
    mavlink_message_t msg;
	uint16_t len;
    mavlink_msg_set_position_target_local_ned_pack(42, MESSAGE_SIGNAL, &msg, 0, 0, 0, signal, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&target_addr, sizeof(struct sockaddr_in));
}

void encode_angle(float theta, const char *target_ip) {
    struct sockaddr_in target_addr;
    memset(&target_addr, 0, sizeof(target_addr));
	target_addr.sin_family = AF_INET;
	target_addr.sin_addr.s_addr = inet_addr(target_ip);
	target_addr.sin_port = htons(14551);

    uint8_t buf[BUFFER_LENGTH];
    socklen_t target_len = sizeof(&target_addr);
    int bytes_sent;
    mavlink_message_t msg;
	uint16_t len;
    mavlink_msg_set_position_target_local_ned_pack(42, MESSAGE_ANGLE, &msg, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, theta, 0, 0, 0, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&target_addr, sizeof(struct sockaddr_in));
}

uint8_t decode_get_message_type(const mavlink_message_t* msg){
    return msg->compid;
}

int decode_get_pos_type(const mavlink_message_t* msg) {
    return (int)(mavlink_msg_set_position_target_local_ned_get_z(msg));
}

float decode_get_pos_x(const mavlink_message_t* msg) {
    return mavlink_msg_set_position_target_local_ned_get_x(msg);
}

float decode_get_pos_y(const mavlink_message_t* msg) {
    return mavlink_msg_set_position_target_local_ned_get_y(msg);
}

float decode_get_vec_x(const mavlink_message_t* msg) {
    return mavlink_msg_set_position_target_local_ned_get_vx(msg);
}

float decode_get_vec_y(const mavlink_message_t* msg) {
    return mavlink_msg_set_position_target_local_ned_get_vy(msg);
}

int decode_get_signal(const mavlink_message_t* msg) {
    return mavlink_msg_set_position_target_local_ned_get_coordinate_frame(msg);
}

float decode_get_angle(const mavlink_message_t* msg) {
    return mavlink_msg_set_position_target_local_ned_get_vz(msg);
}

int read_from_socket(const char *target_ip, mavlink_message_t *msg) {
    uint8_t buf[BUFFER_LENGTH];
    ssize_t recsize;
    struct sockaddr_in inAddr; 
    socklen_t inLen = sizeof(inAddr);
    mavlink_status_t status;

    memset(&inAddr, 0, sizeof(inAddr));
	inAddr.sin_family = AF_INET;
	inAddr.sin_addr.s_addr = inet_addr(target_ip);
	inAddr.sin_port = htons(14550);
    memset(buf, 0, BUFFER_LENGTH);
	recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&inAddr, &inLen);
    if (recsize > 0)
      	{
			// Something received - print out all bytes and parse packet
			mavlink_status_t status;
			printf("aBytes Received from addr1: %d\nDatagram: ", (int)recsize);
			for (int i = 0; i < recsize; ++i)
			{
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], msg, &status))
				{
					return true;
				}
			}
			printf("a%d", decode_get_message_type(&msg));
			printf("\n");
		}
    return false;
}
