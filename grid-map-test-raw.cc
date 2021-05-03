#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <cstdint>

#include <thread>
#include <chrono>

static constexpr int baud = 115200;
static constexpr int timeout = 25;

static uint8_t quad_led_buf[64];

static int dev_fd;

static bool dev_write(const uint8_t *buf, ssize_t nbyte) {
    return write(dev_fd, buf, nbyte) == nbyte;
}

static bool send_map(uint8_t x, uint8_t y, const uint8_t *buf) {
    static struct {
	uint8_t cmd;
	uint8_t x;
	uint8_t y;
	uint8_t d[32];
    } msg;
    msg.cmd = 0x1a;
    msg.x = x;
    msg.y = y;
    const uint8_t *src = buf;
    for (int i=0; i<32; ++i) {
	msg.d[i] = (*src++) & 0xf;
	msg.d[i] |= (*src++) << 4;
    }
    return dev_write((const uint8_t*)&msg, 37);
}

bool open_device(const char *path) {
    struct termios nt, ot;
    
    if( (dev_fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0 ) {
	return false;
    }

    tcgetattr(dev_fd, &ot);
    nt = ot;

    // baud rate
    cfsetispeed(&nt, baud);
    cfsetospeed(&nt, baud);

    // parity (8N1)
    nt.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    nt.c_cflag |=  (CS8 | CLOCAL | CREAD);

    // no line processing
    nt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN);

    // raw input
    nt.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK |
		    INPCK | ISTRIP | IXON);

    // raw output
    nt.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR |
		    OFILL | OPOST);

    nt.c_cc[VMIN]  = 1;
    nt.c_cc[VTIME] = 0;

    if( tcsetattr(dev_fd, TCSANOW, &nt) < 0 ) {
	close(dev_fd);
	return false;
    }
	
    tcflush(dev_fd, TCIOFLUSH);
   
    return true;
}

int main(int argc, const char **argv) {
    if (argc < 2) {
	std::cerr << "device path required; exiting" << std::endl;	    
	return 1;
    }

    const char *dev_path = argv[1];
    if (!open_device(dev_path)) {
	std::cerr << "failed to open device at " << dev_path <<"; exiting" << std::endl;
	return 1;
    }

    int cell = 0;
    int val = 1;
    while (1) {
	quad_led_buf[cell] = val;
	val = (val + 1) % 14 + 1;
	cell = (cell + 1) % 64;
	send_map(0, 0, quad_led_buf);
	send_map(8, 0, quad_led_buf);
	std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(100));
    }
}
