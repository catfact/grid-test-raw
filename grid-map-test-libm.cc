#include <monome.h>

#include <iostream>
#include <cstdint>

#include <thread>
#include <chrono>

uint8_t led_buf[64];

monome_t *m;

int main(int argc, const char **argv) {
    if (argc < 2) {
	std::cerr << "device path required; exiting" << std::endl;	    
	return 1;
    }

    const char *dev_path = argv[1];
    m = monome_open(dev_path);
    if (m == NULL) { 
	std::cerr << "failed to open device at " << dev_path <<"; exiting" << std::endl;
	return 1;
    }

    int cell = 0;
    int val = 1;
    while (1) {
	quad_led_buf[cell] = val;
	val = (val + 1) % 14 + 1;
	cell = (cell + 1) % 64;
	monome_led_level_map(m, 0, 0, quad_led_buf);
	monome_led_level_map(m, 8, 0, quad_led_buf);
	std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(100));
    }
}
