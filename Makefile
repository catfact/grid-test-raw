all:
	g++ grid-map-test-raw.cc -o grid-map-test-raw
	g++ grid-map-test-libm.cc -o grid-map-test-libm -lmonome

