CC=g++

default: dls.cpp
	$(CC) libdls.cpp kbhit.c -fPIC -g -o libdls.so -shared
	$(CC) -o dls dls.cpp -L. -ldls

clean: 
	rm dls
	rm libdls.so

install: 
	cp dls /usr/bin/dls
	cp libdls.so /usr/lib/libdls.so
	cp dls.hpp /usr/include/dls.hpp
