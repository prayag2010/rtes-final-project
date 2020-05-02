INCLUDE_DIRS = 
LIB_DIRS = 
CC=g++
LDFLAGS= 
CDEFS=
CFLAGS= -O0 -pg -g $(INCLUDE_DIRS) $(CDEFS) -I/home/prayag/opencv-3.4.4/modules.core/include -I/home/prayag/opencv-4.3.0/modules/core/include \
	-I/home/prayag/opencv-4.3.0/build/opencv2 -I/home/prayag/opencv-4.3.0/include
LIBS= -lrt -pthread 
CPPLIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video -L/usr/local/lib/ -lopencv_core -lopencv_imgproc -lopencv_highgui

HFILES= 
CFILES= 
CPPFILES= capture.cpp

SRCS= ${HFILES} ${CFILES}
CPPOBJS= ${CPPFILES:.cpp=.o}

all:	capture 

clean:
	-rm -f *.o *.d
	-rm -f capture

distclean:
	-rm -f *.o *.d

capture: capture.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o `pkg-config --libs opencv` $(CPPLIBS) $(LIBS)

depend:

.c.o:
	$(CC) $(CFLAGS) -c $< $(LIBS)

.cpp.o:
	$(CC) $(CFLAGS) -c $< $(LIBS)
