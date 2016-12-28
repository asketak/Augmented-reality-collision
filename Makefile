#  Makefile

INC_DIR= ./include
LIB_DIR= ./lib
BIN_DIR= ./

CC=gcc
LDFLAG=-L/usr/lib/x86_64-linux-gnu -L/usr/lib64 -L$(LIB_DIR)/linux-x86_64 -L$(LIB_DIR)
LIBS= -lcurl -lARgsub_lite -lARvideo -lAR -lARICP -lAR -lglut -lGLU -lGL -lX11 -lm -lpthread -ljpeg
CFLAG= -O3 -fPIC -std=gnu++11 -march=core2 -DHAVE_NFT=1 -I/usr/include/x86_64-linux-gnu -I$(INC_DIR)

OBJS =
HEADERS =

all: $(BIN_DIR)/collision_detection

$(BIN_DIR)/collision_detection: collision_detection.o $(OBJS)
	g++ -o $(BIN_DIR)/collision_detection collision_detection.o curl.o imageloader.o $(OBJS) $(LDFLAG) $(LIBS)

collision_detection.o: collision_detection.c $(HEADERS)
	g++ -c $(CFLAG) collision_detection.c curl.c imageloader.cpp

clean:
	rm -f *.o
	rm -f $(BIN_DIR)/collision_detection
