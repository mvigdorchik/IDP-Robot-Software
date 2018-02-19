TARGETARM=robot.arm
TARGET=robot

#These options are for compiling on the IDP computer, not the intel chip
CCSWITCHES=-I./1BRobot
LIBS=-L./1BRobot 
CC = g++ -O3

#These are for cross compiling the ARM binary
CCSWITCHESARM=-I/usr/arm-unknown-linux-gnueabi/include -I/export/teach/1BRobot -D__arm__ 
LIBSARM=-L/usr/arm-unknown-linux-gnueabi/lib 
CCARM = arm-linux-gnueabi-g++

SRCS := $(wildcard src/*.cpp)
HDRS := $(wildcard src/*.h)
OBJS := $(patsubst src/%.cpp,build/%.o,$(SRCS))
OBJSARM := $(patsubst src/%.cpp,armbuild/%.o,$(SRCS))
#LIBRARIESARM just pulls everything out of the libs folder since -L doesnt seem to do it properly
#LIBRARIESARM := $(wildcard /usr/arm-linux-gnueabi/lib/*.a*)

all: local

arm: $(HDRS) Makefile
	@mkdir -p armbuild
	$(CCARM) $(CCSWITCHESARM) $(LIBSARM) $(SRCS) -o $(TARGETARM) -lrobot

armbuild/%.o: src/%.cpp $(HDRS) Makefile
	@mkdir -p armbuild
	$(CCARM) $(CCSWITCHESARM) $(LIBS) -c $< -o $@

local: $(OBJS) $(HDRS) Makefile
	@mkdir -p build
	$(CC) $(CCSWITCHES) $(LIBS) $(OBJS) ./1BRobot/librobot.a -o $(TARGET)

build/%.o: src/%.cpp $(HDRS) Makefile
	@mkdir -p build
	$(CC) $(CCSWITCHES) $(LIBS) -c $< -o $@

# tidy up
.PHONY clean:
	rm -f $(TARGET) $(TARGETARM) $(OBJS) $(OBJSARM)
