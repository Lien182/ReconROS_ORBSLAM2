CC = gcc

CFLAGS += -g -Wno-unused-variable -Wall -I ../../../../../reconos/reconos -I lib/runtime/comp/ -I /opt/ros/foxy/include/ -I /usr/local/include/ -fpermissive $(pkg-config --cflags --libs opencv)
LDFLAGS += -lpthread -L /opt/ros/foxy/lib/*.so

MKDIRP = mkdir -p

OBJS = lib/comp/ros.o lib/comp/ros_sub.o lib/comp/ros_pub.o application/main.o 

BUILD_OBJS = $(OBJS:%=_build/%)

all: $(BUILD_OBJS)
	$(CC) $(BUILD_OBJS) $(LDFLAGS) -o rosorb_client

install: all
	cp rosorb_client $(PREFIX)

clean:
	rm -rf _build rosorb_client

_build/%.o: %.c
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -x c -c $< -o $@

_build/%.o: %.cc
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

_build/%.o: %.cpp
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -x c -c $< -o $@
