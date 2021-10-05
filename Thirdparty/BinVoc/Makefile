CC = g++
CFLAGS += -march=native -Wall  -I lib/include -I ../ -I ../Pangolin/include -I /usr/include/eigen3/
LDFLAGS += -lpthread -lstdc++ -lm `pkg-config --cflags --libs opencv` -L ../DBoW2/lib/x86/  -L ../g2o/lib/x86/ -L ../Pangolin/lib/x86/ -lg2o -lDBoW2 -lpangolin -lGLEW -lGL -lGLU

MKDIRP = mkdir -p

OBJS =  bin_vocabulary.o

BUILD_OBJS = $(OBJS:%=_build/%)

all: $(BUILD_OBJS)
	$(CC) $(BUILD_OBJS) $(LDFLAGS) -o bin_vocabulary


clean:
	rm -rf _build bin_vocabulary

_build/%.o: %.c
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -x c -c $< -o $@

_build/%.o: %.cpp
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -x c -c $< -o $@

_build/%.o: %.cc
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -c $< -o $@
