CC = g++
CFLAGS += -march=native -Wall -I ../../ReconROS -I lib/include -I /opt/ros/dashing/include/ -I application/orb_slam/include/ -I ../ -I ../Thirdparty/Pangolin/include -I /usr/include/eigen3/
LDFLAGS += -lpthread -lstdc++ -lm `pkg-config --cflags --libs opencv` -L /opt/ros/dashing/lib/*.so -L ../Thirdparty/DBoW2/lib/x86/  -L ../Thirdparty/g2o/lib/x86/ -L ../Thirdparty/Pangolin/lib/x86/ -lg2o -lDBoW2 -lpangolin -lGLEW -lGL -lGLU

MKDIRP = mkdir -p

OBJS =  application/main.o application/orb_slam/src/Optimizer.o application/orb_slam/src/KeyFrame.o application/orb_slam/src/System.o application/orb_slam/src/Viewer.o application/orb_slam/src/PnPsolver.o application/orb_slam/src/MapDrawer.o application/orb_slam/src/Map.o application/orb_slam/src/Frame.o application/orb_slam/src/Converter.o application/orb_slam/src/ORBextractor.o application/orb_slam/src/Initializer.o application/orb_slam/src/FPGA.o application/orb_slam/src/KeyFrameDatabase.o application/orb_slam/src/FrameDrawer.o application/orb_slam/src/Sim3Solver.o application/orb_slam/src/Tracking.o application/orb_slam/src/LoopClosing.o application/orb_slam/src/ORBmatcher.o application/orb_slam/src/LocalMapping.o application/orb_slam/src/MapPoint.o 

BUILD_OBJS = $(OBJS:%=_build/%)

all: $(BUILD_OBJS)
	$(CC) $(BUILD_OBJS) $(LDFLAGS) -o orbslam

install: all
	cp orbslam $(PREFIX)

clean:
	rm -rf _build orbslam

_build/%.o: %.c
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -x c -c $< -o $@

_build/%.o: %.cpp
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -x c -c $< -o $@

_build/%.o: %.cc
	$(MKDIRP) $(@D)
	$(CC) $(CFLAGS) -c $< -o $@
