CC = g++
CFLAGS = -std=gnu++11 -g -O3
WARNS = -w

INC_LOCAL = -I.

LIB_FOLDERS = -L/usr/local/lib

LIBS = -lm
LIBS_OPENCV = -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_aruco -lopencv_imgcodecs -lopencv_videoio -lopencv_ccalib -lopencv_calib3d


all: createboard calibratecamera createmarker viewmarkers computetransformation trackmarkers


.PHONY: createboard
createboard:
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) -o bin/$@ src/$@.cpp $(LIB_FOLDERS) $(LIBS) $(LIBS_OPENCV)

.PHONY: calibratecamera
calibratecamera:
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) -o bin/$@ src/$@.cpp $(LIB_FOLDERS) $(LIBS) $(LIBS_OPENCV)

.PHONY: createmarker
createmarker:
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) -o bin/$@ src/$@.cpp $(LIB_FOLDERS) $(LIBS) $(LIBS_OPENCV)

.PHONY: viewmarkers
viewmarkers:
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) -o bin/$@ src/$@.cpp $(LIB_FOLDERS) $(LIBS) $(LIBS_OPENCV)

.PHONY: computetransformation
computetransformation:
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) -o bin/$@ src/$@.cpp $(LIB_FOLDERS) $(LIBS) $(LIBS_OPENCV)

.PHONY: trackmarkers
trackmarkers:
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) -o bin/$@ src/$@.cpp $(LIB_FOLDERS) $(LIBS) $(LIBS_OPENCV)

clean:
	rm bin/*

