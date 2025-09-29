# Compiler and flags
CC      = gcc
CXX     = g++
CFLAGS  = -c -Wall -Wno-format-truncation -ffunction-sections -fdata-sections
CXXFLAGS= -c -Wall -Wno-format-truncation -std=c++11 $(shell pkg-config --cflags opencv4)
LDFLAGS = $(shell pkg-config --libs opencv4) -lm -pthread -ffunction-sections -fdata-sections

# Sources
SRCS_C   = libs/RPI-serial/RPIserial.c libs/NMEA-gps_utilities/NMEA-GPSutils.c
OBJS_C   = RPIserial.o NMEA.o
SRCS_CPP = main.cpp
OBJS_CPP = main.o

# Target
TARGET   = program

all: $(TARGET)

# Rules for C objects (with correct source path)
RPIserial.o: libs/RPI-serial/RPIserial.c
	$(CC) $(CFLAGS) $< -o $@

NMEA.o: libs/NMEA-gps_utilities/NMEA-GPSutils.c
	$(CC) $(CFLAGS) $< -o $@

# Rule for C++ object
main.o: main.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

# Link final program
$(TARGET): $(OBJS_CPP) $(OBJS_C)
	$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(OBJS_CPP) $(OBJS_C) $(TARGET)
