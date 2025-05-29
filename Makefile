CXX = g++
CXXFLAGS = -std=c++20 -Wall -O2
GTKFLAGS = pkg-config --cflags gtkmm-3.0
GTKLIBS = pkg-config --libs gtkmm-3.0

TARGET = nbody

SRC = nbody.cpp sim.cpp
OBJ = nbody.o sim.o
COMMON_HDR = nbody.hpp

all: $(TARGET)

$(TARGET): $(OBJ)
	@$(CXX) $(CXXFLAGS) $(shell $(GTKFLAGS)) -o $@ $^ $(shell $(GTKLIBS)) -pthread

nbody.o: nbody.cpp $(COMMON_HDR)
	@$(CXX) $(CXXFLAGS) $(shell $(GTKFLAGS)) -c nbody.cpp

sim.o: sim.cpp $(COMMON_HDR)
	@$(CXX) $(CXXFLAGS) $(shell $(GTKFLAGS)) -c sim.cpp

clean:
	@rm -f $(TARGET) *.o
