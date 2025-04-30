CXX = g++
CXXFLAGS = -std=c++11 -Wall -O2 `pkg-config --cflags gtkmm-3.0`
LDFLAGS = `pkg-config --libs gtkmm-3.0`

TARGET = nbody
OBJS = nbody.o sim.o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS)

nbody.o: nbody.cpp nbody.hpp
	$(CXX) $(CXXFLAGS) -c nbody.cpp

sim.o: sim.cpp nbody.hpp
	$(CXX) $(CXXFLAGS) -c sim.cpp

clean:
	rm -f $(TARGET) *.o
