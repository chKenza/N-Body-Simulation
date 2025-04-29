CXX = g++
CXXFLAGS = -std=c++11 -Wall -O2

TARGET = nbody
SRC = nbody.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

clean:
	rm -f $(TARGET)