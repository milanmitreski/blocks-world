CXX = g++
CXXFLAGS = -std=c++17

SRCS = main.cpp sat.cpp blocks_world.cpp planning.cpp
TARGET = blocks_world

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $@

clean:
	rm $(TARGET)

.PHONY: clean