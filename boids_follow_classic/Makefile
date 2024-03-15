# Compiler and flags
CXX = clang++
CXXFLAGS = -std=c++11 -I /opt/homebrew/Cellar/fltk/HEAD-ecb3e40/include -I /usr/local/include/Stage-4.3/

# Libraries and paths
LDFLAGS = -L /opt/homebrew/Cellar/fltk/HEAD-ecb3e40/lib -L /usr/local/lib/
LDLIBS = -l fltk -l stage.4.3.0 -framework OpenGl

# Targets
TARGET = main
SRCS = main.cpp ./bot/ConvoyRobot.cpp ../common/bot/VipRobot.cpp ../common/bot/CircleVisualizer.cpp
OBJS = $(SRCS:.cpp=.o)

# Dependency paths
export DYLD_LIBRARY_PATH := /opt/homebrew/Cellar/fltk/HEAD-ecb3e40/lib:/usr/local/lib:$(DYLD_LIBRARY_PATH)
export STAGE_HEADER := /usr/local/include/Stage-4.3
export STAGEPATH := /usr/local/lib

# Phony targets
.PHONY: build clean

# Default target
build: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $(SRCS) -o $@ $(LDLIBS)

clean:
	rm -f $(OBJS) $(TARGET)
