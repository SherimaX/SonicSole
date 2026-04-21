CXX := g++
CPPFLAGS := -MMD -MP
CXXFLAGS := -std=c++17
LDLIBS := -lwiringPi -lpthread

TARGET := SonicSole_RPi
BUILD_DIR := build/sonicsole_rpi
SRCS := \
	SonicSole_RPi.cpp \
	SonicSole_Core.cpp \
	SonicSole_Pressure.cpp \
	SonicSole_IMU.cpp \
	SonicSole_UDP.cpp \
	SonicSole_Activity.cpp
OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SRCS))
DEPS := $(OBJS:.o=.d)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDLIBS)

$(BUILD_DIR)/%.o: %.cpp SonicSole.h SonicSole_Activity.h RPi_combined_Header.h RPi_Raj_Header.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

-include $(DEPS)
