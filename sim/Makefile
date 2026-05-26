# Builds the laptop simulation. Just run `make` in this directory.
#
# Pulls in the same headers the firmware uses (the GP, the matrix code, the
# PID, the acquisition) by pointing the include paths at lib/ and include/.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter

ROOT     := ..
INCLUDES := -I$(ROOT)/include \
            -I$(ROOT)/lib/Matrix2D \
            -I$(ROOT)/lib/GaussianProcess2D \
            -I$(ROOT)/lib/Acquisition \
            -I$(ROOT)/lib/PIDController \
            -I$(ROOT)/lib/ThermalModel

TARGET := acpto_sim
SRC    := sim_main.cpp

# Rebuild if any shared header changes, not just sim_main.cpp.
HEADERS := $(wildcard $(ROOT)/include/*.h) \
           $(wildcard $(ROOT)/lib/*/*.h)

$(TARGET): $(SRC) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET) $(SRC) -lm

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: run clean
