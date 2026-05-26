# Builds and runs the unit tests with plain g++. No framework needed.
#   make        build
#   make run    build and run

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter

ROOT     := ..
INCLUDES := -I$(ROOT)/include \
            -I$(ROOT)/lib/Matrix2D \
            -I$(ROOT)/lib/GaussianProcess2D \
            -I$(ROOT)/lib/Acquisition \
            -I$(ROOT)/lib/PIDController \
            -I$(ROOT)/lib/ThermalModel

HEADERS := $(wildcard $(ROOT)/include/*.h) $(wildcard $(ROOT)/lib/*/*.h)

test_units: test_units.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o test_units test_units.cpp -lm

run: test_units
	./test_units

clean:
	rm -f test_units

.PHONY: run clean
