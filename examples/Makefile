CXX=g++
# DEBUG_FLAGS=-g
# INCLUDE_FLAGS=-I/usr/include/libusb-1.0
# LIBRARY_FLAGS= -l
OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall -Wmissing-declarations
CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

APT_LIB=-L.. -lapt 
APT_INCLUDE=-I..

TESTS = test_kpz101 test_kst101

all: $(TESTS)

test_kpz101: libapt.a test_kpz101.cpp
	$(CXX) $(CFLAGS) $(APT_INCLUDE) -o test_kpz101 test_kpz101.cpp $(APT_LIB) $(LIBRARY_FLAGS)

test_kst101: libapt.a test_kst101.cpp
	$(CXX) $(CFLAGS) $(APT_INCLUDE) -o test_kst101 test_kst101.cpp $(APT_LIB) $(LIBRARY_FLAGS)

libapt.a:
	make -C ..

clean:
	rm -f *.o *~ $(TESTS)
