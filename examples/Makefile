CXX=g++
# DEBUG_FLAGS=-g
# INCLUDE_FLAGS=-I/usr/include/libusb-1.0
# LIBRARY_FLAGS= -l
OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall -Wmissing-declarations
CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

APT_LIB=-L.. -lapt 
APT_INCLUDE=-I..

TESTS = test_piezo

all: $(TESTS)

test_piezo: libapt.a test_piezo.cpp
	$(CXX) $(CFLAGS) $(APT_INCLUDE) -o test_piezo test_piezo.cpp $(APT_LIB) $(LIBRARY_FLAGS) 

libapt.a:
	make -C ..

clean:
	rm -f *.o *~ $(TESTS)
