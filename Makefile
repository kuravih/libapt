CXX=g++
# DEBUG_FLAGS=-g
INCLUDE_FLAGS=-I
# LIBRARY_FLAGS=-l

OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall -Wmissing-declarations
CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

all: libapt.a

libapt.a: libapt.o kpz101.o
	ar -cr libapt.a libapt.o kpz101.o; rm -r *.o

libapt.o: libapt.cpp
	$(CXX) $(CFLAGS) -c libapt.cpp $(LIBRARY_FLAGS)

kpz101.o: kpz101.cpp
	$(CXX) $(CFLAGS) -c kpz101.cpp $(LIBRARY_FLAGS)

clean:
	rm -f libapt.a *.o

flush:
	rm -f *.o *.a