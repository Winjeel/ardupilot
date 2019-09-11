# This makefile will take all the *.cpp files in the current directory and
# generate an executable from each. As such, it expects that each CPP file is a
# valid program, and has no dependiencies on other object files.


# put pre-processor settings (-I, -D, etc) here
CPPFLAGS =
# put compiler settings here
CXXFLAGS = -g -Wall -Werror

SOURCES=$(wildcard *.cpp)
OBJECTS=$(SOURCES:.cpp=.o)
BINS=$(SOURCES:.cpp=)

all: $(BINS)

.PHONY: clean

clean:
	$(RM) $(OBJECTS) $(BINS)
