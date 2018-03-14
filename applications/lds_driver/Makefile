DIR_ROOT  = ../

CC 				= gcc
CX 				= g++
CCFLAGS		= -Wall -c
CXFLAGS 	= -Wall -c
CP     		= cp
CP_ALL  	= cp -r
RM        = rm
RM_ALL    = rm -f
SRCS			= lds_driver.cpp
OBJS			= lds_driver.o
LIB_DIRS	= -L/usr/lib/x86_64-linux-gnu
INC_DIRS  = -I/usr/include
LIBS      = -lboost_system
TARGET 	  = lds_driver

all : $(TARGET)
	$(CXX) -o $(TARGET) $(OBJS) $(INC_DIRS) $(LIB_DIRS) $(LIBS) 
 
$(TARGET) :
	$(CXX) -c $(SRCS) $(INC_DIRS) $(LIB_DIRS) $(LIBS) 

clean:
	$(RM_ALL) $(OBJS) $(TARGET)
