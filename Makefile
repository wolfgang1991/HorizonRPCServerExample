#List of object files without path
_LINKOBJ = main.o

COMMONLIBPATH = ./CommonLibraries
SRCDIR = .
OBJDIR = $(SRCDIR)/obj
CPPFLAGS = -D_DEBUG=$(DEBUG) -Wall -I$(COMMONLIBPATH)/Common -I$(COMMONLIBPATH)/RPC/JSONRPC2 -I$(COMMONLIBPATH)/RPC
COMMONLIBFLAGS = -L$(COMMONLIBPATH)/RPC/JSONRPC2 -lJSONRPC2 -L$(COMMONLIBPATH)/Common -lCommon -pthread
EXECFILE = ./HorizonRPCServerExample
USEROPTIM = 

all: all_linux

include $(COMMONLIBPATH)/MakefileConsoleCommon

build_deps:
	cd $(COMMONLIBPATH)/RPC/JSONRPC2 && "$(MAKE)" DEBUG=$(DEBUG)
	cd $(COMMONLIBPATH)/Common && "$(MAKE)" DEBUG=$(DEBUG)
	
clean_deps:
	cd $(COMMONLIBPATH)/RPC/JSONRPC2 && "$(MAKE)" clean
	cd $(COMMONLIBPATH)/Common && "$(MAKE)" clean

