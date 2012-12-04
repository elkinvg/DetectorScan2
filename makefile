    PKGFLAGS = $(shell pkg-config --cflags opencv)
    PKGLIBS = $(shell pkg-config --libs opencv)
#    ROOTFLAGS = $(shell root-config --cflags)
#    ROOTLIBS = $(shell root-config --libs)
    RELEASE_DEFINES = -O2
    DEFINEMINPIX  = -DMINNUMPIX=300
    DEBUG_DEFINE  = -O0 -g -DDEBUG -Wall
    CXXFLAGS = $(PKGFLAGS) $(DEBUG_DEFINE)

    CC            = gcc
    CXX           = g++
    LINK          = g++
#    OBJS = tempdetectorscan.o detectorscan.o funcofroot.o foundcircles.o foundofroot.o main.o dsvisual.o dsvisualmethofopencv.o
    OBJS = $(OBJDIR)/detectorscan.o $(OBJDIR)/foundcircles.o $(OBJDIR)/runforinspection.o $(OBJDIR)/main.o $(OBJDIR)/dsvisualmethofopencv.o
    MKDIR         = mkdir -p
    SODIR         = ./SO
#    DIR = if ! [ -d $(SODIR) ] ; then $(MKDIR) $(SODIR) ; fi
#    SOFLAGS = -fpic -shared
    SOFLAGS       = -fPIC -Wl,-O1 -shared
    LFLAGS        = -Wl,-soname,
#    LFLAGS  = -Wl,-O1 -shared -Wl,-soname,libfoundcircles.so.1



    MAKELIBS      = libdetectorscan.so libfoundcircles.so
    MAKELIBS1     = $(SODIR)/libdetectorscan.so $(SODIR)/libfoundcircles.so
    OBJDIR        = ./obj
    BINDIR        = ./bin
    TARGET        = DetectorScan
    TARGETLOC     = $(BINDIR)/$(TARGET)
    
#    TARGET        = libfoundcircles.so.1.0.0
#   lib.so$(((LIBINDEX1)$(LIBINDEX2))$(LIBINDEX))
    LIBINDEX1     = .1
    LIBINDEX2     = $(LIBINDEX1).0
    LIBINDEX      = $(LIBINDEX2).0
    LIBNAME       = lib"$*".so$(LIBINDEX)
    LNMAKE        = @ln -s
    DELFILE       = @rm -f
    STRIP         = strip
    INSTALL_FILE  = install -m 644 -p
    INSTALL_DIR   = $(COPY_DIR)
    INSTALL_PR    = install -m 755 -p




#Detectorscan: 
#$(OBJS)
first: all

release: CXXFLAGS=$(PKGFLAGS) $(RELEASE_DEFINES)
release: all

####### Build rules

all: $(TARGETLOC)
$(TARGETLOC): $(OBJS)
	@if ! [ -d $(BINDIR) ] ; then $(MKDIR) $(BINDIR) ; fi
	$(LINK) $(OBJS) $(PKGLIBS) -o $(TARGETLOC)

####### compile
$(OBJDIR)/%.o: %.cpp
	@if ! [ -d $(OBJDIR) ] ; then $(MKDIR) $(OBJDIR) ; fi
	$(CXX) -c $(CXXFLAGS) -o $@ $<


.SUFFIXES: .cpp .o


libraries: $(MAKELIBS1) 
	
$(MAKELIBS1): $(SODIR)/lib%.so: %.cpp

	@if ! [ -d $(SODIR) ] ; then $(MKDIR) $(SODIR) ; fi
	$(DELFILE) "$@"*
	$(CXX) $(SOFLAGS) $(LFLAGS)$@$(LIBINDEX1) -o $@ $<
	@mv $@ $@$(LIBINDEX)
	$(LNMAKE) $(LIBNAME) $@
	$(LNMAKE) $(LIBNAME) $@$(LIBINDEX1)
	$(LNMAKE) $(LIBNAME) $@$(LIBINDEX2)

# install uninstall

install_target: first FORCE
	@$(INSTALL_PR) "$(TARGETLOC)" "$(INSTALL_ROOT)/usr/bin/$(TARGET)"
	@$(STRIP) --strip-unneeded "$(INSTALL_ROOT)/usr/bin/$(TARGET)"

uninstall_target:
	$(DELFILE) "$(INSTALL_ROOT)/usr/bin/$(TARGET)"

install: install_target  FORCE

uninstall: uninstall_target   FORCE
	
clean: FORCE
	$(DELFILE) ./SO/* ./obj/*.o *.h~ *.cpp~
remake: clean first
remake_release: clean release
remake_libraries: clean libraries

FORCE:
