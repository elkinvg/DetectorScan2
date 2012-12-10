#    TARGET        = libfoundcircles.so.X.X.X libdetectorscan.so.X.X.X
#   lib.so$(((LIBINDEX1)$(LIBINDEX2))$(LIBINDEX))
    LIBINDEX1     = .1
    LIBINDEX2     = $(LIBINDEX1).2
    LIBINDEX      = $(LIBINDEX2).0
    LIBNAME       = lib"$*".so$(LIBINDEX)

#   TARGET        = libqdsandfc.so.X.X.X
#   library with QT dev libqdsandfc.so(((QLIBINDEX1)$(QLIBINDEX2))$(QLIBINDEX))
    QLIB          = libqdsandfc.so
    QLIBINDEX1    = .0
    QLIBINDEX2    = $(QLIBINDEX1).5
    QLIBINDEX     = $(QLIBINDEX2).0
    QLIBNAME      = $(QLIB)$(QLIBINDEX)

# ---------------------------------------------------------------
    OBJDIR        = ./obj
    BINDIR        = ./bin
    TARGET        = DetectorScan
    TARGETLOC     = $(BINDIR)/$(TARGET)
    MKDIR         = mkdir -p
    SODIR         = ./SO
    
    LNMAKE        = @ln -s
    DELFILE       = @rm -f
    STRIP         = strip
#    INSTALL_FILE  = install -m 644 -p
#    INSTALL_DIR   = $(COPY_DIR)
    INSTALL_PR    = install -m 755 -p
    
    PKGFLAGS = $(shell pkg-config --cflags opencv)
    PKGLIBS = $(shell pkg-config --libs opencv)
#    ROOTFLAGS = $(shell root-config --cflags)
#    ROOTLIBS = $(shell root-config --libs)
    QTFLAGS = -I/usr/share/qt4/mkspecs/linux-g++ -I. -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4
    RELEASE_DEFINES = -O2
    DEFINEMINPIX  = -DMINNUMPIX=300
    DEBUG_DEFINE  = -O0 -g -DDEBUG -Wall
    CXXFLAGS = $(PKGFLAGS) $(DEBUG_DEFINE)

    CC            = gcc
    CXX           = g++
    LINK          = g++
#    OBJS = tempdetectorscan.o detectorscan.o funcofroot.o foundcircles.o foundofroot.o main.o dsvisual.o dsvisualmethofopencv.o
    OBJS = $(OBJDIR)/detectorscan.o $(OBJDIR)/foundcircles.o $(OBJDIR)/runforinspection.o $(OBJDIR)/main.o $(OBJDIR)/dsvisualmethofopencv.o

#    DIR = if ! [ -d $(SODIR) ] ; then $(MKDIR) $(SODIR) ; fi
#    SOFLAGS = -fpic -shared
    QOBJS = $(OBJDIR)/libdetectorscan.o $(OBJDIR)/libqdsandfc.o
    SOFLAGS       = -fpic -Wl,-O1 -shared
    LFLAGS        = -Wl,-soname,
    
    QSOFLAGS      = -c -fpic
    QLSOFLAGS     = -shared -Wl,-O1
    
#    LFLAGS  = -Wl,-O1 -shared -Wl,-soname,libfoundcircles.so.1



    MAKELIBS      = libdetectorscan.so libfoundcircles.so
    MAKELIBS1     = $(SODIR)/libdetectorscan.so $(SODIR)/libfoundcircles.so
    MAKEQDSLIBS   = $(SODIR)/libqdsandfc.so




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

####### libraries
libraries: $(MAKELIBS1) 
	
$(MAKELIBS1): $(SODIR)/lib%.so: %.cpp

	@if ! [ -d $(SODIR) ] ; then $(MKDIR) $(SODIR) ; fi
	$(DELFILE) "$@"*
	$(CXX) $(SOFLAGS) $(QTFLAGS) $(LFLAGS)$@$(LIBINDEX1) -o $@ $<
	@mv $@ $@$(LIBINDEX)
	$(LNMAKE) $(LIBNAME) $@
	$(LNMAKE) $(LIBNAME) $@$(LIBINDEX1)
	$(LNMAKE) $(LIBNAME) $@$(LIBINDEX2)

qlib:
#	$(CXX) -fPIC -shared
	@if ! [ -d $(SODIR) ] ; then $(MKDIR) $(SODIR) ; fi
	$(DELFILE) $(SODIR)/$(QLIB)*
	$(CXX) $(QSOFLAGS) detectorscan.cpp -o $(OBJDIR)/libdetectorscan.o
	$(CXX) $(QSOFLAGS) $(QTFLAGS) qdsandfc.cpp -o $(OBJDIR)/libqdsandfc.o
	$(CXX) $(QOBJS) $(QLSOFLAGS) $(LFLAGS)$(QLIB)$(QLIBINDEX1) -o $(SODIR)/$(QLIB)
	@mv $(SODIR)/$(QLIB) $(SODIR)/$(QLIB)$(QLIBINDEX)
	$(LNMAKE) $(QLIBNAME) $(SODIR)/$(QLIB)
	$(LNMAKE) $(QLIBNAME) $(SODIR)/$(QLIB)$(QLIBINDEX1)
	$(LNMAKE) $(QLIBNAME) $(SODIR)/$(QLIB)$(QLIBINDEX2)

# install uninstall

install_target: first FORCE
	@$(INSTALL_PR) "$(TARGETLOC)" "$(INSTALL_ROOT)/usr/bin/$(TARGET)"
	@$(STRIP) --strip-unneeded "$(INSTALL_ROOT)/usr/bin/$(TARGET)"

uninstall_target:
	$(DELFILE) "$(INSTALL_ROOT)/usr/bin/$(TARGET)"

install: install_target  FORCE

uninstall: uninstall_target   FORCE
	
clean: FORCE
	$(DELFILE) ./SO/*.so ./SO/*.so* ./obj/*.o *.h~ *.cpp~
remake: clean first
remake_release: clean release
remake_libraries: clean libraries

FORCE:
