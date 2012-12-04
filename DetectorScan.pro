TEMPLATE = app
CONFIG += console
CONFIG -= qt
MOC_DIR = moc
OBJECTS_DIR = obj
DESTDIR += bin


win32 {
INCLUDEPATH += "c:/opt/opencv/mingw32/include"
#INCLUDEPATH += "c:/dev/OpenCV-2.3.0/mingw32/include"
#INCLUDEPATH += . include
LIBS += "c:/opt/opencv/mingw32/lib/*.a"
#LIBS += "c:/dev/OpenCV-2.3.0/mingw32/lib/*.a"

#LIBS += -L"./lib" -lds
}

#unix
unix {
LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_flann
#INCLUDEPATH += -I/usr/local/include/opencv -I/usr/local/include
INCLUDEPATH += -I/usr/local/include
#LIBS += -L"./lib" -lds
}

SOURCES += main.cpp \
    detectorscan.cpp \
    dsvisualmethofopencv.cpp \
    foundcircles.cpp \
    runforinspection.cpp

HEADERS += \
    detectorscan.h \
    commonstruct.h \
    dsvisualmethofopencv.h \
    foundcircles.h \
    runforinspection.h


