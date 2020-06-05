TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

CONFIG += c++14

#QMAKE_CXXFLAGS_RELEASE += -march=native -msse -msse2
#QMAKE_CXXFLAGS_RELEASE -= -O2

#QMAKE_LFLAGS_RELEASE += -O3

SOURCES += main.cpp \
           bubblesmp/*.cc \
           bubblesmp/environment/*.cc \
           bubblesmp/generators/*.cc

HEADERS += bubblesmp/*.h \
         bubblesmp/environment/*.h \
         bubblesmp/generators/*.h \
    bubblesmp/environment/obj_loader.h

LIBS += -lgflags -lglog -lprotobuf -lboost_filesystem -lboost_serialization \
        -lboost_system -lboost_thread -lboost_unit_test_framework -lm \
        -lflann_cpp -lpthread -lfcl

LIBS += -L$$PWD/pqp/lib/ -lPQP

INCLUDEPATH += $$PWD/pqp/include
DEPENDPATH += $$PWD/pqp/include

PRE_TARGETDEPS += $$PWD/pqp/lib/libPQP.a

OTHER_FILES += bubblesmp/*.proto \
         bubblesmp/environment/*.proto \
         bubblesmp/generators/*.proto \

#INCLUDEPATH += /mnt/HDD/dinko/BubblesRRT/fcl/include

