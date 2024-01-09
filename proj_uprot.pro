TEMPLATE = lib
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += dynamiclib
CONFIG += staticlib

TOOLCHAINPATH=/home/usr/WORKSPACE/SDK/aarch64-linux

INCLUDEPATH += "$$TOOLCHAINPATH/aarch64-xilinx-linux/usr/include"
DEPENDPATH += "$$TOOLCHAINPATH/aarch64-xilinx-linux/usr/include"
INCLUDEPATH += "$$TOOLCHAINPATH/aarch64-xilinx-linux/usr/include/c++/10.2.0"
DEPENDPATH += "$$TOOLCHAINPATH/aarch64-xilinx-linux/usr/include/c++/10.2.0"
INCLUDEPATH += "$$TOOLCHAINPATH/aarch64-xilinx-linux/usr/include/c++/10.2.0/aarch64-xilinx-linux"
DEPENDPATH += "$$TOOLCHAINPATH/aarch64-xilinx-linux/usr/include/c++/10.2.0/aarch64-xilinx-linux"
INCLUDEPATH += "$$TOOLCHAINPATH/aarch64-xilinx-linux/usr/include/c++/10.2.0/backward"
DEPENDPATH += "$$TOOLCHAINPATH/aarch64-xilinx-linux/usr/include/c++/10.2.0/backward"0
INCLUDEPATH += "$$TOOLCHAINPATH/x86_64-petalinux-linux/usr/lib/aarch64-xilinx-linux/gcc/aarch64-xilinx-linux/10.2.0/include"
DEPENDPATH += "$$TOOLCHAINPATH/x86_64-petalinux-linux/usr/lib/aarch64-xilinx-linux/gcc/aarch64-xilinx-linux/10.2.0/include"
INCLUDEPATH += "$$TOOLCHAINPATH/x86_64-petalinux-linux/usr/lib/aarch64-xilinx-linux/gcc/aarch64-xilinx-linux/10.2.0/include-fixed"
DEPENDPATH += "$$TOOLCHAINPATH/x86_64-petalinux-linux/usr/lib/aarch64-xilinx-linux/gcc/aarch64-xilinx-linux/10.2.0/include-fixed"

QMAKE_CFLAGS=\
-Wall\
-O0\
-fpermissive\
-g3\
-c\
-g\
-fmessage-length=0\
-std=c++11

LIBS += -L/home/usr/WORKSPACE/SDK/cortexa72-cortexa53-xilinx-linux/lib\
-L/home/usr/WORKSPACE/SDK/cortexa72-cortexa53-xilinx-linux/usr/lib\
-lpthread

SOURCES += \
    proj_uprot.cpp

HEADERS += \
    proj_uport.hpp

DISTFILES += \
    readme.md
