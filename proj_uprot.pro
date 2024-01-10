TEMPLATE = lib
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += dynamiclib
CONFIG += staticlib

SOURCES += \
    proj_uprot.cpp

HEADERS += \
    proj_uport.hpp

DISTFILES += \
    readme.md

LIBS +=\
"-lpthread"\
