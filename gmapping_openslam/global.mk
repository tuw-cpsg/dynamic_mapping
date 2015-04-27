### You should not need to change anything below.
LINUX=1
MACOSX=0

# Compilers
CC=gcc
CXX=g++

# Paths
MAPPING_ROOT=/home/sbrugger/workspace/src/gmapping_openslam
LIBDIR=/home/sbrugger/workspace/src/gmapping_openslam/lib
BINDIR=/home/sbrugger/workspace/src/gmapping_openslam/bin

# Build tools
PRETTY=/home/sbrugger/workspace/src/gmapping_openslam/build_tools/pretty_compiler
MESSAGE=/home/sbrugger/workspace/src/gmapping_openslam/build_tools/message
TESTLIB=/home/sbrugger/workspace/src/gmapping_openslam/build_tools/testlib

# QT support
MOC=moc-qt3
QT_LIB=-lqt-mt
QT_INCLUDE=-I/usr/include/qt3

# ARIA support
ARIA_LIB=-L/usr/local/Aria/lib -lAria
ARIA_INCLUDE=-I/usr/local/Aria/include


# # KDE support
# KDE_LIB=
# KDE_INCLUDE=
# UIC=

# Generic makefiles
MAKEFILE_GENERIC=/home/sbrugger/workspace/src/gmapping_openslam/build_tools/Makefile.generic-shared-object
MAKEFILE_APP=/home/sbrugger/workspace/src/gmapping_openslam/build_tools/Makefile.app
MAKEFILE_SUBDIRS=/home/sbrugger/workspace/src/gmapping_openslam/build_tools/Makefile.subdirs


# Flags
CPPFLAGS+=-DLINUX -I/home/sbrugger/workspace/src/gmapping_openslam 
CXXFLAGS+=
LDFLAGS+= -Xlinker -rpath /home/sbrugger/workspace/src/gmapping_openslam/lib
CARMENSUPPORT=0
ARIASUPPORT=1



include /home/sbrugger/workspace/src/gmapping_openslam/manual.mk

