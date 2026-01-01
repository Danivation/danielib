################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES := 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=1

# name and version of library (if IS_LIBRARY is 1)
LIBNAME:=danielib
VERSION:=4.4.0

# files to always include in the hot package, or to leave out of the library/cold package
EXCLUDE_SRC_FROM_LIB += $(SRCDIR)/main.cpp

# header files that get distributed with the package, includes all within include/libname
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/**/*.h $(INCDIR)/$(LIBNAME)/**/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk
