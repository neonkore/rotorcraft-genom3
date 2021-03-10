#
# Copyright (c) 2015-2021 LAAS/CNRS
# All rights reserved.
#
# Redistribution and use  in source  and binary  forms,  with or without
# modification, are permitted provided that the following conditions are
# met:
#
#   1. Redistributions of  source  code must retain the  above copyright
#      notice and this list of conditions.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice and  this list of  conditions in the  documentation and/or
#      other materials provided with the distribution.
#
#					Anthony Mallet on Fri Feb 13 2015
#
lib_LTLIBRARIES = librotorcraft_codels.la

librotorcraft_codels_la_SOURCES  =	rotorcraft_c_types.h
librotorcraft_codels_la_SOURCES +=	rotorcraft_codels.c
librotorcraft_codels_la_SOURCES +=	rotorcraft_main_codels.c
librotorcraft_codels_la_SOURCES +=	rotorcraft_comm_codels.c
librotorcraft_codels_la_SOURCES +=	tty.c
librotorcraft_codels_la_SOURCES +=	calibration.cc
librotorcraft_codels_la_SOURCES +=	codels.h

librotorcraft_codels_la_CPPFLAGS =	$(requires_CFLAGS)
librotorcraft_codels_la_LIBADD   =	$(requires_LIBS)
librotorcraft_codels_la_CPPFLAGS+=	$(codels_requires_CFLAGS)
librotorcraft_codels_la_LIBADD  +=	$(codels_requires_LIBS)
librotorcraft_codels_la_LDFLAGS  =	-release $(PACKAGE_VERSION)

librotorcraft_codels_la_CPPFLAGS+=	$(libudev_CFLAGS)
librotorcraft_codels_la_LIBADD  +=	$(libudev_LIBS)

# idl  mappings
BUILT_SOURCES=	rotorcraft_c_types.h
CLEANFILES=	${BUILT_SOURCES}
rotorcraft_c_types.h: ${top_srcdir}/rotorcraft.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/rotorcraft_c_types.h.d
