#
# Copyright (c) 2015 LAAS/CNRS
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
lib_LTLIBRARIES = libmikrokopter_codels.la

libmikrokopter_codels_la_SOURCES=	mikrokopter_c_types.h
libmikrokopter_codels_la_SOURCES+=	mikrokopter_codels.c
libmikrokopter_codels_la_SOURCES+=	mikrokopter_main_codels.c
libmikrokopter_codels_la_SOURCES+=	mikrokopter_comm_codels.c
libmikrokopter_codels_la_SOURCES+=	tty.c
libmikrokopter_codels_la_SOURCES+=	codels.h

libmikrokopter_codels_la_CPPFLAGS=	$(requires_CFLAGS)
libmikrokopter_codels_la_LDFLAGS=	$(requires_LIBS)
libmikrokopter_codels_la_LDFLAGS+=	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	mikrokopter_c_types.h
CLEANFILES=	${BUILT_SOURCES}
mikrokopter_c_types.h: ${top_srcdir}/mikrokopter.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/mikrokopter_c_types.h.d
