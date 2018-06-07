#
# Copyright (c) 2015-2018 LAAS/CNRS
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
#                                      Anthony Mallet on Thu May 31 2018
#
lib_LTLIBRARIES = libuavpos_codels.la

libuavpos_codels_la_SOURCES=	uavpos_c_types.h
libuavpos_codels_la_SOURCES+=	uavpos_codels.c
libuavpos_codels_la_SOURCES+=	uavpos_main_codels.c

libuavpos_codels_la_SOURCES+=	controller.cc
libuavpos_codels_la_SOURCES+=	codels.h

libuavpos_codels_la_CPPFLAGS=	$(requires_CFLAGS)
libuavpos_codels_la_LDFLAGS=	$(requires_LIBS)
libuavpos_codels_la_LDFLAGS+=	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	uavpos_c_types.h
CLEANFILES=	${BUILT_SOURCES}
uavpos_c_types.h: ${top_srcdir}/uavpos.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/uavpos_c_types.h.d
