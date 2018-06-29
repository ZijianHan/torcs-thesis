##############################################################################
#
#    file                 : Makefile
#    created              : tis 27 feb 2018 08:54:58 CET
#    copyright            : (C) 2002 Zijian
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = leftlanekeeper
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp driver.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml car7-trb1.rgb logo.rgb
SHIPSUBDIRS =

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-leftlanekeeper_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-leftlanekeeper_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
