dnl Autoconf file for building rotorcraft codels library.
dnl
dnl Copyright (c) 2015-2021 LAAS/CNRS
dnl All rights reserved.
dnl
dnl Redistribution and use  in source  and binary  forms,  with or without
dnl modification, are permitted provided that the following conditions are
dnl met:
dnl
dnl   1. Redistributions of  source  code must retain the  above copyright
dnl      notice and this list of conditions.
dnl   2. Redistributions in binary form must reproduce the above copyright
dnl      notice and  this list of  conditions in the  documentation and/or
dnl      other materials provided with the distribution.
dnl
dnl					Anthony Mallet on Fri Feb 13 2015
dnl

AC_PREREQ(2.59)

AC_INIT([rotorcraft-genom3],[3.1],[openrobots@laas.fr])
AC_CONFIG_MACRO_DIR([autoconf])
AC_CONFIG_AUX_DIR([autoconf])
AC_CONFIG_HEADERS([autoconf/acrotorcraft.h])
AM_INIT_AUTOMAKE([foreign no-define])
AC_CANONICAL_HOST

dnl Compilers
dnl
LT_INIT([disable-static])
AC_PROG_CC
AC_PROG_CXX


dnl Features
AC_SEARCH_LIBS([aio_write], [rt],, AC_MSG_ERROR([aio_write() not found], 2))


dnl Require GNU make
AC_CACHE_CHECK([for GNU make], [ac_cv_path_MAKE],
  [AC_PATH_PROGS_FEATURE_CHECK([MAKE], [make gmake],
    [case `$ac_path_MAKE --version 2>/dev/null` in
       *GNU*) ac_cv_path_MAKE=$ac_path_MAKE; ac_path_MAKE_found=:;;
     esac],
    [AC_MSG_ERROR([could not find GNU make])])])
AC_SUBST([MAKE], [$ac_cv_path_MAKE])


dnl External packages
PKG_CHECK_MODULES(requires, [
  openrobots2-idl >= 2.0
  genom3 >= 2.99.30
])
PKG_CHECK_MODULES(codels_requires, [
  eigen3
])

AC_PATH_PROG(GENOM3, [genom3], [no])
if test "$GENOM3" = "no"; then
  AC_MSG_ERROR([genom3 tool not found], 2)
fi

dnl --with-templates option
AG_OPT_TEMPLATES([$GENOM3 ],
    [$srcdir/rotorcraft.gen])


# check for TIOCGSERIAL, TIOCSSERIAL
AC_CHECK_HEADERS([sys/ioctl.h linux/serial.h])
if test "$ac_cv_header_sys_ioctl_h$ac_cv_header_linux_serial_h" = yesyes; then
  AC_MSG_CHECKING(ASYNC_LOW_LATENCY)
  AC_EGREP_CPP([yes],
[#include <sys/ioctl.h>
#include <linux/serial.h>

#ifdef TIOCGSERIAL
#ifdef TIOCSSERIAL
#ifdef ASYNC_LOW_LATENCY
       yes
#endif
#endif
#endif
], [AC_MSG_RESULT(yes)
   AC_DEFINE([HAVE_LOW_LATENCY_IOCTL], [], [FTDI low latency ioctl])],
   AC_MSG_RESULT(no))
fi

# libudev for serial <-> tty
case "${host_os}" in
  linux*) PKG_CHECK_MODULES(libudev, [libudev]);;
esac


dnl Doc
AM_MISSING_PROG([ASCIIDOCTOR], [asciidoctor])

dnl Output
AC_CONFIG_FILES([
	rotorcraft-genom3.pc
	rotorcraft-genom3-uninstalled.pc
	Makefile
	codels/Makefile
])
AC_OUTPUT
AG_OUTPUT_TEMPLATES
