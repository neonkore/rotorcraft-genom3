dnl Autoconf file for building mikrokopter codels library.
dnl
dnl Copyright (c) 2015-2019 LAAS/CNRS
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

AC_INIT([mikrokopter-genom3],[2.0],[openrobots@laas.fr])
AC_CONFIG_MACRO_DIR([autoconf])
AC_CONFIG_AUX_DIR([autoconf])
AC_CONFIG_HEADERS([autoconf/acmikrokopter.h])
AM_INIT_AUTOMAKE([foreign no-define])
AC_CANONICAL_HOST

dnl Compilers
dnl
LT_INIT([disable-static])
AC_PROG_CC
AC_PROG_CXX

dnl Features
AC_SEARCH_LIBS([aio_write], [rt],, AC_MSG_ERROR([aio_write() not found], 2))

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
    [$srcdir/mikrokopter.gen])


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
	mikrokopter-genom3.pc
	mikrokopter-genom3-uninstalled.pc
	Makefile
	codels/Makefile
])
AC_OUTPUT
AG_OUTPUT_TEMPLATES
