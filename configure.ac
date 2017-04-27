dnl Autoconf file for building mikrokopter codels library.
dnl
dnl Copyright (c) 2015-2017 LAAS/CNRS
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

AC_INIT([mikrokopter-genom3],[1.5],[openrobots@laas.fr])
AC_CONFIG_MACRO_DIR([autoconf])
AC_CONFIG_AUX_DIR([autoconf])
AC_CONFIG_HEADERS([autoconf/acmikrokopter.h])
AM_INIT_AUTOMAKE([foreign no-define])

dnl Compilers
dnl
LT_INIT
AC_PROG_CC
AC_PROG_CXX


dnl External packages
PKG_CHECK_MODULES(requires, [
  openrobots-idl >= 1.2
  genom3 >= 2.99.26
  eigen3
])

AC_PATH_PROG(GENOM3, [genom3], AC_MSG_ERROR([genom3 tool not found], 2))


dnl --with-templates option
AG_OPT_TEMPLATES([$GENOM3 ],
    [$srcdir/mikrokopter.gen])


dnl Output
AC_CONFIG_FILES([
	mikrokopter-genom3.pc
	mikrokopter-genom3-uninstalled.pc
	Makefile
	codels/Makefile
])
AC_OUTPUT
AG_OUTPUT_TEMPLATES
