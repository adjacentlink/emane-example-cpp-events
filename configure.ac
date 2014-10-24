AC_COPYRIGHT([
 Copyright (c) 2014 - Adjacent Link LLC, Bridgewater, New Jersey
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
 * Neither the name of Adjacent Link LLC nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
])

AC_INIT([emane-example-cpp-events],0.1.1,[emane at adjacentlink dot com])

AC_CONFIG_MACRO_DIR([m4])

AC_CONFIG_SRCDIR([src])

AM_INIT_AUTOMAKE([1.9.6])

AC_PROG_CXX

AC_PROG_CXXCPP

AX_CXX_COMPILE_STDCXX_11([noext],[mandatory])

AC_PROG_INSTALL

AC_PROG_LIBTOOL

AC_LANG([C++])

AC_ARG_WITH(debug,
           [  --with-debug            add debug support (off)])

PKG_CHECK_MODULES([protobuf], protobuf)
PKG_CHECK_MODULES([libuuid], uuid)

AC_CHECK_PROG(PROTOC_CHECK,protoc,yes)
if test x"$PROTOC_CHECK" != x"yes" ; then
    AC_MSG_FAILURE([protoc is required but missing])
fi

# compiler options
CXXFLAGS="$CXXFLAGS -W -Wall -O2"

# options for use with debug
if test "$with_debug" = "yes"
then
CPPFLAGS="$CPPFLAGS -g -O0"
LDFLAGS="$LDFLAGS -g -O0"
fi

LANG=C
AC_SUBST(LANG)

AC_OUTPUT(
 Makefile 
 src/Makefile
)
