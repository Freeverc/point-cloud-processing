#define _CRT_SECURE_NO_WARNINGS

#ifndef ZPIPE_H
#define ZPIPE_H

#if defined(MSDOS) || defined(OS2) || defined(WIN32) || defined(__CYGWIN__)
#include <fcntl.h>
#include <io.h>
#define SET_BINARY_MODE(file) _setmode(_fileno(file), O_BINARY)
#else
#  define SET_BINARY_MODE(file)
#endif


#include <iostream>
#include <cstring>
#include <cassert>
#include "zlib.h"

#define CHUNK 16384

class Zpipe {
  public:
    int def( FILE *, FILE *, int ); // compress
    int inf( FILE *, FILE * ); // decompress
    void zerr( int );
    void usage( char ** );
};
#endif