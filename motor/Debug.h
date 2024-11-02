#ifndef __DEBUG_H
#define __DEBUG_H

#include <stdio.h>

#define USE_DEBUG 1
#if USE_DEBUG
#define DEBUG(__info, ...) printf("Debug : " __info, ##__VA_ARGS__)
#else
#define DEBUG(__info, ...)
#endif

#endif
