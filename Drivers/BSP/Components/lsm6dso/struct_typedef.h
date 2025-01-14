#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#include <stdint.h>

#if defined(__CC_ARM) /* ARM Compiler */
#define __packed __packed
#elif defined(__ICCARM__) /* IAR Compiler */
#define __packed __packed
#elif defined(__GNUC__) /* GNU Compiler */
#define __packed __attribute__((__packed__))
#elif defined(__TASKING__) /* TASKING Compiler */
#define __packed __unaligned
#endif /* __CC_ARM */

// typedef signed char int8_t;
// typedef signed short int int16_t;
// typedef signed int int32_t;
// typedef signed long long int64_t;

/* exact-width unsigned integer types */
// typedef unsigned char uint8_t;
// typedef unsigned short int uint16_t;
// typedef unsigned int uint32_t;
// typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#endif
