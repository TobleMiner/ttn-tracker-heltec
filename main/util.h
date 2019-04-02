#pragma once

#include <stddef.h>

#ifndef offsetof
#define offsetof(type, member)  __builtin_offsetof (type, member)
#endif

#define container_of(ptr, type, member) ({                      \
        const typeof(((type *)0)->member) * __mptr = (ptr);     \
        (type *)((char *)__mptr - offsetof(type, member)); })

#ifndef min
#define min(x, y) (((x) < (y)) ? (x) : (y))
#endif

#ifndef max
#define max(x, y) (((x) > (y)) ? (x) : (y))
#endif

#ifndef swp
#define swp(_a, _b) \
        do { \
                typeof((_a)) tmp = (_a); \
                (_a) = (_b); \
                (_b) = tmp; \
        } while(0)
#endif

#define ARRAY_REVERSE(_arr, _len) \
        do { \
                typeof((_len)) i; \
                for(i = 0; i < _len / 2; i++) swp(((_arr)[i]), ((_arr)[_len - i - 1])); \
        } while(0)


#define ARRAY_LEN(_arr) (sizeof((_arr)) / sizeof(*(_arr)))

#define DIRENT_FOR_EACH(cursor, dir) \
        for((cursor) = readdir((dir)); (cursor); (cursor) = readdir((dir)))

void strntr(char* str, size_t len, char a, char b);

#define strtr(str, a, b) \
        strntr(str, strlen(str), a, b);


#define hex_to_nibble(hex) \
        (((hex) >= '0' && (hex) <= '9' ? (uint8_t)((hex) - '0') : \
                (hex) >= 'A' && (hex) <= 'F' ? (uint8_t)((hex) - 'A' + 0xA) : \
                        (hex) >= 'a' && (hex) <= 'f' ? (uint8_t)((hex) - 'a' + 0xA) : \
                                0 \
        ) & 0xF)

#define hex_to_byte(hex) \
        ((hex_to_nibble((hex)[0]) << 4) | hex_to_nibble((hex)[1]))
