#ifndef DEBUG_H
#define DEBUG_H

#define DEBUG 1

#if DEBUG
#define DBG_PRINT(...)    Serial.printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#endif

#endif // DEBUG_H
