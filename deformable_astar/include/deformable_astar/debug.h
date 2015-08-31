#include "stdlib.h"
#include "stdio.h"
#include "stdarg.h"

inline int debug(int global_debug_level, int current_debug_level, const char * format, ...)
{
    if ((current_debug_level >= 0) && (current_debug_level <= global_debug_level))
    {
        va_list arg;
        va_start(arg,format);
        int done = vprintf(format, arg);
        va_end(arg);
        return done;
    }
    else if (current_debug_level == -1)
    {
        va_list arg;
        va_start(arg,format);
        int done = vprintf(format, arg);
        va_end(arg);
        fflush(stdout);
        return done;
    }
    return 0;
}
