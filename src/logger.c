#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "config.h"
#include "logger.h"

#define _FMT_MAX_SIZE 1024
#define _MESSAGE_MAX_SIZE 4096

#define _DEBUG_STR "[ DEBUG ]\0"
#define _INFO_STR "[ INFO  ]\0"
#define _WARNING_STR "[WARNING]\0"
#define _ERROR_STR "[ ERROR ]\0"
#define _FAILURE_STR "[FAILURE]\0"
#define _SUCCESS_STR "[SUCCESS]\0"

static char _levelStrings[6][10] = {_DEBUG_STR, _INFO_STR, _WARNING_STR, _ERROR_STR, _FAILURE_STR, _SUCCESS_STR};

static char FORMAT[_FMT_MAX_SIZE];
void openLog() {
}


void _logger(LOG_LEVEL level, const char* message, ...) {
	#ifdef NO_LOGGER
		return;
	#else

    memset(FORMAT, 0, _FMT_MAX_SIZE);
    va_list vargs;
    va_start(vargs, message);
    vsprintf(FORMAT, message, vargs);
    va_end(vargs);


    if (level == LDEBUG) {
        #ifdef DEBUG_MODE
    	printf("%s%s", FORMAT, SYS_ENDL);
        #endif // DEBUG_MODE
    }
    else {
        printf("%s%s", FORMAT, SYS_ENDL);
    }
	#endif
}
