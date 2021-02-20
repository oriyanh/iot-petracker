#ifndef IOT_LOGGER_H
#define IOT_LOGGER_H

typedef enum LOG_LEVEL {LDEBUG = 0, LINFO, LWARNING, LERROR, LFAIL, LSUCCESS} LOG_LEVEL;

void _logger(LOG_LEVEL level, const char* message, ...);

#define logDebug(args...) _logger(LDEBUG, args)
#define logInfo(args...) _logger(LINFO, args)
#define logError(args...) _logger(LERROR, args)
#define logWarn(args...) _logger(LWARNING, args)
#define logFail(args...) _logger(LFAIL, args)
#define logSuccess(args...) _logger(LSUCCESS, args)

#endif //IOT_LOGGER_H
