// log.cpp -- see log.h for the macro-based logging API.
#include "log.h"

static_assert(LOG_LEVEL >= LOG_NONE && LOG_LEVEL <= LOG_TRACE,
              "LOG_LEVEL must be one of LOG_NONE..LOG_TRACE (see log.h)");
