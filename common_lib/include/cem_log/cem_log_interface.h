#pragma once

#include "cem_log.h"

#define __SHORT_FILE__ (__FILE__ + CemLog::require_at_compile_time<CemLog::basename_index(__FILE__)>::value)

#define SPEC_LOG(LOG_LEVEL, CONTEXTID, fmt, ...) CemLog::CemLogPt::Instance().Print(CONTEXTID, LOG_LEVEL, "(%s|%s:%d) " fmt, __FUNCTION__, __SHORT_FILE__, __LINE__, ##__VA_ARGS__)
#define SPEC_LOG_DEBUG(CONTEXTID, fmt, ...) SPEC_LOG(CemLogLevel::DEBUG, CONTEXTID, fmt, ##__VA_ARGS__)
#define SPEC_LOG_INFO(CONTEXTID, fmt, ...)  SPEC_LOG(CemLogLevel::INFO, CONTEXTID, fmt, ##__VA_ARGS__)
#define SPEC_LOG_WARN(CONTEXTID, fmt, ...)  SPEC_LOG(CemLogLevel::WARN, CONTEXTID, fmt, ##__VA_ARGS__)
#define SPEC_LOG_ERROR(CONTEXTID, fmt, ...) SPEC_LOG(CemLogLevel::ERROR, CONTEXTID, fmt, ##__VA_ARGS__)
#define SPEC_LOG_FATAL(CONTEXTID, fmt, ...) SPEC_LOG(CemLogLevel::FATAL, CONTEXTID, fmt, ##__VA_ARGS__)

#define SSLOG(LOG_LEVEL, ...) CemLog::CemSsLog(LOG_LEVEL, ##__VA_ARGS__) << "(" << __FUNCTION__ << "|" << __SHORT_FILE__ << ":" << __LINE__ << ") "
#define SSLOG_DEBUG(...)  SSLOG(CemLogLevel::DEBUG, ##__VA_ARGS__)
#define SSLOG_INFO(...)  SSLOG(CemLogLevel::INFO, ##__VA_ARGS__)
#define SSLOG_WARN(...)  SSLOG(CemLogLevel::WARN, ##__VA_ARGS__)
#define SSLOG_ERROR(...)  SSLOG(CemLogLevel::ERROR, ##__VA_ARGS__)
#define SSLOG_FATAL(...)  SSLOG(CemLogLevel::FATAL, ##__VA_ARGS__)

#define SIMP_LOG(LOG_LEVEL, fmt, ...) SPEC_LOG(LOG_LEVEL, CemLog::CemLogPt::DefaultContextId_, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...)  SIMP_LOG(CemLogLevel::DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)  SIMP_LOG(CemLogLevel::INFO, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  SIMP_LOG(CemLogLevel::WARN, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)  SIMP_LOG(CemLogLevel::ERROR, fmt, ##__VA_ARGS__)
#define LOG_FATAL(fmt, ...)  SIMP_LOG(CemLogLevel::FATAL, fmt, ##__VA_ARGS__)

#define LOG_INIT(...) CemLog::CemLogPt::Instance().Init(__VA_ARGS__)
#define LOG_ADDCONTEXT(...) CemLog::CemLogPt::Instance().AddContext(__VA_ARGS__)