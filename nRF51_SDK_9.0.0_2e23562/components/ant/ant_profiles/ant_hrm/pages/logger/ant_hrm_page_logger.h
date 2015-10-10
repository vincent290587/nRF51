#ifndef ANT_HRM_PAGE_LOGGER_H__
#define ANT_HRM_PAGE_LOGGER_H__

#ifdef TRACE_ANT_GENERAL_ENABLE
#include "app_trace.h"
#define LOG_ANT printf
#else
#define LOG_ANT(...)
#endif // TRACE_HRM_GENERAL_ENABLE 

#ifdef TRACE_ANT_GENERAL_ENABLE
#include "app_trace.h"
#define LOG_PAGE0 printf
#else
#define LOG_PAGE0(...)
#endif // TRACE_HRM_PAGE_0_ENABLE

#ifdef TRACE_ANT_GENERAL_ENABLE
#include "app_trace.h"
#define LOG_PAGE1 printf
#else
#define LOG_PAGE1(...)
#endif // TRACE_HRM_PAGE_1_ENABLE 

#ifdef TRACE_ANT_GENERAL_ENABLE
#include "app_trace.h"
#define LOG_PAGE2 printf
#else
#define LOG_PAGE2(...)
#endif // TRACE_HRM_PAGE_2_ENABLE 

#ifdef TRACE_ANT_GENERAL_ENABLE
#include "app_trace.h"
#define LOG_PAGE3 printf
#else
#define LOG_PAGE3(...)
#endif // TRACE_HRM_PAGE_3_ENABLE 

#ifdef TRACE_ANT_GENERAL_ENABLE
#include "app_trace.h"
#define LOG_PAGE4 printf
#else
#define LOG_PAGE4(...)
#endif // TRACE_HRM_PAGE_4_ENABLE 

#endif // ANT_HRM_UTILS_H__
