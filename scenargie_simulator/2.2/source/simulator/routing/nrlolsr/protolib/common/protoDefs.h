/*********************************************************************
 *
 * AUTHORIZATION TO USE AND DISTRIBUTE
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that: 
 *
 * (1) source code distributions retain this paragraph in its entirety, 
 *  
 * (2) distributions including binary code include this paragraph in
 *     its entirety in the documentation or other materials provided 
 *     with the distribution, and 
 *
 * (3) all advertising materials mentioning features or use of this 
 *     software display the following acknowledgment:
 * 
 *      "This product includes software written and developed 
 *       by Code 5520 of the Naval Research Laboratory (NRL)." 
 *         
 *  The name of NRL, the name(s) of NRL  employee(s), or any entity
 *  of the United States Government may not be used to endorse or
 *  promote  products derived from this software, nor does the 
 *  inclusion of the NRL written and developed software  directly or
 *  indirectly suggest NRL or United States  Government endorsement
 *  of this product.
 * 
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 ********************************************************************/

#ifndef _PROTO_DEFS
#define _PROTO_DEFS


#ifdef WIN32
//
// Windwos definition
//
#include <stdint.h>

#include <time.h>  // for mktime()
#include <basetsd.h>  // for UINT32/INT32 types
#ifdef _WIN32_WCE
#include <winbase.h>  // for FILETIME,etc
#endif // _WIN32_WCE

#include <winsock2.h>  // for struct timeval definition
#undef interface //ScenSim-Port://

const char PROTO_PATH_DELIMITER = '\\';

// Note: Windows already defines INT32 and UINT32 types
typedef signed char INT8;
typedef unsigned char UINT8;
typedef short INT16;
typedef unsigned short UINT16;

#else
//
// Linux/Unix definition
//
#include <sys/types.h> // must include this first to make SGI's compiler happy
#include <sys/time.h>  // for UNIX gettimeofday(), etc

#ifdef FREEBSD4
#include <wchar.h>
#else
#include <stdint.h>
//ScenSim-Port:// #include <stdint.h>    // for proper uint32_t, etc definitions
#endif // if/else freebsd4/other Unix

#include <limits.h>    // for PATH_MAX
const char PROTO_PATH_DELIMITER = '/';
#endif // if/else WIN32/UNIX


#ifdef SIMULATE

#ifdef SCENSIM_NRLOLSR //ScenSim-Port://
#include <assert.h> //ScenSim-Port://
inline void ProtoSystemTime(struct timeval& theTime) //ScenSim-Port://
{//ScenSim-Port://
    assert(false && "'Global' ProtoSystemTime is disabled for Scenargie.");
} //ScenSim-Port://
#endif // SCENSIM_NRLOLSR //ScenSim-Port://


#ifdef NS2
#include "scheduler.h"
inline void ProtoSystemTime(struct timeval& theTime)
{
    double now = Scheduler::instance().clock();
    theTime.tv_sec = (unsigned long)(now);
    theTime.tv_usec = (unsigned long)((now - theTime.tv_sec) * 1.0e06);
}
#endif // NS2

#ifdef OPNET
#include "opnet.h"
inline void ProtoSystemTime(struct timeval& theTime)
{
    double current_time = op_sim_time();
    theTime.tv_sec = (unsigned long)(current_time);
    theTime.tv_usec = (unsigned long)((current_time - theTime.tv_sec) * 1.0e06);
}
#endif // OPNET

#else  // !SIMULATE



#ifdef WIN32

// These funcs/variables are defined/inited in "protoTimer.cpp"
inline LARGE_INTEGER ProtoGetPerformanceCounterFrequency()
{
    extern bool proto_performance_counter_init;
    extern LARGE_INTEGER proto_performance_counter_frequency;
    extern long proto_system_time_last_sec;
    extern unsigned long proto_system_count_roll_sec;
    extern LARGE_INTEGER proto_system_count_last;
    if (!proto_performance_counter_init)
    {
        QueryPerformanceFrequency(&proto_performance_counter_frequency);
#ifdef _WIN32_WCE
        // Determine perf counter offset from system time
        extern long proto_performance_counter_offset;
        if (0 != proto_performance_counter_frequency.QuadPart)
        {
            SYSTEMTIME st;
            GetSystemTime(&st);
            LARGE_INTEGER count;
            QueryPerformanceCounter(&count);
            FILETIME ft;
            SystemTimeToFileTime(&st, &ft);
            ULARGE_INTEGER systemTime = {ft.dwLowDateTime, ft.dwHighDateTime};
            // Convert system time to Jan 1, 1970 epoch
            const ULARGE_INTEGER epochTime = {0xD53E8000, 0x019DB1DE};
            systemTime.QuadPart -= epochTime.QuadPart;
            // Convert sytem time to seconds
            systemTime.QuadPart /= 10000000;
            // Save time and counter to detect possible counter rollover
            proto_system_time_last_sec = systemTime.LowPart;
            proto_system_count_last = count;
            proto_system_count_roll_sec = 0xffffffff / proto_performance_counter_frequency.LowPart;
            // Convert counter value to seconds
            count.QuadPart /= proto_performance_counter_frequency.QuadPart;
            proto_performance_counter_offset = systemTime.LowPart - count.LowPart;
        }
#endif // _WIN32_WCE
        proto_performance_counter_init = true;
    }
    return proto_performance_counter_frequency;
}  // end ProtoGetPerformanceCounterFrequency()

inline void ProtoSystemTime(struct timeval& theTime)
{
#ifdef _WIN32_WCE
    // We use the perf counter on WinCE so we can get better than
    // the one second granularity offered by WinCE GetSystemTime()
    // (TBD) This breaks if the user changes the system time _while_
    // a Protolib app is running, so this needs a better fix in
    // the long run.  For example, we could update the
    // "proto_performance_counter_offset" once in a while or if
    // we detect a significant delta in the counter-based time
    // and the actual system time ...
    LARGE_INTEGER counterFreq = ProtoGetPerformanceCounterFrequency();
    if (0 != counterFreq.QuadPart)
    {
        extern long proto_performance_counter_offset;
        extern long proto_system_time_last_sec;
        extern unsigned long proto_system_count_roll_sec;
        extern LARGE_INTEGER proto_system_count_last;
        // 1) Get current system time and performance counter count
        SYSTEMTIME st;
        GetSystemTime(&st);
        LARGE_INTEGER count;
        QueryPerformanceCounter(&count);
        FILETIME ft;
        SystemTimeToFileTime(&st, &ft);
        ULARGE_INTEGER usec = {ft.dwLowDateTime, ft.dwHighDateTime};
        const ULARGE_INTEGER epochTime = {0xD53E8000, 0x019DB1DE};
        usec.QuadPart -= epochTime.QuadPart;
        ULARGE_INTEGER systemSecs;
        systemSecs.QuadPart =  (usec.QuadPart / 10000000);

        // Count rollover correction, if applicable
        if (count.HighPart < proto_system_count_last.HighPart)
            count.HighPart = proto_system_count_last.HighPart;
        // Rollover detection
        long timeDeltaSec = systemSecs.LowPart - proto_system_time_last_sec;
        bool rollover = false;
        if (count.QuadPart < proto_system_count_last.QuadPart)
        {
            rollover = true;
        }
        else
        {
            LARGE_INTEGER countDelta;
            countDelta.QuadPart = count.QuadPart - proto_system_count_last.QuadPart;
            countDelta.QuadPart /= counterFreq.QuadPart;  // delta in seconds
            long countDeltaSecs = countDelta.LowPart;
            if ((timeDeltaSec - countDeltaSecs) > (long)(proto_system_count_roll_sec >> 1))
                rollover = true;
        }
        if (rollover)
        {
            long rolloverCount = (timeDeltaSec / proto_system_count_roll_sec);
            if (0 != rolloverCount)
                count.HighPart += rolloverCount;
            else
                count.HighPart += 1;
        }
        proto_system_count_last = count;
        proto_system_time_last_sec = systemSecs.LowPart;

        // Compute whole seconds using (possibly corrected) counter value
        LARGE_INTEGER counterSec;
        counterSec.QuadPart = (count.QuadPart / counterFreq.QuadPart);
        theTime.tv_sec = counterSec.LowPart;
        // Adjust whole seconds by stored offset
        theTime.tv_sec += proto_performance_counter_offset;
        // Double-check our offset in case the user has changed system time
        // (Note counter drift may make this change, but only over _hopefully_
        //  a long interval time as "backwards" times might result
        long systemDelta = systemSecs.LowPart - theTime.tv_sec;
        if (labs(systemDelta) > 2)
        {
            theTime.tv_sec += systemDelta;
            proto_performance_counter_offset += systemDelta;
        }
        // Compute whole second count
        counterSec.QuadPart = counterSec.QuadPart * counterFreq.QuadPart;
        // Subtract to get fractional second count
        count.QuadPart = count.QuadPart - counterSec.QuadPart;
        // Scale to usec count
        count.QuadPart = 1000000 * count.QuadPart;
        // Compute usec
        LARGE_INTEGER counterUsec;
        counterUsec.QuadPart = count.QuadPart / counterFreq.QuadPart;
        theTime.tv_usec = counterUsec.LowPart;
    }
    else
#endif // _WIN32_WCE
    {
        FILETIME ft;
#ifdef _WIN32_WCE
        SYSTEMTIME st;
        GetSystemTime(&st);
        SystemTimeToFileTime(&st, &ft);
#else
        GetSystemTimeAsFileTime(&ft);
#endif // if/else _WIN32_WCE
        ULARGE_INTEGER usecTotal = {ft.dwLowDateTime, ft.dwHighDateTime};
        //epochTime = time for 00:00:0.00 1 Jan, 1970
        const ULARGE_INTEGER epochTime = {0xD53E8000, 0x019DB1DE};
        usecTotal.QuadPart -= epochTime.QuadPart;
        usecTotal.QuadPart /= 10;  // convert to microseconds
        theTime.tv_sec = (long)(usecTotal.QuadPart / 1000000);
        theTime.tv_usec = (long)(usecTotal.QuadPart - theTime.tv_sec * 1000000);
    }
}

#else

inline void ProtoSystemTime(struct timeval& theTime)
{
    struct timezone tz;
    gettimeofday(&theTime, &tz);
}



#endif  // if/else WIN32/UNIX

#endif // if/else SIMULATE



#ifndef PATH_MAX
#ifdef WIN32
#define PATH_MAX MAX_PATH
#else
#define PATH_MAX _POSIX_PATH_MAX
#endif // if/else WIN32/UNIX
#endif // !PATH_MAX


#ifndef NULL
#define NULL 0
#endif // !NULL


#ifndef WIN32
// Note: These are all already defined in WIN32 <basetsd.h>
typedef int8_t INT8;
typedef int16_t INT16;

#ifdef _USING_X11
typedef long int INT32;
#else
typedef int32_t INT32;
#endif // if/else _USING_X11

typedef uint8_t UINT8;
typedef uint16_t UINT16;
typedef uint32_t UINT32;
#endif  // !WIN32


#ifndef MAX
#define MAX(X,Y) ((X>Y)?(X):(Y))
#endif //!MAX
#ifndef MIN
#define MIN(X,Y) ((X<Y)?(X):(Y))
#endif //!MIN



#endif // _PROTO_DEFS
