// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_TIME_H
#define SCENSIM_TIME_H

#include <assert.h>
#include <limits.h>
#include <float.h>
#include <string>
#include <sstream>

namespace ScenSim {

using std::string;
using std::ostringstream;
using std::ios_base;

//=============================================================================
//
// Simulation Clock Declarations
//
//=============================================================================


typedef long long int SimTime;   // Note: "Time" is too generic a name.
typedef SimTime TimeType;  // Depreciated Style


const SimTime NANO_SECOND = 1;
const SimTime MICRO_SECOND = 1000 * NANO_SECOND;
const SimTime MILLI_SECOND = 1000 * MICRO_SECOND;
const SimTime SECOND = 1000 * MILLI_SECOND;


const SimTime ZERO_TIME = 0;
const SimTime INFINITE_TIME = LLONG_MAX;
const SimTime EPSILON_TIME = 1;

SimTime ConvertStringToTime(const string& aString);

void ConvertStringToTime(const string& aString, SimTime& timeValue, bool& success);

inline
void OutputTimeInSecsToStream(const SimTime& time, std::ostream& outStream)
{
    const unsigned int fractionalDigits = 9;

    assert(time >= ZERO_TIME);

    SimTime seconds = time / SECOND;
    unsigned int nanoseconds = static_cast<unsigned int>(time % SECOND);

    outStream << seconds << '.';

    // Pad with zeroes.

    const unsigned int divisors[fractionalDigits-1] =
        {100000000, 10000000, 1000000,  100000, 10000, 1000, 100, 10};

    for(size_t i = 0; (i < (fractionalDigits-1)); i++) {
        if (nanoseconds >= divisors[i]) {
            break;
        }//if//
        outStream << '0';
    }//for//

    outStream << nanoseconds;

}//OutputTimeInSecsToStream//



inline
string ConvertTimeToStringSecs(const SimTime& time)
{
    ostringstream outStream;
    OutputTimeInSecsToStream(time, outStream);
    return outStream.str();

}//ConvertTimeToString//


inline
double ConvertTimeToDoubleSecs(const SimTime& time) { return (static_cast<double>(time) / SECOND); }

const double doubleInfiniteTimeSecsCutoff =
    (static_cast<double>(INFINITE_TIME)/SECOND) - (DBL_EPSILON * (static_cast<double>(INFINITE_TIME)/SECOND));

inline
SimTime ConvertDoubleSecsToTime(const double& timeSecs)
{
    if (timeSecs > doubleInfiniteTimeSecsCutoff) {
        return INFINITE_TIME;
    }
    else {
        return static_cast<SimTime>((timeSecs * SECOND) + 0.5);
    }
}


inline
SimTime DivideAndRoundUp(const SimTime& x, const SimTime& y)
{
    return ((x + y - 1) / y);
}

//=============================================================================


}//namespace//

#endif
