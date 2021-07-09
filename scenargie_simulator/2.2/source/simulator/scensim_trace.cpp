// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scensim_user_trace_defs.h"

namespace ScenSim {

const TraceTag numberTraceTags = (numberStandardTraceTags + numberUserTraceTags);

void LookupTraceTagIndex(const string& tagString, TraceTag& theTraceTag, bool& succeeded)
{
    succeeded = false;
    for(TraceTag i = 0; (i < numberStandardTraceTags); i++) {
        if (IsEqualCaseInsensitive(tagString, standardTraceTagNames[i])) {
            succeeded = true;
            theTraceTag = i;
        }//if//
    }//for//

    for(TraceTag i = 0; (i < numberUserTraceTags); i++) {
        if (IsEqualCaseInsensitive(tagString, userTraceTagNames[i])) {
            succeeded = true;
            theTraceTag = FirstUserTraceTag + i;
        }//if//
    }//for//
}//LookupTraceTagIndex//



TraceTag LookupTraceTagIndex(const string& tagString)
{
    TraceTag traceTagIndex;
    bool succeeded;

    LookupTraceTagIndex(tagString, traceTagIndex, succeeded);

    if (!succeeded) {
        cerr << "Error: Unknown trace tag string (trace-enabled-tags): \"" << tagString << "\" not valid." << endl;
        exit(1);
    }//if//

    return traceTagIndex;

}//LookupTraceTagIndex//


const string& GetTraceTagName(const TraceTag& theTraceTag)
{
    assert(theTraceTag < numberTraceTags);
    if (theTraceTag < numberStandardTraceTags) {
        return (standardTraceTagNames[theTraceTag]);
    }
    else {
        return (userTraceTagNames[theTraceTag - FirstUserTraceTag]);
    }//if//

}//GetTraceTagName//


unsigned int GetNumberTraceTags() { return numberTraceTags; }

}//namespace


