// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef INSITEGLUE_CHOOSER_H
#define INSITEGLUE_CHOOSER_H

#if defined (FUPM) || defined (HFPM)
#include "insiteglue.h"
#elif defined (HFPM2)
#include "insiteglue2.h"
#else
#include "insiteglue_notavailable.h"
#endif

#endif
