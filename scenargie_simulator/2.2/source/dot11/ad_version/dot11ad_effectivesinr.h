// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11AD_EFFECTIVE_SINR_H
#define DOT11AD_EFFECTIVE_SINR_H

#include "scensim_support.h"
#include <array>
#include <vector>

namespace Dot11ad {

using std::array;
using std::vector;
using ScenSim::RoundToUint;

enum EffectiveSinrModulationChoices {
    EffectiveSinrModulationBpsk,
    EffectiveSinrModulationQpsk,
    EffectiveSinrModulation16Qam,
    EffectiveSinrModulation64Qam,
    EffectiveSinrModulation256Qam,
};


const int RbirEsmMappingTableValuesPerDb = 10;
const int RbirEsmMappingTableRangeFirstSinrDb = -20;

extern const array<double, 301> RbirEsmMappingTableForBpsk;
extern const array<double, 331> RbirEsmMappingTableForQpsk;
extern const array<double, 401> RbirEsmMappingTableFor16Qam;
extern const array<double, 471> RbirEsmMappingTableFor64Qam;
extern const array<double, 521> MiesmMmibMappingTableFor256Qam;


template<const size_t aSize> inline
double LookupRbirEsmValue(
    const array<double, aSize>& RbirEsmMappingTable,
    const double& sinrDb)
{
    if (sinrDb < RbirEsmMappingTableRangeFirstSinrDb) {
        return 0.0;
    }//if//

    const unsigned int tableIndex =
        RoundToUint(
            (sinrDb - RbirEsmMappingTableRangeFirstSinrDb) * RbirEsmMappingTableValuesPerDb);

    if (tableIndex >= RbirEsmMappingTable.size()) {
        return 1.0;
    }//if//

    return (RbirEsmMappingTable[tableIndex]);

}//LookupRbirEsmValue//


// Note for LookupRbirEsmValue: There is a Difference between 801.11ax Evaluation Methodology and
// the "historic" 802.16m Evaluation Methodology;  The former's RBIR tables are per bit and the
// latter are per symbol.  Using per symbol.

inline
double LookupRbirEsmValue(const EffectiveSinrModulationChoices& modulation, const double& sinrDb)
{
    switch (modulation) {
    case EffectiveSinrModulationBpsk:
        return (LookupRbirEsmValue(RbirEsmMappingTableForBpsk, sinrDb));
        break;

    case EffectiveSinrModulationQpsk:
        return (LookupRbirEsmValue(RbirEsmMappingTableForQpsk, sinrDb));
        break;

    case EffectiveSinrModulation16Qam:
        return (LookupRbirEsmValue(RbirEsmMappingTableFor16Qam, sinrDb));
        break;

    case EffectiveSinrModulation64Qam:
        return (LookupRbirEsmValue(RbirEsmMappingTableFor64Qam, sinrDb));
        break;

    case EffectiveSinrModulation256Qam:
        return (LookupRbirEsmValue(MiesmMmibMappingTableFor256Qam, sinrDb));
        break;

    default:
        assert(false && "Unknown Modulation Type"); abort(); return 0.0;
        break;
    }//switch//

}//LookupRbirEsmValue//




template<const size_t aSize> inline
double ReverseLookupSinrDbFromRbirEsmValue(
    const array<double, aSize>& RbirEsmMappingTable,
    const double& rbirValue)
{
    typedef typename array<double, aSize>::const_iterator IterType;

    assert((rbirValue >= 0.0) && (rbirValue <= 1.0));

    if (rbirValue <= RbirEsmMappingTable[0]) {
        return (RbirEsmMappingTableRangeFirstSinrDb);
    }//if//

    const IterType iter =
        std::lower_bound(
            RbirEsmMappingTable.begin(),
            RbirEsmMappingTable.end(),
            rbirValue);

    assert(iter != RbirEsmMappingTable.begin());

    const unsigned int index = static_cast<const unsigned int>(iter - RbirEsmMappingTable.begin());

    const double value1 =
        (RbirEsmMappingTableRangeFirstSinrDb +
         (static_cast<double>(index-1) / RbirEsmMappingTableValuesPerDb));

    const double value2 =
        (RbirEsmMappingTableRangeFirstSinrDb +
         (static_cast<double>(index) / RbirEsmMappingTableValuesPerDb));

    return
        (ScenSim::CalcInterpolatedValue(
            RbirEsmMappingTable[index-1],
            value1,
            RbirEsmMappingTable[index],
            value2,
            rbirValue));

}//ReverseLookupSinrDbFromRbirEsmTable//


inline
double ReverseLookupSinrDbFromRbirEsmValue(
    const EffectiveSinrModulationChoices& modulation,
    const double& rbirValue)
{
    switch (modulation) {
    case EffectiveSinrModulationBpsk:
        return (ReverseLookupSinrDbFromRbirEsmValue(RbirEsmMappingTableForBpsk, rbirValue));
        break;

    case EffectiveSinrModulationQpsk:
        return (ReverseLookupSinrDbFromRbirEsmValue(RbirEsmMappingTableForQpsk, rbirValue));
        break;

    case EffectiveSinrModulation16Qam:
        return (ReverseLookupSinrDbFromRbirEsmValue(RbirEsmMappingTableFor16Qam, rbirValue));
        break;

    case EffectiveSinrModulation64Qam:
        return (ReverseLookupSinrDbFromRbirEsmValue(RbirEsmMappingTableFor64Qam, rbirValue));
        break;

    case EffectiveSinrModulation256Qam:
        return (ReverseLookupSinrDbFromRbirEsmValue(MiesmMmibMappingTableFor256Qam, rbirValue));
        break;

    default:
        assert(false && "Unknown Modulation Type"); abort(); return 0.0;
        break;

    }//switch//

}//ReverseLookupSinrDbFromRbirEsmValue//


inline
double CalculateEffectiveSinrDb(
    const EffectiveSinrModulationChoices& modulation,
    const vector<double>& subpartSinrsDb)
{
    double total = 0.0;

    for(unsigned int i = 0; (i < subpartSinrsDb.size()); i++) {

        total += LookupRbirEsmValue(modulation, subpartSinrsDb[i]);

    }//for//

    assert(!subpartSinrsDb.empty());

    return (ReverseLookupSinrDbFromRbirEsmValue(modulation, (total / subpartSinrsDb.size())));

}//CalculateEffectiveSinr//


}//namespace//

#endif
