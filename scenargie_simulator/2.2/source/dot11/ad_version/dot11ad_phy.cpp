// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "dot11ad_phy.h"

namespace Dot11ad {

const string Dot11Phy::modelName = "Dot11adPhy";


inline
const InterferenceFactorTableType MakeLowBandwidthChannelOnHighInterferenceFactorTable()
{
    InterferenceFactorTableType table;

    for(size_t i = 0; (i < table.size()); i++) {
        for(size_t j = 0; (j < table[i].size()); j++) {
            for(size_t k = 0; (k < table[i][j].size()); k++) {
                table[i][j][k] = InvalidInterferenceFactorTableValue;
            }//for//
        }//for//
    }//for//

    // Only set legal combinations

    for(unsigned int i = 0; (i < 2); i++) {
        table[0][0][i] =
            CalcSubcarrierInterferenceFactorForNarrowChannelOnWide(
                subcarrierIsUsedOnOneChannelBandwidth,
                subcarrierIsDataOnTwoChannelBandwidth,
                i);
    }//for//


    for(unsigned int i = 0; (i < 4); i++) {
        table[0][2][i] =
            CalcSubcarrierInterferenceFactorForNarrowChannelOnWide(
                subcarrierIsUsedOnOneChannelBandwidth,
                subcarrierIsDataOnFourChannelBandwidth,
                i);
    }//for//

    table[1][2][0] =
        CalcSubcarrierInterferenceFactorForNarrowChannelOnWide(
            subcarrierIsUsedOnTwoChannelBandwidth,
            subcarrierIsDataOnFourChannelBandwidth,
            0);

    table[1][2][2] =
        CalcSubcarrierInterferenceFactorForNarrowChannelOnWide(
            subcarrierIsUsedOnTwoChannelBandwidth,
            subcarrierIsDataOnFourChannelBandwidth,
            2);

    return (table);

}//MakeLowBandwidthChannelOnHighInterferenceFactorTable//


inline
const NonHtInterferenceFactorTableType MakeNonHtChannelOnHighInterferenceFactorTable()
{
    NonHtInterferenceFactorTableType table;

    for(size_t i = 0; (i < table.size()); i++) {
        for(size_t j = 0; (j < table[i].size()); j++) {
                table[i][j] = InvalidInterferenceFactorTableValue;
        }//for//
    }//for//

    // Only set legal combinations

    assert(table.size() >= 3);
    assert(table[0].size() >= 2);

    for(unsigned int i = 0; (i < 2); i++) {
        table[0][i] =
            CalcSubcarrierInterferenceFactorForNarrowChannelOnWide(
                subcarrierIsUsedOnNonHtChannelBandwidth,
                subcarrierIsDataOnTwoChannelBandwidth,
                i);
    }//for//


    assert(table[2].size() >= 4);

    for(unsigned int i = 0; (i < 4); i++) {
        table[2][i] =
            CalcSubcarrierInterferenceFactorForNarrowChannelOnWide(
                subcarrierIsUsedOnNonHtChannelBandwidth,
                subcarrierIsDataOnFourChannelBandwidth,
                i);
    }//for//

    return (table);

}//MakeNonHtChannelOnHighInterferenceFactorTable//



inline
const InterferenceFactorTableType MakeHighBandwidthChannelOnLowInterferenceFactorTable()
{
    InterferenceFactorTableType table;

    for(size_t i = 0; (i < table.size()); i++) {
        for(size_t j = 0; (j < table[i].size()); j++) {
            for(size_t k = 0; (k < table[i][j].size()); k++) {
                table[i][j][k] = InvalidInterferenceFactorTableValue;
            }//for//
        }//for//
    }//for//

    // Only set legal combinations

    for(unsigned int i = 0; (i < 2); i++) {
        table[0][0][i] =
            CalcSubcarrierInterferenceFactorForWideChannelOnNarrow(
                subcarrierIsUsedOnTwoChannelBandwidth,
                i,
                subcarrierIsDataOnOneChannelBandwidth);
    }//for//


    for(unsigned int i = 0; (i < 4); i++) {
        table[0][2][i] =
                CalcSubcarrierInterferenceFactorForWideChannelOnNarrow(
                subcarrierIsUsedOnFourChannelBandwidth,
                i,
                subcarrierIsDataOnOneChannelBandwidth);
    }//for//

    table[1][2][0] =
            CalcSubcarrierInterferenceFactorForWideChannelOnNarrow(
            subcarrierIsUsedOnFourChannelBandwidth,
            0,
            subcarrierIsDataOnTwoChannelBandwidth);

    table[1][2][2] =
            CalcSubcarrierInterferenceFactorForWideChannelOnNarrow(
            subcarrierIsUsedOnFourChannelBandwidth,
            2,
            subcarrierIsDataOnTwoChannelBandwidth);

    return (table);

}//MakeHighBandwidthChannelOnLowInterferenceFactorTable//


inline
const NonHtInterferenceFactorTableType MakeHighBandwidthChannelOnNonHtInterferenceFactorTable()
{
    NonHtInterferenceFactorTableType table;

    for(size_t i = 0; (i < table.size()); i++) {
        for(size_t j = 0; (j < table[i].size()); j++) {
                table[i][j] = InvalidInterferenceFactorTableValue;
        }//for//
    }//for//

    // Only set legal combinations

    assert(table.size() >= 3);
    assert(table[0].size() >= 2);

    for(unsigned int i = 0; (i < 2); i++) {
        table[0][i] =
            CalcSubcarrierInterferenceFactorForWideChannelOnNarrow(
                subcarrierIsUsedOnTwoChannelBandwidth,
                i,
                subcarrierIsDataOnNonHtChannelBandwidth);
    }//for//

    assert(table[2].size() >= 4);

    for(unsigned int i = 0; (i < 4); i++) {
        table[2][i] =
                CalcSubcarrierInterferenceFactorForWideChannelOnNarrow(
                subcarrierIsUsedOnFourChannelBandwidth,
                i,
                subcarrierIsDataOnNonHtChannelBandwidth);
    }//for//

    return (table);

}//MakeHighBandwidthChannelOnNonHtInterferenceFactorTable//



//-------------------------------------------------------------------------------------------------

const InterferenceFactorTableType
    LowBandwidthChannelOnHighInterferenceFactors =
        MakeLowBandwidthChannelOnHighInterferenceFactorTable();

const InterferenceFactorTableType
    HighBandwidthChannelOnLowInterferenceFactors =
        MakeHighBandwidthChannelOnLowInterferenceFactorTable();

const NonHtInterferenceFactorTableType
    LowBandwidthNonHtChannelOnHighInterferenceFactors = MakeNonHtChannelOnHighInterferenceFactorTable();

const NonHtInterferenceFactorTableType
    HighBandwidthChannelOnLowNonHtInterferenceFactors = MakeHighBandwidthChannelOnNonHtInterferenceFactorTable();

}//namespace
