// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

//--------------------------------------------------------------------------------------------------
// "NotUsed" data items in header structs are placeholders for standard
// 802.11 fields that are not currently used in the model.  The purpose of
// not including the unused standard field names is to make plain the
// features that are and are NOT implemented.  The "Not Used" fields should always be
// zeroed so that packets do not include random garbage.  Likewise, only
// frame types and codes that are used in model logic will be defined.
//
// This code ignores machine endian issues because it is a model, i.e. fields are the
// correct sizes but the actual bits will not be in "network order" on small endian machines.
//

#ifndef DOT11AD_PHY_H
#define DOT11AD_PHY_H

#include <sstream>
#include <list>
#include <numeric>
#include <complex>
#include <array>
#include <limits>

// Disable some accuracy checks that cause exceptions in UBLAS during Matrix Inversion.

#undef BOOST_UBLAS_TYPE_CHECK
#define BOOST_UBLAS_TYPE_CHECK 0
#include "boost/numeric/ublas/matrix.hpp"
#include "boost/numeric/ublas/lu.hpp"
#include "boost/numeric/ublas/io.hpp"

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "scensim_prop.h"
#include "scensim_bercurves.h"
#include "scensim_mimo_channel.h"
#include "scensim_freqsel_channel.h"

#include "dot11ad_common.h"
#include "dot11ad_tracedefs.h"
#include "dot11ad_info_interface.h"
#include "dot11ad_effectivesinr.h"

namespace Dot11ad {

using std::ostringstream;
using std::endl;
using std::cerr;
using std::cout;
using std::setprecision;
using std::list;
using std::accumulate;
using std::shared_ptr;
using std::enable_shared_from_this;
using std::unique_ptr;
using std::move;
using std::array;
using std::map;
using std::deque;
using std::complex;
using namespace boost::numeric;

using ScenSim::SimTime;
using ScenSim::INFINITE_TIME;
using ScenSim::EPSILON_TIME;
using ScenSim::ZERO_TIME;
using ScenSim::SECOND;
using ScenSim::MICRO_SECOND;
using ScenSim::NANO_SECOND;
using ScenSim::ConvertTimeToStringSecs;
using ScenSim::SimulationEngineInterface;
using ScenSim::SimulationEvent;
using ScenSim::EventRescheduleTicket;
using ScenSim::SimplePropagationModelForNode;
using ScenSim::CustomAntennaModel;
using ScenSim::AntennaPatternDatabase;
using ScenSim::AntennaPattern;
using ScenSim::MimoChannelModel;
using ScenSim::MimoChannelModelInterface;
using ScenSim::FrequencySelectiveFadingModel;
using ScenSim::FrequencySelectiveFadingModelInterface;
using ScenSim::InterfaceOrInstanceId;
using ScenSim::BitOrBlockErrorRateCurveDatabase;
using ScenSim::BitErrorRateCurve;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeId;
using ScenSim::InvalidNodeId;
using ScenSim::InterfaceId;
using ScenSim::Packet;
using ScenSim::PacketId;
using ScenSim::IntegralPower;
using ScenSim::CounterStatistic;
using ScenSim::RealStatistic;
using ScenSim::RandomNumberGenerator;
using ScenSim::RandomNumberGeneratorSeed;
using ScenSim::HashInputsToMakeSeed;
using ScenSim::CalculateThermalNoisePowerWatts;
using ScenSim::ConvertToNonDb;
using ScenSim::ConvertToDb;
using ScenSim::ConvertIntToDb;
using ScenSim::TracePhy;
using ScenSim::TracePhyInterference;
using ScenSim::ConvertDoubleSecsToTime;
using ScenSim::DivideAndRoundUp;
using ScenSim::ConvertStringToLowerCase;
using ScenSim::ConvertStringToUpperCase;
using ScenSim::StringIsAllUpperCase;
using ScenSim::MakeUpperCaseString;
using ScenSim::MakeLowerCaseString;
using ScenSim::RoundToUint;
using ScenSim::ConvertTimeToDoubleSecs;
using ScenSim::ConvertAStringSequenceOfANumericTypeIntoAVector;
using ScenSim::DeleteTrailingSpaces;
using ScenSim::MimoChannelMatrix;
using ScenSim::ThreeDbFactor;
using ScenSim::SixDbFactor;
using ScenSim::PI;
using ScenSim::CalcComplexExp;
using ScenSim::ConvertStringToNonNegativeInt;
using ScenSim::ConvertToUChar;
using ScenSim::InvalidChannelNumber;

class Dot11Mac;
class Dot11Phy;

//--------------------------------------------------------------------------------------------------

enum ModulationAndCodingSchemesType {
    AdMcs0,
    AdMcs1,
    AdMcs2,
    AdMcs3,
    AdMcs4,
    AdMcs5,
    AdMcs6,
    AdMcs7,
    AdMcs8,
    AdMcs9,
    AdMcs10,
    AdMcs11,
    AdMcs12,
    AdMcs13,
    AdMcs14,
    AdMcs15,
    AdMcs16,
    AdMcs17,
    AdMcs18,
    AdMcs19,
    AdMcs20,
    AdMcs21,
    AdMcs22,
    AdMcs23,
    AdMcs24,
    McsInvalid,
};

const unsigned int MaxNumMimoSpatialStreams = 4;

struct ModAndCodingSchemeWithNumSpatialStreams {
    ModulationAndCodingSchemesType modAndCoding;
    unsigned char numberSpatialStreams;

    ModAndCodingSchemeWithNumSpatialStreams() : modAndCoding(McsInvalid), numberSpatialStreams(0) { }
};

const unsigned int NumberModAndCodingSchemes = 25;

const ModulationAndCodingSchemesType minModulationAndCodingScheme = AdMcs0;
const ModulationAndCodingSchemesType maxModulationAndCodingScheme = AdMcs24;


inline
string GetModulationAndCodingName(const ModulationAndCodingSchemesType modulationAndCoding)
{
    static const array<string, NumberModAndCodingSchemes> mcsName =
        {"DOT11AD-MCS0", "DOT11AD-MCS1", "DOT11AD-MCS2", "DOT11AD-MCS3", "DOT11AD-MCS4",
        "DOT11AD-MCS5", "DOT11AD-MCS6", "DOT11AD-MCS7", "DOT11AD-MCS8", "DOT11AD-MCS9",
        "DOT11AD-MCS10", "DOT11AD-MCS11", "DOT11AD-MCS12", "DOT11AD-MCS13", "DOT11AD-MCS14",
        "DOT11AD-MCS15", "DOT11AD-MCS16", "DOT11AD-MCS17", "DOT11AD-MCS18", "DOT11AD-MCS19",
        "DOT11AD-MCS20", "DOT11AD-MCS21", "DOT11AD-MCS22", "DOT11AD-MCS23", "DOT11AD-MCS24"};

    return (mcsName.at(static_cast<size_t>(modulationAndCoding)));
}


inline
void IncrementModAndCodingScheme(ModulationAndCodingSchemesType& mcs)
{
    assert((minModulationAndCodingScheme <= mcs) && (mcs <= maxModulationAndCodingScheme));
    mcs = static_cast<ModulationAndCodingSchemesType>(static_cast<unsigned int>(mcs) + 1);
}


inline
void DecrementModAndCodingScheme(ModulationAndCodingSchemesType& mcs)
{
    assert((minModulationAndCodingScheme < mcs) && (mcs <= maxModulationAndCodingScheme));
    mcs = static_cast<ModulationAndCodingSchemesType>(static_cast<unsigned int>(mcs) - 1);
}



inline
void GetModulationAndCodingSchemeFromName(
    const string& modAndCodingNameString,
    bool& wasFound,
    ModulationAndCodingSchemesType& modulationAndCoding)
{
    assert(StringIsAllUpperCase(modAndCodingNameString));

    wasFound = false;
    for(ModulationAndCodingSchemesType mcs = minModulationAndCodingScheme;
        (mcs <= maxModulationAndCodingScheme);
        IncrementModAndCodingScheme(mcs)) {

        if (GetModulationAndCodingName(mcs) == modAndCodingNameString) {
            wasFound = true;
            modulationAndCoding = mcs;
            break;
        }//if//
    }//for//

}//GetModulationAndCodingSchemeFromName//



inline
void GetModulationAndCodingSchemeAndNumStreamsFromName(
    const string& modAndCodingWithSpatialStreamsName,
    bool& wasFound,
    ModAndCodingSchemeWithNumSpatialStreams& modulationAndCodingWithNumStreams)
{
    assert(StringIsAllUpperCase(modAndCodingWithSpatialStreamsName));

    wasFound = false;

    const size_t spatialStreamsSeparatorPos = modAndCodingWithSpatialStreamsName.find_last_of('X');

    if (spatialStreamsSeparatorPos == string::npos) {
        // No num spatial streams part, assume 1.

        GetModulationAndCodingSchemeFromName(
            MakeUpperCaseString(modAndCodingWithSpatialStreamsName),
            wasFound,
            modulationAndCodingWithNumStreams.modAndCoding);

        modulationAndCodingWithNumStreams.numberSpatialStreams = 1;
    }
    else {
        if ((spatialStreamsSeparatorPos == 0) ||
            (modAndCodingWithSpatialStreamsName[spatialStreamsSeparatorPos-1] != '_')) {

            // Bad format.
            return;
        }//if//

        if ((spatialStreamsSeparatorPos + 2) != modAndCodingWithSpatialStreamsName.size()) {
            // Assuming Spatial Streams is one digit.
            return;
        }//if//


        GetModulationAndCodingSchemeFromName(
            MakeUpperCaseString(modAndCodingWithSpatialStreamsName.substr(0, (spatialStreamsSeparatorPos-1))),
            wasFound,
            modulationAndCodingWithNumStreams.modAndCoding);

        if (!wasFound) {
            return;
        }//if//

        unsigned int numberSpatialStreams = 0;

        ConvertStringToNonNegativeInt(
            modAndCodingWithSpatialStreamsName.substr(spatialStreamsSeparatorPos+1, 1),
            numberSpatialStreams,
            wasFound);

        if (!wasFound) {
            return;
        }//if//

        if ((numberSpatialStreams == 0) ||
            (numberSpatialStreams > MaxNumMimoSpatialStreams)) {

            wasFound = false;
            return;
        }//if//

        modulationAndCodingWithNumStreams.numberSpatialStreams = ConvertToUChar(numberSpatialStreams);
    }//if//

}//GetModulationAndCodingSchemeAndNumStreamsFromName//


inline
ModulationAndCodingSchemesType ConvertNameToModulationAndCodingScheme(
    const string& modAndCodingNameString,
    const string& parameterNameForErrorOutput)
{
    bool wasFound;
    ModulationAndCodingSchemesType modulationAndCoding;

    GetModulationAndCodingSchemeFromName(
        MakeUpperCaseString(modAndCodingNameString), wasFound, modulationAndCoding);

    if (!wasFound) {
        cerr << "Error in " << parameterNameForErrorOutput << " parameter: "
             << "Modulation and coding name \"" << modAndCodingNameString << "\" not valid." << endl;
        exit(1);
    }//if//

    return (modulationAndCoding);

}//ConvertNameToModulationAndCodingScheme//




inline
ModAndCodingSchemeWithNumSpatialStreams ConvertNameToModAndCodingSchemeWithNumStreams(
    const string& modAndCodingWithNumStreamsName,
    const string& parameterNameForErrorOutput)
{
    ModAndCodingSchemeWithNumSpatialStreams modAndCodingWithNumStreams;
    bool wasFound;

    GetModulationAndCodingSchemeAndNumStreamsFromName(
        MakeUpperCaseString(modAndCodingWithNumStreamsName),
        wasFound,
        modAndCodingWithNumStreams);

    if (!wasFound) {
        cerr << "Error in " << parameterNameForErrorOutput << " parameter: "
             << "Modulation and coding name \"" << modAndCodingWithNumStreamsName << "\" not valid." << endl;
        exit(1);
    }//if//

    return (modAndCodingWithNumStreams);

}//ConvertNameToModAndCodingSchemeWithNumStreams//



inline
EffectiveSinrModulationChoices GetModulation(
    const ModulationAndCodingSchemesType& modAndCoding)
{
    assert(false && "11ad");

    return (EffectiveSinrModulationBpsk);

    //11ad// switch (modAndCoding) {
    //11ad// case McsBpsk1Over2:
    //11ad// case McsBpsk3Over4:
    //11ad//     return (EffectiveSinrModulationBpsk);
    //11ad//     break;
    //11ad// case McsQpsk1Over2:
    //11ad// case McsQpsk3Over4:
    //11ad//     return (EffectiveSinrModulationQpsk);
    //11ad//     break;
    //11ad// case Mcs16Qam1Over2:
    //11ad// case Mcs16Qam3Over4:
    //11ad//     return (EffectiveSinrModulation16Qam);
    //11ad//     break;
    //11ad// case Mcs64Qam2Over3:
    //11ad// case Mcs64Qam3Over4:
    //11ad// case Mcs64Qam5Over6:
    //11ad//     return (EffectiveSinrModulation64Qam);
    //11ad//     break;
    //11ad// case Mcs256Qam3Over4:
    //11ad// case Mcs256Qam5Over6:
    //11ad//     return (EffectiveSinrModulation256Qam);
    //11ad//     break;
    //11ad// default:
    //11ad//     assert(false); abort(); return (EffectiveSinrModulationBpsk); break;
    //11ad// }//switch//

}//ExtractEffectiveSinrModulation//


inline
unsigned int GetBitsPerSymbolForModulation(const ModulationAndCodingSchemesType modulationAndCoding)
{
    assert(false); abort(); return 0;

    //11ad// static const array<unsigned int, NumberModAndCodingSchemes> bitsPerSymbol =
    //11ad//     {{1, 1, 2, 2, 4, 4, 6, 6, 6, 8, 8}};
    //11ad// return (bitsPerSymbol.at(static_cast<size_t>(modulationAndCoding)));
}

inline
double GetCodingRate(const ModulationAndCodingSchemesType modulationAndCoding)
{
    assert(false); abort(); return 0;

    //11ad// static const array<double, NumberModAndCodingSchemes> codingRates =
    //11ad// { { 0.5, 0.75, 0.5, 0.75, 0.5, 0.75, (2.0 / 3.0), 0.75, (5.0 / 6.0), 0.75, (5.0 / 6.0)} };

    //11ad// return (codingRates.at(static_cast<size_t>(modulationAndCoding)));
}

inline
double GetFractionalBitsPerSymbol(const ModulationAndCodingSchemesType modulationAndCoding)
{
    return (GetBitsPerSymbolForModulation(modulationAndCoding) * GetCodingRate(modulationAndCoding));
}


//------------------------------------------------------------------------------

const unsigned int numOfdmSubcarriers = 64;
const unsigned int numUsedOfdmSubcarriers = 52;
const unsigned int numOfdmDataSubcarriers = 48;

// const unsigned int numOfdmPilotSubcarriers = (numUsedOfdmSubcarriers - numOfdmDataSubcarriers);

const unsigned int numHighThroughputOfdmDataSubcarriers = 52;
const unsigned int numUsedHighThroughputOfdmSubcarriers = 56;

const unsigned int numOfdmSubcarriersAtTwoChannelBandwidth = 128;
const unsigned int numUsedOfdmSubcarriersAtTwoChannelBandwidth = 114;
const unsigned int numOfdmDataSubcarriersAtTwoChannelBandwidth = 108;

const unsigned int numOfdmSubcarriersAtFourChannelBandwidth = 256;
const unsigned int numUsedOfdmSubcarriersAtFourChannelBandwidth = 242;
const unsigned int numOfdmDataSubcarriersAtFourChannelBandwidth = 234;

// Note: 160Mhz BW (8 ch) is just two 80Mhz channels (4 ch) (no special 160Mhz BW channel mapping).

const double shortGuardIntervalOfdmSymbolDurationShrinkFactor = 0.9;

const double ofdm20MhzHtOn20MhzNhtSubcarrierInterferenceFactor = (48.0 / 56);

// MakeReversedString is for reversing std::bitset initialization strings to
// make std::bitset in "array order".

inline
string MakeReversedString(const string& aString)
{
    string reversedString = aString;
    std::reverse(reversedString.begin(), reversedString.end());
    return (reversedString);
}


template<typename T, size_t N, size_t M> inline
const array<T, N> MakeIndexTable(const std::bitset<M>& set)
{
    array<T, N> indexTable;
    // To stop warning.
    indexTable.fill(0);
    T index = 0;
    for(unsigned int i = 0; (i < N); i++) {
        while (!set[index]) {
            index++;
            assert(index < set.size());
        }//while//

        indexTable[i] = index;
        index++;
    }//for//

    return (indexTable);

}//MakeIndexTable//


// Unmapped values in inverse are mapped to an "out of range" value.

template<size_t N, typename T, size_t M> inline
const array<T, N> MakeInverseIndexTable(const array<T, M>& indexTable)
{
    assert(!std::numeric_limits<T>::is_signed);
    assert(std::numeric_limits<T>::max() >= N);
    array<T, N> reverseIndexTable;
    // Bad value.
    reverseIndexTable.fill(static_cast<T>(M));
    for(T i = 0; (i < M); i++) {
        reverseIndexTable.at(indexTable[i]) = i;
    }//for//

    return (reverseIndexTable);

}//MakeInverseIndexTable//


//
// Example: 20 Mhz (1 Ch) and 40 Mhz tables (2 ch)
//
// Legacy "Non-High throughput" (NHT) 20Mhz BW carrier positions:
//     -3                                                              3
//      2                               0                              1
//           0    1    1    2    2    3    3    4    4    5    5    6
//      0****5****0****5****0****5****0****5****0****5****0****5****0***
// NHT  NNNNNNDDDDDPDDDDDDDDDDDDDPDDDDDDNDDDDDDPDDDDDDDDDDDDDPDDDDDNNNNN
//  HT  NNNNDDDDDDDPDDDDDDDDDDDDDPDDDDDDNDDDDDDPDDDDDDDDDDDDDPDDDDDDDNNN
//
//      20HT-20NHT = 56 TX with 48 data receivers.
//      20NHT-20HT = 52 TX with 48 data receivers.
//
// High Throughput 20Mhz (twice for comparison) and 40mhz:
//
//     -3                                                              3
//      2                               0                              1
//           0    1    1    2    2    3    3    4    4    5    5    6
//      0****5****0****5****0****5****0****5****0****5****0****5****0***
// 40:0 NNDDDDDDDDDPDDDDDDDDDDDDDPDDDDDDDDDDDDDDDDDDDDDDDDDDDPDDDDDNNNNN
// 20:  NNNNDDDDDDDPDDDDDDDDDDDDDPDDDDDDNDDDDDDPDDDDDDDDDDDDDPDDDDDDDNNN
//        --+++++++-+++++++++++++-++++++-++++++-+++++++++++++-+++++
// 40:0->20: 57 Tx subcarriers (1/2 40Mhz BW) and 50 data receivers (20 Mhz HT BW)
//          +++++++-+++++++++++++-++++++ ++++++++++++++++++++-+++++--
// 20->40:0 56 Tx subcarriers and 51 data receivers.
//
//     -3                                                             +3
//      2                               0                              1
//         -6   -5   -5   -4   -4   -3   -3   -2   -2   -1   -1   -0    0
//      ****0****5****0****5****0****5****0****5****0****5****0****5****0
// 40:1 NNNNNNDDDDDPDDDDDDDDDDDDDDDDDDDDDDDDDDDPDDDDDDDDDDDDDPDDDDDDDDDN
// 20:  NNNNDDDDDDDPDDDDDDDDDDDDDPDDDDDDNDDDDDDPDDDDDDDDDDDDDPDDDDDDDNNN


template<size_t N, size_t W> inline
double CalcSubcarrierInterferenceFactorForNarrowChannelOnWide(
    const std::bitset<N>& narrowChannelUsedSubcarriers,
    const std::bitset<W>& wideChannelDataSubcarriers,
    const unsigned int wideChannelSubchannelNumber)
{
    const unsigned int wideChannelSubcarrierOffset = (wideChannelSubchannelNumber * numOfdmSubcarriers);

    assert((wideChannelSubcarrierOffset + narrowChannelUsedSubcarriers.size() - 1) <
           wideChannelDataSubcarriers.size());

    unsigned int numberInterferringSubcarriers = 0;

    for(unsigned int i = 0; (i < narrowChannelUsedSubcarriers.size()); i++) {
        if ((narrowChannelUsedSubcarriers[i]) &&
            (wideChannelDataSubcarriers[wideChannelSubcarrierOffset + i])) {

            numberInterferringSubcarriers++;

        }//if//
    }//for//

    return (static_cast<double>(numberInterferringSubcarriers) / narrowChannelUsedSubcarriers.count());

}//CalcSubcarrierInterferenceFactorNarrowChannelOnWide//


template<size_t N, size_t W> inline
double CalcSubcarrierInterferenceFactorForWideChannelOnNarrow(
    const std::bitset<W>& wideChannelUsedSubcarriers,
    const unsigned int wideChannelSubchannelNumber,
    const std::bitset<N>& narrowChannelDataSubcarriers)
{
    const unsigned int wideChannelSubcarrierOffset = (wideChannelSubchannelNumber * numOfdmSubcarriers);

    assert((wideChannelSubcarrierOffset + narrowChannelDataSubcarriers.size() - 1) <
           wideChannelUsedSubcarriers.size());

    unsigned int numberTransmittingSubcarriers = 0;
    unsigned int numberInterferringSubcarriers = 0;

    for(unsigned int i = 0; (i < narrowChannelDataSubcarriers.size()); i++) {
        if (wideChannelUsedSubcarriers[wideChannelSubcarrierOffset + i]) {
            numberTransmittingSubcarriers++;
            if (narrowChannelDataSubcarriers[i]) {
                numberInterferringSubcarriers++;
            }//if//
        }//if//
    }//for//

    return (static_cast<double>(numberInterferringSubcarriers) / numberTransmittingSubcarriers);

}//CalcSubchanelInterferenceFactorWideChannelOnNarrow//




// Strings are reversed so std::bitset is in "array order".

const std::bitset<numOfdmSubcarriers> subcarrierIsDataOnNonHtChannelBandwidth(
    MakeReversedString("0000001111101111111111111011111101111110111111111111101111100000"));

const std::bitset<numOfdmSubcarriers> subcarrierIsUsedOnNonHtChannelBandwidth(
    MakeReversedString("0000001111111111111111111111111101111111111111111111111111100000"));

const array<unsigned char, numOfdmDataSubcarriers>
    rawDataSubcarrierIndicesForNonHt =
        MakeIndexTable<unsigned char, numOfdmDataSubcarriers>(subcarrierIsDataOnNonHtChannelBandwidth);

const array<unsigned char, numOfdmSubcarriers>
    rawToDataSubcarrierIndexMapNonHt = MakeInverseIndexTable<numOfdmSubcarriers>(rawDataSubcarrierIndicesForNonHt);


//------------------------------------------------

const std::bitset<numOfdmSubcarriers>
    subcarrierIsDataOnOneChannelBandwidth(
        MakeReversedString("0000111111101111111111111011111101111110111111111111101111111000"));

const std::bitset<numOfdmSubcarriers>
    subcarrierIsUsedOnOneChannelBandwidth(
        MakeReversedString("0000111111111111111111111111111101111111111111111111111111111000"));

const array<unsigned char, numHighThroughputOfdmDataSubcarriers>
    rawDataSubcarrierIndices =
        MakeIndexTable<unsigned char, numHighThroughputOfdmDataSubcarriers>(
            subcarrierIsDataOnOneChannelBandwidth);

const array<unsigned char, numOfdmSubcarriers>
    rawToDataSubcarrierIndexMap =
        MakeInverseIndexTable<numOfdmSubcarriers>(rawDataSubcarrierIndices);

//------------------------------------------------

const std::bitset<numOfdmSubcarriersAtTwoChannelBandwidth>
    subcarrierIsDataOnTwoChannelBandwidth(
        MakeReversedString(
            "0000001111101111111111111111111111111110111111111111101111111110"
            "0011111111101111111111111011111111111111111111111111101111100000"));

const std::bitset<numOfdmSubcarriersAtTwoChannelBandwidth>
    subcarrierIsUsedOnTwoChannelBandwidth(
        MakeReversedString(
            "0000001111111111111111111111111111111111111111111111111111111110"
            "0011111111111111111111111111111111111111111111111111111111100000"));

const array<unsigned char, numOfdmDataSubcarriersAtTwoChannelBandwidth>
    rawDataSubcarrierIndicesForTwoChannelBandwidth =
        MakeIndexTable<unsigned char, numOfdmDataSubcarriersAtTwoChannelBandwidth>(
            subcarrierIsDataOnTwoChannelBandwidth);

const array<unsigned char, numOfdmSubcarriersAtTwoChannelBandwidth>
    rawToDataSubcarrierIndexMapForTwoChannelBandwidth =
        MakeInverseIndexTable<numOfdmSubcarriersAtTwoChannelBandwidth>(
            rawDataSubcarrierIndicesForTwoChannelBandwidth);

//------------------------------------------------


const std::bitset<numOfdmSubcarriersAtFourChannelBandwidth>
    subcarrierIsDataOnFourChannelBandwidth(
        MakeReversedString(
            "0000001111111111111111111011111111111111111111111111101111111111"
            "1111111111111111111111111011111111111111111111111111101111111110"
            "0011111111101111111111111111111111111110111111111111111111111111"
            "1111111111101111111111111111111111111110111111111111111111100000"));

const std::bitset<numOfdmSubcarriersAtFourChannelBandwidth>
    subcarrierIsUsedOnFourChannelBandwidth(
        MakeReversedString(
            "0000001111111111111111111111111111111111111111111111111111111111"
            "1111111111111111111111111111111111111111111111111111111111111110"
            "0011111111111111111111111111111111111111111111111111111111111111"
            "1111111111111111111111111111111111111111111111111111111111100000"));

const array<unsigned short, numOfdmDataSubcarriersAtFourChannelBandwidth>
    rawDataSubcarrierIndicesForFourChannelBandwidth =
        MakeIndexTable<unsigned short, numOfdmDataSubcarriersAtFourChannelBandwidth>(
            subcarrierIsDataOnFourChannelBandwidth);

const array<unsigned short, numOfdmSubcarriersAtFourChannelBandwidth>
    rawToDataSubcarrierIndexMapForFourChannelBandwidth =
        MakeInverseIndexTable<numOfdmSubcarriersAtFourChannelBandwidth>(
            rawDataSubcarrierIndicesForFourChannelBandwidth);


//------------------------------------------------
//
// Note: 160Mhz BW is just two 80Mhz channels even if they are contingous.
//
//------------------------------------------------


const unsigned int maxBandwidthNumberChannelsForTables = 4;


// Index Mapping Interference Factor table:
//    Low bandwidth # channels: 1, 2 (20 Mhz, 40Mhz): Indices: 0, 1.
//    High bandwidth # channels: 2, 4 (40Mhz, 80Mhz): Indices  0, 2. (1 illegal).
//    High bandwidth subchannel location indices: 0,1,2,3 (80Mhz).

const double InvalidInterferenceFactorTableValue = 0.0;

typedef array<array<array<double,
            maxBandwidthNumberChannelsForTables>,
            (maxBandwidthNumberChannelsForTables - 1)>,
            (maxBandwidthNumberChannelsForTables - 2)> InterferenceFactorTableType;


// Index Mapping Non-HT (legacy) Interference Factor table (low bandwidth only 1 channel):
//    High bandwidth # channels: 2, 4 (40Mhz, 80Mhz): Indices  0, 2. (1 illegal).
//    High bandwidth subchannel location indices: 0,1,2,3 (80Mhz).


typedef array<array<double,
            maxBandwidthNumberChannelsForTables>,
            (maxBandwidthNumberChannelsForTables - 1)> NonHtInterferenceFactorTableType;

extern
const InterferenceFactorTableType LowBandwidthChannelOnHighInterferenceFactors;

extern
const InterferenceFactorTableType HighBandwidthChannelOnLowInterferenceFactors;

extern
const NonHtInterferenceFactorTableType LowBandwidthNonHtChannelOnHighInterferenceFactors;

extern
const NonHtInterferenceFactorTableType HighBandwidthChannelOnLowNonHtInterferenceFactors;


//------------------------------------------------


typedef vector<ublas::matrix<complex<double> > > MimoPrecodingMatrices;


struct TransmissionParameters {
    unsigned int firstChannelNumber;

    // Using # channels instead of bandwidth Mhz to support variants such as 802.11ah that
    // change the base channel bandwidth size from 20 Mhz.

    unsigned int bandwidthNumberChannels;
    bool isHighThroughputFrame;

    ModulationAndCodingSchemesType modulationAndCodingScheme;
    unsigned int numberSpatialStreams;

    TransmissionParameters() : firstChannelNumber(InvalidChannelNumber),
        bandwidthNumberChannels(0), isHighThroughputFrame(false),
        modulationAndCodingScheme(McsInvalid),
        numberSpatialStreams(1) { }

};//TransmissionParameters//


//-----------------------------------------------


inline
double CalcPilotSubcarriersPowerAdjustmentFactor(
    const unsigned int bandwidthNumberChannels,
    const bool isHighThroughputFrame)
{
    //802.11ad Hack

    return 1.0;

    //11ad// assert(((isHighThroughputFrame) || (bandwidthNumberChannels == 1)) &&
    //11ad//        "High Throughput Mode Must be enabled for 40Mhz or greater bandwidth.");

    //11ad// switch (bandwidthNumberChannels) {
    //11ad// case 1:
    //11ad//     if (!isHighThroughputFrame) {
    //11ad//         return (static_cast<double>(numOfdmDataSubcarriers) / numUsedOfdmSubcarriers);
    //11ad//     }
    //11ad//     else {
    //11ad//         return
    //11ad//             (static_cast<double>(numHighThroughputOfdmDataSubcarriers) /
    //11ad//              numUsedHighThroughputOfdmSubcarriers);
    //11ad//     }//if//
    //11ad//     break;

    //11ad// case 2:
    //11ad//     return
    //11ad//         (static_cast<double>(numOfdmDataSubcarriersAtTwoChannelBandwidth) /
    //11ad//          numUsedOfdmSubcarriersAtTwoChannelBandwidth);
    //11ad//     break;

    //11ad// case 4:
    //11ad// case 8:
    //11ad//     return
    //11ad//         (static_cast<double>(numOfdmDataSubcarriersAtFourChannelBandwidth) /
    //11ad//          numUsedOfdmSubcarriersAtFourChannelBandwidth);
    //11ad//     break;

    //11ad// default:
    //11ad//     assert(false && "Illegal bandwidth"); abort();
    //11ad//     break;

    //11ad// }//switch//

    //11ad// assert(false); abort(); return 0.0;

}//CalcPilotSubcarriersPowerAdjustmentFactor//



inline
double CalcPilotSubcarriersPowerAdjustmentFactor(const TransmissionParameters& txParameters)
{
    return (
        CalcPilotSubcarriersPowerAdjustmentFactor(
            txParameters.bandwidthNumberChannels,
            txParameters.isHighThroughputFrame));

}


inline
double CalcLowBandwidthSignalOnHighInterferenceFactor(
    const unsigned int lowBandwidthNumberChannels,
    const bool lowBandwidthChannelIsHt,
    const unsigned int highBandwidthNumberChannels,
    const unsigned int highBandwidthChannelLocation)
{
    assert(lowBandwidthNumberChannels >= 1);
    assert(lowBandwidthChannelIsHt || (lowBandwidthNumberChannels == 1));
    assert(highBandwidthNumberChannels >= 2);

    double value;
    if (highBandwidthNumberChannels <= maxBandwidthNumberChannelsForTables) {
        if (lowBandwidthChannelIsHt) {
            value =
                LowBandwidthChannelOnHighInterferenceFactors
                    [lowBandwidthNumberChannels - 1]
                    [highBandwidthNumberChannels - 2]
                    [highBandwidthChannelLocation];
        }
        else {
            value =
               LowBandwidthNonHtChannelOnHighInterferenceFactors
                    [highBandwidthNumberChannels - 2]
                    [highBandwidthChannelLocation];
        }//if//
    }
    else {
        // Dual 80 Mhz (4) channels

        assert(highBandwidthNumberChannels == (2*maxBandwidthNumberChannelsForTables));
        assert(highBandwidthChannelLocation < (2*maxBandwidthNumberChannelsForTables));

        if (lowBandwidthNumberChannels == 4) {

            // 80Mhz(4) channel on 160Mhz(8) channel special case (everything matches).

            return (CalcPilotSubcarriersPowerAdjustmentFactor(lowBandwidthNumberChannels, true));
        }//if//

        assert(lowBandwidthNumberChannels <= 2);

        const unsigned int subchannelLocation =
            (highBandwidthChannelLocation % maxBandwidthNumberChannelsForTables);

        if (lowBandwidthChannelIsHt) {
            value =
                LowBandwidthChannelOnHighInterferenceFactors
                    [lowBandwidthNumberChannels - 1]
                    [maxBandwidthNumberChannelsForTables - 2]
                    [subchannelLocation];
        }
        else {
            value =
               LowBandwidthNonHtChannelOnHighInterferenceFactors
                    [maxBandwidthNumberChannelsForTables - 2]
                    [subchannelLocation];
        }//if//
    }//if//

    assert(value != InvalidInterferenceFactorTableValue);
    return (value);

}//GetLowBandwidthChannelOnHighFactor//



inline
double CalcHighBandwidthSignalOnLowInterferenceFactor(
    const unsigned int lowBandwidthNumberChannels,
    const bool lowBandwidthChannelIsHt,
    const unsigned int highBandwidthNumberChannels,
    const unsigned int highBandwidthChannelLocation)
{
    assert(lowBandwidthNumberChannels >= 1);
    assert(lowBandwidthNumberChannels <= 4);
    assert(lowBandwidthChannelIsHt || (lowBandwidthNumberChannels == 1));
    assert(highBandwidthNumberChannels >= 2);
    assert(highBandwidthNumberChannels <= (2*maxBandwidthNumberChannelsForTables));
    assert(highBandwidthChannelLocation < maxBandwidthNumberChannelsForTables);

    double value;

    if (highBandwidthNumberChannels <= maxBandwidthNumberChannelsForTables) {
        if (lowBandwidthChannelIsHt) {
            value =
                HighBandwidthChannelOnLowInterferenceFactors
                    [lowBandwidthNumberChannels - 1]
                    [highBandwidthNumberChannels - 2]
                    [highBandwidthChannelLocation];
        }
        else {
            value =
                HighBandwidthChannelOnLowNonHtInterferenceFactors
                    [highBandwidthNumberChannels - 2]
                    [highBandwidthChannelLocation];

        }//if//
    }
    else {
        // Dual 80 Mhz channels

        assert(highBandwidthNumberChannels == (2*maxBandwidthNumberChannelsForTables));
        assert(highBandwidthChannelLocation < (2*maxBandwidthNumberChannelsForTables));

        if (lowBandwidthNumberChannels == 4) {

            // 80Mhz(4) channel on 160Mhz(8) channel special case (everything matches).

            return (CalcPilotSubcarriersPowerAdjustmentFactor(lowBandwidthNumberChannels, true));
        }//if//

        const unsigned int subchannelLocation =
            (highBandwidthChannelLocation % maxBandwidthNumberChannelsForTables);

        if (lowBandwidthChannelIsHt) {
            value =
                HighBandwidthChannelOnLowInterferenceFactors
                    [lowBandwidthNumberChannels - 1]
                    [maxBandwidthNumberChannelsForTables - 2]
                    [subchannelLocation];
        }
        else {
            value =
                HighBandwidthChannelOnLowNonHtInterferenceFactors
                    [maxBandwidthNumberChannelsForTables - 2]
                    [subchannelLocation];

        }//if//
    }//if//

    assert(value != InvalidInterferenceFactorTableValue);
    return (value);

}//CalcHighBandwidthSignalOnLowInterferenceFactor//



inline
bool SignalSubcarrierLocationsMatch(
    const TransmissionParameters& signal1TxParameters,
    const TransmissionParameters& signal2TxParameters)
{
    return (
        (signal1TxParameters.firstChannelNumber == signal2TxParameters.firstChannelNumber) &&
        (signal1TxParameters.bandwidthNumberChannels == signal2TxParameters.bandwidthNumberChannels) &&
        (signal1TxParameters.isHighThroughputFrame ==  signal2TxParameters.isHighThroughputFrame));

}//SignalSubcarrierLocationsMatch//



inline
unsigned int CalcNumberOfMimoCodewords(const MimoPrecodingMatrices& precodingMatrices)
{
    const unsigned int luckySubcarrierIndex = 7;
    assert(precodingMatrices.at(luckySubcarrierIndex).size2() != 0);

    return (static_cast<unsigned int>(precodingMatrices[luckySubcarrierIndex].size2()));
}



inline
unsigned int CalcNumberOfDataSubcarriers(
    const unsigned int bandwidthNumberChannels,
    const bool isHighThroughputFrame)
{
    assert(isHighThroughputFrame || (bandwidthNumberChannels == 1));
    switch (bandwidthNumberChannels) {
    case 1:
        if (!isHighThroughputFrame) {
            return (numOfdmDataSubcarriers);
        }
        else {
            return (numHighThroughputOfdmDataSubcarriers);
        }//if//
        break;

    case 2:
        return (numOfdmDataSubcarriersAtTwoChannelBandwidth);
        break;

    case 4:
        return (numOfdmDataSubcarriersAtFourChannelBandwidth);
        break;

    case 8:
        return (2 * numOfdmDataSubcarriersAtFourChannelBandwidth);
        break;

    default:
        assert(false && "Channel Bandwidth not supported"); abort(); return 0;
        break;

    }//switch//

}//CalcNumberOfDataSubcarriers//



inline
unsigned int CalcNumberOfUsedSubcarriers(
    const unsigned int bandwidthNumberChannels,
    const bool isHighThroughputFrame)
{
    assert(isHighThroughputFrame || (bandwidthNumberChannels == 1));
    switch (bandwidthNumberChannels) {
    case 1:
        if (!isHighThroughputFrame) {
            return (numUsedOfdmSubcarriers);
        }
        else {
            return (numUsedHighThroughputOfdmSubcarriers);
        }//if//
        break;

    case 2:
        return (numUsedOfdmSubcarriersAtTwoChannelBandwidth);
        break;

    case 4:
        return (numUsedOfdmSubcarriersAtFourChannelBandwidth);
        break;

    case 8:
        return (2 * numUsedOfdmSubcarriersAtFourChannelBandwidth);
        break;

    default:
        assert(false && "Channel Bandwidth not supported"); abort(); return 0;
        break;

    }//switch//

}//CalcNumberOfDataSubcarriers//



inline
bool IsADataSubcarrier(
    const unsigned int bandwidthNumberChannels,
    const bool isHighThroughputFrame,
    const unsigned subcarrierIndex)
{
    assert(isHighThroughputFrame || (bandwidthNumberChannels == 1));

    switch (bandwidthNumberChannels) {
    case 1:
        if (!isHighThroughputFrame) {
            return (subcarrierIsDataOnNonHtChannelBandwidth[subcarrierIndex]);
        }
        else {
            return (subcarrierIsDataOnOneChannelBandwidth[subcarrierIndex]);
        }//if//
        break;

    case 2:
        return (subcarrierIsDataOnTwoChannelBandwidth[subcarrierIndex]);
        break;

    case 4:
        return (subcarrierIsDataOnFourChannelBandwidth[subcarrierIndex]);
        break;

    case 8:
        return
            (subcarrierIsDataOnFourChannelBandwidth[
                subcarrierIndex % subcarrierIsDataOnFourChannelBandwidth.size()]);

        break;

    default:
        assert(false && "Channel Bandwidth not supported"); abort(); return false;
        break;

    }//switch//

}//SubcarrierIsUsed//



inline
bool SubcarrierIsUsed(
    const unsigned int bandwidthNumberChannels,
    const bool isHighThroughputFrame,
    const unsigned subcarrierIndex)
{
    assert(isHighThroughputFrame || (bandwidthNumberChannels == 1));

    switch (bandwidthNumberChannels) {
    case 1:
        if (!isHighThroughputFrame) {
            return (subcarrierIsUsedOnNonHtChannelBandwidth[subcarrierIndex]);
        }
        else {
            return (subcarrierIsUsedOnOneChannelBandwidth[subcarrierIndex]);
        }//if//
        break;

    case 2:
        return (subcarrierIsUsedOnTwoChannelBandwidth[subcarrierIndex]);
        break;

    case 4:
        return (subcarrierIsUsedOnFourChannelBandwidth[subcarrierIndex]);
        break;

    case 8:
        return
            (subcarrierIsUsedOnFourChannelBandwidth[
                subcarrierIndex % subcarrierIsUsedOnFourChannelBandwidth.size()]);

        break;

    default:
        assert(false && "Channel Bandwidth not supported"); abort(); return false;
        break;

    }//switch//

}//SubcarrierIsUsed//



inline
bool SubcarrierIsUsedBySignal(
    const TransmissionParameters& txParameters,
    const unsigned int channelNumber,
    const unsigned int channelSubcarrierIndex)
{
    assert(channelSubcarrierIndex < numOfdmSubcarriers);

    if ((channelNumber < txParameters.firstChannelNumber) ||
        (channelNumber >= (txParameters.firstChannelNumber + txParameters.bandwidthNumberChannels))) {

        return false;
    }//if//

    const unsigned int rawSubcarrierIndex =
        (((channelNumber - txParameters.firstChannelNumber) * numOfdmSubcarriers) +
         channelSubcarrierIndex);

    return (
        SubcarrierIsUsed(
            txParameters.bandwidthNumberChannels,
            txParameters.isHighThroughputFrame,
            rawSubcarrierIndex));

}//SubcarrierIsUsedBySignal//



inline
unsigned int CalcRawSubcarrierFromDataIndex(
    const unsigned int bandwidthNumberChannels,
    const bool isHighThroughputFrame,
    const unsigned int dataSubcarrierIndex)
{
    assert(isHighThroughputFrame || (bandwidthNumberChannels == 1));

    switch (bandwidthNumberChannels) {
    case 1:
        if (!isHighThroughputFrame) {
            return (rawDataSubcarrierIndicesForNonHt.at(dataSubcarrierIndex));
        }
        else {
            return (rawDataSubcarrierIndices.at(dataSubcarrierIndex));
        }//if//
        break;

    case 2:
        return (rawDataSubcarrierIndicesForTwoChannelBandwidth.at(dataSubcarrierIndex));
        break;

    case 4:
        return (rawDataSubcarrierIndicesForFourChannelBandwidth.at(dataSubcarrierIndex));
        break;

    case 8: {
        const unsigned int fourChSize = static_cast<unsigned int>(
            rawDataSubcarrierIndicesForFourChannelBandwidth.size());

        if (dataSubcarrierIndex < fourChSize) {
            return (rawDataSubcarrierIndicesForFourChannelBandwidth.at(dataSubcarrierIndex));
        }
        else {
            return
                ((numOfdmSubcarriers * 4) +
                 rawDataSubcarrierIndicesForFourChannelBandwidth.at(dataSubcarrierIndex - fourChSize));
        }//if//
        break;
    }

    default:
        cerr << "Error illegal channel bandwidth value:" << bandwidthNumberChannels << " Channels" << endl;
        exit(1);
        return 0;
        break;
    }//switch//

}//CalcRawSubcarrierFromDataIndex//



inline
unsigned int CalcDataSubcarrierFromRawIndex(
    const unsigned int bandwidthNumberChannels,
    const bool isHighThroughputFrame,
    const unsigned int rawSubcarrierIndex)
{
    assert(isHighThroughputFrame || (bandwidthNumberChannels == 1));

    switch (bandwidthNumberChannels) {
    case 1:
        if (!isHighThroughputFrame) {
            return (rawToDataSubcarrierIndexMapNonHt.at(rawSubcarrierIndex));
        }
        else {
            return (rawToDataSubcarrierIndexMap.at(rawSubcarrierIndex));
        }//if//
        break;

    case 2:
        return (rawToDataSubcarrierIndexMapForTwoChannelBandwidth.at(rawSubcarrierIndex));
        break;

    case 4:
        return (rawToDataSubcarrierIndexMapForFourChannelBandwidth.at(rawSubcarrierIndex));
        break;

    case 8: {
        const unsigned int fourChSize = static_cast<unsigned int>(
            rawToDataSubcarrierIndexMapForFourChannelBandwidth.size());

        if (rawSubcarrierIndex < fourChSize) {
            return (rawToDataSubcarrierIndexMapForFourChannelBandwidth.at(rawSubcarrierIndex));
        }
        else {

            return (
                fourChSize +
                rawToDataSubcarrierIndexMapForFourChannelBandwidth.at(rawSubcarrierIndex - fourChSize));
        }//if//

        break;
    }


    default:
        cerr << "Error illegal channel bandwidth value:" << bandwidthNumberChannels << " Channels" << endl;
        exit(1);
        return 0;
        break;
    }//switch//

}//CalcDataSubcarrierFromRawIndex//



// Factor adjusts "full bandwidth" noise power down to "data subcarrier bandwidth" noise.

inline
double CalcThermalNoiseSubcarrierAdjustmentFactor(const TransmissionParameters& txParameters)
{
    //802.11ad Hack
    return 1.0;

    //11ad// assert(((txParameters.isHighThroughputFrame) || (txParameters.bandwidthNumberChannels == 1)) &&
    //11ad//        "High Throughput Mode Must be enabled for 40Mhz or greater bandwidth.");

    //11ad// switch (txParameters.bandwidthNumberChannels) {
    //11ad// case 1:
    //11ad//     if (!txParameters.isHighThroughputFrame) {
    //11ad//         return (static_cast<double>(numOfdmDataSubcarriers) / numOfdmSubcarriers);
    //11ad//     }
    //11ad//     else {
    //11ad//         return
    //11ad//             (static_cast<double>(numHighThroughputOfdmDataSubcarriers) / numOfdmSubcarriers);
    //11ad//     }//if//
    //11ad//     break;

    //11ad// case 2:
    //11ad//     return
    //11ad//         (static_cast<double>(numOfdmDataSubcarriersAtTwoChannelBandwidth) /
    //11ad//          numOfdmSubcarriersAtTwoChannelBandwidth);
    //11ad//     break;

    //11ad// case 4:
    //11ad// case 8:
    //11ad//     return
    //11ad//         (static_cast<double>(numOfdmDataSubcarriersAtFourChannelBandwidth) /
    //11ad//          numOfdmSubcarriersAtFourChannelBandwidth);
    //11ad//     break;

    //11ad// default:
    //11ad//     assert(false && "Illegal bandwidth"); abort(); return 0.0;
    //11ad//     break;

    //11ad// }//switch//

}//CalcThermalNoiseSubcarrierAdjustmentFactor//

//
// CalcDefaultThermalNoiseSubcarrierAdjustmentFactor is used only when not receiving a frame.
// Mostly for trace output.
//

inline
double CalcDefaultThermalNoiseSubcarrierAdjustmentFactor(const unsigned int numberBaseChannels)
{
    TransmissionParameters txParameters;
    txParameters.bandwidthNumberChannels = numberBaseChannels;
    txParameters.isHighThroughputFrame = true;
    return (CalcThermalNoiseSubcarrierAdjustmentFactor(txParameters));
}


// "CalcNumberOfBitsPerOfdmSymbol" takes account of MIMO spatial streams.

inline
unsigned int CalcNumberOfBitsPerOfdmSymbol(const TransmissionParameters& txParameters)
{
    unsigned int bandwidthNumberChannels = 1;

    assert(((txParameters.isHighThroughputFrame) || (bandwidthNumberChannels == 1)) &&
           "High Throughput Mode Must be enabled for 40Mhz or greater bandwidth.");

    unsigned int numberSubcarriers;

    switch (bandwidthNumberChannels) {
    case 1:
        if (!txParameters.isHighThroughputFrame) {
            numberSubcarriers = numOfdmDataSubcarriers;
        }
        else {
            numberSubcarriers = numHighThroughputOfdmDataSubcarriers;
        }//if//
        break;

    case 2:
        numberSubcarriers =  numOfdmDataSubcarriersAtTwoChannelBandwidth;
        break;

    case 4:
        numberSubcarriers =  numOfdmDataSubcarriersAtFourChannelBandwidth;
        break;

    case 8:
        numberSubcarriers = (2 * numOfdmDataSubcarriersAtFourChannelBandwidth);
        break;

    default:
        assert(false && "Illegal bandwidth"); abort(); numberSubcarriers = 0;
        break;

    }//switch//

    return (
        static_cast<unsigned int>(
            numberSubcarriers *
            GetFractionalBitsPerSymbol(txParameters.modulationAndCodingScheme)) *
            txParameters.numberSpatialStreams);

}//CalcNumberOfBitsPerOfdmSymbol//


//11ad// inline
//11ad// DatarateBitsPerSec CalcDatarateBitsPerSecond(
//11ad//     const SimTime& ofdmSymbolDuration,
//11ad//     const TransmissionParameters& txParameters)
//11ad// {
//11ad//     const unsigned int ofdmSymbolsPerSecond =
//11ad//         static_cast<unsigned int>(SECOND / ofdmSymbolDuration);

//11ad//     return (static_cast<DatarateBitsPerSec>(
//11ad//         ofdmSymbolsPerSecond * CalcNumberOfBitsPerOfdmSymbol(txParameters)));

//11ad// }//CalcDatarateBitsPerSecond//



//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

class Dot11MacInterfaceForPhy {
public:
    virtual void BusyChannelAtPhysicalLayerNotification() = 0;
    virtual void ClearChannelAtPhysicalLayerNotification() = 0;
    virtual void TransmissionIsCompleteNotification() = 0;
    virtual void DoSuccessfulTransmissionPostProcessing(const bool wasJustTransmitting) = 0;

    virtual void ReceiveFrameFromPhy(
        const Packet& aFrame, const TransmissionParameters& receivedFrameTxParameters) = 0;

    virtual void ReceiveAggregatedSubframeFromPhy(
        unique_ptr<Packet>& subframePtr,
        const TransmissionParameters& receivedFrameTxParameters,
        const unsigned int aggregateFrameSubframeIndex,
        const unsigned int numberSubframes) = 0;

    virtual void NotifyThatPhyReceivedCorruptedFrame() = 0;

    virtual void NotifyThatPhyReceivedCorruptedAggregatedSubframe(
        const TransmissionParameters& receivedFrameTxParameters,
        const unsigned int aggregateFrameSubframeIndex,
        const unsigned int numberSubframes) = 0;

    virtual bool AggregatedSubframeIsForThisNode(const Packet& frame) const = 0;

};//Dot11MacInterfaceForPhy//

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

class Dot11Phy {
public:
    static const string modelName;

    static const unsigned int phyFrameDataPaddingBits = 16 + 6; // Service and Tail

    struct PropFrame {
        TransmissionParameters txParameters;
        // For trace:
        PacketId thePacketId;

        // One of:
        unique_ptr<ScenSim::Packet> macFramePtr;
        unique_ptr<vector<unique_ptr<ScenSim::Packet> > > aggregateFramePtr;

        //For MIMO mode:
        unique_ptr<MimoPrecodingMatrices> mimoPrecodingMatricesWithPowerAllocationPtr;

    };//PropFrame//


    typedef SimplePropagationModelForNode<PropFrame>::IncomingSignal IncomingSignal;

    Dot11Phy(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const AntennaPatternDatabase& theAntennaPatternDatabase,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<PropFrame> >& propModelInterfacePtr,
        const shared_ptr<MimoChannelModelInterface>& mimoChannelModelInterfacePtr,
        const shared_ptr<FrequencySelectiveFadingModelInterface>& frequencySelectiveFadingModelInterfacePtr,
        const shared_ptr<CustomAntennaModel>& antennaModelPtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
        const shared_ptr<Dot11MacInterfaceForPhy> macLayerPtr,
        const RandomNumberGeneratorSeed& nodeSeed);


    ~Dot11Phy() {
        propagationModelInterfacePtr->DisconnectThisInterface();
    }

    shared_ptr<Dot11InfoInterface> GetDot11InfoInterface() const
    {
        return shared_ptr<Dot11InfoInterface>(new Dot11InfoInterface(this));
    }

    bool IsReceivingAFrame() const { return (phyState == PhyReceiving); }

    bool IsTransmittingAFrame() const {
        return ((phyState == PhyTxStarting) || (phyState == PhyTransmitting));
    }

    bool ChannelIsClear() const { return (!currentlySensingBusyMedium); }

    void StopReceivingSignalSoCanTransmit();

    void TransmitFrame(
        unique_ptr<Packet>& packetPtr,
        const TransmissionParameters& txParameters,
        const double& transmitPowerDbm,
        const SimTime& delayUntilAirborne);

    void TransmitDirectionalFrame(
        unique_ptr<Packet>& framePtr,
        const TransmissionParameters& txParameters,
        const double& transmitPowerDbm,
        const unsigned int antennaSectorId,
        const SimTime& delayUntilAirborne);

    void TransmitDirectionalFrame(
        unique_ptr<Packet>& framePtr,
        const TransmissionParameters& txParameters,
        const double& transmitPowerDbm,
        const unsigned int antennaSectorId,
        const SimTime& delayUntilAirborne,
        SimTime& transmissionEndTime);

    void TransmitAggregateFrame(
        unique_ptr<vector<unique_ptr<Packet> > >& aggregatedFramePtr,
        const TransmissionParameters& txParameters,
        const double& transmitPowerDbm,
        const SimTime& delayUntilAirborne);

    void TransmitDirectionalAggregateFrame(
        unique_ptr<vector<unique_ptr<Packet> > >& aggregateFramePtr,
        const TransmissionParameters& txParameters,
        const double& transmitPowerDbm,
        const unsigned int antennaSectorId,
        const SimTime& delayUntilAirborne);

    // Take back only after transmitted, propagated and received by all nodes.


    void TakeOwnershipOfLastTransmittedFrame(unique_ptr<Packet>& framePtr)
    {
        framePtr = move((*this).currentPropagatedFramePtr->macFramePtr);
    }

    bool LastSentFrameWasAggregate() const
        {  return (currentPropagatedFramePtr->aggregateFramePtr != nullptr); }

    void TakeOwnershipOfLastTransmittedAggregateFrame(
        unique_ptr<vector<unique_ptr<Packet> > >& aggregateFramePtr)
    {
        aggregateFramePtr = move((*this).currentPropagatedFramePtr->aggregateFramePtr);
    }

    SimTime CalculatePhysicalLayerHeaderDuration(
        const TransmissionParameters& txParameters) const;

    SimTime CalculateFrameDataDuration(
        const unsigned int frameLengthBytes,
        const TransmissionParameters& txParameters) const;

    SimTime CalculateFrameDataDurationWithPaddingBits(
        const unsigned int frameLengthBytes,
        const TransmissionParameters& txParameters) const;

    SimTime CalculateFrameBitsDuration(
        const unsigned int frameLengthBytes,
        const TransmissionParameters& txParameters) const;

    SimTime CalculateFrameTransmitDuration(
        const unsigned int frameLengthBytes,
        const TransmissionParameters& txParameters) const;

    SimTime CalculateAggregateFrameTransmitDuration(
        const vector<unique_ptr<ScenSim::Packet> >& aggregateFrame,
        const TransmissionParameters& txParameters) const;


    DatarateBitsPerSec CalculateDatarateBitsPerSecond(
        const TransmissionParameters& txParameters) const;

    // Modelling hack having to do with mobility and consecutive frames.  Mobility and
    // time-granularity of propagation delay can cause start signal events to show up
    // before end signal events.  Current value of "5ns" is overkill.

    SimTime GetDelayBetweenConsecutiveFrames() const { return (5 * NANO_SECOND); }

    SimTime GetSlotDuration() const { return aSlotTimeDuration; }
    SimTime GetShortInterframeSpaceDuration() const { return aShortInterframeSpaceDuration; }
    SimTime GetRxTxTurnaroundTime() const { return aRxTxTurnaroundTime; }
    SimTime GetPhyRxStartDelay() const { return aPhyRxStartDelay; }

    // Part of Slot Duration, but this completely separate parameter provided here for
    // new "TDMA-esque"  802.11 extensions such as 11ad.

    SimTime GetAirPropagationTimeDuration() const { return aAirPropagationTimeDuration; }

    unsigned int GetBaseChannelBandwidthMhz() const { return (baseChannelBandwidthMhz); }
    unsigned int GetChannelCount() const { return propagationModelInterfacePtr->GetChannelCount(); }

    unsigned int GetCurrentChannelNumber() const {
        assert(!currentBondedChannelList.empty());
        return (currentBondedChannelList.front());
    }

    unsigned int GetCurrentBandwidthNumChannels() const {
        return (static_cast<unsigned int>(currentBondedChannelList.size())); }
    unsigned int GetMaxChannelBandwidthMhz() const { return (maxChannelBandwidthMhz); }
    unsigned int GetMaxBandwidthNumChannels() const
        { return (maxChannelBandwidthMhz / baseChannelBandwidthMhz); }

    bool GetIsAHighThroughputStation() const { return (isAHighThroughputStation); }

    const vector<unsigned int>& GetCurrentBondedChannelList() const {
        return (currentBondedChannelList);
    }


    bool DistributedEmulationChannelIsLocal(const unsigned int channelNum) const {
        return (propagationModelInterfacePtr->ChannelIsBeingUsed(channelNum));
    }

    void SwitchToChannels(const vector<unsigned int>& bondedChannelList);

    void SwitchToChannelNumber(const unsigned int channelNumber)
    {
        vector<unsigned int> channels(1);
        channels[0] = channelNumber;
        (*this).SwitchToChannels(channels);
    }

    double GetRssiOfLastFrameDbm() const { return lastReceivedPacketRssiDbm; }
    double GetSinrOfLastFrameDb() const { return lastReceivedPacketSinrDb; }
    SimTime GetStartTimeOfLastFrame() const { return lastReceivedPacketStartTime; }
    SimTime GetDurationOfLastFrame() const { return lastReceivedPacketDuration; }

    //double GetMovingAverageOfRssiDbm() const;

    // These methods allow node to stop (and start) receiving frames. Interference is still accumulated.

    void StopReceivingFrames();
    void StartReceivingFrames();
    bool IsNotReceivingFrames() const { return (ignoreIncomingFrames); }

    // Used by emulator:
    const ScenSim::ObjectMobilityPosition GetPosition() const {
        return (propagationModelInterfacePtr->GetCurrentMobilityPosition());
    }

    unsigned int GetNumberOfReceivedFrames() const { return numberOfReceivedFrames; }
    unsigned int GetNumberOfFramesWithErrors() const { return numberOfFramesWithErrors; }
    unsigned int GetNumberOfSignalCaptures() const { return numberOfSignalCaptures; }

    SimTime GetTotalIdleChannelTime() const { return totalIdleChannelTime; }
    SimTime GetTotalBusyChannelTime() const { return totalBusyChannelTime; }
    SimTime GetTotalTransmissionTime() const { return totalTransmissionTime; }

    // 802.11ad:

    SimTime GetShortBeamformingInterframeSpaceDuration() const {
        return (shortBeamformingInterframeSpaceDuration);
    }

    SimTime GetMediumBeamformingInterframeSpaceDuration() const {
        return (mediumBeamformingInterframeSpaceDuration);
    }

    unsigned int GetNumberAntennaSectors() const {
        return (static_cast<unsigned int>(infoForAntennaSectors.size()));
    }

    void SwitchToQuasiOmniAntennaMode(
        const bool setRxPattern);


    void SwitchToDirectionalAntennaMode(
        const unsigned int sectorId,
        const bool setRxPattern);

    // sectorId:
    // QuasiOmniSectorId => QuasiOmni

    // setRxPattern:
    // true  => Set both Tx and Rx pattern
    // false => Set Tx pattern only

private:
    static const int SEED_HASH = 23788567;
    static const int DroperSeedHash = 185724713;

    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;

    enum PhyModellingModeType {
        SimpleModelling,
        FrequencySelectiveFadingModelling,
        MimoModelling
    };

    PhyModellingModeType phyModellingMode;

    shared_ptr<SimplePropagationModelForNode<PropFrame> > propagationModelInterfacePtr;

    // One of:

    shared_ptr<MimoChannelModelInterface> mimoChannelModelInterfacePtr;
    shared_ptr<FrequencySelectiveFadingModelInterface> frequencySelectiveFadingModelInterfacePtr;

    // Is 11n or 11ac.

    bool isAHighThroughputStation;

    // Allow MIMO datarates without modelling MIMO (which is nonsense).

    bool allowMimoTxWithoutMimoModel;

    // 11ad
    shared_ptr<CustomAntennaModel> antennaModelPtr;

    struct AntennaSectorInfoType {
        double sectorCenterAzimuthDegrees;
        shared_ptr<AntennaPattern> antennaPatternPtr;
    };

    vector<AntennaSectorInfoType> infoForAntennaSectors;
    bool isQuasiOmniSectorInfoSpecified;
    AntennaSectorInfoType quasiOmniSectorInfo;


    shared_ptr<BitOrBlockErrorRateCurveDatabase> berCurveDatabasePtr;

    shared_ptr<Dot11MacInterfaceForPhy> macLayerPtr;

    // If (unsynchronized) noise energy is over this threshold, then the station
    // is sensing a "busy medium"
    // Note: these variables should always be the same, but in different units.

    double energyDetectionPowerThresholdDbm;
    double energyDetectionPowerThresholdMw;

    // If the signal energy is over the "preambleDetectionPowerThresholdDbm",
    // then the station will lock on the signal and will try to recieve the
    // frame unless it does not pass the optional SINR based preamble detection
    // condition.  The parameter is also used to differentiate between
    // weak and non-weak signals in a few statistics.

    double preambleDetectionPowerThresholdDbm;

    // Supports optional probablistic preamble detection based on SINR, must first pass
    // the RSSI threshold in "preambleDetectionPowerThresholdDbm" parameter which is
    // the minimum receive power with no interference (thermal noise only).

    ScenSim::InterpolatedTable preambleDetectionProbBySinrTable;

    // This is only used when the preamble was missed (for example, if station was
    // transmitting), the station will do a degraded "lock" on a carrier.
    // Note in OFDM, the radio (as defined in the standard) can detect much lower energy
    // levels when it is locked onto the signal (preamble detection) than when it
    // is not (100X=20dB more sensitive).

    double carrierDetectionPowerThresholdDbm;
    SimTime currentLastCarrierDetectableSignalEnd;

    // Delays

    SimTime aSlotTimeDuration;
    SimTime aShortInterframeSpaceDuration;
    SimTime aRxTxTurnaroundTime;
    SimTime aPreambleLengthDuration;
    SimTime shortTrainingFieldDuration;
    SimTime aPhysicalLayerCpHeaderLengthDuration; // aka aPLCPHeaderLength
    SimTime highThroughputPhyHeaderAdditionalDuration;
    SimTime highThroughputPhyHeaderAdditionalPerStreamDuration;
    SimTime ofdmSymbolDuration;

    static const SimTime defaultAirPropagationTimeDuration = 1 * MICRO_SECOND;
    SimTime aAirPropagationTimeDuration;

    SimTime aPhyRxStartDelay;

    unsigned int baseChannelBandwidthMhz;
    unsigned int maxChannelBandwidthMhz;
    unsigned int subcarrierBandwidthHz;

    vector<unsigned int> currentBondedChannelList;
    unsigned int firstChannelNumber;

    // Standard radio parameter for how much noise the radio circuitry adds.
    double radioNoiseFigureDb;
    double thermalNoisePowerPerBaseChannelMw;
    double thermalNoisePowerPerSubcarrierMw;

    // Only used when not receiving a signal.

    double defaultThermalNoisePowerMw;

    // New signal must be at least this dB over the signal currently being received to preempt.

    double signalCaptureRatioThresholdDb;

    double mimoMaxGainDbForCaptureCalcBypass;

    bool useShortOfdmGuardInterval;

    // 802.11ad Stuff:

    unsigned int currentReceiveSectorId;
    unsigned int currentSectorId;

    SimTime shortBeamformingInterframeSpaceDuration;

    static const unsigned int mediumBeamformingIfsToSifsRatio = 3;

    SimTime mediumBeamformingInterframeSpaceDuration;

    // Used for error messages:

    NodeId theNodeId;
    InterfaceId theInterfaceId;

    // Model State variabbles

    enum PhyStateType { PhyScanning, PhyReceiving, PhyTxStarting, PhyTransmitting, PhyIgnoring };

    PhyStateType phyState;

    bool ignoreIncomingFrames;

    string phyProtocolString;

    shared_ptr<PropFrame> currentPropagatedFramePtr;

    // To keep the frame alive for long propagation delays.

    shared_ptr<PropFrame> previousPropagatedFramePtr;

    DatarateBitsPerSec outgoingTransmissionDatarateBitsPerSecond;
    double outgoingTransmissionPowerDbm;

    double currentSignalPowerDbm;

    // Subtracts power wasted on guard and pilots. Converted to milliwatts.

    double currentAdjustedSignalPowerMilliwatts;

    DatarateBitsPerSec currentIncomingSignalDatarateBitsPerSec;
    double currentThermalNoisePowerMw;

    // Total Interference energy during a receive (For mean SINR statistics only).

    double totalInterferenceEnergyMillijoules;

    vector<shared_ptr<BitErrorRateCurve> > bitErrorCurves;
    vector<DatarateBitsPerSec> bitsPerSecondDatarateForMcs;

    bool currentPacketHasAnError;

    vector<IntegralPower> currentInterferencePowers;

    SimTime lastErrorCalculationUpdateTime;

    bool currentlySensingBusyMedium;

    SimTime currentIncomingSignalStartTime;

    // Actually this is the end of L-SIG/H-SIG fields when the PHY header errors are detected.

    SimTime currentIncomingSignalEndOfPhyHeaderTime;
    SimTime currentIncomingSignalEndTime;

    NodeId currentIncomingSignalSourceNodeId;

    // Only used in MIMO/Frequency Selective modes for Phy Header error terminated frames.

    shared_ptr<const PropFrame> currentIncomingSignalFramePtr;

    TransmissionParameters currentIncomingSignalTxParameters;

    TransmissionParameters originalIncomingSignalTxParameters;
    double originalSignalPowerDbm;

    PacketId currentLockedOnFramePacketId;

    // Means that the initial non-binding calculation of the Phy header receive
    // is calculated (random) to have failed.  This determination becomes
    // the actual status if SINR does not change (new signals).

    bool tentativePhyHeaderReceiveHasFailed;

    // Aggregation

    struct AggregateFrameSubframeInfoElement {
        unique_ptr<Packet> macFramePtr;
        bool hasError;
        unsigned int lengthBytes;

        AggregateFrameSubframeInfoElement() : hasError(false) {}

        void operator=(AggregateFrameSubframeInfoElement&& right)
        {
            assert(this != &right);
            hasError = right.hasError;
            lengthBytes = right.lengthBytes;
            macFramePtr = move(right.macFramePtr);
        }
        AggregateFrameSubframeInfoElement(AggregateFrameSubframeInfoElement&& right)
            {  (*this) = move(right); }
    };

    vector<AggregateFrameSubframeInfoElement> aggregateFrameSubframeInfo;

    unsigned int currentAggregateFrameSubframeIndex;

    // Simulation optimization to avoid copying frames in Phy.

    unique_ptr<Packet> notForMeHeaderOnlyFramePtr;

    //------------------------------------------------------

    // Required for MIMO and Frequency Selective Fading.

    struct InterferingSignalInfo {
        shared_ptr<const PropFrame> framePtr;
        NodeId theNodeId;
        // Pathloss only "bulk" receive power.
        double bulkReceivePowerPerDataSubcarrierMw;

        // For expiration and garbage collection.
        SimTime signalEndTime;

        bool subcarrierLocationsMatchIncomingFrame;

        vector<vector<double> > subcarrierInterferenceLevelsMw;

        InterferingSignalInfo() :
            theNodeId(InvalidNodeId),
            bulkReceivePowerPerDataSubcarrierMw(0.0),
            signalEndTime(ZERO_TIME) { }

        InterferingSignalInfo(
            const shared_ptr<const PropFrame>& initFramePtr,
            const NodeId& initNodeId,
            const double& initBulkReceivePowerPerDataSubcarrierMw,
            const SimTime& initSignalEndTime,
            const bool initsubcarrierLocationsMatchIncomingFrame)
            :
            framePtr(initFramePtr),
            theNodeId(initNodeId),
            bulkReceivePowerPerDataSubcarrierMw(initBulkReceivePowerPerDataSubcarrierMw),
            signalEndTime(initSignalEndTime),
            subcarrierLocationsMatchIncomingFrame(initsubcarrierLocationsMatchIncomingFrame)
        {}

    };//InterferingSignalInfo//

    deque<InterferingSignalInfo> interferingSignalList;

    // Optimized MIMO weights for current frame being received.
    // Current algorithm is Minimum Mean-Square Error (MMSE) channel estimation.

    vector<ublas::matrix<complex<double> > > currentMimoReceiveWeightMatrices;

    // Power and Interference values by subcarrier and codeword.
    // Only 1 "codeword" for frequency selective fading modelling.

    vector<vector<double> > incomingFramePowerMw;
    vector<vector<double> > mimoThermalNoiseMw;
    vector<vector<double> > currentSubcarrierInterferenceLevelsMw;
    vector<vector<double> > totalSubcarrierInterferenceEnergyMillijoules;

    double currentIncomingFrameEffectiveSinrDb;


    // Only for calbration runs.

    bool useCalibrationAbstractCapacityEffectiveSinr;

    //------------------------------------------------------

    RandomNumberGenerator aRandomNumberGenerator;

    //------------------------------------------------------
    // For testing:

    RandomNumberGenerator artificialFrameDropRandomNumberGenerator;
    double artificialFrameDropProbability;
    double artificialSubframeDropProbability;

    //For Stats

    shared_ptr<CounterStatistic> transmittedFramesStatPtr;

    double lastReceivedPacketRssiDbm;
    double lastReceivedPacketSinrDb;
    SimTime lastReceivedPacketDuration;
    SimTime lastReceivedPacketStartTime;

    shared_ptr<RealStatistic> receivedFrameRssiMwStatPtr;
    shared_ptr<RealStatistic> receivedFrameSinrStatPtr;
    shared_ptr<RealStatistic> receivedFrameEffectiveSinrStatPtr;

    int numberOfReceivedFrames;
    shared_ptr<CounterStatistic> receivedFramesStatPtr;

    int numberOfFramesWithErrors;
    shared_ptr<CounterStatistic> framesWithErrorsStatPtr;

    int numberOfSignalCaptures;
    shared_ptr<CounterStatistic> signalCaptureStatPtr;

    SimTime lastChannelStateTransitionTime;
    SimTime totalIdleChannelTime;
    SimTime totalBusyChannelTime;
    SimTime totalTransmissionTime;


    shared_ptr<CounterStatistic> signalsDuringTransmissionStatPtr;
    shared_ptr<CounterStatistic> weakSignalsStatPtr;
    shared_ptr<CounterStatistic> interferingSignalsStatPtr;

    bool redundantTraceInformationModeIsOn;

    int tracePrecisionDigitsForDbm;

    //-----------------------------------------------------
    class SignalArrivalHandler: public SimplePropagationModelForNode<PropFrame>::SignalHandler {
    public:
        SignalArrivalHandler(Dot11Phy* initPhyPtr) : phyPtr(initPhyPtr) { }
        void ProcessSignal(const IncomingSignal& aSignal) { phyPtr->ProcessSignalArrivalFromChannel(aSignal); }
    private:
        Dot11Phy* phyPtr;
    };//SignalArrivalHandler//

    class SignalEndHandler: public SimplePropagationModelForNode<PropFrame>::SignalHandler {
    public:
        SignalEndHandler(Dot11Phy* initPhyPtr) : phyPtr(initPhyPtr) { }
        void ProcessSignal(const IncomingSignal& aSignal) { phyPtr->ProcessSignalEndFromChannel(aSignal); }
    private:
        Dot11Phy* phyPtr;
    };//SignalEndHandler//


    class TransmissionTimerEvent: public SimulationEvent {
    public:
        TransmissionTimerEvent(Dot11Phy* initPhyPtr) : phyPtr(initPhyPtr) { }
        void ExecuteEvent() { phyPtr->StartOrEndTransmission(); }
    private:
        Dot11Phy* phyPtr;
    };//EndOfTransmissionEvent//


    shared_ptr<SimulationEvent> transmissionTimerEventPtr;

    class AggregatedMpduFrameEndEvent: public SimulationEvent {
    public:
        AggregatedMpduFrameEndEvent(Dot11Phy* initPhyPtr) : phyPtr(initPhyPtr) { }
        void ExecuteEvent() { phyPtr->ProcessAggregatedMpduFrameEndEvent(); }
    private:
        Dot11Phy* phyPtr;

    };//AggregatedMpduFrameEndEvent//


    shared_ptr<SimulationEvent> aggregatedMpduFrameEndEventPtr;
    EventRescheduleTicket aggregatedMpduFrameEndEventTicket;

    class EndPhyHeaderEvent: public SimulationEvent {
    public:
        EndPhyHeaderEvent(Dot11Phy* initPhyPtr) : phyPtr(initPhyPtr) { }
        void ExecuteEvent() { phyPtr->ProcessEndPhyHeaderEvent(); }
    private:
        Dot11Phy* phyPtr;

    };//EndPhyHeaderEvent//


    // Used Only if preamble and Phy Header is "interrupted" (SINR changed) by another signal.

    shared_ptr<SimulationEvent> endPhyHeaderEventPtr;
    EventRescheduleTicket endPhyHeaderEventTicket;

    //-----------------------------------------------------

    shared_ptr<BitErrorRateCurve> GetBerCurve(
        const ModulationAndCodingSchemesType& modulationAndCodingScheme);

    void ReadInAntennaSectorInformation(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const AntennaPatternDatabase& theAntennaPatternDatabase);

    void PerformBernoulliTrialBitErrorCalculation(
        const BitErrorRateCurve& bitErrorRateCurve,
        const double& signalToNoiseAndInterferenceRatio,
        const unsigned int numberBits,
        bool& foundAnError);

    double CalcCurrentInterferencePowerMw(
        const unsigned int firstChannelNum,
        const unsigned int numberChannels) const;

    double CalcCurrentInterferencePowerMw() const;

    bool CurrentInterferencePowerIsAboveEnergyDetectionThreshold() const;

    bool SignalIsAboveCarrierDetectionThreshold(
        const TransmissionParameters& txParameters,
        const double& receivedPowerDbm) const;

    bool SignalIsAboveCarrierDetectionThreshold(const IncomingSignal& aSignal) const;

    void UpdatePacketReceptionCalculation();

    void UpdateSubcarrierInterferenceEnergyTotals(const double& durationSecs);

    void AddMimoInfoToPropFrame(PropFrame& propagatedFrame);

    void StartTransmission();
    void EndCurrentTransmission();
    void StartOrEndTransmission();
    void ProcessEndPhyHeaderEvent();
    void ProcessAggregatedMpduFrameEndEvent();

    void SetupReceiveOfMpduAggregateFrame(const PropFrame& incomingFrame);

    void StartReceivingThisSignal(const IncomingSignal& aSignal);
    void SetupMimoCalculationForReceive(const IncomingSignal& aSignal);
    void SetupFrequencySelectiveFadingCalculationForReceive(const IncomingSignal& aSignal);
    void ProcessNewSignal(const IncomingSignal& aSignal);

    double CalcCurrentEffectiveSignalToInterferenceAndNoiseRatioDb() const;
    double CalcMeanEffectiveSignalToInterferenceAndNoiseRatioDb(const SimTime& frameDuration) const;

    void AddSignalPowerToInterferenceLevel(
        const unsigned int signalStartChannelNumber,
        const unsigned int signalNumberChannels,
        const double& signalPowerMw);

    void SubtractSignalPowerFromInterferenceLevel(
        const unsigned int signalStartChannelNumber,
        const unsigned int signalNumberChannels,
        const double& signalPowerMw);

    // For MIMO or frequency selective fading.

    void AddSignalToSubcarrierInterferenceLevel(const IncomingSignal& aSignal);

    void CalcSignalsMimoSubcarrierInterference(
        const unsigned int dataSubcarrierIndex,
        const unsigned int channelNumber,
        const unsigned int interferingSignalRawSubcarrierIndex,
        const PropFrame& aSignalFrame,
        const NodeId sourceNodeId,
        const double& bulkReceivePowerPerDataSubcarrierMw,
        vector<double>& interferenceLevelForCodewordsMw) const;

    double CalcSignalsFrequencySelectiveFadingSubcarrierInterferenceMw(
        const unsigned int channelNumber,
        const unsigned int interferingSignalRawSubcarrierIndex,
        const NodeId sourceNodeId,
        const double& bulkReceivePowerPerDataSubcarrierMw) const;

    void AddSignalToInterferenceLevel(const IncomingSignal& aSignal);

    void SubtractSignalFromInterferenceLevel(const IncomingSignal& aSignal);
    void SubtractSignalFromSubcarrierInterferenceLevel(const IncomingSignal& aSignal);
    unsigned int FindInterferingSignalIndex(const IncomingSignal& aSignal) const;

    void SubtractSignalFromInterferenceLevelAndNotifyMacIfMediumIsClear(const IncomingSignal& aSignal);

    bool IsCurrentlyReceivingThisSignal(const IncomingSignal& aSignal) const;

    void MoveCurrentIncomingSignalToMimoInterferingSignalList();

    void StopReceivingDueToPhyHeaderFailure();

    void ProcessEndOfTheSignalCurrentlyBeingReceived(const IncomingSignal& aSignal);

    void ProcessSignalArrivalFromChannel(const IncomingSignal& aSignal);
    void ProcessSignalEndFromChannel(const IncomingSignal& aSignal);

    void OutputTraceAndStatsForAddSignalToInterferenceLevel(
        const double& signalPowerDbm,
        const double& adjustedSignalPowerMw,
        const NodeId& signalSourceNodeId,
        const PacketId& signalPacketId) const;

    void OutputTraceForSubtractSignalFromInterferenceLevel(
        const IncomingSignal& aSignal,
        const double& receivedSignalPowerMw) const;

    void OutputTraceAndStatsForTxStart(
        const Packet& aPacket,
        const double& txPowerDbm,
        const TransmissionParameters& txParameters,
        const SimTime& duration) const;

    void OutputTraceAndStatsForRxStart(const Packet& aPacket);

    void OutputTraceAndStatsForRxEnd(
        const PacketId& thePacketId,
        const bool& rxIsEndedByCapture,
        const bool& capturedInShortTrainingField);

    void ProcessStatsForTxStartingStateTransition();
    void ProcessStatsForEndCurrentTransmission();
    void ProcessStatsForTransitionToBusyChannel();
    void ProcessStatsForTransitionToIdleChannel();

    void DiscardOldestMovingAverageRecord();

    unsigned int GetFirstChannelNumberForChannelBandwidth(
        const unsigned int signalChannelBandwidthMhz) const;

    unsigned int FindChannelIndexForChannelNum(const unsigned int channelNumber) const;

    bool SignalOverlapsCurrentChannels(
        const unsigned int signalStartChannelNum,
        const unsigned int signalNumberChannels) const;

    bool SignalOverlapsPrimaryChannel(
        const unsigned int signalStartChannelNum,
        const unsigned int signalNumberChannels) const;

    bool SignalChannelsMatchForReceive(const TransmissionParameters& txParameters) const;

    const vector<unsigned int> MakeChannelListFromTxParameters(
        const TransmissionParameters& txParameters) const;

    double CalcSubcarrierPowerAdjustmentFactorForInterferringFrame(
        const TransmissionParameters& interferringFrameTxParameters);

    void CalcIfPhyHeaderHasBeenReceived(bool& phyHeaderHasBeenReceived);

    double CalculatePreambleReceivedPowerDbm(const IncomingSignal& aSignal);

    void CheckForFrameCaptureInMimoMode(
        const IncomingSignal& aSignal,
        bool& captureTheSignal);

    void GarbageCollectInterferingSignalList();

    SimTime CalculatePreamblePlusSignalFieldDuration(
        const TransmissionParameters& txParameters) const;

    // Parallelism Stuff:

    //Parallel: unsigned int eotIndex;

};//Dot11Phy//



//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

inline
Dot11Phy::Dot11Phy(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const AntennaPatternDatabase& theAntennaPatternDatabase,
    const NodeId& initNodeId,
    const InterfaceId& initInterfaceId,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<SimplePropagationModelForNode<PropFrame> >& initPropModelInterfacePtr,
    const shared_ptr<MimoChannelModelInterface>& initMimoChannelModelInterfacePtr,
    const shared_ptr<FrequencySelectiveFadingModelInterface>& initFrequencySelectiveFadingModelInterfacePtr,
    const shared_ptr<CustomAntennaModel>& initAntennaModelPtr,
    const shared_ptr<BitOrBlockErrorRateCurveDatabase>& initBerCurveDatabasePtr,
    const shared_ptr<Dot11MacInterfaceForPhy> initMacLayerPtr,
    const RandomNumberGeneratorSeed& nodeSeed)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    propagationModelInterfacePtr(initPropModelInterfacePtr),
    antennaModelPtr(initAntennaModelPtr),
    isQuasiOmniSectorInfoSpecified(false),
    mimoChannelModelInterfacePtr(initMimoChannelModelInterfacePtr),
    frequencySelectiveFadingModelInterfacePtr(initFrequencySelectiveFadingModelInterfacePtr),
    allowMimoTxWithoutMimoModel(false),
    berCurveDatabasePtr(initBerCurveDatabasePtr),
    macLayerPtr(initMacLayerPtr),
    theNodeId(initNodeId),
    theInterfaceId(initInterfaceId),
    phyState(PhyScanning),
    ignoreIncomingFrames(false),
    phyModellingMode(SimpleModelling),
    outgoingTransmissionDatarateBitsPerSecond(0),
    outgoingTransmissionPowerDbm(0.0),
    currentlySensingBusyMedium(false),
    lastErrorCalculationUpdateTime(ZERO_TIME),
    currentIncomingSignalStartTime(ZERO_TIME),
    currentIncomingSignalEndTime(ZERO_TIME),
    currentLastCarrierDetectableSignalEnd(ZERO_TIME),
    currentIncomingSignalDatarateBitsPerSec(0),
    currentSignalPowerDbm(-DBL_MAX),
    currentAdjustedSignalPowerMilliwatts(0.0),
    currentPropagatedFramePtr(new PropFrame()),
    previousPropagatedFramePtr(new PropFrame()),
    currentPacketHasAnError(false),
    currentIncomingSignalSourceNodeId(0),
    currentIncomingFrameEffectiveSinrDb(-DBL_MAX),
    aRandomNumberGenerator(HashInputsToMakeSeed(nodeSeed, initInterfaceId, SEED_HASH)),
    numberOfReceivedFrames(0),
    numberOfFramesWithErrors(0),
    numberOfSignalCaptures(0),
    lastChannelStateTransitionTime(simulationEngineInterfacePtr->CurrentTime()),
    totalIdleChannelTime(ZERO_TIME),
    totalBusyChannelTime(ZERO_TIME),
    totalTransmissionTime(ZERO_TIME),
    currentReceiveSectorId(QuasiOmniSectorId),
    totalInterferenceEnergyMillijoules(0.0),
    useCalibrationAbstractCapacityEffectiveSinr(false),
    artificialFrameDropRandomNumberGenerator(
        HashInputsToMakeSeed(nodeSeed, initInterfaceId, DroperSeedHash)),
    artificialFrameDropProbability(0.0),
    artificialSubframeDropProbability(0.0),
    tentativePhyHeaderReceiveHasFailed(false),
    transmittedFramesStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_FramesTransmitted"))),
    receivedFrameRssiMwStatPtr(
        simulationEngineInterfacePtr->CreateRealStatWithDbConversion(
            (modelName + '_' + theInterfaceId + "_ReceivedFrameRssiDbm"))),
    receivedFrameSinrStatPtr(
        simulationEngineInterfacePtr->CreateRealStatWithDbConversion(
            (modelName + '_' + theInterfaceId + "_ReceivedFrameSinrDb"))),
    receivedFramesStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_FramesReceived"))),
    framesWithErrorsStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_FramesWithErrors"))),
    signalCaptureStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_SignalsCaptured"))),
    signalsDuringTransmissionStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_SignalsDuringTransmission"))),
    weakSignalsStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_TooWeakToReceiveSignals"))),
    interferingSignalsStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + theInterfaceId + "_InterferingSignals"))),
    redundantTraceInformationModeIsOn(false),
    tracePrecisionDigitsForDbm(8)
{
    if (mimoChannelModelInterfacePtr != nullptr) {
        phyModellingMode = MimoModelling;
        propagationModelInterfacePtr->TurnOnSignalsGetFramePtrSupport();

        receivedFrameEffectiveSinrStatPtr =
            simulationEngineInterfacePtr->CreateRealStatWithDbConversion(
                (modelName + '_' + theInterfaceId + "_ReceivedFrameEffectiveSinrDb"));
    }//if//

    assert((phyModellingMode == SimpleModelling) && "11ad");

    phyProtocolString =
        theParameterDatabaseReader.ReadString(
            (parameterNamePrefix + "phy-protocol"),
            theNodeId,
            theInterfaceId);

    ConvertStringToUpperCase(phyProtocolString);

    (*this).ReadInAntennaSectorInformation(theParameterDatabaseReader, theAntennaPatternDatabase);

    (*this).SwitchToQuasiOmniAntennaMode(true/*setRxPattern*/);

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "base-channel-bandwidth-mhz"), theNodeId, theInterfaceId)) {

        // For the unusual scenario of 802.11a/g/n/ac and 802.11p coexistence.

        baseChannelBandwidthMhz =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "base-channel-bandwidth-mhz"), theNodeId, theInterfaceId);
    }
    else {
        // Initialize channel bandwidth to first channel.

        const double firstChannelsChannelBandwidthMhz =
           propagationModelInterfacePtr->GetChannelBandwidthMhz(
               propagationModelInterfacePtr->GetBaseChannelNumber());

        baseChannelBandwidthMhz = RoundToUint(firstChannelsChannelBandwidthMhz);

        if (fabs(firstChannelsChannelBandwidthMhz - baseChannelBandwidthMhz) > DBL_EPSILON) {

            cerr << "Error in Dot11 Model: Channel bandwidth must be multiple of 1 MHz." << endl;
            exit(1);
        }//if//
    }//if//

    subcarrierBandwidthHz = (baseChannelBandwidthMhz * 1000000) /  numOfdmSubcarriers;

    maxChannelBandwidthMhz = baseChannelBandwidthMhz;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "max-channel-bandwidth-mhz"),theNodeId, theInterfaceId)) {

        maxChannelBandwidthMhz =
            theParameterDatabaseReader.ReadNonNegativeInt(
                (parameterNamePrefix + "max-channel-bandwidth-mhz"), theNodeId, theInterfaceId);
    }//if//

    useShortOfdmGuardInterval = false;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "phy-use-short-guard-interval-and-shrink-ofdm-symbol-duration")) &&
        (theParameterDatabaseReader.ReadBool(
            (parameterNamePrefix + "phy-use-short-guard-interval-and-shrink-ofdm-symbol-duration")))) {

        useShortOfdmGuardInterval = true;

        ofdmSymbolDuration =
            static_cast<SimTime>(
                (ofdmSymbolDuration * shortGuardIntervalOfdmSymbolDurationShrinkFactor));
    }//if//

    isAHighThroughputStation = false;

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "enable-high-throughput-mode"), theNodeId, theInterfaceId)) {

        isAHighThroughputStation =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "enable-high-throughput-mode"), theNodeId, theInterfaceId);
    }//if//

    radioNoiseFigureDb =
        theParameterDatabaseReader.ReadDouble(
            (parameterNamePrefix + "radio-noise-figure-db"),
            theNodeId,
            theInterfaceId);

    thermalNoisePowerPerBaseChannelMw =
        CalculateThermalNoisePowerWatts(radioNoiseFigureDb, baseChannelBandwidthMhz) * 1000.0;

    const unsigned int numberBaseChannels = maxChannelBandwidthMhz / baseChannelBandwidthMhz;

    defaultThermalNoisePowerMw =
        thermalNoisePowerPerBaseChannelMw * numberBaseChannels *
        CalcDefaultThermalNoiseSubcarrierAdjustmentFactor(numberBaseChannels);

    thermalNoisePowerPerSubcarrierMw =
         1000.0 *
         CalculateThermalNoisePowerWatts(
             radioNoiseFigureDb,
             (static_cast<double>(baseChannelBandwidthMhz) / numOfdmSubcarriers));

    currentThermalNoisePowerMw = defaultThermalNoisePowerMw;

    energyDetectionPowerThresholdDbm =
        theParameterDatabaseReader.ReadDouble(
            (parameterNamePrefix + "energy-detection-power-threshold-dbm"),
            theNodeId,
            theInterfaceId);

    energyDetectionPowerThresholdMw = ConvertToNonDb(energyDetectionPowerThresholdDbm);

    preambleDetectionPowerThresholdDbm =
        theParameterDatabaseReader.ReadDouble(
            (parameterNamePrefix + "preamble-detection-power-threshold-dbm"),
            theNodeId,
            theInterfaceId);

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "preamble-detection-probability-for-sinr-db-table"),
            theNodeId,
            theInterfaceId)) {

        string tableString =
            theParameterDatabaseReader.ReadString(
                (parameterNamePrefix + "preamble-detection-probability-for-sinr-db-table"),
                theNodeId,
                theInterfaceId);

        DeleteTrailingSpaces(tableString);

        bool success;
        map<double, double> preambleDetectionProbForSinrDbMap;

        ScenSim::ConvertAStringSequenceOfRealValuedPairsIntoAMap(
            tableString,
            success,
            preambleDetectionProbForSinrDbMap);

        if (!success) {
            cerr << "Error in "
                 << (parameterNamePrefix + "preamble-detection-probability-for-sinr-db-table")
                 << " parameter:" << endl;
            cerr << "    Value= \"" << tableString << "\"" << endl;
            cerr << "    Example string: \"-5.0:0.0 -4.0:0.25 -3.0:0.5 -2.0:0.75 -1.0:1.0\"" << endl;
            exit(1);
        }//if//

        typedef map<double, double>::const_iterator IterType;

        for(IterType iter = preambleDetectionProbForSinrDbMap.begin();
             (iter != preambleDetectionProbForSinrDbMap.end()); ++iter) {

             preambleDetectionProbBySinrTable.AddDatapoint(
                 ConvertToNonDb(iter->first),
                 iter->second);
        }//for//
    }//if//


    //Default Disabled
    carrierDetectionPowerThresholdDbm = DBL_MAX;

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "carrier-detection-power-threshold-dbm"), theNodeId, theInterfaceId)) {

        carrierDetectionPowerThresholdDbm =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "carrier-detection-power-threshold-dbm"),
                theNodeId,
                theInterfaceId);
    }//if//

    signalCaptureRatioThresholdDb =
        theParameterDatabaseReader.ReadDouble(
            (parameterNamePrefix + "signal-capture-ratio-threshold-db"),
            theNodeId,
            theInterfaceId);

    mimoMaxGainDbForCaptureCalcBypass = 100.0;

    ofdmSymbolDuration =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "ofdm-symbol-duration"),
            theNodeId,
            theInterfaceId);

    aSlotTimeDuration =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "slot-time"),
            theNodeId,
            theInterfaceId);

    aAirPropagationTimeDuration = defaultAirPropagationTimeDuration;
    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "air-propagation-time"), theNodeId, theInterfaceId)) {

        aAirPropagationTimeDuration =
            theParameterDatabaseReader.ReadTime(
                (parameterNamePrefix + "air-propagation-time"),
                theNodeId,
                theInterfaceId);
    }//if//


    aShortInterframeSpaceDuration =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "sifs-time"),
            theNodeId,
            theInterfaceId);

    aRxTxTurnaroundTime =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "rx-tx-turnaround-time"),
            theNodeId,
            theInterfaceId);

    aPhyRxStartDelay =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "phy-rx-start-delay"),
            theNodeId,
            theInterfaceId);

    aPreambleLengthDuration =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "preamble-length-duration"),
            theNodeId,
            theInterfaceId);

    shortTrainingFieldDuration = aPreambleLengthDuration / 2;

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "short-training-field-duration"),
            theNodeId,
            theInterfaceId)) {

        shortTrainingFieldDuration =
            theParameterDatabaseReader.ReadTime(
                (parameterNamePrefix + "short-training-field-duration"),
                theNodeId,
                theInterfaceId);
    }//if//


    aPhysicalLayerCpHeaderLengthDuration =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "plcp-header-length-duration"),
            theNodeId,
            theInterfaceId);

    highThroughputPhyHeaderAdditionalDuration =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "phy-high-throughput-header-additional-duration"),
            theNodeId,
            theInterfaceId);

    highThroughputPhyHeaderAdditionalPerStreamDuration =
        theParameterDatabaseReader.ReadTime(
            (parameterNamePrefix + "phy-high-throughput-header-additional-per-stream-duration"),
            theNodeId,
            theInterfaceId);

    assert((NumberModAndCodingSchemes-1) == static_cast<unsigned int>(maxModulationAndCodingScheme));

    bitErrorCurves.resize(NumberModAndCodingSchemes);
    bitsPerSecondDatarateForMcs.resize(NumberModAndCodingSchemes);

    for(ModulationAndCodingSchemesType mcs = minModulationAndCodingScheme;
        (mcs <= maxModulationAndCodingScheme);
        IncrementModAndCodingScheme(mcs)) {

        bitErrorCurves.at(static_cast<size_t>(mcs)) =
            berCurveDatabasePtr->GetBerCurve(
                MakeUpperCaseString(phyProtocolString),
                MakeUpperCaseString(GetModulationAndCodingName(mcs)));

        bitsPerSecondDatarateForMcs.at(static_cast<size_t>(mcs)) =
            static_cast<DatarateBitsPerSec>(
                1000000.0 *
                theParameterDatabaseReader.ReadDouble(
                    (adParamNamePrefix + "mbps-datarate-for-" +
                     MakeLowerCaseString(GetModulationAndCodingName(mcs))),
                    theNodeId,
                    theInterfaceId));
    }//for//

    shortBeamformingInterframeSpaceDuration =
        theParameterDatabaseReader.ReadTime(
                (adParamNamePrefix + "short-beamforming-interframe-space-duration"),
                theNodeId, theInterfaceId);

    mediumBeamformingInterframeSpaceDuration =
        (mediumBeamformingIfsToSifsRatio * aShortInterframeSpaceDuration);


    transmissionTimerEventPtr.reset(new TransmissionTimerEvent(this));
    propagationModelInterfacePtr->RegisterSignalHandler(
        unique_ptr<SimplePropagationModelForNode<PropFrame>::SignalHandler>(
            new SignalArrivalHandler(this)));
    propagationModelInterfacePtr->RegisterSignalEndHandler(
        unique_ptr<SimplePropagationModelForNode<PropFrame>::SignalHandler>(
            new SignalEndHandler(this)));

    aggregatedMpduFrameEndEventPtr.reset(new AggregatedMpduFrameEndEvent(this));
    endPhyHeaderEventPtr.reset(new EndPhyHeaderEvent(this));

    // Parallelism Stuff:

    //Parallel: (*this).eotIndex = (*this).simulationEngineInterfacePtr->AllocateEarliestOutputTimeIndex();
    //Parallel: simulationEngineInterfacePtr->SetAnEarliestOutputTimeForThisNode(INFINITE_TIME, eotIndex);

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "redundant-trace-information-mode"))) {

        redundantTraceInformationModeIsOn =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "redundant-trace-information-mode"));
    }//if//

    notForMeHeaderOnlyFramePtr = Packet::CreatePacketWithoutSimInfo(CommonFrameHeader());

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "allow-mimo-speed-transmissions-without-mimo-model"))) {

        allowMimoTxWithoutMimoModel =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "allow-mimo-speed-transmissions-without-mimo-model"));
    }//if//

    useCalibrationAbstractCapacityEffectiveSinr = false;

    if (theParameterDatabaseReader.ParameterExists(
        (parameterNamePrefix + "use-calibration-abstract-capacity-effective-sinr"))) {

        useCalibrationAbstractCapacityEffectiveSinr =
            theParameterDatabaseReader.ReadBool(
                (parameterNamePrefix + "use-calibration-abstract-capacity-effective-sinr"));
    }//if//

    artificialFrameDropProbability = 0.0;

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "phy-artificial-frame-drop-probability-for-test"),
            theNodeId,
            theInterfaceId)) {

        artificialFrameDropProbability =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "phy-artificial-frame-drop-probability-for-test"),
                theNodeId,
                theInterfaceId);
    }//if//

    artificialSubframeDropProbability = 0.0;

    if (theParameterDatabaseReader.ParameterExists(
            (parameterNamePrefix + "phy-artificial-subframe-drop-probability-for-test"),
            theNodeId,
            theInterfaceId)) {

        artificialSubframeDropProbability =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "phy-artificial-subframe-drop-probability-for-test"),
                theNodeId,
                theInterfaceId);
    }//if//

}//Dot11Phy()//


inline
void Dot11Phy::ReadInAntennaSectorInformation(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const AntennaPatternDatabase& theAntennaPatternDatabase)
{
    const unsigned int numberSectors =
        theParameterDatabaseReader.ReadNonNegativeInt(
                (adParamNamePrefix + "number-directional-sectors"),
                theNodeId, theInterfaceId);

    if (numberSectors > MaxDirectionalSectors) {
        cerr << "Error in parameter \"" << (adParamNamePrefix + "number-directional-sectors")
             << " illegal value (too high)." << endl;
        exit(1);
    }//if//

    infoForAntennaSectors.resize(numberSectors);

    const string antennaModelName = MakeLowerCaseString(
        theParameterDatabaseReader.ReadString("antenna-model", theNodeId, theInterfaceId));

    if (theParameterDatabaseReader.ParameterExists(
        (adParamNamePrefix + "custom-sectored-antenna-model-sector-azimuths-degs"), theNodeId, theInterfaceId)) {

        string antennaAzimuthList =
            theParameterDatabaseReader.ReadString(
                (adParamNamePrefix + "custom-sectored-antenna-model-sector-azimuths-degs"),
                theNodeId, theInterfaceId);

        DeleteTrailingSpaces(antennaAzimuthList);

        bool success;
        vector<double> azimuthsDegs;
        ConvertAStringSequenceOfANumericTypeIntoAVector(antennaAzimuthList, success, azimuthsDegs);

        if (!success) {
            cerr << "Error reading \"custom-sectored-antenna-model-sector-azimuths-degs\" parameter: " << endl;
            cerr << "   " << antennaAzimuthList << endl;
            exit(1);
        }//if//

        if (azimuthsDegs.size() != numberSectors) {
            cerr << "Error in \"" << adParamNamePrefix
                 << "custom-sectored-antenna-model-sector-azimuths-degs\" parameter: " << endl;
            cerr << "     size does not agree with the number of sectors." << endl;
            exit(1);
        }//if//

        string antennaModelNamesString =
            theParameterDatabaseReader.ReadString(
                (adParamNamePrefix + "custom-sectored-antenna-model-sector-pattern-names"),
                theNodeId, theInterfaceId);

        DeleteTrailingSpaces(antennaModelNamesString);

        vector<string> antennaModelNames;
        ConvertAStringSequenceOfANumericTypeIntoAVector(
            antennaModelNamesString, success, antennaModelNames);

        if (!success) {
            cerr << "Error reading \"" << adParamNamePrefix
                 << "custom-sectored-antenna-model-sector-pattern-names\" parameter: " << endl;
            cerr << "   " << antennaModelNamesString << endl;
            exit(1);
        }//if//

        if (antennaModelNames.size() != numberSectors) {
            cerr << "Error in \"" << adParamNamePrefix
                 << "custom-sectored-antenna-model-sector-names\" parameter: " << endl;
            cerr << "    size does not agree with the number of sectors." << endl;
            exit(1);
        }//if//

        InterfaceOrInstanceId channelInstanceId = theInterfaceId;
        if (theParameterDatabaseReader.ParameterExists("channel-instance-id", theNodeId, theInterfaceId)) {
            channelInstanceId = MakeLowerCaseString(
                theParameterDatabaseReader.ReadString("channel-instance-id", theNodeId, theInterfaceId));
        }//if//

        const string propagationModel = MakeLowerCaseString(
            theParameterDatabaseReader.ReadString("propagation-model", channelInstanceId));

        bool disableSectorAzimuthsDegs = false;
        if (propagationModel == "hfpm") {
            bool useNonUanAntennaFormat = false;
            if (theParameterDatabaseReader.ParameterExists("hfpm-use-non-uan-antenna-format")) {
                useNonUanAntennaFormat = theParameterDatabaseReader.ReadBool("hfpm-use-non-uan-antenna-format");
            }//if//
            if (!useNonUanAntennaFormat) {
                disableSectorAzimuthsDegs = true;
            }//if//
        }//if//

        for(unsigned int i = 0; (i < numberSectors); i++) {
            infoForAntennaSectors[i].sectorCenterAzimuthDegrees = azimuthsDegs[i];
            ConvertStringToLowerCase(antennaModelNames[i]);
            if (disableSectorAzimuthsDegs) {
                if (antennaModelName != antennaModelNames[i]) {
                    cerr << "Error: Please set \"hfpm-use-non-uan-antenna-format\" true to use \""
                         << (adParamNamePrefix + "custom-sectored-antenna-model-sector-pattern-name\" = ")
                         << antennaModelNamesString << endl;
                    exit(1);
                }//if//
            }//if//
            infoForAntennaSectors[i].antennaPatternPtr =
                theAntennaPatternDatabase.GetAntennaPattern(antennaModelNames[i]);
        }//for//
    }
    else {
        const shared_ptr<AntennaPattern> antennaPatternPtr =
            theAntennaPatternDatabase.GetAntennaPattern(antennaModelName);

        for(unsigned int i = 0; (i < infoForAntennaSectors.size()); i++) {
            infoForAntennaSectors[i].sectorCenterAzimuthDegrees = ((360.0 / numberSectors) * i);
            infoForAntennaSectors[i].antennaPatternPtr = antennaPatternPtr;
        }//for//
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            (adParamNamePrefix + "custom-quasi-omni-antenna-model-name"), theNodeId, theInterfaceId)) {

        string antennaModelNamesString =
            theParameterDatabaseReader.ReadString(
                (adParamNamePrefix + "custom-quasi-omni-antenna-model-name"),
                theNodeId, theInterfaceId);

        DeleteTrailingSpaces(antennaModelNamesString);
        ConvertStringToLowerCase(antennaModelNamesString);

        isQuasiOmniSectorInfoSpecified = true;

        quasiOmniSectorInfo.sectorCenterAzimuthDegrees = 0.0;
        quasiOmniSectorInfo.antennaPatternPtr =
            theAntennaPatternDatabase.GetAntennaPattern(antennaModelNamesString);
    }//if//

}//ReadInAntennaSectorInformation//





inline
SimTime Dot11Phy::CalculatePhysicalLayerHeaderDuration(
    const TransmissionParameters& txParameters) const
{
    if (txParameters.isHighThroughputFrame) {
        return
            (aPreambleLengthDuration +
             aPhysicalLayerCpHeaderLengthDuration +
             highThroughputPhyHeaderAdditionalDuration +
             (highThroughputPhyHeaderAdditionalPerStreamDuration * txParameters.numberSpatialStreams));
    }
    else {
        return (aPreambleLengthDuration + aPhysicalLayerCpHeaderLengthDuration);

    }//if//

}//CalculatePhysicalLayerHeaderDuration//



inline
SimTime Dot11Phy::CalculatePreamblePlusSignalFieldDuration(
    const TransmissionParameters& txParameters) const
{
    if (txParameters.isHighThroughputFrame) {
        return
            (aPreambleLengthDuration +
             aPhysicalLayerCpHeaderLengthDuration +
             highThroughputPhyHeaderAdditionalDuration);
    }
    else {
        return (aPreambleLengthDuration + aPhysicalLayerCpHeaderLengthDuration);

    }//if//

}//CalculatePhysicalLayerHeaderDuration//



inline
SimTime Dot11Phy::CalculateFrameDataDuration(
    const unsigned int frameLengthBytes,
    const TransmissionParameters& txParameters) const
{
    const unsigned int numberFrameBits = (frameLengthBytes * 8);

    return CalculateFrameBitsDuration(numberFrameBits, txParameters);

}//CalculateFrameDataDuration//

inline
SimTime Dot11Phy::CalculateFrameDataDurationWithPaddingBits(
    const unsigned int frameLengthBytes,
    const TransmissionParameters& txParameters) const
{
    const unsigned int numberFrameBits = ((frameLengthBytes * 8) + phyFrameDataPaddingBits);

    return (CalculateFrameBitsDuration(numberFrameBits, txParameters));

}//CalculateFrameDataDurationWithPaddingBits//

inline
SimTime Dot11Phy::CalculateFrameBitsDuration(
    const unsigned int numberFrameBits,
    const TransmissionParameters& txParameters) const
{
    //11ad// // Note: "CalcNumberOfBitsPerOfdmSymbol" takes account of MIMO spatial streams.

    //11ad// const unsigned int numberOfOfdmBitsPerSymbol = CalcNumberOfBitsPerOfdmSymbol(txParameters);

    //11ad// const unsigned int numberOfOfdmSymbols =
    //11ad//     DivideAndRoundUp(numberFrameBits, numberOfOfdmBitsPerSymbol);

    //11ad// return (numberOfOfdmSymbols * ofdmSymbolDuration);

    const DatarateBitsPerSec& datarate =
        bitsPerSecondDatarateForMcs.at(txParameters.modulationAndCodingScheme);

    return (ConvertDoubleSecsToTime(static_cast<double>(numberFrameBits) / datarate));

}//CalculateFrameBitsDuration//


inline
SimTime Dot11Phy::CalculateFrameTransmitDuration(
    const unsigned int frameLengthBytes,
    const TransmissionParameters& txParameters) const
{
    return
        (CalculatePhysicalLayerHeaderDuration(txParameters) +
         CalculateFrameDataDurationWithPaddingBits(frameLengthBytes, txParameters));

}//CalculateFrameTransmitDuration//


inline
SimTime Dot11Phy::CalculateAggregateFrameTransmitDuration(
    const vector<unique_ptr<ScenSim::Packet> >& aggregateFrame,
    const TransmissionParameters& txParameters) const
{
    assert(!aggregateFrame.empty());

    SimTime duration =
        CalculateFrameTransmitDuration(aggregateFrame[0]->LengthBytes(), txParameters);

    for(unsigned int i = 1; (i < aggregateFrame.size()); i++) {
        duration +=
            CalculateFrameDataDuration(aggregateFrame[i]->LengthBytes(), txParameters);
    }//for//

    return (duration);

}//CalculateAggregateFrameTransmitDuration//





inline
void Dot11Phy::ProcessStatsForTxStartingStateTransition()
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
    if (currentlySensingBusyMedium) {
        (*this).totalBusyChannelTime += (currentTime - lastChannelStateTransitionTime);
    }
    else {
        (*this).totalIdleChannelTime += (currentTime - lastChannelStateTransitionTime);
    }//if//

    (*this).lastChannelStateTransitionTime = currentTime;

}//ProcessStatsForTxStartingStateTransition//


inline
void Dot11Phy::StopReceivingSignalSoCanTransmit()
{
    signalsDuringTransmissionStatPtr->IncrementCounter();

    const double signalPowerMw =
        (ConvertToNonDb(originalSignalPowerDbm) *
         CalcSubcarrierPowerAdjustmentFactorForInterferringFrame(originalIncomingSignalTxParameters));

    (*this).AddSignalPowerToInterferenceLevel(
        originalIncomingSignalTxParameters.firstChannelNumber,
        originalIncomingSignalTxParameters.bandwidthNumberChannels,
        signalPowerMw);

    if ((phyModellingMode == MimoModelling) ||
        (phyModellingMode == FrequencySelectiveFadingModelling)) {

        (*this).MoveCurrentIncomingSignalToMimoInterferingSignalList();

    }//if//

    OutputTraceAndStatsForAddSignalToInterferenceLevel(
        currentSignalPowerDbm,
        currentAdjustedSignalPowerMilliwatts,
        currentIncomingSignalSourceNodeId,
        currentLockedOnFramePacketId);

    (*this).currentSignalPowerDbm = -DBL_MAX;
    (*this).currentAdjustedSignalPowerMilliwatts = 0.0;
    (*this).currentThermalNoisePowerMw = defaultThermalNoisePowerMw;
    (*this).totalInterferenceEnergyMillijoules = 0.0;
    (*this).currentIncomingSignalSourceNodeId = 0;
    (*this).currentLockedOnFramePacketId = PacketId::nullPacketId;
    (*this).currentIncomingSignalDatarateBitsPerSec = 0;

    (*this).phyState = PhyTxStarting;

}//StopReceivingSignalSoCanTransmit//



inline
void Dot11Phy::TransmitFrame(
    unique_ptr<Packet>& packetPtr,
    const TransmissionParameters& txParameters,
    const double& transmitPowerDbm,
    const SimTime& delayUntilAirborne)
{
    assert((phyState != PhyTxStarting) && (phyState != PhyTransmitting));

    if ((!allowMimoTxWithoutMimoModel) && (mimoChannelModelInterfacePtr == nullptr) &&
        (txParameters.numberSpatialStreams > 1)) {

        cerr << "Error: MIMO transmission (multiple spatial streams) without a MIMO channel model." << endl;
        exit(1);

    }//if//

    if (phyState == PhyReceiving) {
        (*this).StopReceivingSignalSoCanTransmit();
    }//if//

    (*this).phyState = PhyTxStarting;

    (*this).ProcessStatsForTxStartingStateTransition();

    std::swap((*this).currentPropagatedFramePtr, (*this).previousPropagatedFramePtr);

    (*this).currentPropagatedFramePtr->macFramePtr = move(packetPtr);

    if (currentPropagatedFramePtr->aggregateFramePtr != nullptr) {
        (*this).currentPropagatedFramePtr->aggregateFramePtr.reset();
    }//if//

    (*this).currentPropagatedFramePtr->txParameters = txParameters;

    (*this).currentPropagatedFramePtr->txParameters.firstChannelNumber =
        GetFirstChannelNumberForChannelBandwidth(txParameters.bandwidthNumberChannels);

    assert((txParameters.firstChannelNumber == InvalidChannelNumber) ||
           (txParameters.firstChannelNumber ==
            currentPropagatedFramePtr->txParameters.firstChannelNumber));

    (*this).currentPropagatedFramePtr->thePacketId = currentPropagatedFramePtr->macFramePtr->GetPacketId();

    outgoingTransmissionPowerDbm = transmitPowerDbm;

    const SimTime startTransmissionTime = simulationEngineInterfacePtr->CurrentTime() + delayUntilAirborne;
    simulationEngineInterfacePtr->ScheduleEvent(transmissionTimerEventPtr, startTransmissionTime);

    (*this).ProcessStatsForTxStartingStateTransition();

    // Parallelism Stuff: Transmission is imminent.

    //Parallel simulationEngineInterfacePtr->SetAnEarliestOutputTimeForThisNode(
    //Parallel     (simulationEngineInterfacePtr->CurrentTime() + delayUntilAirborne),
    //Parallel    eotIndex);

}//TransmitFrame//


inline
void Dot11Phy::SwitchToQuasiOmniAntennaMode(const bool setRxPattern)
{
    (*this).currentSectorId = QuasiOmniSectorId;

    if (setRxPattern) {
        (*this).currentReceiveSectorId = QuasiOmniSectorId;
    }//if//

    if (isQuasiOmniSectorInfoSpecified) {
        antennaModelPtr->SwitchToDirectionalMode();
        antennaModelPtr->SetAntennaPattern(quasiOmniSectorInfo.antennaPatternPtr);

        propagationModelInterfacePtr->SetRelativeAntennaAttitudeAzimuth(quasiOmniSectorInfo.sectorCenterAzimuthDegrees);
    }
    else {
        antennaModelPtr->SwitchToQuasiOmniMode();

        // Call is necessary to disable cached proploss values.

        propagationModelInterfacePtr->SetRelativeAntennaAttitudeAzimuth(0.0);
    }//if//

}//SwitchToQuasiOmniAntennaMode()//



inline
void Dot11Phy::SwitchToDirectionalAntennaMode(
    const unsigned int sectorId,
    const bool setRxPattern)
{
    if (sectorId == QuasiOmniSectorId) {
        (*this).SwitchToQuasiOmniAntennaMode(setRxPattern);
        return;
    }//if//

    AntennaSectorInfoType& sectorInfo = infoForAntennaSectors.at(sectorId);

    (*this).currentSectorId = sectorId;

    if (setRxPattern) {
        (*this).currentReceiveSectorId = sectorId;
    }//if//

    antennaModelPtr->SwitchToDirectionalMode();
    antennaModelPtr->SetAntennaPattern(sectorInfo.antennaPatternPtr);

    propagationModelInterfacePtr->SetRelativeAntennaAttitudeAzimuth(sectorInfo.sectorCenterAzimuthDegrees);

}//SwitchToDirectionalAntennaMode//


inline
void Dot11Phy::GarbageCollectInterferingSignalList()
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    while((!interferingSignalList.empty()) &&
          (interferingSignalList.front().signalEndTime <= currentTime)) {

        interferingSignalList.pop_front();

    }//while//

}//GarbageCollectInterferingSignalList//


inline
void Dot11Phy::TransmitDirectionalFrame(
    unique_ptr<Packet>& framePtr,
    const TransmissionParameters& txParameters,
    const double& transmitPowerDbm,
    const unsigned int antennaSectorId,
    const SimTime& delayUntilAirborne)
{
    // Set Tx pattern only

    (*this).SwitchToDirectionalAntennaMode(antennaSectorId, false/*setRxPattern*/);

    (*this).TransmitFrame(framePtr, txParameters, transmitPowerDbm, delayUntilAirborne);

}//TransmitDirectionalFrame//



inline
void Dot11Phy::TransmitDirectionalFrame(
    unique_ptr<Packet>& framePtr,
    const TransmissionParameters& txParameters,
    const double& transmitPowerDbm,
    const unsigned int antennaSectorId,
    const SimTime& delayUntilAirborne,
    SimTime& transmissionEndTime)
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    transmissionEndTime =
        (currentTime + delayUntilAirborne + CalculateFrameTransmitDuration(framePtr->LengthBytes(), txParameters));

    (*this).TransmitDirectionalFrame(
        framePtr, txParameters, transmitPowerDbm, antennaSectorId, delayUntilAirborne);

}//TransmitDirectionalFrame//


inline
void Dot11Phy::TransmitDirectionalAggregateFrame(
    unique_ptr<vector<unique_ptr<Packet> > >& aggregateFramePtr,
    const TransmissionParameters& txParameters,
    const double& transmitPowerDbm,
    const unsigned int antennaSectorId,
    const SimTime& delayUntilAirborne)
{
    // Set Tx pattern only

    (*this).SwitchToDirectionalAntennaMode(antennaSectorId, false/*setRxPattern*/);

    (*this).TransmitAggregateFrame(aggregateFramePtr, txParameters, transmitPowerDbm, delayUntilAirborne);

}//TransmitDirectionalAggregateFrame//


inline
void Dot11Phy::TransmitAggregateFrame(
    unique_ptr<vector<unique_ptr<Packet> > >& aggregateFramePtr,
    const TransmissionParameters& txParameters,
    const double& transmitPowerDbm,
    const SimTime& delayUntilAirborne)
{
    assert((phyState != PhyTxStarting) && (phyState != PhyTransmitting));

    if ((!allowMimoTxWithoutMimoModel) && (mimoChannelModelInterfacePtr == nullptr) &&
        (txParameters.numberSpatialStreams > 1)) {

        cerr << "Error: MIMO transmission (multiple spatial streams) without a MIMO channel model." << endl;
        exit(1);

    }//if//

    if (phyState == PhyReceiving) {
        (*this).StopReceivingSignalSoCanTransmit();
    }//if//

    (*this).phyState = PhyTxStarting;

    (*this).ProcessStatsForTxStartingStateTransition();

    std::swap((*this).currentPropagatedFramePtr, (*this).previousPropagatedFramePtr);

    (*this).currentPropagatedFramePtr->aggregateFramePtr = move(aggregateFramePtr);

    if (currentPropagatedFramePtr->macFramePtr != nullptr) {
        (*this).currentPropagatedFramePtr->macFramePtr.reset();
    }//if//

    (*this).currentPropagatedFramePtr->txParameters = txParameters;

    (*this).currentPropagatedFramePtr->txParameters.firstChannelNumber =
        GetFirstChannelNumberForChannelBandwidth(txParameters.bandwidthNumberChannels);

    assert((txParameters.firstChannelNumber == InvalidChannelNumber) ||
           (txParameters.firstChannelNumber ==
            currentPropagatedFramePtr->txParameters.firstChannelNumber));

    (*this).currentPropagatedFramePtr->thePacketId =
        currentPropagatedFramePtr->aggregateFramePtr->front()->GetPacketId();

    outgoingTransmissionPowerDbm = transmitPowerDbm;

    SimTime startTransmissionTime = simulationEngineInterfacePtr->CurrentTime() + delayUntilAirborne;
    simulationEngineInterfacePtr->ScheduleEvent(transmissionTimerEventPtr, startTransmissionTime);

    (*this).ProcessStatsForTxStartingStateTransition();

    // Parallelism Stuff: Transmission is imminent.

    //Parallel simulationEngineInterfacePtr->SetAnEarliestOutputTimeForThisNode(
    //Parallel    (simulationEngineInterfacePtr->CurrentTime() + delayUntilAirborne),
    //Parallel    eotIndex);

}//TransmitAggregateFrame//



inline
unsigned int Dot11Phy::GetFirstChannelNumberForChannelBandwidth(
    const unsigned int frameNumberChannels) const
{
    assert(frameNumberChannels <= currentBondedChannelList.size());

    unsigned int firstChannelNumberForChannelBandwidth = currentBondedChannelList[0];
    for(unsigned int i = 1; (i < frameNumberChannels); i++) {
        if (firstChannelNumberForChannelBandwidth > currentBondedChannelList[i]) {
            firstChannelNumberForChannelBandwidth = currentBondedChannelList[i];
        }//if//
    }//for//

    return (firstChannelNumberForChannelBandwidth);

}//GetFirstChannelNumberForChannelBandwidth//


inline
bool Dot11Phy::SignalOverlapsCurrentChannels(
    const unsigned int signalStartChannelNum,
    const unsigned int signalNumberChannels) const
{
    return ((signalStartChannelNum < (firstChannelNumber + currentBondedChannelList.size())) &&
            ((signalStartChannelNum + signalNumberChannels) > firstChannelNumber));

}//SignalOverlapsCurrentChannels//


inline
bool Dot11Phy::SignalOverlapsPrimaryChannel(
    const unsigned int signalStartChannelNum,
    const unsigned int signalNumberChannels) const
{
    const unsigned int primaryChannel = GetFirstChannelNumberForChannelBandwidth(1);

    return ((signalStartChannelNum <=  primaryChannel) &&
            (primaryChannel < (signalStartChannelNum + signalNumberChannels)));

}//SignalOverlapsPrimaryChannel//



inline
bool Dot11Phy::SignalChannelsMatchForReceive(const TransmissionParameters& txParameters) const
{
    if (currentBondedChannelList.size() == txParameters.bandwidthNumberChannels) {
        return (txParameters.firstChannelNumber == firstChannelNumber);
    }
    else if (currentBondedChannelList.size() < txParameters.bandwidthNumberChannels) {
        return false;
    }
    else {
        return (txParameters.firstChannelNumber ==
                GetFirstChannelNumberForChannelBandwidth(txParameters.bandwidthNumberChannels));
    }//if//

}//SignalChannelsMatchForReceive//



inline
unsigned int Dot11Phy::FindChannelIndexForChannelNum(const unsigned int channelNumber) const
{
    for(unsigned int i = 0; (i < currentBondedChannelList.size()); i++) {
        if (channelNumber == currentBondedChannelList[i]) {
            return (i);
        }//if//
    }//for//

    assert(false); abort(); return 0;

}//FindChannelIndexForChannelNum//


inline
const vector<unsigned int> Dot11Phy::MakeChannelListFromTxParameters(
    const TransmissionParameters& txParameters) const
{
    assert(txParameters.bandwidthNumberChannels > 0);
    vector<unsigned int> channelList(txParameters.bandwidthNumberChannels);
    for (unsigned int i = 0; (i < channelList.size()); i++) {
        channelList[i] = txParameters.firstChannelNumber + i;
    }//for//

    return channelList;

}//MakeChannelListFromTxParameters//



inline
double Dot11Phy::CalcSubcarrierPowerAdjustmentFactorForInterferringFrame(
    const TransmissionParameters& interferringFrameTxParameters)
{
    //802.11ad Hack

    return 1.0;

    // Assume any current or future incoming frame to this node will use the whole channel
    // with respect to subcarrier assignments.

    //11ad// const unsigned int numReceiveChannels =
    //11ad//     static_cast<unsigned int>(currentBondedChannelList.size());

    //11ad// if (interferringFrameTxParameters.bandwidthNumberChannels == numReceiveChannels) {

    //11ad//     // Abstraction inaccuracy: Assume matching "High Throughput" flags or
    //11ad//     // equivalently future receive frame is HT.
    //11ad//     // (Otherwise interference signal info would have to be stored).
    //11ad//     // Note assuming NHT->HT and NHT->NHT are equal leaving
    //11ad//     // HT->NHT (assuming HT->HT) overestimates interference
    //11ad//     // (most likely receiving beacon frames).

    //11ad//     assert(interferringFrameTxParameters.firstChannelNumber == firstChannelNumber);

    //11ad//     if ((interferringFrameTxParameters.isHighThroughputFrame) && (!isAHighThroughputStation)) {
    //11ad//         return (ofdm20MhzHtOn20MhzNhtSubcarrierInterferenceFactor);
    //11ad//     }
    //11ad//     else {
    //11ad//         return (CalcPilotSubcarriersPowerAdjustmentFactor(interferringFrameTxParameters));
    //11ad//     }//if//
    //11ad// }
    //11ad// else if (interferringFrameTxParameters.bandwidthNumberChannels < numReceiveChannels) {

    //11ad//     assert(isAHighThroughputStation);

    //11ad//     return (
    //11ad//         CalcLowBandwidthSignalOnHighInterferenceFactor(
    //11ad//             interferringFrameTxParameters.bandwidthNumberChannels,
    //11ad//             interferringFrameTxParameters.isHighThroughputFrame,
    //11ad//             numReceiveChannels,
    //11ad//             FindChannelIndexForChannelNum(interferringFrameTxParameters.firstChannelNumber)));
    //11ad// }
    //11ad// else {

    //11ad//     assert(interferringFrameTxParameters.firstChannelNumber <= firstChannelNumber);
    //11ad//     const unsigned int channelIndex =
    //11ad//         (firstChannelNumber - interferringFrameTxParameters.firstChannelNumber);

    //11ad//     return (
    //11ad//         CalcHighBandwidthSignalOnLowInterferenceFactor(
    //11ad//             numReceiveChannels,
    //11ad//             isAHighThroughputStation,
    //11ad//             interferringFrameTxParameters.bandwidthNumberChannels,
    //11ad//             channelIndex));
    //11ad// }//if//

}//CalcSubcarrierPowerAdjustmentFactorForInterferringFrame//



inline
void Dot11Phy::AddMimoInfoToPropFrame(PropFrame& propagatedFrame)
{
    const unsigned int numAntennas = mimoChannelModelInterfacePtr->GetNumberOfAntennas();

    const unsigned int numberChannels = propagatedFrame.txParameters.bandwidthNumberChannels;
    const unsigned int numberSubcarriers = numberChannels  * numOfdmSubcarriers;
    const bool isHighThroughputFrame = propagatedFrame.txParameters.isHighThroughputFrame;

    propagatedFrame.mimoPrecodingMatricesWithPowerAllocationPtr.reset(
        new MimoPrecodingMatrices(numberSubcarriers));

    for(unsigned int s = 0; (s < numberSubcarriers); s++) {
        if (!SubcarrierIsUsed(numberChannels, isHighThroughputFrame, s)) {
            continue;
        }//if//

        const int centeredSubcarrierNumber = s - (numberSubcarriers / 2);
        const double subcarrierOffsetHz =
            static_cast<double>(subcarrierBandwidthHz) * centeredSubcarrierNumber;
        const double twoPiTimesSubcarrierOffsetHz = 2.0 * PI * subcarrierOffsetHz;

        ublas::matrix<complex<double> >& precodingMatrix =
            (*propagatedFrame.mimoPrecodingMatricesWithPowerAllocationPtr)[s];

        precodingMatrix.resize(
            numAntennas,
            propagatedFrame.txParameters.numberSpatialStreams,
            false);


        if (numAntennas < propagatedFrame.txParameters.numberSpatialStreams) {
            cerr << "Error in Dot11 MIMO model: More spatial streams ("
                 << propagatedFrame.txParameters.numberSpatialStreams << ") than antennas ("
                 << numAntennas << ")." << endl;
            exit(1);
        }//if//


        // "Factor" terms include both spatial expansion and stream power allocation factors.
        // Note using spatial allocation EXAMPLE from standard.  Expansion balances antenna power
        // not spatial stream power (some cases).

        switch (propagatedFrame.txParameters.numberSpatialStreams) {
        case 1:
            switch (numAntennas) {
            case 1: {
                precodingMatrix(0,0) = 1.0;

                break;
            }
            case 2: {
                // Spatial expansion factor only

                const double factor = (1.0 / sqrt(2.0));
                const double delaySecs = 4.0e-7;

                precodingMatrix(0,0) = factor;
                precodingMatrix(1,0) =
                    (factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delaySecs));

                break;
            }
            case 3: {
                // Spatial expansion factor only

                const double factor = (1.0 / sqrt(3.0));
                const double delay1Secs = 4.0e-7;
                const double delay2Secs = 2.0e-7;
                precodingMatrix(0,0) = factor;
                precodingMatrix(1,0) =
                    (factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delay1Secs));
                precodingMatrix(2,0) =
                    (factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delay2Secs));

                break;
            }
            case 4: {
                // Spatial expansion factor only

                const double factor = 0.5;
                const double delay1Secs = 4.0e-7;
                const double delay2Secs = 2.0e-7;
                const double delay3Secs = 6.0e-7;
                precodingMatrix(0,0) = factor;
                precodingMatrix(1,0) =
                    (factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delay1Secs));
                precodingMatrix(2,0) =
                    (factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delay2Secs));
                precodingMatrix(3,0) =
                    (factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delay3Secs));

                break;
            }
            default:
                assert(false); abort(); break;
            }//switch//

            break;
        case 2:
            switch (numAntennas) {
            case 2: {
                // Power allocation factor only:

                const double factor = (1.0/sqrt(2.0));
                const double delaySecs = 4.0e-7;

                precodingMatrix(0,0) = factor;
                precodingMatrix(0,1) = 0.0;
                precodingMatrix(1,0) = 0.0;
                precodingMatrix(1,1) = factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delaySecs);

                break;
            }
            case 3: {
                // Both power and spatial allocation factors

                const double factor = 1.0 / sqrt(3.0); // 1/sqrt(2) * sqrt(2/3)
                const double delaySecs = 2.0e-7;

                precodingMatrix(0,0) = factor;
                precodingMatrix(0,1) = 0.0;
                precodingMatrix(1,0) = 0.0;
                precodingMatrix(1,1) = factor;
                precodingMatrix(2,0) =
                    (factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delaySecs));
                precodingMatrix(2,1) = 0.0;

                break;
            }
            case 4: {
                // Both power and spatial allocation factors

                const double factor = 0.5;  // (1/sqrt(2)) * (1/sqrt(2));
                const double delaySecs = 2.0e-7;

                precodingMatrix(0,0) = factor;
                precodingMatrix(0,1) = 0.0;
                precodingMatrix(1,0) = 0.0;
                precodingMatrix(1,1) = factor;
                precodingMatrix(2,0) =
                    (factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delaySecs));
                precodingMatrix(2,1) = 0.0;
                precodingMatrix(3,0) = 0.0;
                precodingMatrix(3,1) =
                    (factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delaySecs));

                break;
            }
            default:
                assert(false); abort(); break;
            }//switch//

            break;

        case 3:
            switch (numAntennas) {
            case 3: {
                // Power allocation factor only:

                const double factor = (1.0/sqrt(3.0));
                const double delay1Secs = 4.0e-7;
                const double delay2Secs = 2.0e-7;

                precodingMatrix(0,0) = factor;
                precodingMatrix(0,1) = 0.0;
                precodingMatrix(0,2) = 0.0;
                precodingMatrix(1,0) = 0.0;
                precodingMatrix(1,1) = (factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delay1Secs));
                precodingMatrix(1,2) = 0.0;
                precodingMatrix(2,0) = 0.0;
                precodingMatrix(2,1) = 0.0;
                precodingMatrix(2,2) = (factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delay2Secs));

                break;
            }
            case 4: {
                // Both power and spatial allocation factors

                const double factor = 0.5;  // (1/sqrt(3))*(sqrt(3)/2)
                const double delaySecs = 6.0e-7;

                precodingMatrix(0,0) = factor;
                precodingMatrix(0,1) = 0.0;
                precodingMatrix(0,2) = 0.0;
                precodingMatrix(1,0) = 0.0;
                precodingMatrix(1,1) = factor;
                precodingMatrix(1,2) = 0.0;
                precodingMatrix(2,0) = 0.0;
                precodingMatrix(2,1) = 0.0;
                precodingMatrix(2,2) = factor;
                precodingMatrix(3,0) =
                    (factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delaySecs));
                precodingMatrix(3,1) = 0.0;
                precodingMatrix(3,2) = 0.0;

                break;
            }
            default:
                assert(false); abort(); break;
            }//switch//

            break;

        case 4: {
            assert(numAntennas == 4);

            // Power allocation factor only:

            const double factor = 0.5;

            const double delay1Secs = 4.0e-7;
            const double delay2Secs = 2.0e-7;
            const double delay3Secs = 6.0e-7;

            precodingMatrix(0,0) = factor;
            precodingMatrix(0,1) = 0.0;
            precodingMatrix(0,2) = 0.0;
            precodingMatrix(0,3) = 0.0;
            precodingMatrix(1,0) = 0.0;
            precodingMatrix(1,1) = (factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delay1Secs));
            precodingMatrix(1,2) = 0.0;
            precodingMatrix(1,3) = 0.0;
            precodingMatrix(2,0) = 0.0;
            precodingMatrix(2,1) = 0.0;
            precodingMatrix(2,2) = (factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delay2Secs));
            precodingMatrix(2,3) = 0.0;
            precodingMatrix(3,0) = 0.0;
            precodingMatrix(3,1) = 0.0;
            precodingMatrix(3,2) = 0.0;
            precodingMatrix(3,3) = (factor * CalcComplexExp(twoPiTimesSubcarrierOffsetHz * delay3Secs));

            break;
        }
        default:
            assert(false); abort(); break;
        }//switch//
    }//for//

}//AddMimoInfoToPropFrame//



inline
void Dot11Phy::StartTransmission()
{
    assert(phyState == PhyTxStarting);
    phyState = PhyTransmitting;

    // The medium is busy because we are transmitting on it!
    (*this).currentlySensingBusyMedium = true;


    SimTime duration;

    if (currentPropagatedFramePtr->macFramePtr != nullptr) {
        duration =
            CalculateFrameTransmitDuration(
                currentPropagatedFramePtr->macFramePtr->LengthBytes(),
                currentPropagatedFramePtr->txParameters);
    }
    else {
        duration =
            CalculateAggregateFrameTransmitDuration(
                *currentPropagatedFramePtr->aggregateFramePtr,
                currentPropagatedFramePtr->txParameters);
    }//if//


    if (currentPropagatedFramePtr->macFramePtr != nullptr) {
        OutputTraceAndStatsForTxStart(
            *currentPropagatedFramePtr->macFramePtr,
            outgoingTransmissionPowerDbm,
            currentPropagatedFramePtr->txParameters,
            duration);
    }
    else {
        assert(currentPropagatedFramePtr->aggregateFramePtr != nullptr);

        for(unsigned int i = 0; (i < currentPropagatedFramePtr->aggregateFramePtr->size()); i++) {

            OutputTraceAndStatsForTxStart(
                *(*currentPropagatedFramePtr->aggregateFramePtr)[i],
                outgoingTransmissionPowerDbm,
                currentPropagatedFramePtr->txParameters,
                duration);

        }//for//
    }//if//

    if (phyModellingMode == MimoModelling) {

        AddMimoInfoToPropFrame(*currentPropagatedFramePtr);

    }//if//


    if (currentBondedChannelList.empty()) {
        propagationModelInterfacePtr->TransmitSignal(
            outgoingTransmissionPowerDbm, duration, currentPropagatedFramePtr);
    }
    else {
        propagationModelInterfacePtr->TransmitSignal(
            MakeChannelListFromTxParameters(currentPropagatedFramePtr->txParameters),
            outgoingTransmissionPowerDbm,
            duration,
            currentPropagatedFramePtr);
    }//if//

    SimTime endOfTransmissionTime = simulationEngineInterfacePtr->CurrentTime() + duration;

    simulationEngineInterfacePtr->ScheduleEvent(transmissionTimerEventPtr, endOfTransmissionTime);

    // Parallelism: Delete stale Earliest Output Time.

    // Parallelism: (*this).simulationEngineInterfacePtr->SetAnEarliestOutputTimeForThisNode(INFINITE_TIME, eotIndex);

}//StartTransmission//



inline
void Dot11Phy::StartOrEndTransmission()
{
    if (phyState == PhyTxStarting) {
        (*this).StartTransmission();
    }
    else if (phyState == PhyTransmitting) {
        (*this).EndCurrentTransmission();
    }
    else {
        assert(false); abort();
    }//if//
}

inline
shared_ptr<BitErrorRateCurve> Dot11Phy::GetBerCurve(
    const ModulationAndCodingSchemesType& modulationAndCodingScheme)
{
    const size_t bitErrorCurveIndex = static_cast<size_t>(modulationAndCodingScheme);

    if (bitErrorCurves.at(bitErrorCurveIndex) == nullptr) {
        bitErrorCurves.at(bitErrorCurveIndex) =
            berCurveDatabasePtr->GetBerCurve(
                MakeUpperCaseString(phyProtocolString),
                MakeUpperCaseString(GetModulationAndCodingName(modulationAndCodingScheme)));
    }//if//

    return bitErrorCurves.at(bitErrorCurveIndex);

}//GetBerCurve//

inline
void Dot11Phy::PerformBernoulliTrialBitErrorCalculation(
    const BitErrorRateCurve& bitErrorRateCurve,
    const double& signalToNoiseAndInterferenceRatio,
    const unsigned int numberBits,
    bool& foundAnError)
{
    if (numberBits == 0) {
        foundAnError = false;
    }
    else {
        const double bitErrorRate = bitErrorRateCurve.CalculateBitErrorRate(signalToNoiseAndInterferenceRatio);

        const double probabilityOfZeroErrors = pow((1.0-bitErrorRate), static_cast<int>(numberBits));
        const double randomNumber = (*this).aRandomNumberGenerator.GenerateRandomDouble();

        foundAnError = (randomNumber > probabilityOfZeroErrors);

    }//if//

}//PerformBernoulliTrialBitErrorCalculation//


// Restrict interference to subset of channels.

inline
double Dot11Phy::CalcCurrentInterferencePowerMw(
    const unsigned int firstChannelNum,
    const unsigned int numberChannels) const
{
    IntegralPower totalInterferencePower(0.0);

    for(unsigned int i = 0; (i < numberChannels); i++) {
        const unsigned int channelNumber = firstChannelNumber + i;
        if ((channelNumber >= firstChannelNumber) &&
            ((channelNumber - firstChannelNumber) < currentInterferencePowers.size())) {

            totalInterferencePower += currentInterferencePowers[channelNumber - firstChannelNumber];

        }//if//
    }//for//

    return (totalInterferencePower.ConvertToMilliwatts());

}//CalcCurrentInterferencePowerMw//


inline
double Dot11Phy::CalcCurrentInterferencePowerMw() const
{
    if (currentLockedOnFramePacketId != PacketId::nullPacketId) {

        return (
            CalcCurrentInterferencePowerMw(
                currentIncomingSignalTxParameters.firstChannelNumber,
                currentIncomingSignalTxParameters.bandwidthNumberChannels));
    }
    else {
        // Don't restrict interference power to current signal's channels.

        IntegralPower totalInterferencePower(0.0);

        for(unsigned int i = 0; (i < currentInterferencePowers.size()); i++) {
            totalInterferencePower += currentInterferencePowers[i];
        }//for//

        return (totalInterferencePower.ConvertToMilliwatts());

    }//if//

}//CalcCurrentInterferencePowerMw//


inline
bool Dot11Phy::CurrentInterferencePowerIsAboveEnergyDetectionThreshold() const
{
    assert(!currentBondedChannelList.empty());
    const unsigned int primaryChannelNum = currentBondedChannelList[0];
    const double primaryChannelInterferenceMw =
        currentInterferencePowers[primaryChannelNum - firstChannelNumber].ConvertToMilliwatts();

    if (primaryChannelInterferenceMw > energyDetectionPowerThresholdMw) {
        return true;
    }//if//

    if (currentBondedChannelList.size() == 1) {
        return false;
    }//if//

    const unsigned int secondaryChannelNum = currentBondedChannelList[1];
    const double secondaryChannelInterferenceMw =
        currentInterferencePowers[secondaryChannelNum - firstChannelNumber].ConvertToMilliwatts();

    if (secondaryChannelInterferenceMw > energyDetectionPowerThresholdMw) {
        return true;
    }//if//

    if (currentBondedChannelList.size() == 2) {
        return false;
    }//if//

    assert(currentBondedChannelList.size() >= 4);

    const IntegralPower secondaryTwoChannelBwInterferencePower =
        (currentInterferencePowers.at(currentBondedChannelList[2] - firstChannelNumber) +
         currentInterferencePowers.at(currentBondedChannelList[3] - firstChannelNumber));

    const double secondaryTwoChannelBwInterferencePowerMw =
        secondaryTwoChannelBwInterferencePower.ConvertToMilliwatts();

    if (secondaryTwoChannelBwInterferencePowerMw > (ThreeDbFactor * energyDetectionPowerThresholdMw)) {
        return true;
    }//if//

    if (currentBondedChannelList.size() == 4) {
        return false;
    }//if//

    assert(currentBondedChannelList.size() == 8);

    const IntegralPower secondaryFourChannelBwInterferencePower =
        (currentInterferencePowers.at(currentBondedChannelList[4] - firstChannelNumber) +
         currentInterferencePowers.at(currentBondedChannelList[5] - firstChannelNumber) +
         currentInterferencePowers.at(currentBondedChannelList[6] - firstChannelNumber) +
         currentInterferencePowers.at(currentBondedChannelList[7] - firstChannelNumber));

    const double secondaryFourChannelBwInterferencePowerMw =
        secondaryFourChannelBwInterferencePower.ConvertToMilliwatts();

    return (secondaryFourChannelBwInterferencePowerMw > (SixDbFactor * energyDetectionPowerThresholdMw));

}//CurrentInterferencePowerIsAboveEnergyDetectionThreshold//



inline
bool Dot11Phy::SignalIsAboveCarrierDetectionThreshold(
    const TransmissionParameters& txParameters,
    const double& receivedPowerDbm) const
{
    if (carrierDetectionPowerThresholdDbm == DBL_MAX) {
        return false;
    }//if//

    // Allowing carrier detection on primary channels for partial signals (radio turned on,
    // or just changed channels, etc).
    //
    // Using standard channel mapping restrictions (no straddling).

    //Jay const PropFrame& aFrame = aSignal.GetFrame();

    const unsigned int numberReceiveChannels =
        static_cast<unsigned int>(currentBondedChannelList.size());

    // Using Raw power without OFDM Guard or Subcarrier mapping adjustment factors.


    const double receivedPowerPerChannelDbm =
        (receivedPowerDbm - ConvertIntToDb(txParameters.bandwidthNumberChannels));

    if (txParameters.bandwidthNumberChannels == numberReceiveChannels) {
        // Signal channels completely match or cover receive channels.

        return (receivedPowerPerChannelDbm > carrierDetectionPowerThresholdDbm);
    }
    else {
        if (numberReceiveChannels == 2) {
            return (receivedPowerPerChannelDbm > carrierDetectionPowerThresholdDbm);
        }
        else {
            const size_t pos =
                (find(currentBondedChannelList.begin(),
                     currentBondedChannelList.end(),
                     txParameters.firstChannelNumber) - currentBondedChannelList.begin());

            switch (pos) {
            case 0:
            case 1:
                // Signal on primary channel or secondary channel.
                return (receivedPowerPerChannelDbm > carrierDetectionPowerThresholdDbm);
                break;

            case 2:
            case 3:
                // Signal on secondary 40Mhz channel

                if (txParameters.bandwidthNumberChannels == 1) {
                    return (receivedPowerPerChannelDbm > (carrierDetectionPowerThresholdDbm + 3.0));
                }
                else {
                    return (receivedPowerPerChannelDbm > (carrierDetectionPowerThresholdDbm));
                }//if//
                break;

            case 4:
            case 5:
            case 6:
            case 7:
                // Signal on secondary 80Mhz channel.

                if (txParameters.bandwidthNumberChannels == 1) {
                    return (receivedPowerPerChannelDbm > (carrierDetectionPowerThresholdDbm + 6.0));
                }
                else if (txParameters.bandwidthNumberChannels == 2) {
                    return (receivedPowerPerChannelDbm > (carrierDetectionPowerThresholdDbm + 3.0));
                }
                else {
                    return (receivedPowerPerChannelDbm > carrierDetectionPowerThresholdDbm);
                }//if//

                break;

            default:
                assert(false); abort(); return (false); break;
            }//switch//
        }//if//
    }//if//

}//SignalIsAboveCarrierDetectionThreshold//



inline
bool Dot11Phy::SignalIsAboveCarrierDetectionThreshold(const IncomingSignal& aSignal) const
{
    const PropFrame& aFrame = aSignal.GetFrame();

    return
        (SignalIsAboveCarrierDetectionThreshold(
            aFrame.txParameters,
            aSignal.GetReceivedPowerDbm()));
}



inline
void Dot11Phy::UpdateSubcarrierInterferenceEnergyTotals(const double& durationSecs)
{
    for(unsigned int i = 0; (i < totalSubcarrierInterferenceEnergyMillijoules.size()); i++) {
        for (unsigned int j = 0; (j < totalSubcarrierInterferenceEnergyMillijoules[i].size()); j++) {
           totalSubcarrierInterferenceEnergyMillijoules[i][j] +=
               (durationSecs * currentSubcarrierInterferenceLevelsMw[i][j]);
        }//for//
    }//for//
}




inline
void Dot11Phy::UpdatePacketReceptionCalculation()
{
    using std::max;
    using std::min;

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
    const SimTime startHeaderTime = (currentIncomingSignalStartTime + aPreambleLengthDuration);
    const SimTime startDataTime = startHeaderTime + aPhysicalLayerCpHeaderLengthDuration;

    if (currentTime <= startHeaderTime) {
        // Still in preamble, nothing to do.
        return;
    }//if//

    const double interferencePowerMw = CalcCurrentInterferencePowerMw();
    const double totalNoiseAndInterferencePowerMw =
        (currentThermalNoisePowerMw + interferencePowerMw);

    double signalToNoiseAndInterferenceRatio;

    if (phyModellingMode == SimpleModelling) {
        signalToNoiseAndInterferenceRatio =
            (currentAdjustedSignalPowerMilliwatts / totalNoiseAndInterferencePowerMw);
    }
    else {
        (*this).currentIncomingFrameEffectiveSinrDb =
            CalcCurrentEffectiveSignalToInterferenceAndNoiseRatioDb();

        signalToNoiseAndInterferenceRatio =
            ConvertToNonDb(currentIncomingFrameEffectiveSinrDb);

        if (interferencePowerMw > 0.0) {
            (*this).UpdateSubcarrierInterferenceEnergyTotals(
                ConvertTimeToDoubleSecs(currentTime - lastErrorCalculationUpdateTime));
        }//if/

    }//if//

    (*this).totalInterferenceEnergyMillijoules +=
        (interferencePowerMw *
         ConvertTimeToDoubleSecs(currentTime - lastErrorCalculationUpdateTime));

    if (!currentPacketHasAnError) {
        if (lastErrorCalculationUpdateTime < startDataTime) {
            const SimTime timeInHeader =
                min(currentTime, startDataTime) -
                max(lastErrorCalculationUpdateTime, startHeaderTime);

            const unsigned int numberOfBits =
                RoundToUint(
                    (ConvertTimeToDoubleSecs(timeInHeader) * currentIncomingSignalDatarateBitsPerSec));

            (*this).PerformBernoulliTrialBitErrorCalculation(
                *(*this).GetBerCurve(minModulationAndCodingScheme),
                signalToNoiseAndInterferenceRatio,
                numberOfBits,
                currentPacketHasAnError);
        }//if//

        if ((!currentPacketHasAnError) && (currentTime > startDataTime)) {
            const SimTime timeInDataPart =
                currentTime -
                max(lastErrorCalculationUpdateTime, startDataTime);

            const unsigned int numberOfBits =
                RoundToUint(
                    (ConvertTimeToDoubleSecs(timeInDataPart) * currentIncomingSignalDatarateBitsPerSec));

            (*this).PerformBernoulliTrialBitErrorCalculation(
                *(*this).GetBerCurve(
                    currentIncomingSignalTxParameters.modulationAndCodingScheme),
                signalToNoiseAndInterferenceRatio,
                numberOfBits,
                currentPacketHasAnError);
        }//if//
    }//if//

    (*this).lastErrorCalculationUpdateTime = simulationEngineInterfacePtr->CurrentTime();

}//UpdatePacketReceptionCalculation//


inline
void Dot11Phy::AddSignalPowerToInterferenceLevel(
    const unsigned int signalStartChannelNumber,
    const unsigned int signalNumberChannels,
    const double& signalPowerMw)
{
    const double signalPowerPerChannel = (signalPowerMw / signalNumberChannels);
    const unsigned int numberChannels = static_cast<unsigned int>(currentBondedChannelList.size());
    const unsigned int lastChannelNumber = firstChannelNumber + numberChannels - 1;

    for (unsigned int i = 0; (i < signalNumberChannels); i++) {
        const unsigned int channelNum = (signalStartChannelNumber + i);
        if ((channelNum >= firstChannelNumber) && (channelNum <= lastChannelNumber)) {
            (*this).currentInterferencePowers[channelNum - firstChannelNumber] += signalPowerPerChannel;
        }//if//
    }//for//

}//AddSignalPowerToInterferenceLevel//


inline
void Dot11Phy::SubtractSignalPowerFromInterferenceLevel(
    const unsigned int signalStartChannelNumber,
    const unsigned int signalNumberChannels,
    const double& signalPowerMw)
{
    const double signalPowerPerChannel = (signalPowerMw / signalNumberChannels);
    const unsigned int numberChannels = static_cast<unsigned int>(currentBondedChannelList.size());
    const unsigned int lastChannelNumber = firstChannelNumber + numberChannels - 1;

    for (unsigned int i = 0; (i < signalNumberChannels); i++) {
        const unsigned int channelNum = (signalStartChannelNumber + i);
        if ((channelNum >= firstChannelNumber) && (channelNum <= lastChannelNumber)) {
            (*this).currentInterferencePowers[channelNum - firstChannelNumber] -= signalPowerPerChannel;
        }//if//
    }//for//

}//SubtractSignalPowerFromInterferenceLevel//


inline
void Dot11Phy::CalcSignalsMimoSubcarrierInterference(
    const unsigned int dataSubcarrierIndex,
    const unsigned int channelNumber,
    const unsigned int interferingSignalRawSubcarrierIndex,
    const PropFrame& aSignalFrame,
    const NodeId sourceNodeId,
    const double& bulkReceivePowerPerDataSubcarrierMw,
    vector<double>& interferenceLevelForCodewordsMw) const
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    MimoChannelMatrix interfererChannelMatrix;

    const unsigned int channelSubcarrierIndex =
        (interferingSignalRawSubcarrierIndex % numOfdmSubcarriers);

    mimoChannelModelInterfacePtr->GetChannelMatrix(
        currentTime, channelNumber, sourceNodeId, channelSubcarrierIndex, interfererChannelMatrix);

    const ublas::matrix<complex<double> >& interfererPrecodingMatrix =
        (*aSignalFrame.mimoPrecodingMatricesWithPowerAllocationPtr).at(
            interferingSignalRawSubcarrierIndex);

    const ublas::matrix<complex<double> > interfererChannelMatrixWithPrecoding =
        prod(interfererChannelMatrix, interfererPrecodingMatrix);

    const unsigned int numInterfererCodewords =
        static_cast<const unsigned int>(interfererPrecodingMatrix.size2());

    const ublas::matrix<complex<double> > channelMatrixWithPreAndPostcoding =
        prod(currentMimoReceiveWeightMatrices[dataSubcarrierIndex], interfererChannelMatrixWithPrecoding);

    const unsigned int numIncomingCodewords =
        static_cast<const unsigned int>(currentMimoReceiveWeightMatrices[0].size1());

    for(unsigned int l = 0; (l < numIncomingCodewords); l++) {
        interferenceLevelForCodewordsMw.at(l) = 0.0;
        for(unsigned int j = 0; (j < numInterfererCodewords); j++) {
            interferenceLevelForCodewordsMw.at(l) +=
                (std::norm(channelMatrixWithPreAndPostcoding(l, j)) *
                 bulkReceivePowerPerDataSubcarrierMw);
        }//for//
    }//for//

}//CalcSignalsMimoSubcarrierInterference//



inline
double Dot11Phy::CalcSignalsFrequencySelectiveFadingSubcarrierInterferenceMw(
    const unsigned int channelNumber,
    const unsigned int interferingSignalRawSubcarrierIndex,
    const NodeId sourceNodeId,
    const double& bulkReceivePowerPerDataSubcarrierMw) const
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
    frequencySelectiveFadingModelInterfacePtr->SetTime(channelNumber, currentTime);
    const double fadingValueDb =
        frequencySelectiveFadingModelInterfacePtr->GetSubcarrierOrSubchannelDbFadingVector(
            channelNumber, sourceNodeId).at(interferingSignalRawSubcarrierIndex);

    return (ConvertToNonDb(fadingValueDb) * bulkReceivePowerPerDataSubcarrierMw);

}//CalcSignalsFrequencySelectiveFadingSubcarrierInterferenceMw//



inline
void Dot11Phy::AddSignalToSubcarrierInterferenceLevel(const IncomingSignal& aSignal)
{
    using std::max;
    using std::min;

    assert((phyModellingMode == MimoModelling) ||
           (phyModellingMode == FrequencySelectiveFadingModelling));

    (*this).GarbageCollectInterferingSignalList();

    const PropFrame& aFrame = aSignal.GetFrame();
    assert(aFrame.mimoPrecodingMatricesWithPowerAllocationPtr != nullptr);

    const double bulkReceivePowerPerDataSubcarrierMw =
        (ConvertToNonDb(aSignal.GetReceivedPowerDbm()) /
         CalcNumberOfUsedSubcarriers(
             aFrame.txParameters.bandwidthNumberChannels,
             aFrame.txParameters.isHighThroughputFrame));

    bool subcarrierLocationsMatchIncomingFrame = false;
    if (phyState == PhyReceiving) {
        subcarrierLocationsMatchIncomingFrame =
            SignalSubcarrierLocationsMatch(aFrame.txParameters, currentIncomingSignalTxParameters);
    }//if//

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    interferingSignalList.push_back(
        InterferingSignalInfo(
            aSignal.GetFramePtr(),
            aSignal.GetSourceNodeId(),
            bulkReceivePowerPerDataSubcarrierMw,
            (currentTime + aSignal.GetDuration()),
            subcarrierLocationsMatchIncomingFrame));

    if (phyState == PhyReceiving) {

        const unsigned int numberDataSubcarriers =
            static_cast<const unsigned int>(currentSubcarrierInterferenceLevelsMw.size());

        const unsigned int numIncomingCodewords =
            static_cast<const unsigned int>(currentSubcarrierInterferenceLevelsMw.front().size());

        // Pulled out of loop (created once).

        vector<double> interferenceForCodewordsMw(numIncomingCodewords);

        bool mustSaveSignalsInterference = false;

        if (interferingSignalList.back().signalEndTime <= currentIncomingSignalEndTime) {
            // Only save interference additions if necessary.
            interferingSignalList.back().subcarrierInterferenceLevelsMw.resize(numberDataSubcarriers);
            mustSaveSignalsInterference = true;

        }//if//

        const unsigned int firstChannel =
            max(aFrame.txParameters.firstChannelNumber,
                currentIncomingSignalTxParameters.firstChannelNumber);

        const unsigned int lastChannel =
            (firstChannel +
             min(aFrame.txParameters.bandwidthNumberChannels,
                 currentIncomingSignalTxParameters.bandwidthNumberChannels) - 1);

        for(unsigned int chan = firstChannel; (chan <= lastChannel); chan++) {
            const unsigned int subcarrierIndexOffset =
                (chan - currentIncomingSignalTxParameters.firstChannelNumber) * numOfdmSubcarriers;

            const unsigned int interferrerSubcarrierIndexOffset =
                (chan - aFrame.txParameters.firstChannelNumber) * numOfdmSubcarriers;

            for(unsigned int s = 0; (s < numOfdmSubcarriers); s++) {
                if ((IsADataSubcarrier(
                        currentIncomingSignalTxParameters.bandwidthNumberChannels,
                        currentIncomingSignalTxParameters.isHighThroughputFrame,
                        (subcarrierIndexOffset + s)) &&
                    (SubcarrierIsUsed(
                        aFrame.txParameters.bandwidthNumberChannels,
                        aFrame.txParameters.isHighThroughputFrame,
                        interferrerSubcarrierIndexOffset + s)))) {

                    const unsigned int dataSubcarrierIndex =
                        CalcDataSubcarrierFromRawIndex(
                            currentIncomingSignalTxParameters.bandwidthNumberChannels,
                            currentIncomingSignalTxParameters.isHighThroughputFrame,
                            (subcarrierIndexOffset + s));

                    if (phyModellingMode == MimoModelling) {

                        CalcSignalsMimoSubcarrierInterference(
                            dataSubcarrierIndex,
                            chan,
                            (interferrerSubcarrierIndexOffset + s),
                            aFrame,
                            aSignal.GetSourceNodeId(),
                            bulkReceivePowerPerDataSubcarrierMw,
                            interferenceForCodewordsMw);
                    }
                    else {
                        assert(phyModellingMode == FrequencySelectiveFadingModelling);
                        assert(numIncomingCodewords == 1);
                        assert(interferenceForCodewordsMw.size() == 1);

                        interferenceForCodewordsMw[0] =
                            CalcSignalsFrequencySelectiveFadingSubcarrierInterferenceMw(
                                chan,
                                (interferrerSubcarrierIndexOffset + s),
                                aSignal.GetSourceNodeId(),
                                bulkReceivePowerPerDataSubcarrierMw);
                    }//if//


                    for(unsigned int i = 0; (i < numIncomingCodewords); i++) {
                        (*this).currentSubcarrierInterferenceLevelsMw[dataSubcarrierIndex][i] +=
                            interferenceForCodewordsMw[i];
                    }//for//

                    if (mustSaveSignalsInterference) {
                        interferingSignalList.back().subcarrierInterferenceLevelsMw[dataSubcarrierIndex] =
                            interferenceForCodewordsMw;
                    }//if//
                }//if//
            }//for//
        }//for//
    }//if//

}//AddSignalPowerToMimoInterferenceLevel//





inline
void Dot11Phy::MoveCurrentIncomingSignalToMimoInterferingSignalList()
{
    assert((phyModellingMode == MimoModelling) ||
           (phyModellingMode == FrequencySelectiveFadingModelling));

    const double bulkReceivePowerPerDataSubcarrierMw =
        (ConvertToNonDb(currentSignalPowerDbm) /
         CalcNumberOfUsedSubcarriers(
             originalIncomingSignalTxParameters.bandwidthNumberChannels,
             originalIncomingSignalTxParameters.isHighThroughputFrame));

    interferingSignalList.push_back(
        InterferingSignalInfo(
            currentIncomingSignalFramePtr,
            currentIncomingSignalSourceNodeId,
            bulkReceivePowerPerDataSubcarrierMw,
            currentIncomingSignalEndTime,
            false));

    currentIncomingSignalFramePtr.reset();

}//MoveCurrentIncomingSignalToMimoInterferingSignalList//


inline
void Dot11Phy::StopReceivingDueToPhyHeaderFailure()
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    assert(phyState == PhyReceiving);
    (*this).phyState = PhyScanning;

    const double signalPowerMw =
        (ConvertToNonDb(originalSignalPowerDbm) *
         CalcSubcarrierPowerAdjustmentFactorForInterferringFrame(originalIncomingSignalTxParameters));

    (*this).AddSignalPowerToInterferenceLevel(
        originalIncomingSignalTxParameters.firstChannelNumber,
        originalIncomingSignalTxParameters.bandwidthNumberChannels,
        signalPowerMw);

    OutputTraceAndStatsForAddSignalToInterferenceLevel(
        currentSignalPowerDbm,
        currentAdjustedSignalPowerMilliwatts,
        currentIncomingSignalSourceNodeId,
        currentLockedOnFramePacketId);

    if ((phyModellingMode == MimoModelling) ||
        (phyModellingMode == FrequencySelectiveFadingModelling)) {

        (*this).MoveCurrentIncomingSignalToMimoInterferingSignalList();

    }//if//

    // Carrier Detection logic.

    if (SignalIsAboveCarrierDetectionThreshold(currentIncomingSignalTxParameters, currentSignalPowerDbm)) {
        if (currentIncomingSignalEndTime > currentLastCarrierDetectableSignalEnd) {
            currentLastCarrierDetectableSignalEnd = currentIncomingSignalEndTime;
        }//if//
    }//if//

    (*this).currentPacketHasAnError = true;
    (*this).currentIncomingSignalStartTime = ZERO_TIME;
    (*this).currentIncomingSignalEndOfPhyHeaderTime = ZERO_TIME;
    (*this).currentIncomingSignalEndTime = ZERO_TIME;
    (*this).currentIncomingSignalSourceNodeId = InvalidNodeId;
    (*this).currentLockedOnFramePacketId = PacketId::nullPacketId;
    (*this).tentativePhyHeaderReceiveHasFailed = false;
    (*this).aggregateFrameSubframeInfo.clear();
    (*this).currentAggregateFrameSubframeIndex = 0;

    if ((currentTime >= currentLastCarrierDetectableSignalEnd) &&
        (!CurrentInterferencePowerIsAboveEnergyDetectionThreshold())) {

        (*this).currentlySensingBusyMedium = false;
        (*this).ProcessStatsForTransitionToIdleChannel();

        if (phyState != PhyIgnoring) {
            macLayerPtr->ClearChannelAtPhysicalLayerNotification();
        }//if//

    }//if//

}//StopReceivingDueToPhyHeaderFailure//



inline
void Dot11Phy::AddSignalToInterferenceLevel(const IncomingSignal& aSignal)
{
    PacketId thePacketId;
    double signalPowerMw;

    if (phyState == PhyReceiving) {
        (*this).UpdatePacketReceptionCalculation();
    }//if//

    if (aSignal.HasAFrame()) {
        const PropFrame& aFrame = aSignal.GetFrame();

        if (!SignalOverlapsCurrentChannels(
                aFrame.txParameters.firstChannelNumber,
                aFrame.txParameters.bandwidthNumberChannels)) {

            // No interference to add. (Occurs only during channel switching)
            return;

        }//if//

        signalPowerMw =
           (ConvertToNonDb(aSignal.GetReceivedPowerDbm()) *
            CalcSubcarrierPowerAdjustmentFactorForInterferringFrame(aFrame.txParameters));

        (*this).AddSignalPowerToInterferenceLevel(
            aFrame.txParameters.firstChannelNumber,
            aFrame.txParameters.bandwidthNumberChannels,
            signalPowerMw);

        if ((phyModellingMode == MimoModelling) ||
            (phyModellingMode == FrequencySelectiveFadingModelling)) {

            (*this).AddSignalToSubcarrierInterferenceLevel(aSignal);

        }//if//

        thePacketId = aFrame.thePacketId;

        // Carrier Detection logic.

        if (SignalIsAboveCarrierDetectionThreshold(aSignal)) {
            const SimTime signalEndTime =
                (simulationEngineInterfacePtr->CurrentTime() + aSignal.GetDuration());

            if (signalEndTime > currentLastCarrierDetectableSignalEnd) {
                currentLastCarrierDetectableSignalEnd = signalEndTime;
            }//if//
        }//if//
    }
    else {
        // Raw noise/interference signal (Not a Dot11 frame signal).

        assert((phyModellingMode == SimpleModelling) && "TBD");

        signalPowerMw = ConvertToNonDb(aSignal.GetReceivedPowerDbm());

        (*this).AddSignalPowerToInterferenceLevel(
            aSignal.GetChannelNumber(),
            aSignal.GetNumberBondedChannels(),
            signalPowerMw);

    }//if//

    OutputTraceAndStatsForAddSignalToInterferenceLevel(
        aSignal.GetReceivedPowerDbm(),
        signalPowerMw,
        aSignal.GetSourceNodeId(),
        thePacketId);

}//AddSignalToInterferenceLevel//


inline
unsigned int Dot11Phy::FindInterferingSignalIndex(const IncomingSignal& aSignal) const
{
    for(unsigned int i = 0; (i < interferingSignalList.size()); i++) {
        if (interferingSignalList[i].framePtr == aSignal.GetFramePtr()) {
            return i;
        }//if//
    }//for//

    assert(false && "Interfering Signal Not Found"); abort(); return 0;
}


inline
void Dot11Phy::SubtractSignalFromSubcarrierInterferenceLevel(const IncomingSignal& aSignal)
{
    // MIMO/Freq. Fading processing: No need to delete from signal list as it is garbage collected
    // (using simulation time).

    if (simulationEngineInterfacePtr->CurrentTime() == currentIncomingSignalStartTime) {
        // Signal ends on exact same time tick as incoming frame start. Ignore.
        return;
    }//if//

    const vector<vector<double> >& subcarrierInterferenceLevelsMw =
        interferingSignalList[FindInterferingSignalIndex(aSignal)].subcarrierInterferenceLevelsMw;

    assert(currentSubcarrierInterferenceLevelsMw.size() == subcarrierInterferenceLevelsMw.size());

    for(unsigned int s = 0; (s < currentSubcarrierInterferenceLevelsMw.size()); s++) {
        for(unsigned int l = 0; (l < currentSubcarrierInterferenceLevelsMw[s].size()); l++) {
            if (subcarrierInterferenceLevelsMw[s].empty()) {
                continue;
            }//if//

            const double newLevelMw =
                (currentSubcarrierInterferenceLevelsMw[s][l] - subcarrierInterferenceLevelsMw[s][l]);

            if (newLevelMw < 0.0) {
                assert((-newLevelMw) < DBL_EPSILON);
                currentSubcarrierInterferenceLevelsMw[s][l] = 0.0;
            }
            else {
                currentSubcarrierInterferenceLevelsMw[s][l] = newLevelMw;
            }//if//
        }//for//
    }//for//

}//SubtractSignalFromSubcarrierInterferenceLevel//



inline
void Dot11Phy::SubtractSignalFromInterferenceLevel(const IncomingSignal& aSignal)
{
    if (phyState == PhyReceiving) {
        (*this).UpdatePacketReceptionCalculation();
    }//if//

    double signalPowerMw;

    if (aSignal.HasAFrame()) {
        const PropFrame& aFrame = aSignal.GetFrame();

        if (!SignalOverlapsCurrentChannels(
                aFrame.txParameters.firstChannelNumber,
                aFrame.txParameters.bandwidthNumberChannels)) {

            // No interference to subtract. (Occurs only during channel switching).

            return;
        }//if//


        // Assuming pilots don't contribute to interference.

        signalPowerMw =
            (ConvertToNonDb(aSignal.GetReceivedPowerDbm()) *
             CalcSubcarrierPowerAdjustmentFactorForInterferringFrame(aFrame.txParameters));

        (*this).SubtractSignalPowerFromInterferenceLevel(
            aFrame.txParameters.firstChannelNumber,
            aFrame.txParameters.bandwidthNumberChannels,
            signalPowerMw);

        if ((phyState == PhyReceiving) &&
            ((phyModellingMode == MimoModelling) ||
             (phyModellingMode == FrequencySelectiveFadingModelling))) {

            (*this).SubtractSignalFromSubcarrierInterferenceLevel(aSignal);

        }//if//
    }
    else {
        // Raw noise/interference signal (Not a Dot11 frame signal).

        assert((phyModellingMode == SimpleModelling) && "TBD");

        signalPowerMw = ConvertToNonDb(aSignal.GetReceivedPowerDbm());

        (*this).SubtractSignalPowerFromInterferenceLevel(
            aSignal.GetChannelNumber(),
            aSignal.GetNumberBondedChannels(),
            signalPowerMw);
    }//if//

    OutputTraceForSubtractSignalFromInterferenceLevel(aSignal, signalPowerMw);

}//SubtractSignalFromInterferenceLevel//



inline
void Dot11Phy::SubtractSignalFromInterferenceLevelAndNotifyMacIfMediumIsClear(
    const IncomingSignal& aSignal)
{
    (*this).SubtractSignalFromInterferenceLevel(aSignal);

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (((phyState == PhyScanning) || (phyState == PhyIgnoring)) &&
        (currentlySensingBusyMedium) &&
        ((currentTime >= currentLastCarrierDetectableSignalEnd) &&
         (!CurrentInterferencePowerIsAboveEnergyDetectionThreshold()))) {

        (*this).currentlySensingBusyMedium = false;
        (*this).ProcessStatsForTransitionToIdleChannel();

        if (phyState != PhyIgnoring) {
            macLayerPtr->ClearChannelAtPhysicalLayerNotification();
        }//if//

    }//if//

}//SubtractSignalFromInterferenceLevelAndNotifyMacIfMediumIsClear//



inline
void Dot11Phy::SetupReceiveOfMpduAggregateFrame(const PropFrame& incomingFrame)
{
    assert(incomingFrame.macFramePtr == nullptr);

    // Simulation Optimization: Peek (extra-simulation) at frame to avoid useless packet copies.

    const bool frameIsForMe =
        ((!currentPacketHasAnError) &&
         macLayerPtr->AggregatedSubframeIsForThisNode(*incomingFrame.aggregateFramePtr->front()));

    if (!frameIsForMe) {
        // Just copy header into bogus "header-only" frame.

        (*this).notForMeHeaderOnlyFramePtr->GetAndReinterpretPayloadData<CommonFrameHeader>() =
            incomingFrame.aggregateFramePtr->front()->GetAndReinterpretPayloadData<CommonFrameHeader>(
                sizeof(MpduDelimiterFrame));

        assert(aggregateFrameSubframeInfo.empty());
        return;
    }//if//

    (*this).aggregateFrameSubframeInfo.resize(incomingFrame.aggregateFramePtr->size());

    for(unsigned int i = 0; (i < incomingFrame.aggregateFramePtr->size()); i++) {
        AggregateFrameSubframeInfoElement& info = (*this).aggregateFrameSubframeInfo[i];

        info.lengthBytes = (*incomingFrame.aggregateFramePtr)[i]->LengthBytes();
        info.hasError = false;

        assert(info.macFramePtr == nullptr);

        info.macFramePtr.reset(new Packet(*(*incomingFrame.aggregateFramePtr)[i]));
    }//for//

    (*this).currentAggregateFrameSubframeIndex = 0;

    if (incomingFrame.aggregateFramePtr->size() > 1) {

        const SimTime subframeEndTime =
            (simulationEngineInterfacePtr->CurrentTime() +
             CalculateFrameTransmitDuration(
                 incomingFrame.aggregateFramePtr->front()->LengthBytes(),
                 incomingFrame.txParameters));

        simulationEngineInterfacePtr->ScheduleEvent(
            aggregatedMpduFrameEndEventPtr,
            subframeEndTime,
            aggregatedMpduFrameEndEventTicket);
    }//if//

}//SetupReceiveOfAggregateFrame//


// Only for calibration.

inline
double CalcCalibrationAbstractCapacityEffectiveSinrDb(const vector<double>& subpartSinrsDb)
{
    using std::pow;
    using std::log;
    using std::log10;

    // No "log2" yet (C++11).
    const double oneOverLog2Value = (1.0 / log(2.0));

    double total = 0.0;
    for(unsigned int i = 0; (i < subpartSinrsDb.size()); i++) {
        total += log(1.0 + ConvertToNonDb(subpartSinrsDb[i])) * oneOverLog2Value;
    }//for//

    const double numberTones = static_cast<double>(subpartSinrsDb.size());

    return (10.0 * log10(pow(2.0, (total / numberTones)) - 1));

}//CalcCalibrationAbstractCapacityEffectiveSinrDb//



inline
double Dot11Phy::CalcCurrentEffectiveSignalToInterferenceAndNoiseRatioDb() const
{
    assert((phyModellingMode == MimoModelling) ||
           (phyModellingMode == FrequencySelectiveFadingModelling));

    assert(!incomingFramePowerMw.empty());
    assert(!mimoThermalNoiseMw.empty());
    const unsigned int numDataSubcarriers = static_cast<const unsigned int>(incomingFramePowerMw.size());
    const unsigned int numIncomingCodewords = static_cast<const unsigned int>(incomingFramePowerMw.front().size());

    vector<double> signalToInterferenceAndNoiseRatiosDb(numDataSubcarriers * numIncomingCodewords);

    for(unsigned int s = 0; (s < numDataSubcarriers); s++) {
        for(unsigned int l = 0; (l < numIncomingCodewords); l++) {
            signalToInterferenceAndNoiseRatiosDb[((s * numIncomingCodewords) + l)] =
                ConvertToDb(
                    (incomingFramePowerMw[s][l] /
                     (currentSubcarrierInterferenceLevelsMw[s][l] + mimoThermalNoiseMw[s][l])));
        }//for//
    }//for//

    if (!useCalibrationAbstractCapacityEffectiveSinr) {
        return (
            CalculateEffectiveSinrDb(
                GetModulation(currentIncomingSignalTxParameters.modulationAndCodingScheme),
                signalToInterferenceAndNoiseRatiosDb));
    }
    else {
        return (
            CalcCalibrationAbstractCapacityEffectiveSinrDb(signalToInterferenceAndNoiseRatiosDb));
    }//if//

}//CalcCurrentEffectiveSignalToInterferenceAndNoiseRatioDb()//



inline
void OutputSubcarrierSinrsForCalibration(const vector<double>& signalToInterferenceAndNoiseRatiosDb)
{
    for(unsigned int i = 0; (i < signalToInterferenceAndNoiseRatiosDb.size()); i++) {
        cout << setprecision(10) << signalToInterferenceAndNoiseRatiosDb[i] << endl;
    }//for//
}



// This routine calculates a mean interference level from the accumulated total energy
// and is used by the stat routines for frame stats.

inline
double Dot11Phy::CalcMeanEffectiveSignalToInterferenceAndNoiseRatioDb(
    const SimTime& frameDuration) const
{
    assert((phyModellingMode == MimoModelling) ||
           (phyModellingMode == FrequencySelectiveFadingModelling));

    assert(!incomingFramePowerMw.empty());
    assert(!mimoThermalNoiseMw.empty());

    const double frameDurationSecs = ConvertTimeToDoubleSecs(frameDuration);

    const unsigned int numDataSubcarriers = static_cast<const unsigned int>(incomingFramePowerMw.size());
    const unsigned int numIncomingCodewords = static_cast<const unsigned int>(incomingFramePowerMw.front().size());

    vector<double> signalToInterferenceAndNoiseRatiosDb(numDataSubcarriers * numIncomingCodewords);

    for(unsigned int s = 0; (s < numDataSubcarriers); s++) {
        for(unsigned int l = 0; (l < numIncomingCodewords); l++) {

            const double meanInterferenceMw =
               (totalSubcarrierInterferenceEnergyMillijoules[s][l] / frameDurationSecs);

            signalToInterferenceAndNoiseRatiosDb[((s * numIncomingCodewords) + l)] =
                ConvertToDb(
                    (incomingFramePowerMw[s][l] /
                     (meanInterferenceMw + mimoThermalNoiseMw[s][l])));
        }//for//
    }//for//

    if (!useCalibrationAbstractCapacityEffectiveSinr) {
        return (
            CalculateEffectiveSinrDb(
                GetModulation(currentIncomingSignalTxParameters.modulationAndCodingScheme),
                signalToInterferenceAndNoiseRatiosDb));
    }
    else {
        // OutputSubcarrierSinrsForCalibration(signalToInterferenceAndNoiseRatiosDb);

        return (
            CalcCalibrationAbstractCapacityEffectiveSinrDb(signalToInterferenceAndNoiseRatiosDb));
    }//if//

}//CalcCurrentEffectiveSignalToInterferenceAndNoiseRatioDb()//




template<class T> inline
void CalcMatrixInverse(
   const ublas::matrix<T>& theMatrix,
   bool& success,
   ublas::matrix<T>& inverseMatrix)
{
   ublas::matrix<T> matrixCopy(theMatrix);
   ublas::permutation_matrix<std::size_t> permMatrix(theMatrix.size1());

   size_t res = lu_factorize(matrixCopy, permMatrix);

   if (res != 0) {
       // Matrix is singular, can't be inverted.
       success = false;
       return;
   }//if//

   // create identity matrix of "inverse"

   inverseMatrix.assign(ublas::identity_matrix<T>(theMatrix.size1()));

   // Backsubstitute to get the inverse

   lu_substitute(matrixCopy, permMatrix, inverseMatrix);

   success = true;

}//CalcMatrixInverse//


inline
void NormalizeWeightMatrix(ublas::matrix<complex<double> >& weightMatrix)
{   //
    // Decrease weights so that all weights (power is the square of the complex weight)
    // from an antenna does not exceed the power received at the antenna.
    //
    using std::max;

    double maxTotal = 0.0;

    for(unsigned int j = 0; (j < weightMatrix.size2()); j++) {
        double antennaTotal = 0.0;
        for(unsigned int i = 0; (i < weightMatrix.size1()); i++) {
           antennaTotal += std::norm(weightMatrix(i, j));
        }//for//
        maxTotal = max(maxTotal, antennaTotal);
    }//for//

    for(unsigned int i = 0; (i < weightMatrix.size1()); i++) {
        for(unsigned int j = 0; (j < weightMatrix.size2()); j++) {
           weightMatrix(i,j) /= sqrt(maxTotal);
        }//for//
    }//for//

}//NormalizeWeightMatrix//


inline
void Dot11Phy::SetupMimoCalculationForReceive(const IncomingSignal& aSignal)
{
    assert(phyModellingMode == MimoModelling);

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    (*this).currentIncomingSignalFramePtr = aSignal.GetFramePtr();

    (*this).GarbageCollectInterferingSignalList();

    const PropFrame& aFrame = aSignal.GetFrame();

    assert((aFrame.txParameters.bandwidthNumberChannels ==
           currentIncomingSignalTxParameters.bandwidthNumberChannels));

    // Optimization for signals with same subcarrier mappings.

    for(unsigned i = 0; (i < interferingSignalList.size()); i++) {
        InterferingSignalInfo& aSignalInfo = interferingSignalList[i];
        if (aSignalInfo.signalEndTime > currentTime) {
            const PropFrame& aSignalFrame = *aSignalInfo.framePtr;

            aSignalInfo.subcarrierLocationsMatchIncomingFrame =
                SignalSubcarrierLocationsMatch(
                    currentIncomingSignalTxParameters,
                    aSignalFrame.txParameters);
        }//if//
    }//for//

    assert(aFrame.mimoPrecodingMatricesWithPowerAllocationPtr != nullptr);

    const unsigned int numReceiveAntennas = mimoChannelModelInterfacePtr->GetNumberOfAntennas();

    const unsigned int numberDataSubcarriers =
        CalcNumberOfDataSubcarriers(
            currentIncomingSignalTxParameters.bandwidthNumberChannels,
            currentIncomingSignalTxParameters.isHighThroughputFrame);

    const unsigned int numIncomingCodewords =
        CalcNumberOfMimoCodewords(*aFrame.mimoPrecodingMatricesWithPowerAllocationPtr);

    const double desiredSignalBulkReceivePowerPerSubcarrierMw =
        ((ConvertToNonDb(aSignal.GetReceivedPowerDbm()) *
          CalcPilotSubcarriersPowerAdjustmentFactor(currentIncomingSignalTxParameters)) /
         numberDataSubcarriers);

    (*this).incomingFramePowerMw.resize(numberDataSubcarriers);
    (*this).mimoThermalNoiseMw.resize(numberDataSubcarriers);
    (*this).currentSubcarrierInterferenceLevelsMw.resize(numberDataSubcarriers);
    (*this).totalSubcarrierInterferenceEnergyMillijoules.resize(numberDataSubcarriers);
    (*this).currentMimoReceiveWeightMatrices.resize(numberDataSubcarriers);

    MimoChannelMatrix desiredSignalChannelMatrix;

    for(unsigned int s = 0; (s < numberDataSubcarriers); s++) {

        // "Raw" subcarrier index covers all 64 subcarriers (nulls and pilots).

        const unsigned int rawSubcarrierIndex =
            CalcRawSubcarrierFromDataIndex(
                currentIncomingSignalTxParameters.bandwidthNumberChannels,
                currentIncomingSignalTxParameters.isHighThroughputFrame,
                s);

        const unsigned int channelSubcarrierIndex = (rawSubcarrierIndex % numOfdmSubcarriers);

        const unsigned int channelNumber =
            (currentIncomingSignalTxParameters.firstChannelNumber + (rawSubcarrierIndex / numOfdmSubcarriers));

        vector<vector<double> >
            signalToInterferenceAndNoiseRatios(
                numIncomingCodewords, vector<double>(vector<double>(numberDataSubcarriers, 0.0)));

        // Calculate interference levels for each antenna.

        // Initialize with thermal noise power.

        ublas::vector<double> interferenceAndNoiseMwForAntenna(
            numReceiveAntennas,
            thermalNoisePowerPerSubcarrierMw);

        for(unsigned i = 0; (i < interferingSignalList.size()); i++) {
            const InterferingSignalInfo& aSignalInfo = interferingSignalList[i];
            if (aSignalInfo.signalEndTime > currentTime) {

                const PropFrame& aSignalFrame = *aSignalInfo.framePtr;

                if ((aSignalInfo.subcarrierLocationsMatchIncomingFrame) ||
                    (SubcarrierIsUsedBySignal(
                        aSignalFrame.txParameters,
                        channelNumber,
                        channelSubcarrierIndex))) {

                    MimoChannelMatrix interfererChannelMatrix;

                    mimoChannelModelInterfacePtr->GetChannelMatrix(
                        currentTime, channelNumber, aSignalInfo.theNodeId, channelSubcarrierIndex,
                        interfererChannelMatrix);

                    unsigned int interferingSignalRawSubcarrierIndex = rawSubcarrierIndex;
                    if (!aSignalInfo.subcarrierLocationsMatchIncomingFrame) {
                        interferingSignalRawSubcarrierIndex =
                            (((channelNumber - aSignalFrame.txParameters.firstChannelNumber) *
                              baseChannelBandwidthMhz) + channelSubcarrierIndex);
                    }//if//

                    const ublas::matrix<complex<double> >& interfererPrecodingMatrix =
                        (*aSignalFrame.mimoPrecodingMatricesWithPowerAllocationPtr).at(
                            interferingSignalRawSubcarrierIndex);

                    const ublas::matrix<complex<double> > interfererChannelMatrixWithPrecoding =
                        prod(interfererChannelMatrix, interfererPrecodingMatrix);

                    const unsigned int numInterfererCodewords =
                        static_cast<const unsigned int>(interfererPrecodingMatrix.size2());

                    for(unsigned int l = 0; (l < numInterfererCodewords); l++) {
                        for(unsigned int j = 0; (j < numReceiveAntennas); j++) {

                            interferenceAndNoiseMwForAntenna[j] +=
                                (std::norm(interfererChannelMatrixWithPrecoding(j, l)) *
                                 aSignalInfo.bulkReceivePowerPerDataSubcarrierMw);

                        }//for//
                    }//for//
                }//if//
            }//if//
        }//for//

        mimoChannelModelInterfacePtr->GetChannelMatrix(
            currentTime, channelNumber, aSignal.GetSourceNodeId(), channelSubcarrierIndex,
            /*out*/desiredSignalChannelMatrix);

        const ublas::matrix<complex<double> >& precodingMatrix =
            (*aFrame.mimoPrecodingMatricesWithPowerAllocationPtr)[rawSubcarrierIndex];

        const ublas::matrix<complex<double> > desiredSignalChannelMatrixWithPrecoding =
            prod(desiredSignalChannelMatrix, precodingMatrix);

        // MMSE Equalizer calculation.

        ublas::matrix<double>
            diagonalNormalizedNoiseMatrix(numReceiveAntennas, numReceiveAntennas, 0.0);

        for(unsigned int i = 0; (i < numReceiveAntennas); i++) {

            // Normalizing by dividing by received power of desired signal.
            // In "y = Hx + N", (H matrix is defined to not contain bulk pathloss and
            // "x" does not contain TX power (is unit power)). Note that "x" is in
            // signal units, but in MMSE formula the signal part for the noise
            // matrix is squared (which is the same as the power units).

            diagonalNormalizedNoiseMatrix(i,i) =
                (interferenceAndNoiseMwForAntenna(i) / desiredSignalBulkReceivePowerPerSubcarrierMw);

        }//for//

        // "herm()" calculates conjugate transpose.

        const ublas::matrix<complex<double> >
            hermitianOfChannelMatrix = ublas::herm(desiredSignalChannelMatrixWithPrecoding);

        const ublas::matrix<complex<double> >
            covarianceMatrix =
                 (prod(desiredSignalChannelMatrixWithPrecoding, hermitianOfChannelMatrix) +
                  diagonalNormalizedNoiseMatrix);

        ublas::matrix<complex<double> >
            covarianceMatrixInverse(numReceiveAntennas, numReceiveAntennas);

        bool success;
        CalcMatrixInverse(covarianceMatrix, success, covarianceMatrixInverse);

        if (!success) {
            (*this).currentMimoReceiveWeightMatrices[s] =
                ublas::zero_matrix<complex<double> >(
                    hermitianOfChannelMatrix.size1(),
                    covarianceMatrix.size2());
            return;
        }
        else {
            (*this).currentMimoReceiveWeightMatrices[s] =
                prod(hermitianOfChannelMatrix, covarianceMatrixInverse);

            NormalizeWeightMatrix((*this).currentMimoReceiveWeightMatrices[s]);

        }//if//

        // std::cout << "Channel Matrix  = " << desiredSignalChannelMatrix << endl;
        // std::cout << "Precoding Matrix  = " << precodingMatrix << endl;
        // std::cout << "Channel with Precoding Matrix  = " << desiredSignalChannelMatrixWithPrecoding<< endl;
        // std::cout << "Noise Matrix  = " << diagonalNormalizedNoiseMatrix << endl;
        // std::cout << "Conjugate Transpose/Herm Matrix  = " << hermitianOfChannelMatrix << endl;
        // std::cout << "Covariance Matrix = " << covarianceMatrix << endl;
        // std::cout << "Covariance Matrix Inv= " << covarianceMatrixInverse << endl;
        // std::cout << "MMSE Receive Weights = " << currentMimoReceiveWeightMatrix << endl;


        // Calculate info for desired signal.

        const ublas::matrix<complex<double> > channelMatrixWithPreAndPostcoding =
            prod(
                currentMimoReceiveWeightMatrices[s],
                desiredSignalChannelMatrixWithPrecoding);

        (*this).incomingFramePowerMw[s].resize(numIncomingCodewords);

        for(unsigned int l = 0; (l < numIncomingCodewords); l++) {
            // Note: Diagonal only (off-diagonal is interference).
            (*this).incomingFramePowerMw[s][l] =
                (std::norm(channelMatrixWithPreAndPostcoding(l, l)) *
                 desiredSignalBulkReceivePowerPerSubcarrierMw);

        }//for//

        (*this).mimoThermalNoiseMw[s].resize(numIncomingCodewords);

        for(unsigned int l = 0; (l < numIncomingCodewords); l++) {

            // Calculate per stream thermal noise.
            // Add up power from weights from antennas to stream/codeword.

            double total = 0.0;
            for(unsigned int i = 0; (i < numReceiveAntennas); i++) {
                total += std::norm(currentMimoReceiveWeightMatrices[s](l, i));
            }//for//

            (*this).mimoThermalNoiseMw[s][l] = total * thermalNoisePowerPerSubcarrierMw;

        }//for//


        (*this).currentSubcarrierInterferenceLevelsMw[s].resize(numIncomingCodewords);

        std::fill(
            (*this).currentSubcarrierInterferenceLevelsMw[s].begin(),
            (*this).currentSubcarrierInterferenceLevelsMw[s].end(),
            0.0);

        (*this).totalSubcarrierInterferenceEnergyMillijoules[s].resize(numIncomingCodewords);

        std::fill(
            (*this).totalSubcarrierInterferenceEnergyMillijoules[s].begin(),
            (*this).totalSubcarrierInterferenceEnergyMillijoules[s].end(),
            0.0);

        // Calculate "self interference" from other codewords.

        for(unsigned int j = 0; (j < numIncomingCodewords); j++) {
            for(unsigned int l = 0; (l < numIncomingCodewords); l++) {
                if (j != l) {
                    (*this).currentSubcarrierInterferenceLevelsMw[s][l] +=
                        (std::norm(channelMatrixWithPreAndPostcoding(l, j)) *
                         desiredSignalBulkReceivePowerPerSubcarrierMw);
                }//if//
            }//for//
        }//for//

        // Calculate Interference levels with the new MMSE weights for each codeword

        for(unsigned i = 0; (i < interferingSignalList.size()); i++) {

            const InterferingSignalInfo& signalInfo = interferingSignalList[i];

            if (signalInfo.signalEndTime <= currentTime) {
                // Signal ends before or at current time => ignore.
                continue;
            }//if//

            const PropFrame& anInterferringFrame = *signalInfo.framePtr;

            bool mustSaveSignalsInterference = false;

            if (interferingSignalList[i].signalEndTime <= currentIncomingSignalEndTime) {
                // Only save interference additions if necessary.
                interferingSignalList[i].subcarrierInterferenceLevelsMw.resize(numberDataSubcarriers);
                mustSaveSignalsInterference = true;

            }//if//

            if ((signalInfo.subcarrierLocationsMatchIncomingFrame) ||
                (SubcarrierIsUsedBySignal(
                        anInterferringFrame.txParameters,
                        channelNumber,
                        channelSubcarrierIndex))) {

                unsigned int interferingSignalRawSubcarrierIndex = rawSubcarrierIndex;
                if (signalInfo.subcarrierLocationsMatchIncomingFrame) {
                    interferingSignalRawSubcarrierIndex =
                        (((channelNumber - anInterferringFrame.txParameters.firstChannelNumber) *
                          baseChannelBandwidthMhz) + channelSubcarrierIndex);
                }//if//

                vector<double> interferenceForCodewordsMw(numIncomingCodewords);

                CalcSignalsMimoSubcarrierInterference(
                    s,
                    channelNumber,
                    interferingSignalRawSubcarrierIndex,
                    *signalInfo.framePtr,
                    signalInfo.theNodeId,
                    signalInfo.bulkReceivePowerPerDataSubcarrierMw,
                    interferenceForCodewordsMw);

                for(unsigned int l = 0; (l < numIncomingCodewords); l++) {
                    (*this).currentSubcarrierInterferenceLevelsMw[s][l] += interferenceForCodewordsMw[l];
                }//for//

                if (mustSaveSignalsInterference) {
                    interferingSignalList[i].subcarrierInterferenceLevelsMw[s] =
                        interferenceForCodewordsMw;
                }//if//
            }//if//
        }//for//
    }//for//

}//SetupMimoCalculationForReceive//



inline
void Dot11Phy::SetupFrequencySelectiveFadingCalculationForReceive(const IncomingSignal& aSignal)
{
    assert(phyModellingMode == FrequencySelectiveFadingModelling);

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    (*this).currentIncomingSignalFramePtr = aSignal.GetFramePtr();

    (*this).GarbageCollectInterferingSignalList();

    const PropFrame& aFrame = aSignal.GetFrame();

    assert((aFrame.txParameters.bandwidthNumberChannels ==
           currentIncomingSignalTxParameters.bandwidthNumberChannels));

    // Optimization for signals with same subcarrier mappings.

    for(unsigned i = 0; (i < interferingSignalList.size()); i++) {
        InterferingSignalInfo& aSignalInfo = interferingSignalList[i];
        if (aSignalInfo.signalEndTime > currentTime) {
            const PropFrame& aSignalFrame = *aSignalInfo.framePtr;

            aSignalInfo.subcarrierLocationsMatchIncomingFrame =
                SignalSubcarrierLocationsMatch(
                    currentIncomingSignalTxParameters,
                    aSignalFrame.txParameters);
        }//if//
    }//for//

    const unsigned int numberDataSubcarriers =
        CalcNumberOfDataSubcarriers(
            currentIncomingSignalTxParameters.bandwidthNumberChannels,
            currentIncomingSignalTxParameters.isHighThroughputFrame);

    for(unsigned int i = 0; (i < currentBondedChannelList.size()); i++) {
        frequencySelectiveFadingModelInterfacePtr->SetTime((firstChannelNumber + i), currentTime);
    }//for//

    const double desiredSignalBulkReceivePowerPerSubcarrierMw =
        ((ConvertToNonDb(aSignal.GetReceivedPowerDbm()) *
          CalcPilotSubcarriersPowerAdjustmentFactor(currentIncomingSignalTxParameters)) /
         numberDataSubcarriers);

    (*this).incomingFramePowerMw.resize(numberDataSubcarriers);
    (*this).currentSubcarrierInterferenceLevelsMw.resize(numberDataSubcarriers);
    (*this).totalSubcarrierInterferenceEnergyMillijoules.resize(numberDataSubcarriers);

    for(unsigned int s = 0; (s < numberDataSubcarriers); s++) {

        // "Raw" subcarrier index covers all 64 subcarriers (nulls and pilots).

        const unsigned int rawSubcarrierIndex =
            CalcRawSubcarrierFromDataIndex(
                currentIncomingSignalTxParameters.bandwidthNumberChannels,
                currentIncomingSignalTxParameters.isHighThroughputFrame,
                s);

        const unsigned int channelSubcarrierIndex = (rawSubcarrierIndex % numOfdmSubcarriers);

        const unsigned int channelNumber =
            (currentIncomingSignalTxParameters.firstChannelNumber + (rawSubcarrierIndex / numOfdmSubcarriers));

        // Calculate interference level.

        (*this).currentSubcarrierInterferenceLevelsMw[s].resize(1);
        (*this).currentSubcarrierInterferenceLevelsMw[s][0] = 0.0;
        (*this).totalSubcarrierInterferenceEnergyMillijoules[s].resize(1);
        (*this).totalSubcarrierInterferenceEnergyMillijoules[s][0] = 0.0;

        for(unsigned i = 0; (i < interferingSignalList.size()); i++) {
            const InterferingSignalInfo& aSignalInfo = interferingSignalList[i];
            if (aSignalInfo.signalEndTime > currentTime) {

                const PropFrame& aSignalFrame = *aSignalInfo.framePtr;

                if ((aSignalInfo.subcarrierLocationsMatchIncomingFrame) ||
                    (SubcarrierIsUsedBySignal(
                        aSignalFrame.txParameters,
                        channelNumber,
                        channelSubcarrierIndex))) {

                    unsigned int interferingSignalRawSubcarrierIndex = rawSubcarrierIndex;
                    if (!aSignalInfo.subcarrierLocationsMatchIncomingFrame) {
                        interferingSignalRawSubcarrierIndex =
                            (((channelNumber - aSignalFrame.txParameters.firstChannelNumber) *
                              baseChannelBandwidthMhz) + channelSubcarrierIndex);
                    }//if//

                    (*this).currentSubcarrierInterferenceLevelsMw[s][0] +=
                        CalcSignalsFrequencySelectiveFadingSubcarrierInterferenceMw(
                            channelNumber,
                            interferingSignalRawSubcarrierIndex,
                            aSignalInfo.theNodeId,
                            aSignalInfo.bulkReceivePowerPerDataSubcarrierMw);
                }//if//
            }//if//
        }//for//
    }//for//

}//SetupFrequencySelectiveFadingCalculationForReceive//



inline
void Dot11Phy::StartReceivingThisSignal(const IncomingSignal& aSignal)
{
    const PropFrame& incomingFrame = aSignal.GetFrame();

    (*this).currentIncomingSignalStartTime = simulationEngineInterfacePtr->CurrentTime();
    (*this).currentIncomingSignalEndOfPhyHeaderTime =
        (currentIncomingSignalStartTime +
         CalculatePreamblePlusSignalFieldDuration(incomingFrame.txParameters));
    (*this).currentIncomingSignalEndTime = (currentIncomingSignalStartTime + aSignal.GetDuration());
    (*this).currentIncomingSignalSourceNodeId = aSignal.GetSourceNodeId();

    // Saved because "currentIncomingSignalTxParameters" may be modified (below).

    (*this).originalIncomingSignalTxParameters = incomingFrame.txParameters;
    (*this).originalSignalPowerDbm = aSignal.GetReceivedPowerDbm();
    (*this).currentIncomingSignalTxParameters = incomingFrame.txParameters;
    (*this).currentSignalPowerDbm = aSignal.GetReceivedPowerDbm();
    (*this).currentPacketHasAnError = false;

    if (!SignalChannelsMatchForReceive(incomingFrame.txParameters)) {
        (*this).currentPacketHasAnError = true;
    }//if//

    (*this).currentIncomingSignalDatarateBitsPerSec =
        bitsPerSecondDatarateForMcs.at(currentIncomingSignalTxParameters.modulationAndCodingScheme);

    (*this).currentAdjustedSignalPowerMilliwatts =
        (ConvertToNonDb(currentSignalPowerDbm) *
         CalcPilotSubcarriersPowerAdjustmentFactor(
             incomingFrame.txParameters));

    (*this).currentThermalNoisePowerMw =
        (incomingFrame.txParameters.bandwidthNumberChannels *
         thermalNoisePowerPerBaseChannelMw *
         CalcThermalNoiseSubcarrierAdjustmentFactor(incomingFrame.txParameters));

    (*this).totalInterferenceEnergyMillijoules = 0.0;

    // Start interference calculation.

    (*this).lastErrorCalculationUpdateTime = simulationEngineInterfacePtr->CurrentTime();

    (*this).currentLockedOnFramePacketId = incomingFrame.thePacketId;

    if (!currentPacketHasAnError) {
        if (incomingFrame.macFramePtr != nullptr) {
            OutputTraceAndStatsForRxStart(*incomingFrame.macFramePtr);
        }
        else {
            assert(!incomingFrame.aggregateFramePtr->empty());

            OutputTraceAndStatsForRxStart(*incomingFrame.aggregateFramePtr->front());

            (*this).SetupReceiveOfMpduAggregateFrame(incomingFrame);
        }//if//
    }//if//

    if (phyModellingMode == MimoModelling) {

        (*this).SetupMimoCalculationForReceive(aSignal);
    }
    else if (phyModellingMode == FrequencySelectiveFadingModelling) {

        (*this).SetupFrequencySelectiveFadingCalculationForReceive(aSignal);

    }//if//

}//StartReceivingThisSignal//




inline
void Dot11Phy::ProcessSignalArrivalFromChannel(const IncomingSignal& aSignal)
{
    switch (phyState) {
    case PhyScanning:
    case PhyReceiving:
        (*this).ProcessNewSignal(aSignal);

        break;
    case PhyTransmitting:
    case PhyTxStarting:
    case PhyIgnoring:
        (*this).AddSignalToInterferenceLevel(aSignal);

        break;
    default:
        assert(false); abort(); break;
    }//switch//

}//ProcessSignalArrivalFromChannel//


inline
bool Dot11Phy::IsCurrentlyReceivingThisSignal(const IncomingSignal& aSignal) const
{
    assert(phyState == PhyReceiving);
    return (
        (aSignal.GetSourceNodeId() == currentIncomingSignalSourceNodeId) &&
        (aSignal.HasACompleteFrame()));
}



inline
void Dot11Phy::ProcessSignalEndFromChannel(const IncomingSignal& aSignal)
{
    switch (phyState) {
    case PhyReceiving:
        if (IsCurrentlyReceivingThisSignal(aSignal)) {
            (*this).ProcessEndOfTheSignalCurrentlyBeingReceived(aSignal);
        }
        else {
            (*this).SubtractSignalFromInterferenceLevelAndNotifyMacIfMediumIsClear(aSignal);
        }//if//

        break;

    case PhyScanning:
    case PhyTransmitting:
    case PhyTxStarting:
    case PhyIgnoring:

        (*this).SubtractSignalFromInterferenceLevelAndNotifyMacIfMediumIsClear(aSignal);

        break;

    default:
        assert(false); abort(); break;
    }//switch//

}//ProcessSignalEndFromChannel//


inline
void Dot11Phy::StopReceivingFrames()
{
    assert((phyState == PhyScanning) || (phyState == PhyReceiving));

    if (phyState == PhyReceiving) {

        const double signalPowerMw =
            (ConvertToNonDb(originalSignalPowerDbm) *
             CalcSubcarrierPowerAdjustmentFactorForInterferringFrame(originalIncomingSignalTxParameters));

        (*this).AddSignalPowerToInterferenceLevel(
            originalIncomingSignalTxParameters.firstChannelNumber,
            originalIncomingSignalTxParameters.bandwidthNumberChannels,
            signalPowerMw);

        OutputTraceAndStatsForAddSignalToInterferenceLevel(
            currentSignalPowerDbm,
            currentAdjustedSignalPowerMilliwatts,
            currentIncomingSignalSourceNodeId,
            currentLockedOnFramePacketId);

        if ((phyModellingMode == MimoModelling) ||
            (phyModellingMode == FrequencySelectiveFadingModelling)) {

            (*this).MoveCurrentIncomingSignalToMimoInterferingSignalList();

        }//if//

        (*this).currentSignalPowerDbm = -DBL_MAX;
        (*this).currentAdjustedSignalPowerMilliwatts = 0.0;
        (*this).currentIncomingSignalSourceNodeId = 0;
        (*this).currentLockedOnFramePacketId = PacketId::nullPacketId;
        (*this).currentThermalNoisePowerMw = defaultThermalNoisePowerMw;
        (*this).totalInterferenceEnergyMillijoules = 0.0;
    }//if//

    (*this).phyState = PhyIgnoring;
    (*this).ignoreIncomingFrames = true;

}//StopReceivingFrames//


inline
void Dot11Phy::StartReceivingFrames()
{
    assert(phyState == PhyIgnoring);
    (*this).phyState = PhyScanning;
    (*this).ignoreIncomingFrames = false;
}



inline
void Dot11Phy::SwitchToChannels(const vector<unsigned int>& bondedChannelList)
{
    assert(!bondedChannelList.empty());

    if (currentBondedChannelList == bondedChannelList) {
        return;
    }

    (*this).currentBondedChannelList = bondedChannelList;

    vector<unsigned int> sortedBondedChannelList = bondedChannelList;
    std::sort(sortedBondedChannelList.begin(), sortedBondedChannelList.end());
    (*this).firstChannelNumber = sortedBondedChannelList[0];

    for(unsigned int i = 1; (i < sortedBondedChannelList.size()); i++) {
        assert((sortedBondedChannelList[i] == (firstChannelNumber + i))
               && "Dot11 bonded channel list is not continguous");

        const unsigned int channelBandwidthMhz =
            RoundToUint(
                propagationModelInterfacePtr->GetChannelBandwidthMhz(sortedBondedChannelList[i]));

        assert((channelBandwidthMhz == baseChannelBandwidthMhz) &&
               "Bonded channel bandwidths must be same as the Base channel bandwidth.");

    }//for//

    (*this).currentSignalPowerDbm = -DBL_MAX;
    (*this).currentAdjustedSignalPowerMilliwatts = 0.0;

    // May change in channel width.

    (*this).defaultThermalNoisePowerMw =
        thermalNoisePowerPerBaseChannelMw * bondedChannelList.size() *
        CalcDefaultThermalNoiseSubcarrierAdjustmentFactor(
            static_cast<unsigned int>(bondedChannelList.size()));

    (*this).currentThermalNoisePowerMw = defaultThermalNoisePowerMw;
    (*this).currentIncomingSignalDatarateBitsPerSec = 0;
    (*this).currentIncomingSignalSourceNodeId = 0;
    (*this).currentLockedOnFramePacketId = PacketId::nullPacketId;
    (*this).totalInterferenceEnergyMillijoules = 0.0;
    (*this).currentInterferencePowers.clear();
    (*this).currentInterferencePowers.resize(bondedChannelList.size(), IntegralPower(0.0));
    (*this).currentlySensingBusyMedium = false;

    switch (phyState) {
    case PhyTransmitting:
    case PhyTxStarting:
    case PhyIgnoring:

        assert(false && "Should be prevented from occuring"); abort();
        break;

    case PhyReceiving:
    case PhyScanning:

        phyState = PhyScanning;

        propagationModelInterfacePtr->SwitchToChannelNumbers(sortedBondedChannelList);

        break;

    default:
        assert(false); abort(); break;
    }//switch//

}//SwitchToChannels//




inline
void Dot11Phy::ProcessStatsForEndCurrentTransmission()
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    (*this).totalTransmissionTime += (currentTime - lastChannelStateTransitionTime);
    (*this).lastChannelStateTransitionTime = currentTime;
}


inline
void Dot11Phy::EndCurrentTransmission()
{
    assert(phyState == PhyTransmitting);

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (!ignoreIncomingFrames) {
        phyState = PhyScanning;
    }
    else {
        phyState = PhyIgnoring;
    }//if

    (*this).ProcessStatsForEndCurrentTransmission();

    if (currentSectorId != currentReceiveSectorId) {
        (*this).SwitchToDirectionalAntennaMode(currentReceiveSectorId, true/*setRxPattern*/);
    }//if//

    macLayerPtr->TransmissionIsCompleteNotification();

    // Note that the MAC may (theoretically) have issued a transmit command
    // in the immediately previous statement (TransmissionIsCompleteNotification call).

    if (((phyState == PhyScanning) || (phyState == PhyIgnoring)) &&
        ((currentTime >= currentLastCarrierDetectableSignalEnd) &&
         (!CurrentInterferencePowerIsAboveEnergyDetectionThreshold()))) {

        (*this).currentlySensingBusyMedium = false;

        if (phyState != PhyIgnoring) {
            macLayerPtr->ClearChannelAtPhysicalLayerNotification();
        }//if//

    }//if//

}//EndCurrentTransmission//



inline
void Dot11Phy::ProcessStatsForTransitionToBusyChannel()
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    (*this).totalIdleChannelTime += (currentTime - lastChannelStateTransitionTime);
    (*this).lastChannelStateTransitionTime = currentTime;
}

inline
void Dot11Phy::ProcessStatsForTransitionToIdleChannel()
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    (*this).totalBusyChannelTime += (currentTime - lastChannelStateTransitionTime);
    (*this).lastChannelStateTransitionTime = currentTime;
}


inline
void Dot11Phy::CheckForFrameCaptureInMimoMode(
    const IncomingSignal& aSignal,
    bool& captureTheSignal)
{
    captureTheSignal = false;

}//CheckForFrameCaptureInMimoMode//



inline
void Dot11Phy::CalcIfPhyHeaderHasBeenReceived(bool& phyHeaderHasBeenReceived)
{
    if (!preambleDetectionProbBySinrTable.HasData()) {
        phyHeaderHasBeenReceived = true;
        return;
    }//if//

    const unsigned int numberChannels = currentIncomingSignalTxParameters.bandwidthNumberChannels;

    const double thermalNoisePowerMw =
        (numberChannels * thermalNoisePowerPerBaseChannelMw *
         CalcThermalNoiseSubcarrierAdjustmentFactor(currentIncomingSignalTxParameters));

    const double totalNoiseAndInterferencePowerMw =
        (thermalNoisePowerMw +
         CalcCurrentInterferencePowerMw(
             currentIncomingSignalTxParameters.firstChannelNumber,
             numberChannels));

    const double signalToNoiseAndInterferenceRatio =
        (ConvertToNonDb(currentSignalPowerDbm) / totalNoiseAndInterferencePowerMw);

    const double probabilityOfDetection =
        preambleDetectionProbBySinrTable.CalcValue(signalToNoiseAndInterferenceRatio);

    if (probabilityOfDetection == 0.0) {
        phyHeaderHasBeenReceived = false;
    }
    else if (probabilityOfDetection == 1.0) {
        phyHeaderHasBeenReceived = true;
    }
    else {
        const double randomNumber = (*this).aRandomNumberGenerator.GenerateRandomDouble();

        phyHeaderHasBeenReceived = (randomNumber < probabilityOfDetection);
    }//if//

}//CalcIfPhyHeaderHasBeenReceived//




inline
void Dot11Phy::ProcessNewSignal(const IncomingSignal& aSignal)
{
    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (phyState == PhyScanning) {

        bool preambleHasBeenDetected = false;

        if (aSignal.HasACompleteFrame()) {

            const PropFrame& incomingFrame = aSignal.GetFrame();
            const unsigned int incomingFrameFirstChannelNumber = incomingFrame.txParameters.firstChannelNumber;
            const unsigned int numberChannels = incomingFrame.txParameters.bandwidthNumberChannels;

            assert(SignalOverlapsCurrentChannels(incomingFrameFirstChannelNumber, numberChannels));

            if (!SignalOverlapsCurrentChannels(incomingFrameFirstChannelNumber, numberChannels)) {
                return;
            }//if//

            // Legacy Preamble detection (sent on every channel).

            const double bandwidthNormalizedReceivedPowerDbm =
                aSignal.GetReceivedPowerDbm() - ConvertIntToDb(numberChannels);

            if (bandwidthNormalizedReceivedPowerDbm >= preambleDetectionPowerThresholdDbm) {
                preambleHasBeenDetected = true;
            }//if//
        }//if//

        if (!preambleHasBeenDetected) {

            (*this).AddSignalToInterferenceLevel(aSignal);

            if ((!currentlySensingBusyMedium) &&
                ((currentTime < currentLastCarrierDetectableSignalEnd) ||
                 (CurrentInterferencePowerIsAboveEnergyDetectionThreshold()))) {

                (*this).currentlySensingBusyMedium = true;
                (*this).ProcessStatsForTransitionToBusyChannel();

                macLayerPtr->BusyChannelAtPhysicalLayerNotification();

            }//if//
        }
        else {
            (*this).phyState = PhyReceiving;
            (*this).tentativePhyHeaderReceiveHasFailed = false;

            (*this).StartReceivingThisSignal(aSignal);

            if (!currentlySensingBusyMedium) {

                (*this).currentlySensingBusyMedium = true;
                (*this).ProcessStatsForTransitionToBusyChannel();

                macLayerPtr->BusyChannelAtPhysicalLayerNotification();

            }//if//

            bool tentativePhyHeaderIsGood = false;

            // Frame can already be in error state when signal and STA channels and bandwidth
            // do not match.

            if (!currentPacketHasAnError) {
                (*this).CalcIfPhyHeaderHasBeenReceived(/*out*/tentativePhyHeaderIsGood);
            }//if//}

            if (!tentativePhyHeaderIsGood) {

                assert(endPhyHeaderEventTicket.IsNull());

                (*this).tentativePhyHeaderReceiveHasFailed = true;

                const PropFrame& incomingFrame = aSignal.GetFrame();

                const SimTime endPhyHeaderTime =
                    (currentTime +
                     CalculatePreamblePlusSignalFieldDuration(currentIncomingSignalTxParameters));

                simulationEngineInterfacePtr->ScheduleEvent(
                    endPhyHeaderEventPtr,
                    endPhyHeaderTime,
                    endPhyHeaderEventTicket);
            }//if//
        }//if//
    }
    else if ((phyState == PhyReceiving) && (aSignal.HasACompleteFrame())) {

        bool captureTheSignal = false;
        bool captureIsInShortTrainingField = false;

        const bool isInPreambleOrPhyHeader = (currentTime < currentIncomingSignalEndOfPhyHeaderTime);

        if (isInPreambleOrPhyHeader) {

            const SimTime timeSinceStartOfSignal = (currentTime - currentIncomingSignalStartTime);

            if ((timeSinceStartOfSignal < shortTrainingFieldDuration) &&
                (currentSignalPowerDbm < aSignal.GetReceivedPowerDbm())) {

                // New signal is more powerful (in STF preamble) go to new signal.

                captureTheSignal = true;
                captureIsInShortTrainingField = true;
            }
            else {

                // Tentative error is now invalid, must recalc at end of header.

                (*this).tentativePhyHeaderReceiveHasFailed = false;

                if ((preambleDetectionProbBySinrTable.HasData()) &&
                    (endPhyHeaderEventTicket.IsNull())) {

                    const SimTime endPhyHeaderTime =
                        (currentIncomingSignalStartTime +
                         CalculatePreamblePlusSignalFieldDuration(currentIncomingSignalTxParameters));

                    simulationEngineInterfacePtr->ScheduleEvent(
                        endPhyHeaderEventPtr,
                        endPhyHeaderTime,
                        endPhyHeaderEventTicket);
                }//if//
            }//if//
        }//if//

        const bool isInLtfPreambleOrPhyHeader = !endPhyHeaderEventTicket.IsNull();

        if ((!captureTheSignal) && (!isInPreambleOrPhyHeader)) {
            if (phyModellingMode != MimoModelling) {
                captureTheSignal =
                    (aSignal.GetReceivedPowerDbm() > (currentSignalPowerDbm + signalCaptureRatioThresholdDb));
            }
            else {
                if ((aSignal.GetReceivedPowerDbm() + mimoMaxGainDbForCaptureCalcBypass) >
                    (currentSignalPowerDbm + signalCaptureRatioThresholdDb)) {

                    // Above condition filters very weak signals to bypass expensive MIMO calculation.

                    (*this).CheckForFrameCaptureInMimoMode(aSignal, captureTheSignal);

                }//if//
            }//if//
        }//if//

        if (captureTheSignal) {

            (*this).currentPacketHasAnError = true;

            OutputTraceAndStatsForRxEnd(
                currentLockedOnFramePacketId,
                true/*Capture*/,
                captureIsInShortTrainingField);

            // Assuming Pilot energy doesn't interfere.

            // Calculate "bulk" interference value (pathloss only, no MIMO channel) even in MIMO
            // mode for comparison statistics.

            const double signalPowerMw =
                (ConvertToNonDb(originalSignalPowerDbm) *
                 CalcSubcarrierPowerAdjustmentFactorForInterferringFrame(
                     originalIncomingSignalTxParameters));

            (*this).AddSignalPowerToInterferenceLevel(
                originalIncomingSignalTxParameters.firstChannelNumber,
                originalIncomingSignalTxParameters.bandwidthNumberChannels,
                signalPowerMw);

            OutputTraceAndStatsForAddSignalToInterferenceLevel(
                currentSignalPowerDbm,
                signalPowerMw,
                currentIncomingSignalSourceNodeId,
                currentLockedOnFramePacketId);

            if ((phyModellingMode == MimoModelling) ||
                (phyModellingMode == FrequencySelectiveFadingModelling)) {

                (*this).MoveCurrentIncomingSignalToMimoInterferingSignalList();

            }//if//

            // Clear preempted aggregate frame if it exists.

            (*this).aggregateFrameSubframeInfo.clear();

            if (!aggregatedMpduFrameEndEventTicket.IsNull()) {
                simulationEngineInterfacePtr->CancelEvent(aggregatedMpduFrameEndEventTicket);
            }//if//

            (*this).StartReceivingThisSignal(aSignal);
        }
        else {
            (*this).AddSignalToInterferenceLevel(aSignal);

        }//if//
    }
    else {

        // Present signal keeps going

        (*this).AddSignalToInterferenceLevel(aSignal);
    }//if//

}//ProcessNewSignal//



inline
void Dot11Phy::ProcessEndOfTheSignalCurrentlyBeingReceived(const IncomingSignal& aSignal)
{
    const PropFrame& incomingFrame = aSignal.GetFrame();

    assert(phyState == PhyReceiving);

    (*this).UpdatePacketReceptionCalculation();

    if (incomingFrame.macFramePtr != nullptr) {

        assert(incomingFrame.aggregateFramePtr == nullptr);

        if ((artificialFrameDropProbability != 0.0) && (!currentPacketHasAnError)) {
            // For testing only:

            (*this).currentPacketHasAnError =
                (artificialFrameDropRandomNumberGenerator.GenerateRandomDouble() <
                 artificialFrameDropProbability);
        }//if//

        OutputTraceAndStatsForRxEnd(currentLockedOnFramePacketId, false/*No Capture*/, false);

        (*this).phyState = PhyScanning;

        if (!currentPacketHasAnError) {

            (*this).lastReceivedPacketRssiDbm = currentSignalPowerDbm;
            (*this).lastReceivedPacketStartTime = aSignal.GetStartTime();
            (*this).lastReceivedPacketDuration = aSignal.GetDuration();

            // Use original incoming frame TX Parameters here.

            macLayerPtr->ReceiveFrameFromPhy(*incomingFrame.macFramePtr, incomingFrame.txParameters);
        }
        else {
            macLayerPtr->NotifyThatPhyReceivedCorruptedFrame();

        }//if//
    }
    else {
        (*this).phyState = PhyScanning;

        // Check if aggregate frame has been "pre-screened" by Phy.

        if (!aggregateFrameSubframeInfo.empty()) {
            assert(currentAggregateFrameSubframeIndex == (aggregateFrameSubframeInfo.size()-1));

            AggregateFrameSubframeInfoElement& subframeInfo =
                (*this).aggregateFrameSubframeInfo[currentAggregateFrameSubframeIndex];


            if ((artificialSubframeDropProbability != 0.0) && (!currentPacketHasAnError)) {
                // For testing only:

                (*this).currentPacketHasAnError =
                    (artificialFrameDropRandomNumberGenerator.GenerateRandomDouble() <
                     artificialSubframeDropProbability);
            }//if//

            OutputTraceAndStatsForRxEnd(subframeInfo.macFramePtr->GetPacketId(), false/*No Capture*/, false);

            subframeInfo.hasError = currentPacketHasAnError;

            if (!currentPacketHasAnError) {

                (*this).lastReceivedPacketRssiDbm = currentSignalPowerDbm;

                macLayerPtr->ReceiveAggregatedSubframeFromPhy(
                    subframeInfo.macFramePtr,
                    currentIncomingSignalTxParameters,
                    currentAggregateFrameSubframeIndex,
                    static_cast<unsigned int>(aggregateFrameSubframeInfo.size()));
            }
            else {
                macLayerPtr->NotifyThatPhyReceivedCorruptedAggregatedSubframe(
                    currentIncomingSignalTxParameters,
                    currentAggregateFrameSubframeIndex,
                    static_cast<unsigned int>(aggregateFrameSubframeInfo.size()));

                subframeInfo.macFramePtr.reset();

            }//if//

            aggregateFrameSubframeInfo.clear();
        }
        else {
            // Optimimistic assumption: At least one subframe header was received.
            // Used only for setting NAV.

            macLayerPtr->ReceiveFrameFromPhy(
                (*notForMeHeaderOnlyFramePtr),
                currentIncomingSignalTxParameters);
        }//if//
    }//if//

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    // Note that the MAC may have issued a transmit command in the immediately
    // previous statement (ReceiveFrameFromPhy call).

    if (((phyState == PhyScanning) || (phyState == PhyIgnoring)) &&
        ((currentTime >= currentLastCarrierDetectableSignalEnd) &&
         (!CurrentInterferencePowerIsAboveEnergyDetectionThreshold()))) {

        assert(currentlySensingBusyMedium);
        (*this).currentlySensingBusyMedium = false;
        (*this).ProcessStatsForTransitionToIdleChannel();

        if (phyState != PhyIgnoring) {
            macLayerPtr->ClearChannelAtPhysicalLayerNotification();
        }//if//
    }//if//

    (*this).currentSignalPowerDbm = -DBL_MAX;
    (*this).currentAdjustedSignalPowerMilliwatts = 0.0;

    (*this).currentIncomingSignalSourceNodeId = 0;
    (*this).currentLockedOnFramePacketId = PacketId::nullPacketId;

}//ProcessEndOfTheSignalCurrentlyBeingReceived//



inline
void Dot11Phy::ProcessEndPhyHeaderEvent()
{
    endPhyHeaderEventTicket.Clear();

    assert(phyState == PhyReceiving);

    bool phyHeaderHasBeenReceived = false;
    if (!tentativePhyHeaderReceiveHasFailed) {
        (*this).CalcIfPhyHeaderHasBeenReceived(/*out*/phyHeaderHasBeenReceived);
    }//if//

    if (!phyHeaderHasBeenReceived) {

        assert(currentlySensingBusyMedium);

        (*this).StopReceivingDueToPhyHeaderFailure();
    }//if//

    (*this).tentativePhyHeaderReceiveHasFailed = false;

}//ProcessEndPhyHeaderEvent//





inline
void Dot11Phy::ProcessAggregatedMpduFrameEndEvent()
{
    assert(phyState == PhyReceiving);

    (*this).aggregatedMpduFrameEndEventTicket.Clear();

    (*this).UpdatePacketReceptionCalculation();

    if ((artificialSubframeDropProbability != 0.0) && (!currentPacketHasAnError)) {
        // For testing only:

        (*this).currentPacketHasAnError =
            (artificialFrameDropRandomNumberGenerator.GenerateRandomDouble() <
             artificialSubframeDropProbability);
    }//if//


    AggregateFrameSubframeInfoElement& subframeInfo =
        (*this).aggregateFrameSubframeInfo[currentAggregateFrameSubframeIndex];

    OutputTraceAndStatsForRxEnd(subframeInfo.macFramePtr->GetPacketId(), false/*No Capture*/, false);

    subframeInfo.hasError = currentPacketHasAnError;

    if (!currentPacketHasAnError) {

        (*this).lastReceivedPacketRssiDbm = currentSignalPowerDbm;

        // Check for pre-filtering.
        if (subframeInfo.macFramePtr != nullptr) {
            macLayerPtr->ReceiveAggregatedSubframeFromPhy(
                subframeInfo.macFramePtr,
                currentIncomingSignalTxParameters,
                currentAggregateFrameSubframeIndex,
                static_cast<unsigned int>(aggregateFrameSubframeInfo.size()));
        }//if//
    }
    else {
        macLayerPtr->NotifyThatPhyReceivedCorruptedAggregatedSubframe(
            currentIncomingSignalTxParameters,
            currentAggregateFrameSubframeIndex,
            static_cast<unsigned int>(aggregateFrameSubframeInfo.size()));

        subframeInfo.macFramePtr.reset();
    }//if//

    // Reset for next MPDU subframe.  Assumes perfect MPDU delimiter acquisition.

    (*this).currentPacketHasAnError = false;

    (*this).currentAggregateFrameSubframeIndex++;

    // Rely on end of signal event for final subframe.

    if (currentAggregateFrameSubframeIndex < (aggregateFrameSubframeInfo.size() - 1)) {

        const AggregateFrameSubframeInfoElement& nextSubframeInfo =
            aggregateFrameSubframeInfo[currentAggregateFrameSubframeIndex];

        const SimTime subframeEndTime =
            (simulationEngineInterfacePtr->CurrentTime() +
             CalculateFrameDataDuration(
                 nextSubframeInfo.macFramePtr->LengthBytes(),
                 currentIncomingSignalTxParameters));

        simulationEngineInterfacePtr->ScheduleEvent(
            aggregatedMpduFrameEndEventPtr,
            subframeEndTime,
            aggregatedMpduFrameEndEventTicket);
    }//if//

}//ProcessAggregatedMpduFrameEndEvent//



//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


inline
void Dot11Phy::OutputTraceAndStatsForAddSignalToInterferenceLevel(
    const double& signalPowerDbm,
    const double& adjustedSignalPowerMw,
    const NodeId& signalSourceNodeId,
    const PacketId& signalPacketId) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TracePhyInterference)) {

        const double currentInterferencePowerMw = CalcCurrentInterferencePowerMw();

        const double currentInterferenceAndNoisePowerDbm =
            ConvertToDb(currentThermalNoisePowerMw + currentInterferencePowerMw);

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {
            NoiseStartTraceRecord traceData;

            traceData.sourceNodeId = signalSourceNodeId;
            traceData.rxPower = ConvertToDb(adjustedSignalPowerMw);
            traceData.interferenceAndNoisePower = currentInterferenceAndNoisePowerDbm;

            assert(!redundantTraceInformationModeIsOn);// no implementation for redundant trace

            assert(sizeof(traceData) == NOISE_START_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName,
                                                              theInterfaceId,
                                                              "NoiseStart",
                                                              traceData);
        }
        else {
            ostringstream msgStream;
            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "SrcN= " << signalSourceNodeId
                      << " RxPow= " << ConvertToDb(adjustedSignalPowerMw)
                      << " I&NPow= " << currentInterferenceAndNoisePowerDbm;

            if (redundantTraceInformationModeIsOn) {
                msgStream << " PktId= " << signalPacketId;
                if (signalPacketId != currentLockedOnFramePacketId) {
                    msgStream << " LockedOnPacketId= " << currentLockedOnFramePacketId;
                }
                else {
                    // Preempted Frame Receive output null packet Id.
                    msgStream << " LockedOnPacketId= " << PacketId::nullPacketId;
                }//if//
            }//if//

            simulationEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "NoiseStart", msgStream.str());
        }//if//
    }//if//

    interferingSignalsStatPtr->IncrementCounter();

    if (signalPowerDbm < preambleDetectionPowerThresholdDbm) {

        weakSignalsStatPtr->IncrementCounter();
    }//if//


    if (((phyState == PhyTxStarting) || (phyState == PhyTransmitting)) &&
        (signalPowerDbm >= preambleDetectionPowerThresholdDbm)) {

        // Count all signals that the PHY could lock on to staring during a transmission.

        signalsDuringTransmissionStatPtr->IncrementCounter();

    }//if//

}//OutputTraceForAddSignalToInterferenceLevel//



inline
void Dot11Phy::OutputTraceForSubtractSignalFromInterferenceLevel(
    const IncomingSignal& aSignal,
    const double& signalPowerMw) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TracePhyInterference)) {

        const double currentInterferencePowerMw = CalcCurrentInterferencePowerMw();

        const double currentInterferenceAndNoisePowerDbm =
            ConvertToDb(currentThermalNoisePowerMw + currentInterferencePowerMw);

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            NoiseEndTraceRecord traceData;

            traceData.rxPower = ConvertToDb(signalPowerMw);
            traceData.interferenceAndNoisePower = currentInterferenceAndNoisePowerDbm;

            assert(!redundantTraceInformationModeIsOn);// no implementation for redundant trace

            assert(sizeof(traceData) == NOISE_END_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName,
                                                              theInterfaceId,
                                                              "NoiseEnd",
                                                              traceData);
        }
        else {
            ostringstream msgStream;
            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "RxPow= " << ConvertToDb(signalPowerMw)
                      << " I&NPow= " << currentInterferenceAndNoisePowerDbm;

            if (redundantTraceInformationModeIsOn) {
                msgStream << " PktId= " << aSignal.GetFrame().thePacketId
                          << " LockedOnPacketId= " << currentLockedOnFramePacketId;
            }//if//

            simulationEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "NoiseEnd", msgStream.str());
        }//if//
    }//if//

}//OutputTraceForSubtractSignalFromInterferenceLevel//


inline
void Dot11Phy::OutputTraceAndStatsForTxStart(
    const Packet& aPacket,
    const double& txPowerDbm,
    const TransmissionParameters& txParameters,
    const SimTime& duration) const
{
    // Future idea: More detailed output from txParameters data.

    if (simulationEngineInterfacePtr->TraceIsOn(TracePhy)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            TxStartTraceRecord traceData;
            const PacketId& thePacketId = aPacket.GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            traceData.txPower = txPowerDbm;
            traceData.dataRate =
                bitsPerSecondDatarateForMcs.at(txParameters.modulationAndCodingScheme);
            traceData.duration = duration;

            assert(sizeof(traceData) == TX_START_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theInterfaceId, "TxStart", traceData);
        }
        else {
            ostringstream msgStream;
            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "PktId= " << aPacket.GetPacketId()
                      << " TxPow= " << txPowerDbm
                      << " Rate= " << bitsPerSecondDatarateForMcs.at(txParameters.modulationAndCodingScheme)
                      << " Dur= " << ConvertTimeToStringSecs(duration);

            simulationEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "TxStart", msgStream.str());
        }//if//
    }//if//

    transmittedFramesStatPtr->IncrementCounter();

}//OutputTraceForTxStart//

inline
void Dot11Phy::OutputTraceAndStatsForRxStart(const Packet& aPacket)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TracePhy)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            RxStartTraceRecord traceData;
            const PacketId& thePacketId = aPacket.GetPacketId();
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            traceData.rxPower = currentSignalPowerDbm;

            assert(!redundantTraceInformationModeIsOn); // no implementation for redundant trace

            assert(sizeof(traceData) == RX_START_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theInterfaceId, "RxStart", traceData);

        }
        else {
            ostringstream msgStream;
            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "PktId= " << aPacket.GetPacketId() << " RxPow= " << currentSignalPowerDbm;

            if (redundantTraceInformationModeIsOn) {
                const double currentInterferencePowerMw = CalcCurrentInterferencePowerMw();
                const double currentInterferenceAndNoisePowerDbm =
                    ConvertToDb(currentThermalNoisePowerMw + currentInterferencePowerMw);

                msgStream << " I&NPow= " << currentInterferenceAndNoisePowerDbm;
            }//if//

            simulationEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "RxStart", msgStream.str());

        }//if//

    }//if//

}//OutputTraceForRxStart//

inline
void Dot11Phy::OutputTraceAndStatsForRxEnd(
    const PacketId& thePacketId,
    const bool& receiveIsEndedByCapture,
    const bool& capturedInShortTrainingField)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TracePhy)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            RxEndTraceRecord traceData;
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            traceData.error = currentPacketHasAnError;

            if (receiveIsEndedByCapture) {
                if (capturedInShortTrainingField) {
                    traceData.captureType = TRACE_SIGNAL_CAPTURE_IN_SHORT_TRAINING_FIELD;
                }
                else {
                    traceData.captureType = TRACE_SIGNAL_CAPTURE;
                }//if//
            }
            else {
                traceData.captureType = TRACE_NO_SIGNAL_CAPTURE;
            }//if//

            if (!currentPacketHasAnError) {
                assert(!receiveIsEndedByCapture);
            }

            assert(!redundantTraceInformationModeIsOn); // no implementation for redundant trace

            assert(sizeof(traceData) == RX_END_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, theInterfaceId, "RxEnd", traceData);
        }
        else {

            ostringstream msgStream;

            msgStream.precision(tracePrecisionDigitsForDbm);

            msgStream << "PktId= " << thePacketId;

            if (currentPacketHasAnError) {
                if (receiveIsEndedByCapture) {
                    if (capturedInShortTrainingField) {
                        msgStream << " Err= YesStfCapture";
                    }
                    else {
                        msgStream << " Err= YesCapture";
                    }//if/
                }
                else {
                    msgStream << " Err= Yes";

                }//if//
            }
            else {
                assert(!receiveIsEndedByCapture);
                msgStream << " Err= No";
            }//if//

            if (redundantTraceInformationModeIsOn) {
                const double currentInterferencePowerMw = CalcCurrentInterferencePowerMw();
                const double currentInterferenceAndNoisePowerDbm =
                    ConvertToDb(currentThermalNoisePowerMw + currentInterferencePowerMw);

                msgStream << " I&NPow= " << currentInterferenceAndNoisePowerDbm;
            }//if//

            simulationEngineInterfacePtr->OutputTrace(modelName, theInterfaceId, "RxEnd", msgStream.str());

        }//if//
    }//if//

    if (!receiveIsEndedByCapture) {

        receivedFrameRssiMwStatPtr->RecordStatValue(ConvertToNonDb(currentSignalPowerDbm));

        const SimTime signalDuration =
            (currentIncomingSignalEndTime - currentIncomingSignalStartTime);

        const double meanInterferencePowerMw =
            (totalInterferenceEnergyMillijoules / ConvertTimeToDoubleSecs(signalDuration));

        const double interferenceAndNoisePowerMw =
            (currentThermalNoisePowerMw + meanInterferencePowerMw);

        const double signalToNoiseAndInterferenceRatio =
            (currentAdjustedSignalPowerMilliwatts / interferenceAndNoisePowerMw);

        receivedFrameSinrStatPtr->RecordStatValue(signalToNoiseAndInterferenceRatio);

        (*this).lastReceivedPacketSinrDb = ConvertToDb(signalToNoiseAndInterferenceRatio);

        if (phyModellingMode != SimpleModelling) {

            receivedFrameEffectiveSinrStatPtr->RecordStatValue(
                ConvertToNonDb(CalcMeanEffectiveSignalToInterferenceAndNoiseRatioDb(signalDuration)));

        }//if//

        if (!currentPacketHasAnError) {
            (*this).numberOfReceivedFrames++;
            receivedFramesStatPtr->UpdateCounter(numberOfReceivedFrames);
        }
        else {
            (*this).numberOfFramesWithErrors++;
            framesWithErrorsStatPtr->UpdateCounter(numberOfFramesWithErrors);
        }//if//
    }
    else {
        if (!capturedInShortTrainingField) {

            // Not counting STF preamble capture as "real" captures.

            (*this).numberOfSignalCaptures++;
            signalCaptureStatPtr->UpdateCounter(numberOfSignalCaptures);
        }//if//
    }//if//

}//OutputTraceAndStatsForRxEnd//



inline
DatarateBitsPerSec Dot11Phy::CalculateDatarateBitsPerSecond(
    const TransmissionParameters& txParameters) const
{
    return bitsPerSecondDatarateForMcs.at(txParameters.modulationAndCodingScheme);
}//CalcDatarateBitsPerSecond//

//-------------------------------------------------------------------------------------------------



inline
void ChooseAndCreateChannelModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceOrInstanceId& propagationModeInstanceId,
    const unsigned int baseChannelNumber,
    const unsigned int numberChannels,
    const vector<double>& channelCarrierFrequenciesMhz,
    const vector<double>& channelBandwidthsMhz,
    const RandomNumberGeneratorSeed& runSeed,
    shared_ptr<MimoChannelModel>& mimoChannelModelPtr,
    shared_ptr<FrequencySelectiveFadingModel>& frequencySelectiveFadingModelPtr)
{
    mimoChannelModelPtr.reset();

    if (theParameterDatabaseReader.ParameterExists(parameterNamePrefix + "channel-model")) {
        string modelName =
            theParameterDatabaseReader.ReadString(parameterNamePrefix + "channel-model");

        ConvertStringToLowerCase(modelName);

        if (modelName == "mimo") {

            mimoChannelModelPtr =
                make_shared<ScenSim::CannedFileMimoChannelModel>(
                    theParameterDatabaseReader,
                    propagationModeInstanceId,
                    baseChannelNumber,
                    numberChannels);
        }
        else if (modelName == "tgnmimo") {

            mimoChannelModelPtr =
                ScenSim::OnDemandTgnMimoChannelModel::Create(
                    theParameterDatabaseReader,
                    propagationModeInstanceId,
                    baseChannelNumber,
                    numberChannels,
                    channelCarrierFrequenciesMhz,
                    channelBandwidthsMhz,
                    numOfdmSubcarriers,
                    runSeed);
        }
        else if (modelName == "freqsel") {
            frequencySelectiveFadingModelPtr =
                make_shared<ScenSim::CannedFileFrequencySelectiveFadingModel>(
                    theParameterDatabaseReader,
                    propagationModeInstanceId,
                    baseChannelNumber,
                    numberChannels);
        }
        else {
            cerr << "Error in \"" << parameterNamePrefix
                 << "channel-model\" parameter: Unknown channel model: "
                 << modelName << "." << endl;
            exit(1);
        }//if//
    }//if//

}//ChooseAndCreateChannelModel//


//--------------------------------------------------------------------------------------------------


}//namespace//

#endif
