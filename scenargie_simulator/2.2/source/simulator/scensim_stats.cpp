// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <utility>
#include "scensim_support.h"
#include "scensim_stats.h"
#include "scensim_engine.h"
#include "scensim_gui_interface_constants.h"

namespace ScenSim {

using std::map;
using std::cerr;
using std::endl;
using std::ostream;
using std::ifstream;
using std::ofstream;
using std::istringstream;
using std::vector;
using std::setw;
using std::make_pair;

class CounterStatistic::Implementation {
public:
    ~Implementation() {}
private:
    friend class RuntimeStatisticsSystem;
    friend class CounterStatistic;

    Implementation(const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr)
        : simEngineInterfacePtr(initSimEngineInterfacePtr), lastCounterValue(0)
        {}

    long long int lastCounterValue;

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    vector<shared_ptr<CounterStatView> > statViews;

};//CounterStatistic::Implementation//


class CounterStatViewIterator::Implementation {
public:
    ~Implementation();
private:
    friend class CounterStatViewIterator;
    friend class CounterStatView;

    Implementation() : currentIndex(0) { }

    shared_ptr<const CounterStatView> statViewPtr;
    size_t currentIndex;

};//CounterStatViewIterator//



class CounterStatView::Implementation {
public:
    ~Implementation() { }
private:
    friend class CounterStatView;
    friend class CounterStatistic;
    friend class CounterStatViewIterator;
    friend class RuntimeStatisticsSystem;

    Implementation();

    void IncrementCounter(const SimTime& currentTime, const long long int incrementNumber);

    // Member variables:

    NodeId theNodeId;
    string statName;

    SimTime startTime;
    SimTime endTime;

    SimTime timeBinDuration;
    vector<long long int> binnedStatValues;
    SimTime currentBinEndTime;


    struct TimeAndValueRecord {
        SimTime time;
        long long int value;
        long long int totalCount;
        TimeAndValueRecord(const SimTime& theTime, const long long int theValue)
            : time(theTime), value(theValue), totalCount(0) { }
    };

    vector<TimeAndValueRecord> binlessStatValues;
    SimTime lastTime;
    long long int lastTotalCount;

};//CounterStatView::Implementation//


CounterStatView::Implementation::Implementation()
    :
    startTime(ZERO_TIME),
    endTime(INFINITE_TIME),
    timeBinDuration(ZERO_TIME),
    currentBinEndTime(ZERO_TIME),
    lastTime(ZERO_TIME),
    lastTotalCount(0)
{}


inline
void CounterStatView::Implementation::IncrementCounter(
    const SimTime& currentTime,
    const long long int incrementNumber)
{
    if ((currentTime < startTime) || (currentTime >= endTime)) {
        return;
    }//if//

    lastTotalCount += incrementNumber;

    if (timeBinDuration == ZERO_TIME) {
        // Probably useless (too much memory).

        if ((lastTime == currentTime) && (binlessStatValues.size() != 0)) {

            (*this).binlessStatValues.back().value += incrementNumber;
        }
        else {
            (*this).binlessStatValues.push_back(TimeAndValueRecord(currentTime, incrementNumber));
            lastTime = currentTime;

        }//if//
        (*this).binlessStatValues.back().totalCount = lastTotalCount;

    }
    else {
        // Lazy initialization.

        if (currentBinEndTime == ZERO_TIME) {
            if (timeBinDuration == INFINITE_TIME) {
                currentBinEndTime = INFINITE_TIME;
                (*this).binnedStatValues.push_back(0);
            }
            else {
                // Start with Null bin.
                currentBinEndTime = startTime;
            }//if//
        }//if//


        if (currentTime < currentBinEndTime) {
            (*this).binnedStatValues.back() += incrementNumber;
        }
        else {
            currentBinEndTime += timeBinDuration;

            while (currentBinEndTime <= currentTime) {
                binnedStatValues.push_back(0);
                currentBinEndTime += timeBinDuration;
            }//while//

            (*this).binnedStatValues.push_back(incrementNumber);

        }//if//

    }//if//

}//IncrementCounter//


CounterStatView::CounterStatView(
    const NodeId& theNodeId,
    const string& statName)
    :
    implPtr(new CounterStatView::Implementation())
{
    implPtr->theNodeId = theNodeId;
    implPtr->statName = statName;
}


CounterStatView::~CounterStatView() { }


NodeId CounterStatView::GetNodeId() const { return implPtr->theNodeId; }
string CounterStatView::GetStatName() const { return implPtr->statName; }

SimTime CounterStatView::GetCollectionStartTime() const { return (implPtr->startTime); }
SimTime CounterStatView::GetCollectionEndTime() const { return (implPtr->endTime); }
SimTime CounterStatView::GetAggregationInterval() const
    { return (implPtr->timeBinDuration); }
SimTime CounterStatView::GetCurrentBinEndTime() const { return (implPtr->currentBinEndTime); }


long long int CounterStatView::CurrentCount() const { return implPtr->lastTotalCount; }

long long int CounterStatView::GetCountOfLastBin(const SimTime& currentTime) const
{
    if (implPtr->binnedStatValues.empty()) {
        return 0;
    }

    vector<long long int>::const_reverse_iterator binIter = implPtr->binnedStatValues.rbegin();
    SimTime binEndTime = implPtr->currentBinEndTime;
    while (binIter != implPtr->binnedStatValues.rend()) {
        if (binEndTime <= currentTime) {
            break;
        }
        ++binIter;
        binEndTime -= implPtr->timeBinDuration;
    }

    if (binIter == implPtr->binnedStatValues.rend()) {
        return 0;
    }

    const long long int count = std::accumulate(binIter, implPtr->binnedStatValues.crend(), 0LL);

    return count;
}


void CounterStatView::GetPeriodEvents(
    const SimTime& periodStartTime,
    vector<pair<SimTime, long long int> >& result) const
{
    typedef CounterStatView::Implementation::TimeAndValueRecord TimeAndValueRecord;

    vector<pair<SimTime, long long int> > buffer;

    if (!(implPtr->binlessStatValues.empty())) {
        vector<TimeAndValueRecord>::const_iterator iter =
            implPtr->binlessStatValues.end();
        while (iter != implPtr->binlessStatValues.begin()) {
            --iter;
            if (iter->time < periodStartTime) {
                ++iter;
                break;
            }
        }
        const size_t eventCount = implPtr->binlessStatValues.end() - iter;
        buffer.reserve(eventCount);
        for ( ; iter != implPtr->binlessStatValues.end(); ++iter) {
            buffer.push_back(make_pair(iter->time, iter->totalCount));
        }
    }
    result.swap(buffer);
}

bool CounterStatView::IsEmpty() const
{

    if (implPtr->timeBinDuration == ZERO_TIME) {
        return (implPtr->binlessStatValues.empty());
    }
    else {
        return (implPtr->binnedStatValues.empty());
    }//if//

}

CounterStatViewIterator CounterStatView::begin() const
{
    CounterStatViewIterator newIterator;
    newIterator.implPtr->statViewPtr = shared_from_this();
    return newIterator;
}


CounterStatViewIterator CounterStatView::end() const
{
    CounterStatViewIterator newIterator;

    newIterator.implPtr->statViewPtr = shared_from_this();

    if (implPtr->timeBinDuration == ZERO_TIME) {
        newIterator.implPtr->currentIndex = implPtr->binlessStatValues.size();
    }
    else {
        newIterator.implPtr->currentIndex = implPtr->binnedStatValues.size();
    }//if//

    return newIterator;
}

bool CounterStatView::IsEqualToEnd(const const_iterator& anIterator) const
{
    assert(anIterator.implPtr->statViewPtr.get() == this);

    if (implPtr->timeBinDuration == ZERO_TIME) {
        return (anIterator.implPtr->currentIndex == implPtr->binlessStatValues.size());
    }
    else {
        return (anIterator.implPtr->currentIndex == implPtr->binnedStatValues.size());
    }//if//
}

void CounterStatView::SetCollectionStartTime(const SimTime& newStartTime)
{
    implPtr->startTime = newStartTime;
}

void CounterStatView::SetCollectionEndTime(const SimTime& newEndTime)
{
    implPtr->endTime = newEndTime;
}

void CounterStatView::SetAggregationInterval(const SimTime& newTimeBinDuration)
{
    implPtr->timeBinDuration = newTimeBinDuration;
}



CounterStatViewIterator::CounterStatViewIterator()
    : implPtr(new Implementation())
{ }

CounterStatViewIterator::CounterStatViewIterator(const CounterStatViewIterator& source)
    : implPtr(new Implementation())
{
    this->implPtr->statViewPtr = source.implPtr->statViewPtr;
    this->implPtr->currentIndex =  source.implPtr->currentIndex;
}

void CounterStatViewIterator::operator=(const CounterStatViewIterator& right)
{
    this->implPtr->statViewPtr = right.implPtr->statViewPtr;
    this->implPtr->currentIndex =  right.implPtr->currentIndex;
}


CounterStatViewIterator::~CounterStatViewIterator() { }


bool CounterStatViewIterator::operator==(const CounterStatViewIterator& right) const
{
    assert(implPtr->statViewPtr == right.implPtr->statViewPtr);
    return (implPtr->currentIndex == right.implPtr->currentIndex);
}


void CounterStatViewIterator::operator++()
{
    implPtr->currentIndex++;
}


long long int CounterStatViewIterator::CounterValue() const
{
    CounterStatView::Implementation& statView = *implPtr->statViewPtr->implPtr;

    if (statView.timeBinDuration == ZERO_TIME) {
        return statView.binlessStatValues.at(implPtr->currentIndex).value;
    }
    else {
        return statView.binnedStatValues.at(implPtr->currentIndex);
    }//if//

}//CounterValue//


long long int CounterStatViewIterator::CountForCurrentBinOnly() const
{
    if (implPtr->currentIndex == 0) {
        return CounterValue();
    }
    else {
        CounterStatView::Implementation& statView = *implPtr->statViewPtr->implPtr;

        if (statView.timeBinDuration == ZERO_TIME) {
            return (
                 statView.binlessStatValues.at(implPtr->currentIndex).value -
                 statView.binlessStatValues.at(implPtr->currentIndex - 1).value);
        }
        else {
            return (
                statView.binnedStatValues.at(implPtr->currentIndex) -
                statView.binnedStatValues.at(implPtr->currentIndex - 1));
        }//if//
    }//if//

}//CountForCurrentBinOnly//



SimTime CounterStatViewIterator::TimeForBinEnd() const
{
    CounterStatView::Implementation& statView = *implPtr->statViewPtr->implPtr;

    if (statView.timeBinDuration == ZERO_TIME) {
        return statView.binlessStatValues.at(implPtr->currentIndex).time;
    }
    else if (statView.timeBinDuration == INFINITE_TIME) {
        return INFINITE_TIME;
    }
    else {
        return (statView.startTime + ((implPtr->currentIndex+1) * statView.timeBinDuration));
    }//if//

}//TimeForBinEnd//


CounterStatViewIterator::Implementation::~Implementation() { }



CounterStatistic::CounterStatistic() : statIsEnabled(false) {}
CounterStatistic::~CounterStatistic() {}

void CounterStatistic::AddView(const shared_ptr<CounterStatView>& statViewPtr)
{
    (*this).statIsEnabled = true;
    implPtr->statViews.push_back(statViewPtr);
}


void CounterStatistic::IncrementCounterInternal(const unsigned long long int incrementNumber)
{
    assert(implPtr.get() != nullptr);

    SimTime currentTime = implPtr->simEngineInterfacePtr->CurrentTime();

    implPtr->lastCounterValue += incrementNumber;

    for(size_t i = 0; (i < implPtr->statViews.size()); i++) {
        implPtr->statViews[i]->implPtr->IncrementCounter(currentTime, incrementNumber);
    }//for//

}//IncrementCounter//


void CounterStatistic::UpdateCounterInternal(const long long int newCounterValue)
{
    assert(implPtr.get() != nullptr);

    assert(newCounterValue >= implPtr->lastCounterValue);

    if (newCounterValue > implPtr->lastCounterValue) {
        (*this).IncrementCounter(newCounterValue - implPtr->lastCounterValue);
    }//if//

}//UpdateCounter//



class RealStatistic::Implementation {
public:
    ~Implementation() {}
private:
    friend class RuntimeStatisticsSystem;
    friend class RealStatistic;

    Implementation(const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr)
        : simEngineInterfacePtr(initSimEngineInterfacePtr)
        {}

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    vector<shared_ptr<RealStatView> > statViews;

};//RealStatistic::Implementation//


class RealStatViewIterator::Implementation {
public:
    ~Implementation();
private:
    friend class RealStatViewIterator;
    friend class RealStatView;

    Implementation() : currentIndex(0) { }

    shared_ptr<const RealStatView> statViewPtr;
    size_t currentIndex;

};//IntegerStatViewIterator//


class RealStatView::Implementation {
public:
    ~Implementation() { }
private:
    friend class RealStatView;
    friend class RealStatistic;
    friend class RealStatViewIterator;
    friend class RuntimeStatisticsSystem;

    Implementation();

    void RecordStatValue(const SimTime& currentTime, const double& value);

    // Member variables:

    NodeId theNodeId;
    string statName;
    bool needDbConversion;

    SimTime startTime;
    SimTime endTime;
    SimTime timeBinDuration;

    struct BinnedValueRecord {
        SimTime endTime;
        double totalValue;
        double minValue;
        double maxValue;
        int numberValues;

        BinnedValueRecord(
            const SimTime initEndTime)
            : endTime(initEndTime),
              numberValues(0), totalValue(0.0),
              minValue(FLT_MAX), maxValue(FLT_MIN) {
        }

    };//BinnedValueRecord//

    vector<BinnedValueRecord> binnedStatValues;
    SimTime currentBinEndTime;


    struct TimeAndValueRecord {
        SimTime time;
        double value;
        TimeAndValueRecord(const SimTime& theTime, const double theValue)
            : time(theTime), value(theValue) { }
    };

    vector<TimeAndValueRecord> binlessStatValues;

    bool isLastValueViewMode;
    double lastValue;
    double totalOfAllValues;
    unsigned int totalNumberOfValues;

};//RealStatView::Implementation//


RealStatView::Implementation::Implementation()
    :
    startTime(ZERO_TIME),
    endTime(INFINITE_TIME),
    timeBinDuration(ZERO_TIME),
    currentBinEndTime(ZERO_TIME),
    isLastValueViewMode(false),
    lastValue(0.0),
    totalOfAllValues(0.0),
    totalNumberOfValues(0)
{
}


RealStatView::RealStatView(
    const NodeId& theNodeId,
    const string& statName,
    const bool needDbConversion)
    :
    implPtr(new RealStatView::Implementation())
{
    implPtr->theNodeId = theNodeId;
    implPtr->statName = statName;
    implPtr->needDbConversion = needDbConversion;
}



NodeId RealStatView::GetNodeId() const { return implPtr->theNodeId; }
string RealStatView::GetStatName() const { return implPtr->statName; }
bool RealStatView::NeedDbConversion() const { return implPtr->needDbConversion; }
bool RealStatView::IsLastValueViewMode() const { return implPtr->isLastValueViewMode; }

SimTime RealStatView::GetCollectionStartTime() const { return (implPtr->startTime); }
SimTime RealStatView::GetCollectionEndTime() const { return (implPtr->endTime); }
SimTime RealStatView::GetAggregationInterval() const
    { return (implPtr->timeBinDuration); }
SimTime RealStatView::GetCurrentBinEndTime() const { return (implPtr->currentBinEndTime); }


void RealStatView::SetCollectionStartTime(const SimTime& setToStartTime)
{
    implPtr->startTime = setToStartTime;
}


void RealStatView::SetCollectionEndTime(const SimTime& setToEndTime)
{
    implPtr->endTime = setToEndTime;
}


void RealStatView::SetAggregationInterval(const SimTime& setToTimeBinDuration)
{
    implPtr->timeBinDuration = setToTimeBinDuration;
}

void RealStatView::SetLastValueViewMode()
{
    implPtr->isLastValueViewMode = true;
}

bool RealStatView::IsEmpty() const
{

    if (implPtr->timeBinDuration == ZERO_TIME) {
        return (implPtr->binlessStatValues.empty());
    }
    else {
        return (implPtr->binnedStatValues.empty());
    }//if//

}


RealStatViewIterator RealStatView::begin() const
{
    RealStatViewIterator newIterator;
    newIterator.implPtr->statViewPtr = shared_from_this();
    return newIterator;
}


RealStatViewIterator RealStatView::end() const
{
    RealStatViewIterator newIterator;

    newIterator.implPtr->statViewPtr = shared_from_this();

    if (implPtr->timeBinDuration == ZERO_TIME) {
        newIterator.implPtr->currentIndex = implPtr->binlessStatValues.size();
    }
    else {
        newIterator.implPtr->currentIndex = implPtr->binnedStatValues.size();
    }//if//

    return newIterator;
}



bool RealStatView::IsEqualToEnd(const const_iterator& anIterator) const
{
    assert(anIterator.implPtr->statViewPtr.get() == this);

    if (implPtr->timeBinDuration == ZERO_TIME) {
        return (anIterator.implPtr->currentIndex == implPtr->binlessStatValues.size());
    }
    else {
        return (anIterator.implPtr->currentIndex == implPtr->binnedStatValues.size());
    }//if//
}



void RealStatView::Implementation::RecordStatValue(const SimTime& currentTime, const double& value)
{

    if ((currentTime < startTime) || (currentTime >= endTime)) {
        return;
    }//if//

    lastValue = value;
    totalOfAllValues += value;
    totalNumberOfValues++;

    if (timeBinDuration == ZERO_TIME) {
        // Probably useless (too much memory).

        (*this).binlessStatValues.push_back(
            TimeAndValueRecord(currentTime, static_cast<double>(value)));
    }
    else {
        // Lazy Initialization
        if (currentBinEndTime == ZERO_TIME) {
            if (timeBinDuration == INFINITE_TIME) {
                currentBinEndTime = INFINITE_TIME;
                (*this).binnedStatValues.push_back(BinnedValueRecord(currentBinEndTime));
            } else {
                // Start with Null bin.
                currentBinEndTime = startTime;
            }//if//
        }//if//.

        if (currentTime >= currentBinEndTime) {
            currentBinEndTime += timeBinDuration;

            while (currentBinEndTime <= currentTime) {
                // Append null data records.
                binnedStatValues.push_back(BinnedValueRecord(currentBinEndTime));
                currentBinEndTime += timeBinDuration;
            }//while//

            (*this).binnedStatValues.push_back(BinnedValueRecord(currentBinEndTime));

        }//if//

        BinnedValueRecord& currentRecord = binnedStatValues.back();

        currentRecord.totalValue += static_cast<double>(value);
        currentRecord.numberValues++;

        if (value < currentRecord.minValue) {
            currentRecord.minValue = static_cast<double>(value);
        }//if//

        if (value > currentRecord.minValue) {
            currentRecord.minValue = static_cast<double>(value);
        }//if//
    }//if//

}//RecordStatValue//


double RealStatView::GetLastRawValue() const
{
    return (implPtr->lastValue);
}


double RealStatView::GetAverageOfAllValues() const
{
    if ((*this).NeedDbConversion()) {
        return ConvertToDb((implPtr->totalOfAllValues / implPtr->totalNumberOfValues));
    }
    else {
        return (implPtr->totalOfAllValues / implPtr->totalNumberOfValues);
    }//if//
}//GetAverageOfAllValues//


const bool RealStatView::GetAverageOfLastBin(
    const SimTime& currentTime,
    double& value) const
{
    if (implPtr->binnedStatValues.empty()) {
        return false;

    } else {
        vector<RealStatView::Implementation::BinnedValueRecord>::const_reverse_iterator binIter = implPtr->binnedStatValues.rbegin();
        while (binIter != implPtr->binnedStatValues.rend()) {
            if (binIter->endTime <= currentTime) {
                break;
            }
            ++binIter;
        }

        if (binIter == implPtr->binnedStatValues.rend()) {
            return false;
        }

        const RealStatView::Implementation::BinnedValueRecord& bin = *binIter;
        if (implPtr->timeBinDuration != INFINITE_TIME) {
            if (bin.endTime <= (currentTime - implPtr->timeBinDuration)) {
                // the bin is expired
                return false;
            }
        }
        if (bin.numberValues <= 0) {
            // the bin is empty
            return false;
        }

        value = bin.totalValue / bin.numberValues;
        return true;
    }
}

void RealStatView::GetPeriodEvents(
    const SimTime& periodStartTime,
    vector<pair<SimTime, double> >& result) const
{
    typedef RealStatView::Implementation::TimeAndValueRecord TimeAndValueRecord;

    vector<pair<SimTime, double> > buffer;

    const size_t eventCount = implPtr->binlessStatValues.size();
    if (!(implPtr->binlessStatValues.empty())) {
        vector<TimeAndValueRecord>::const_iterator iter =
            implPtr->binlessStatValues.end();
        while (iter != implPtr->binlessStatValues.begin()) {
            --iter;
            if (iter->time < periodStartTime) {
                ++iter;
                break;
            }
        }
        const size_t count = implPtr->binlessStatValues.end() - iter;
        buffer.reserve(count);
        for ( ; iter != implPtr->binlessStatValues.end(); ++iter) {
            buffer.push_back(make_pair(iter->time, iter->value));
        }
    }
    result.swap(buffer);
}

RealStatViewIterator::RealStatViewIterator()
    : implPtr(new Implementation())

{ }

RealStatViewIterator::RealStatViewIterator(const RealStatViewIterator& source)
    : implPtr(new Implementation())
{
    this->implPtr->statViewPtr = source.implPtr->statViewPtr;
    this->implPtr->currentIndex =  source.implPtr->currentIndex;
}

void RealStatViewIterator::operator=(const RealStatViewIterator& right)
{
    this->implPtr->statViewPtr = right.implPtr->statViewPtr;
    this->implPtr->currentIndex =  right.implPtr->currentIndex;
}


RealStatViewIterator::~RealStatViewIterator() { }


bool RealStatViewIterator::operator==(const RealStatViewIterator& right) const
{
    assert(implPtr->statViewPtr == right.implPtr->statViewPtr);
    return (implPtr->currentIndex == right.implPtr->currentIndex);
}


void RealStatViewIterator::operator++()
{
    implPtr->currentIndex++;
}


int RealStatViewIterator::BinNumberDatapoints() const
{
    RealStatView::Implementation& statView = *implPtr->statViewPtr->implPtr;
    assert (statView.timeBinDuration != ZERO_TIME);
    return (statView.binnedStatValues[implPtr->currentIndex].numberValues);
}


double RealStatViewIterator::BinValueMin() const
{
    RealStatView::Implementation& statView = *implPtr->statViewPtr->implPtr;
    assert (statView.timeBinDuration != ZERO_TIME);
    if (statView.needDbConversion) {
        return (ConvertToDb(statView.binnedStatValues[implPtr->currentIndex].minValue));
    }
    else {
        return (statView.binnedStatValues[implPtr->currentIndex].minValue);
    }//if//
}//BinValueMin//


double RealStatViewIterator::BinValueMax() const
{
    RealStatView::Implementation& statView = *implPtr->statViewPtr->implPtr;
    assert(statView.timeBinDuration != ZERO_TIME);
    if (statView.needDbConversion) {
        return (ConvertToDb(statView.binnedStatValues[implPtr->currentIndex].maxValue));
    }
    else {
        return (statView.binnedStatValues[implPtr->currentIndex].maxValue);
    }//if//
}//BinValueMax//


double RealStatViewIterator::BinValueAverage() const
{
    RealStatView::Implementation& statView = *implPtr->statViewPtr->implPtr;
    if (statView.timeBinDuration == ZERO_TIME) {
        if (statView.needDbConversion) {
            return (ConvertToDb(statView.binlessStatValues[implPtr->currentIndex].value));
        }
        else {
            return (statView.binlessStatValues[implPtr->currentIndex].value);
        }
    }
    else {
        const RealStatView::Implementation::BinnedValueRecord& valueRecord =
            statView.binnedStatValues[implPtr->currentIndex];

        assert(valueRecord.numberValues > 0);

        if (statView.needDbConversion) {
            return (ConvertToDb(double(valueRecord.totalValue) / valueRecord.numberValues));
        }
        else  {
            return (double(valueRecord.totalValue) / valueRecord.numberValues);
        }//if//

    }//if//

}//BinValueAverage//



SimTime RealStatViewIterator::TimeForBinEnd() const
{
    RealStatView::Implementation& statView = *implPtr->statViewPtr->implPtr;
    if (statView.timeBinDuration == ZERO_TIME) {

        return (statView.binlessStatValues[implPtr->currentIndex].time);
    }
    else if (statView.timeBinDuration == INFINITE_TIME) {
        return INFINITE_TIME;
    }
    else {
        return (statView.startTime + ((implPtr->currentIndex+1) * statView.timeBinDuration));

    }//if//

}//TimeForBinEnd//


RealStatViewIterator::Implementation::~Implementation() { }


RealStatistic::RealStatistic(const bool initNeedDbConversion)
    :
    statIsEnabled(false),
    needDbConversion(initNeedDbConversion)
{ }

RealStatistic::~RealStatistic() {}


void RealStatistic::AddView(const shared_ptr<RealStatView>& statViewPtr)
{
    (*this).statIsEnabled = true;
    implPtr->statViews.push_back(statViewPtr);
}


void RealStatistic::RecordStatValueInternal(const double& value)
{
    assert(implPtr.get() != nullptr);

    SimTime currentTime = implPtr->simEngineInterfacePtr->CurrentTime();

    for(size_t i = 0; (i < implPtr->statViews.size()); i++) {
        implPtr->statViews[i]->implPtr->RecordStatValue(currentTime, value);
    }//for//

}//RecordStatValue//





class RuntimeStatisticsSystem::Implementation {
public:
    Implementation(const bool initAllowStatReconnection) : allowStatReconnection(initAllowStatReconnection) {}
    ~Implementation() { }
private:
    friend class RuntimeStatisticsSystem;
    friend class StatNameIterator;
    friend class StatNameIterator::Implementation;

    // Note: specific ordering of the pairs used by program logic.

    struct StatNameNodeIdPairType {
        NodeId theNodeId;
        string statName;

        bool operator<(const StatNameNodeIdPairType& right) const
        {
            return (IsLessThanCaseInsensitive(statName, right.statName) ||
                    ((IsEqualCaseInsensitive(statName, right.statName)) && (theNodeId < right.theNodeId)));
        }

        StatNameNodeIdPairType(const string& initStatName, const NodeId& initNodeId)
            : theNodeId(initNodeId), statName(initStatName) {}
    };//StatNameNodeIdPairType//


    map<StatNameNodeIdPairType, shared_ptr<CounterStatistic> > counterStatistics;
    map<StatNameNodeIdPairType, shared_ptr<RealStatistic> > realStatistics;

    struct StatConfigItem {
        NodeId theNodeId;
        string statNameWithWildcards;
        SimTime startTime;
        SimTime endTime;
        SimTime aggregationInterval;
        bool viewLastValueMode;
        StatViewCollection* theStatViewCollectionPtr;

        bool operator==(const StatConfigItem& right)
        {
            return (
                (theNodeId == right.theNodeId) &&
                (statNameWithWildcards == right.statNameWithWildcards) &&
                (startTime == right.startTime) &&
                (endTime == right.endTime) &&
                (aggregationInterval == right.aggregationInterval) &&
                ((viewLastValueMode && right.viewLastValueMode) || (!viewLastValueMode && !right.viewLastValueMode)) &&
                (theStatViewCollectionPtr == right.theStatViewCollectionPtr));

        }//operator==//

    };//StatConfigItem//

    vector<StatConfigItem> statConfigCache;

    bool allowStatReconnection;

};//RuntimeStatisticsSystem::Implementation//



RuntimeStatisticsSystem::RuntimeStatisticsSystem(const bool allowStatReconnection)
    : implPtr(new RuntimeStatisticsSystem::Implementation(allowStatReconnection))
{
}

RuntimeStatisticsSystem::~RuntimeStatisticsSystem()
{
}

//-------------------------------------------------------------------------------------------------

class StatNameIterator::Implementation {
public:
    ~Implementation() { }
private:
    friend class StatNameIterator;
    friend class RuntimeStatisticsSystem;

    typedef RuntimeStatisticsSystem::Implementation::StatNameNodeIdPairType StatNameNodeIdPairType;

    typedef map<StatNameNodeIdPairType, shared_ptr<CounterStatistic> >::const_iterator CounterStatMapIterType;

    typedef map<StatNameNodeIdPairType, shared_ptr<RealStatistic> >::const_iterator RealStatMapIterType;

    Implementation(
        const RuntimeStatisticsSystem* initStatSystemPtr,
        const CounterStatMapIterType& initCounterMapIter,
        const RealStatMapIterType& initRealMapIter)
        :
        statSystemPtr(initStatSystemPtr),
        counterMapIter(initCounterMapIter),
        realMapIter(initRealMapIter)
    {
        if (counterMapIter == statSystemPtr->implPtr->counterStatistics.end()) {
            currentIsCounterStat = false;
        }
        else if (realMapIter == statSystemPtr->implPtr->realStatistics.end()) {
            currentIsCounterStat = true;
        }
        else {
            currentIsCounterStat =
                ((*counterMapIter).first < (*realMapIter).first);
        }//if//
    }

    const RuntimeStatisticsSystem* statSystemPtr;

    bool currentIsCounterStat;
    CounterStatMapIterType counterMapIter;
    RealStatMapIterType realMapIter;

};//StatNameIterator::Implementation//


StatNameIterator::StatNameIterator() { }
StatNameIterator::StatNameIterator(const StatNameIterator& source)
    :
    implPtr(
        new StatNameIterator::Implementation(
            source.implPtr->statSystemPtr,
            source.implPtr->counterMapIter,
            source.implPtr->realMapIter))
{}

void StatNameIterator::operator=(const StatNameIterator& right)
{
    (*this).implPtr->statSystemPtr = right.implPtr->statSystemPtr;
    (*this).implPtr->counterMapIter = right.implPtr->counterMapIter;
    (*this).implPtr->realMapIter = right.implPtr->realMapIter;
}


StatNameIterator::~StatNameIterator() { }


bool StatNameIterator::operator==(const StatNameIterator& right) const
{
    return
        ((implPtr->counterMapIter == right.implPtr->counterMapIter) &&
         (implPtr->realMapIter == right.implPtr->realMapIter));

}


bool StatNameIterator::IsCounterStat() const
{
    return (implPtr->currentIsCounterStat);
}


bool StatNameIterator::IsRealStat() const
{
    return !(implPtr->currentIsCounterStat);
}



string StatNameIterator::GetName() const
{
    if (implPtr->currentIsCounterStat) {
        return (implPtr->counterMapIter->first.statName);
    }
    else {
        return (implPtr->realMapIter->first.statName);
    }//if//
}

NodeId StatNameIterator::GetNodeId() const
{
    if (implPtr->currentIsCounterStat) {
        return (implPtr->counterMapIter->first.theNodeId);
    }
    else {
        return (implPtr->realMapIter->first.theNodeId);

    }//if//
}


void StatNameIterator::operator++()
{
    if (implPtr->currentIsCounterStat) {
        ++(implPtr->counterMapIter);
    }
    else {
        ++(implPtr->realMapIter);
    }//if//

    if (implPtr->counterMapIter == implPtr->statSystemPtr->implPtr->counterStatistics.end()) {
        implPtr->currentIsCounterStat = false;
        return;
    }//if//

    if (implPtr->realMapIter == implPtr->statSystemPtr->implPtr->realStatistics.end()) {
        implPtr->currentIsCounterStat = true;
        return;
    }//if//

    implPtr->currentIsCounterStat =
        ((*implPtr->counterMapIter).first < (*implPtr->realMapIter).first);

}//operator++//


void RuntimeStatisticsSystem::SetAllowStatReconnectionMode() { implPtr->allowStatReconnection = true; }


StatNameIterator RuntimeStatisticsSystem::begin() const
{
    StatNameIterator iter;

    iter.implPtr = unique_ptr<StatNameIterator::Implementation>(
        new StatNameIterator::Implementation(
            this,
            implPtr->counterStatistics.begin(),
            implPtr->realStatistics.begin()));

    return iter;
}


StatNameIterator RuntimeStatisticsSystem::end() const
{
    StatNameIterator iter;

    iter.implPtr = unique_ptr<StatNameIterator::Implementation>(
        new StatNameIterator::Implementation(
            this,
            implPtr->counterStatistics.end(),
            implPtr->realStatistics.end()));

    return iter;
}


StatNameIterator RuntimeStatisticsSystem::FindFirst(const string& statNamePrefix) const
{
    StatNameIterator iter;

    Implementation::StatNameNodeIdPairType statId(statNamePrefix, 0);

    iter.implPtr = unique_ptr<StatNameIterator::Implementation>(
        new StatNameIterator::Implementation(
            this,
            implPtr->counterStatistics.lower_bound(statId),
            implPtr->realStatistics.lower_bound(statId)));

    return iter;
}



void RuntimeStatisticsSystem::CreateCounterStat(
    const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
    const NodeId& theNodeId,
    const string& statName,
    //const bool useBigInts,
    shared_ptr<CounterStatistic>& counterStatPtr)
{
    typedef Implementation::StatNameNodeIdPairType StatNameNodeIdPairType;
    typedef map<StatNameNodeIdPairType, shared_ptr<CounterStatistic> >::iterator StatIterType;

    Implementation::StatNameNodeIdPairType statId(statName, theNodeId);

    StatIterType iter = implPtr->counterStatistics.find(statId);

    if (iter != implPtr->counterStatistics.end()) {
        if (implPtr->allowStatReconnection) {
            counterStatPtr = iter->second;
            counterStatPtr->implPtr->simEngineInterfacePtr = simEngineInterfacePtr;
            return;
        }
        else {
            cerr << "Error: Duplicate runtime statistic: " << theNodeId << "/" << statName << endl;
            exit(1);
        }//if//
    }//if//

    counterStatPtr = shared_ptr<CounterStatistic>(new CounterStatistic());
    counterStatPtr->implPtr.reset(new CounterStatistic::Implementation(simEngineInterfacePtr));
    (*this).implPtr->counterStatistics[statId] = counterStatPtr;

    CreateCounterStatViewFromCachedStatConfig(theNodeId, statName, counterStatPtr);

}//CreateCounterStat//


void RuntimeStatisticsSystem::CreateRealStat(
    const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
    const NodeId& theNodeId,
    const string& statName,
    const bool needDbConversion,
    shared_ptr<RealStatistic>& realStatPtr)
{
    typedef Implementation::StatNameNodeIdPairType StatNameNodeIdPairType;
    typedef map<StatNameNodeIdPairType, shared_ptr<RealStatistic> >::iterator StatIterType;

    Implementation::StatNameNodeIdPairType statId(statName, theNodeId);

    StatIterType iter = implPtr->realStatistics.find(statId);

    if (iter != implPtr->realStatistics.end()) {
        if (implPtr->allowStatReconnection) {
            realStatPtr = iter->second;
            realStatPtr->implPtr->simEngineInterfacePtr = simEngineInterfacePtr;
            return;
        }
        else {
            cerr << "Error: Duplicate runtime statistic: " << theNodeId << "/" << statName << endl;
            exit(1);
        }//if//
    }//if//

    realStatPtr = shared_ptr<RealStatistic>(new RealStatistic(needDbConversion));
    realStatPtr->implPtr.reset(new RealStatistic::Implementation(simEngineInterfacePtr));
    (*this).implPtr->realStatistics[statId] = realStatPtr;

    CreateRealStatViewFromCachedStatConfig(theNodeId, statName, realStatPtr);

}//CreateRealStat//



void RuntimeStatisticsSystem::DeleteAllStatsFor(const NodeId& theNodeId)
{
    typedef Implementation::StatNameNodeIdPairType StatNameNodeIdPairType;
    typedef map<StatNameNodeIdPairType, shared_ptr<CounterStatistic> >::iterator CounterStatIterType;
    typedef map<StatNameNodeIdPairType, shared_ptr<RealStatistic> >::iterator RealStatIterType;

    CounterStatIterType iter1 = implPtr->counterStatistics.begin();
    while(iter1 != implPtr->counterStatistics.end()) {
        if (iter1->first.theNodeId == theNodeId) {
            CounterStatIterType deleteIter = iter1;
            ++iter1;
            implPtr->counterStatistics.erase(deleteIter);
        }
        else {
            ++iter1;
        }//if//
    }//while//

    RealStatIterType iter2 = implPtr->realStatistics.begin();
    while(iter2 != implPtr->realStatistics.end()) {
        if (iter2->first.theNodeId == theNodeId) {
            RealStatIterType deleteIter = iter2;
            ++iter2;
            implPtr->realStatistics.erase(deleteIter);
        }
        else {
            ++iter2;
        }//if//
    }//while//

}//DeleteAllStatsFor//


void RuntimeStatisticsSystem::CreateCounterStatView(
    const NodeId& theNodeId,
    const string& statName,
    shared_ptr<CounterStatView>& newStatViewPtr)
{
    newStatViewPtr = shared_ptr<CounterStatView>(new CounterStatView(theNodeId, statName));

    Implementation::StatNameNodeIdPairType statId(statName, theNodeId);

    if (implPtr->counterStatistics.find(statId) == implPtr->counterStatistics.end()) {
        cerr << "Error in CreateCounterStatView: Stat: " << statName << " for node " << theNodeId;
        cerr << " not found." << endl;
        exit(1);
    }//if//

    CounterStatistic& statForView = *implPtr->counterStatistics[statId];

    statForView.AddView(newStatViewPtr);

}//CreateCounterStatView//



void RuntimeStatisticsSystem::CreateRealStatView(
    const NodeId& theNodeId,
    const string& statName,
    shared_ptr<RealStatView>& newStatViewPtr)
{

    Implementation::StatNameNodeIdPairType statId(statName, theNodeId);

    if (implPtr->realStatistics.find(statId) == implPtr->realStatistics.end()) {
        cerr << "Error in CreateRealStatView: Stat: " << statName << " for node " << theNodeId;
        cerr << " not found." << endl;
        exit(1);
    }//if//

    RealStatistic& statForView = *implPtr->realStatistics[statId];

    newStatViewPtr = shared_ptr<RealStatView>(new RealStatView(theNodeId, statName, statForView.needDbConversion));

    statForView.AddView(newStatViewPtr);

}//CreateRealStatView//


void RuntimeStatisticsSystem::CacheStatConfig(
    const NodeId& theNodeId,
    const string& statNameWithWildcards,
    const SimTime& startTime,
    const SimTime& endTime,
    const SimTime& aggregationInterval,
    const bool viewLastValueMode,
    StatViewCollection& theStatViewCollection)
{
    Implementation::StatConfigItem newItem;

    newItem.theNodeId = theNodeId;
    newItem.statNameWithWildcards = statNameWithWildcards;
    newItem.startTime = startTime;
    newItem.endTime = endTime;
    newItem.aggregationInterval = aggregationInterval;
    newItem.viewLastValueMode = viewLastValueMode;
    newItem.theStatViewCollectionPtr = &theStatViewCollection;

    for (size_t i = 0; i < implPtr->statConfigCache.size(); ++i) {
        if (implPtr->statConfigCache[i] == newItem) {
            return;
        }//if//
    }//for//

    implPtr->statConfigCache.push_back(newItem);

}//CacheStatConfig//


inline
string GetWildcardFreePrefix(const string& wildString)
{
    size_t pos = wildString.find('*');

    if (pos == string::npos) {
        return wildString;
    }//if//

    return (wildString.substr(0, pos));
}


void RuntimeStatisticsSystem::CreateCounterStatViewFromCachedStatConfig(
    const NodeId& theNodeId,
    const string& statName,
    shared_ptr<CounterStatistic>& counterStatPtr)
{
    for (size_t i = 0; i < implPtr->statConfigCache.size(); ++i) {
        if ((implPtr->statConfigCache[i].theNodeId == ANY_NODEID) ||
            (implPtr->statConfigCache[i].theNodeId == theNodeId)) {

            if ((IsEqualWithWildcardsCaseInsensitive(
                implPtr->statConfigCache[i].statNameWithWildcards, statName))) {

                shared_ptr<CounterStatView> newStatViewPtr(new CounterStatView(theNodeId, statName));

                newStatViewPtr->SetCollectionStartTime(implPtr->statConfigCache[i].startTime);
                newStatViewPtr->SetCollectionEndTime(implPtr->statConfigCache[i].endTime);
                newStatViewPtr->SetAggregationInterval(implPtr->statConfigCache[i].aggregationInterval);

                const StatViewCollection::StatViewIdType statViewId(statName, theNodeId);
                implPtr->statConfigCache[i].theStatViewCollectionPtr->counterStatViewPtrs[statViewId] = newStatViewPtr;

                counterStatPtr->AddView(newStatViewPtr);
            }//if//
        }//if//
    }//for//

}//CreateCounterStatViewFromCachedStatConfig//


void RuntimeStatisticsSystem::CreateRealStatViewFromCachedStatConfig(
    const NodeId& theNodeId,
    const string& statName,
    shared_ptr<RealStatistic>& realStatPtr)
{
    for (size_t i = 0; i < implPtr->statConfigCache.size(); ++i) {
        if ((implPtr->statConfigCache[i].theNodeId == ANY_NODEID) ||
            (implPtr->statConfigCache[i].theNodeId == theNodeId)) {

            if ((IsEqualWithWildcardsCaseInsensitive(
                implPtr->statConfigCache[i].statNameWithWildcards, statName))) {

                shared_ptr<RealStatView> newStatViewPtr(new RealStatView(theNodeId, statName, realStatPtr->needDbConversion));

                newStatViewPtr->SetCollectionStartTime(implPtr->statConfigCache[i].startTime);
                newStatViewPtr->SetCollectionEndTime(implPtr->statConfigCache[i].endTime);
                newStatViewPtr->SetAggregationInterval(implPtr->statConfigCache[i].aggregationInterval);

                const StatViewCollection::StatViewIdType statViewId(statName, theNodeId);
                implPtr->statConfigCache[i].theStatViewCollectionPtr->realStatViewPtrs[statViewId] = newStatViewPtr;

                if (implPtr->statConfigCache[i].viewLastValueMode) {
                    newStatViewPtr->SetLastValueViewMode();
                }//if//

                realStatPtr->AddView(newStatViewPtr);
            }//if//
        }//if//
    }//for//

}//CreateRealStatViewFromCachedStatConfig//


void CreateStatViews(
    RuntimeStatisticsSystem& statRuntime,
    const NodeId& theNodeId,
    const string& statNameWithWildcards,
    const SimTime& startTime,
    const SimTime& endTime,
    const SimTime& aggregationInterval,
    const bool viewLastValueMode,
    StatViewCollection& theStatViewCollection)
{
    typedef StatNameIterator IterType;

    const string prefix = GetWildcardFreePrefix(statNameWithWildcards);

    IterType iter = statRuntime.FindFirst(prefix);

    while (!(iter == statRuntime.end()) &&
           IsEqualCaseInsensitive(prefix, iter.GetName().substr(0, prefix.length()))) {

        if (((theNodeId == ANY_NODEID) || (theNodeId == iter.GetNodeId())) &&
            (IsEqualWithWildcardsCaseInsensitive(statNameWithWildcards, iter.GetName()))) {

            StatViewCollection::StatViewIdType statViewId(iter.GetName(), iter.GetNodeId());

            if (iter.IsCounterStat()) {

                if (theStatViewCollection.counterStatViewPtrs.find(statViewId) ==
                    theStatViewCollection.counterStatViewPtrs.end()) {

                    shared_ptr<CounterStatView> newStatViewPtr;
                    statRuntime.CreateCounterStatView(iter.GetNodeId(), iter.GetName(), newStatViewPtr);

                    newStatViewPtr->SetCollectionStartTime(startTime);
                    newStatViewPtr->SetCollectionEndTime(endTime);
                    newStatViewPtr->SetAggregationInterval(aggregationInterval);

                    theStatViewCollection.counterStatViewPtrs[statViewId] = newStatViewPtr;
                }//if//

            }
            else if (iter.IsRealStat()) {

                if (theStatViewCollection.realStatViewPtrs.find(statViewId) ==
                    theStatViewCollection.realStatViewPtrs.end()) {

                    shared_ptr<RealStatView> newStatViewPtr;
                    statRuntime.CreateRealStatView(iter.GetNodeId(), iter.GetName(), newStatViewPtr);

                    newStatViewPtr->SetCollectionStartTime(startTime);
                    newStatViewPtr->SetCollectionEndTime(endTime);
                    newStatViewPtr->SetAggregationInterval(aggregationInterval);

                    if (viewLastValueMode) {
                        newStatViewPtr->SetLastValueViewMode();
                    }

                    theStatViewCollection.realStatViewPtrs[statViewId] = newStatViewPtr;
                }//if//

            }
            else {
                assert(false && "Unknown Stat Type"); abort();
            }//if//
        }//if//

        ++iter;

    }//while//

}//CreateStatViews//



void ReadStatConfigFile(
    RuntimeStatisticsSystem& statRuntime,
    const string& statConfigFilename,
    StatViewCollection& theStatViewCollection)
{
    ifstream statConfigFile(statConfigFilename.c_str());

    if (!statConfigFile.good()) {
        cerr << "Error: The statistics configuration file: " << statConfigFilename
             << " was not found." << endl;
        exit(1);
    }//if//


    while(!statConfigFile.eof()) {
        string aLine;
        getline(statConfigFile, aLine);
        if (statConfigFile.eof()) {
            break;
        }//if//

        if (!IsAConfigFileCommentLine(aLine)) {

            DeleteTrailingSpaces(aLine);

            istringstream aLineStream(aLine);

            string nodeIdString;
            aLineStream >> nodeIdString;

            string statName;
            aLineStream >> statName;

            string aggregationIntervalString;
            aLineStream >> aggregationIntervalString;

            if (aLineStream.fail()) {
                cerr << "Error: in Statistic Definition file format, line:" << endl;
                cerr << aLine << endl;
                exit(1);
            }//if//

            ConvertStringToLowerCase(statName);

            SimTime aggregationInterval = ZERO_TIME;
            bool succeeded;
            ConvertStringToTime(aggregationIntervalString, aggregationInterval, succeeded);

            if (!succeeded) {
                cerr << "Error: in Statistic Definition file format, line:" << endl;
                cerr << aLine << endl;
                cerr << "       for time string:" << aggregationIntervalString << endl;
                exit(1);
            }//if//

            SimTime startTime = ZERO_TIME;
            SimTime endTime = INFINITE_TIME;
            unsigned int statViewInstanceId = 0;

            if (!aLineStream.eof()) {

                string startTimeString;

                aLineStream >> startTimeString;

                if (aLineStream.fail()) {
                    cerr << "Error: in Statistic Definition file format, line:" << endl;
                    cerr << aLine << endl;
                    exit(1);
                }//if//

                ConvertStringToTime(startTimeString, startTime, succeeded);
                if (!succeeded) {
                    cerr << "Error: in Statistic Definition file format, line:" << endl;
                    cerr << aLine << endl;
                    cerr << "       for time string:" << startTimeString << endl;
                    exit(1);
                }//if//


                if (!aLineStream.eof()) {

                    string endTimeString;

                    aLineStream >> endTimeString;

                    if (aLineStream.fail()) {
                        cerr << "Error: in Statistic Definition file format, line:" << endl;
                        cerr << aLine << endl;
                        exit(1);
                    }//if//

                    ConvertStringToTime(endTimeString, endTime, succeeded);
                    if (!succeeded) {
                        cerr << "Error: in Statistic Definition file format, line:" << endl;
                        cerr << aLine << endl;
                        cerr << "       for time string:" << endTimeString << endl;
                        exit(1);
                    }//if//


                    if (!aLineStream.eof()) {
                        assert(false && "Stat View Instance Id future feature not implemented");
                        aLineStream >> statViewInstanceId;

                        if ((aLineStream.fail()) || (!aLineStream.eof())) {
                            cerr << "Error: in Statistic Definition file format, line:" << endl;
                            cerr << aLine << endl;
                            exit(1);
                        }//if//
                    }//if//
                }//if//
            }//if//

            NodeId theNodeId = ANY_NODEID;

            if (nodeIdString != "*") {

                int intValue;
                ConvertStringToInt(nodeIdString, intValue, succeeded);
                if (!succeeded) {
                    cerr << "Error: in Statistic Definition file format, line:" << endl;
                    cerr << aLine << endl;
                    cerr << "       for Node ID string:" << nodeIdString << endl;
                    exit(1);
                }//if//

                theNodeId = intValue;
            }//if//

            statRuntime.CacheStatConfig(
                theNodeId, statName, startTime, endTime, aggregationInterval, false/*viewLastValueMode*/,
                theStatViewCollection);

        }//if//
    }//while//

}//ReadStatConfigFile//


void OutputCounterStatToStream(
    CounterStatView& statView,
    ostream& outputStream,
    const SimTime& startTime = ZERO_TIME,
    const SimTime& endTime = INFINITE_TIME,
    const bool noDataOutputIsEnabled = true)
{
    typedef CounterStatView::const_iterator IterType;

    outputStream.fill(' ');//temporary fix for mountain lion libc++'s bug

    const SimTime aggregationInterval = statView.GetAggregationInterval();

    if (!noDataOutputIsEnabled && statView.IsEmpty()) {
        return;
    }//if//

    bool headerHasBeenOutputed = false;
    if (aggregationInterval == ZERO_TIME) {
        if (statView.IsEmpty()) {
            headerHasBeenOutputed = true;
            outputStream << setw(NODEID_MIN_STREAM_OUTPUT_WIDTH) << statView.GetNodeId()
                         << ' ' << statView.GetStatName()
                         << " = 0";
        }
        else {
            for(IterType iter = statView.begin(); (!statView.IsEqualToEnd(iter)); ++iter) {
                SimTime datapointTime = iter.TimeForBinEnd();
                if ((datapointTime >= startTime) && (datapointTime <= endTime)) {
                    if (!headerHasBeenOutputed) {
                        headerHasBeenOutputed = true;
                        outputStream << setw(NODEID_MIN_STREAM_OUTPUT_WIDTH) << statView.GetNodeId()
                                     << ' ' << statView.GetStatName()
                                     << " = " << statView.CurrentCount() << "  "
                                     << ConvertTimeToStringSecs(datapointTime) << " 0";
                    }//if//
                    outputStream << ' ' << ConvertTimeToStringSecs(iter.TimeForBinEnd())
                                 << ' ' << iter.CounterValue();
                }//if//
            }//for//
        }//if//
    }
    else if (aggregationInterval == INFINITE_TIME) {
        headerHasBeenOutputed = true;
        outputStream << setw(NODEID_MIN_STREAM_OUTPUT_WIDTH) << statView.GetNodeId()
                << ' ' << statView.GetStatName()
                << " = " << statView.CurrentCount();
    }
    else {
        if (statView.IsEmpty()) {
            headerHasBeenOutputed = true;
            const SimTime collectionStartTime = statView.GetCollectionStartTime();
            outputStream << setw(NODEID_MIN_STREAM_OUTPUT_WIDTH) << statView.GetNodeId()
                         << ' ' << statView.GetStatName()
                         << " = 0  " << ConvertTimeToStringSecs(collectionStartTime)<< ' '
                         << ConvertTimeToStringSecs(aggregationInterval);
        }
        else {
            for(IterType iter = statView.begin(); (!statView.IsEqualToEnd(iter)); ++iter) {
                SimTime dataBinEndTime = iter.TimeForBinEnd();
                SimTime dataBinStartTime = dataBinEndTime - aggregationInterval;

                if ((dataBinEndTime >= startTime) && (dataBinStartTime <= endTime)) {

                    if (!headerHasBeenOutputed) {
                        headerHasBeenOutputed = true;
                        outputStream << setw(NODEID_MIN_STREAM_OUTPUT_WIDTH) << statView.GetNodeId()
                                     << ' ' << statView.GetStatName()
                                     << " = " << statView.CurrentCount() << "  "
                                     << ConvertTimeToStringSecs(dataBinStartTime) << ' '
                                     << ConvertTimeToStringSecs(aggregationInterval);
                    }//if//
                    outputStream << ' ' << iter.CounterValue();
                }//if//
            }//for//
        }//if//

        SimTime lastOutputTime;
        if (statView.IsEmpty()) {
            lastOutputTime = statView.GetCollectionStartTime();
        }
        else {
            lastOutputTime = statView.GetCurrentBinEndTime();
        }//if//
        const SimTime collectionEndTime = statView.GetCollectionEndTime();
        const SimTime outputEndTime = std::min(endTime, collectionEndTime) + aggregationInterval;

        for(SimTime outputTime = (lastOutputTime + aggregationInterval); outputTime <= outputEndTime; outputTime += aggregationInterval) {
                outputStream << " 0";
        }//for//

    }//if//

    assert(headerHasBeenOutputed);
    outputStream << endl;

}//OutputCounterStatToStream//


void OutputRealStatToStream(
    RealStatView& statView,
    ostream& outputStream,
    const bool outputFullStats = false,
    const SimTime& startTime = ZERO_TIME,
    const SimTime& endTime = INFINITE_TIME,
    const bool noDataOutputIsEnabled = true)
{
    typedef RealStatView::const_iterator IterType;

    outputStream.fill(' ');//temporary fix for mountain lion libc++'s bug

    const SimTime aggregationInterval = statView.GetAggregationInterval();

    if (!noDataOutputIsEnabled && statView.IsEmpty()) {
        return;
    }//if//

    bool headerHasBeenOutputed = false;

    if (aggregationInterval == ZERO_TIME) {
        if (statView.IsEmpty()) {
            headerHasBeenOutputed = true;
            outputStream << setw(NODEID_MIN_STREAM_OUTPUT_WIDTH) << statView.GetNodeId()
                         << ' ' << statView.GetStatName()
                         << " Avg= -";
        }
        else {
            for(IterType iter = statView.begin(); (!statView.IsEqualToEnd(iter)); ++iter) {
                SimTime datapointTime = iter.TimeForBinEnd();
                if ((datapointTime >= startTime) && (datapointTime <= endTime)) {
                    if (!headerHasBeenOutputed) {
                        headerHasBeenOutputed = true;
                        outputStream << setw(NODEID_MIN_STREAM_OUTPUT_WIDTH) << statView.GetNodeId()
                                     << ' ' << statView.GetStatName()
                                     << " Avg= " << statView.GetAverageOfAllValues() << "  "
                                     << ConvertTimeToStringSecs(datapointTime) << " 0";
                    }//if//
                    outputStream << ' ' << ConvertTimeToStringSecs(iter.TimeForBinEnd())
                                 << ' ' << iter.BinValueAverage();
                }//if//
            }//for//
        }//if//
    }
    else if (aggregationInterval == INFINITE_TIME) {
        headerHasBeenOutputed = true;
        outputStream << setw(NODEID_MIN_STREAM_OUTPUT_WIDTH) << statView.GetNodeId()
                    << ' ' << statView.GetStatName()
                    << " Avg= ";
        if (statView.IsEmpty()) {
            outputStream << "-";
        }
        else {
            outputStream << statView.GetAverageOfAllValues();
        }//if//
    }
    else {
        if (statView.IsEmpty()) {
            const SimTime collectionStartTime = statView.GetCollectionStartTime();
            headerHasBeenOutputed = true;
            outputStream << setw(NODEID_MIN_STREAM_OUTPUT_WIDTH) << statView.GetNodeId()
                         << ' ' << statView.GetStatName()
                         << " Avg= -  " << ConvertTimeToStringSecs(collectionStartTime) << ' '
                         << ConvertTimeToStringSecs(aggregationInterval);
        }
        else {
            for(IterType iter = statView.begin(); (!statView.IsEqualToEnd(iter)); ++iter) {
                SimTime dataBinEndTime = iter.TimeForBinEnd();
                SimTime dataBinStartTime = dataBinEndTime - aggregationInterval;

                if ((dataBinEndTime >= startTime) && (dataBinStartTime <= endTime)) {

                    if (!headerHasBeenOutputed) {
                        headerHasBeenOutputed = true;
                        outputStream << setw(NODEID_MIN_STREAM_OUTPUT_WIDTH) << statView.GetNodeId()
                                     << ' ' << statView.GetStatName()
                                     << " Avg= " << statView.GetAverageOfAllValues() << "  "
                                     << ConvertTimeToStringSecs(dataBinStartTime) << ' '
                                     << ConvertTimeToStringSecs(aggregationInterval);
                    }//if//

                    if (iter.BinNumberDatapoints() == 0) {
                        // No Data for Bin
                        outputStream << " -";
                    }
                    else {
                        if (outputFullStats) {
                            outputStream << ' ' << iter.BinValueMin();
                        }//if//
                        outputStream << ' ' << iter.BinValueAverage();
                        if (outputFullStats) {
                            outputStream << ' ' << iter.BinValueMax();
                        }//if//
                    }//if//

                }//if//
            }//for//
        }//if//

        SimTime lastOutputTime;
        if (statView.IsEmpty()) {
            lastOutputTime = statView.GetCollectionStartTime();
        }
        else {
            lastOutputTime = statView.GetCurrentBinEndTime();
        }//if//
        const SimTime collectionEndTime = statView.GetCollectionEndTime();
        const SimTime outputEndTime = std::min(endTime, collectionEndTime) + aggregationInterval;

        for(SimTime outputTime = (lastOutputTime + aggregationInterval); outputTime <= outputEndTime; outputTime += aggregationInterval) {
            outputStream << " -";
        }//for//

    }//if//

    assert(headerHasBeenOutputed);
    outputStream << endl;


}//OutputRealStatToStream//


void OutputAStatToStream(
    const StatViewCollection& theStatViewCollection,
    const NodeId& theNodeId,
    const string& statName,
    const bool outputFullStats,
    const SimTime& startTime,
    const SimTime& endTime,
    std::ostream& aStream)
{
    typedef map<StatViewCollection::StatViewIdType, shared_ptr<CounterStatView> >::const_iterator IterType;

    StatViewCollection::StatViewIdType statViewId(statName, theNodeId);

    IterType iter = theStatViewCollection.counterStatViewPtrs.find(statViewId);

    if (iter != theStatViewCollection.counterStatViewPtrs.end()) {
        OutputCounterStatToStream(*iter->second, aStream, startTime, endTime);
    }
    else {
        RealStatView& statView = *theStatViewCollection.realStatViewPtrs.find(statViewId)->second;
        OutputRealStatToStream(statView, aStream, outputFullStats, startTime, endTime);
    }//if//

}//OutputAStatToStream//


void OutputStatsToFile(
    const string& outputFileName,
    StatViewCollection& theStatViewCollection,
    const bool noDataOutputIsEnabled,
    const SimTime& startTime,
    const SimTime& endTime)
{
    typedef StatViewCollection::StatViewIdType StatViewIdType;
    typedef map<StatViewIdType, shared_ptr<CounterStatView> >::const_iterator CounterStatIterType;
    typedef map<StatViewIdType, shared_ptr<RealStatView> >::const_iterator RealStatIterType;

    ofstream outputFileStream(outputFileName.c_str(), ios_base::app);
    if (!outputFileStream.good()) {
        cerr << "Error opening output file: " << outputFileName << endl;
        exit(1);
    }//if//

    outputFileStream.precision(STATISTICS_OUTPUT_PRECISION);

    for(CounterStatIterType iter = theStatViewCollection.counterStatViewPtrs.begin();
        (iter != theStatViewCollection.counterStatViewPtrs.end());
        ++iter) {

        CounterStatView& statView = *iter->second;

        OutputCounterStatToStream(
            statView, outputFileStream, startTime, endTime, noDataOutputIsEnabled);

    }//for//

    for(RealStatIterType iter = theStatViewCollection.realStatViewPtrs.begin();
        (iter != theStatViewCollection.realStatViewPtrs.end());
        ++iter) {

        RealStatView& statView = *iter->second;

        OutputRealStatToStream(
            statView, outputFileStream, false/*full stat*/, startTime, endTime, noDataOutputIsEnabled);

    }//for//

}//OutputStatsToFile//



void OutputLastStatValueToStream(
    const StatViewCollection& theStatViewCollection,
    const NodeId& theNodeId,
    const string& statName,
    const SimTime& currentTime,
    std::ostream& aStream)
{
    typedef map<StatViewCollection::StatViewIdType, shared_ptr<CounterStatView> >::const_iterator CounterStatIterType;
    typedef map<StatViewCollection::StatViewIdType, shared_ptr<RealStatView> >::const_iterator RealStatIterType;

    StatViewCollection::StatViewIdType statViewId(statName, theNodeId);

    CounterStatIterType iter = theStatViewCollection.counterStatViewPtrs.find(statViewId);

    if (iter != theStatViewCollection.counterStatViewPtrs.end()) {
        CounterStatView& statView = *iter->second;
        const long long int value = statView.GetCountOfLastBin(currentTime);
        aStream << ' ' << value;
    }
    else {
        RealStatIterType realStatIter = theStatViewCollection.realStatViewPtrs.find(statViewId);

        if (realStatIter != theStatViewCollection.realStatViewPtrs.end()) {

            RealStatView& statView = *realStatIter->second;

            if (statView.IsLastValueViewMode()) {
                aStream <<  ' ' << statView.GetLastRawValue();
            } else {
                double value;
                const bool isValid =
                    statView.GetAverageOfLastBin(currentTime, value);
                if (!isValid) {
                    aStream << ' ' << STAT_NO_DATA_SPECIAL_FIELD;
                } else {
                    aStream << ' ' << value;
                }
            }
        }
        else {
            cerr << "Error: Statistic with name: " << statName << " does not exist." << endl;
            exit(1);
        }//if//
    }//if//

}//OutputLastStatValueToStream//



void OutputLastStatValuesForAListOfNodesToStream(
    const StatViewCollection& guiStatViewCollection,
    const vector<NodeId>& nodeIds,
    const string& statNameString,
    const SimTime& currentTime,
    std::ostream& outStream)
{
    typedef vector<NodeId>::const_iterator NodeIdIterType;

    for(NodeIdIterType iter = nodeIds.begin(); (iter != nodeIds.end()); ++iter) {

        OutputLastStatValueToStream(
            guiStatViewCollection,
            *iter,
            statNameString,
            currentTime,
            outStream);
    }//for//

}//OutputLastStatValuesForAListOfNodesToStream//


void OutputPeriodStatEventsToStream(
    const StatViewCollection& theStatViewCollection,
    const NodeId& theNodeId,
    const string& statName,
    const SimTime& periodStartTime,
    std::ostream& aStream)
{
    typedef map<StatViewCollection::StatViewIdType, shared_ptr<CounterStatView> >::const_iterator CounterStatIterType;
    typedef map<StatViewCollection::StatViewIdType, shared_ptr<RealStatView> >::const_iterator RealStatIterType;

    StatViewCollection::StatViewIdType statViewId(statName, theNodeId);

    CounterStatIterType iter = theStatViewCollection.counterStatViewPtrs.find(statViewId);

    if (iter != theStatViewCollection.counterStatViewPtrs.end()) {
        CounterStatView& statView = *iter->second;

        vector<pair<SimTime, long long int> > events;
        statView.GetPeriodEvents(periodStartTime, events);
        const size_t eventCount = events.size();
        aStream << ' ' << eventCount;
        for (size_t index = 0; index < eventCount; ++index) {
            const pair<SimTime, long long int>& event = events[index];
            aStream << ' ' << event.first << ' ' << event.second;
        }
    }
    else {
        RealStatIterType realStatIter = theStatViewCollection.realStatViewPtrs.find(statViewId);

        if (realStatIter != theStatViewCollection.realStatViewPtrs.end()) {

            RealStatView& statView = *realStatIter->second;

            vector<pair<SimTime, double> > events;
            statView.GetPeriodEvents(periodStartTime, events);
            const size_t eventCount = events.size();
            aStream << ' ' << eventCount;
            for (size_t index = 0; index < eventCount; ++index) {
                const pair<SimTime, double>& event = events[index];
                aStream << ' ' << event.first << ' ' << event.second;
            }
        }
        else {
            cerr << "Error: Statistic with name: " << statName << " does not exist." << endl;
            exit(1);
        }//if//
    }//if//

}//OutputPeriodStatEventsToStream//

void OutputPeriodStatEventsForAListOfNodesToStream(
    const StatViewCollection& guiStatViewCollection,
    const vector<NodeId>& nodeIds,
    const string& statNameString,
    const SimTime& periodStartTime,
    std::ostream& outStream)
{
    typedef vector<NodeId>::const_iterator NodeIdIterType;

    for(NodeIdIterType iter = nodeIds.begin(); (iter != nodeIds.end()); ++iter) {
        OutputPeriodStatEventsToStream(
            guiStatViewCollection,
            *iter,
            statNameString,
            periodStartTime,
            outStream);
    }//for//

}//OutputPeriodStatEventsForAListOfNodesToStream//


void OutputStatNamesToStream(
    const RuntimeStatisticsSystem& statRuntime,
    const NodeId& theNodeId,
    const string& statNameWithWildcards,
    std::ostream& aStream)
{
    typedef StatNameIterator IterType;

    const string prefix = GetWildcardFreePrefix(statNameWithWildcards);

    IterType iter = statRuntime.FindFirst(prefix);

    while (!(iter == statRuntime.end()) &&
           IsEqualCaseInsensitive(prefix, iter.GetName().substr(0, prefix.length()))) {

        if (((theNodeId == ANY_NODEID) || (theNodeId == iter.GetNodeId())) &&
            (IsEqualWithWildcardsCaseInsensitive(statNameWithWildcards, iter.GetName()))) {

            aStream << "  " << iter.GetNodeId() << ' ' << iter.GetName() << ' ';

            if (iter.IsCounterStat()) {
                aStream << "counter";
            }
            else if (iter.IsRealStat()) {
                aStream << "real";
            }
            else {
                assert(false && "Unknown stat type");
            }//if//

        }//if//

        ++iter;

    }//while//

}//OutputAllStatNamesToStream//

}//namespace//



