// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_STATS_H
#define SCENSIM_STATS_H

#include <vector>
#include <memory>
#include "scensim_parmio.h"
//#include "scensim_engine.h"


namespace ScenSim {

using std::vector;
using std::map;
using std::pair;
using std::shared_ptr;
using std::enable_shared_from_this;

const int STATISTICS_OUTPUT_PRECISION = 10;

class RuntimeStatisticsSystem;
class CounterStatView;

class CounterStatistic {
public:
    bool IsEnabled() const { return statIsEnabled; }

    void SetLinkedStatisticNameAndNodeId(const string& statName, const NodeId& theNodeId);
    string GetLinkedStatisticName() const;
    NodeId GetLinkedStatisticNodeId() const;

    void IncrementCounter(const unsigned long long int incrementNumber = 1);

    // For when counter is kept in the model.
    void UpdateCounter(const long long int newCounterValue);

    ~CounterStatistic();

private:
    friend class RuntimeStatisticsSystem;

    CounterStatistic();

    // For reducing the unused stat overhead to a inlined compare (statIsEnabled) and branch.
    void IncrementCounterInternal(const unsigned long long int);
    void UpdateCounterInternal(const long long int newCounterValue);

    void AddView(const shared_ptr<CounterStatView>& statViewPtr);

    // Abbreviation: impl = implementation

    bool statIsEnabled;

    class Implementation;
    unique_ptr<Implementation> implPtr;

    //Disabling:
    CounterStatistic(const CounterStatistic&);
    void operator=(const CounterStatistic&);

};//CounterStatistic//

inline
void CounterStatistic::IncrementCounter(const unsigned long long int incrementNumber)
{
    if (statIsEnabled) {
        (*this).IncrementCounterInternal(incrementNumber);
    }
}

inline
void CounterStatistic::UpdateCounter(const long long int newCounterValue)
{
    if (statIsEnabled) {
        (*this).UpdateCounterInternal(newCounterValue);
    }
}


class CounterStatViewIterator {
public:
    CounterStatViewIterator(const CounterStatViewIterator& source);
    void operator=(const CounterStatViewIterator& right);

    ~CounterStatViewIterator();

    long long int CounterValue() const;
    long long int CountForCurrentBinOnly() const;
    SimTime TimeForBinEnd() const;

    bool operator==(const CounterStatViewIterator& right) const;
    bool operator!=(const CounterStatViewIterator& right) const { return (!(*this == right)); }

    void operator++();

private:
    friend class CounterStatView;
    CounterStatViewIterator();

    // Abbreviation: impl = implementation
    class Implementation;
    unique_ptr<Implementation> implPtr;

};//CounterStatViewIterator//



class CounterStatView: public enable_shared_from_this<CounterStatView> {
public:
    typedef CounterStatViewIterator const_iterator;
    ~CounterStatView();

    NodeId GetNodeId() const;
    string GetStatName() const;

    void SetCollectionStartTime(const SimTime& startTime);
    void SetCollectionEndTime(const SimTime& endTime);
    void SetAggregationInterval(const SimTime& timeBucketDuration);

    SimTime GetCollectionStartTime() const;
    SimTime GetCollectionEndTime() const;
    SimTime GetAggregationInterval() const;
    SimTime GetCurrentBinEndTime() const;

    bool IsEmpty() const;
    const_iterator begin() const;
    const_iterator end() const;

    bool IsEqualToEnd(const const_iterator& anIterator) const;

    void DeleteData(const SimTime& startTime, const SimTime& endTime);

    long long int CurrentCount() const;
    SimTime CurrentCollectionDuration() const;
    long long int GetCountOfLastBin(const SimTime& currentTime) const;
    void GetPeriodEvents(
        const SimTime& periodStartTime,
        vector<pair<SimTime, long long int> >& result) const;

private:
    CounterStatView(
        const NodeId& theNodeId,
        const string& statName);

    friend class RuntimeStatisticsSystem;
    friend class CounterStatistic;
    friend class CounterStatViewIterator;

    // Abbreviation: impl = implementation
    class Implementation;
    unique_ptr<Implementation> implPtr;

    // Disable:
    CounterStatView(CounterStatView&);
    void operator=(CounterStatView&);

};//CounterStatView//


class RealStatView;

class RealStatistic {
public:
    bool IsEnabled() const { return statIsEnabled; }
    void RecordStatValue(const double& value);

    ~RealStatistic();
private:
    friend class RuntimeStatisticsSystem;
    RealStatistic(const bool initNeedDbConversion);

    // For reducing the unused stat overhead to a inlined compare (statIsEnabled) and branch.
    void RecordStatValueInternal(const double& value);
    void AddView(const shared_ptr<RealStatView>& statViewPtr);

    bool statIsEnabled;
    bool needDbConversion;

    class Implementation;
    unique_ptr<Implementation> implPtr;

    //Disabling:
    RealStatistic(const RealStatistic&);
    void operator=(const RealStatistic&);

};//RealStatistic//

inline
void RealStatistic::RecordStatValue(const double& value)
{
    if (statIsEnabled) {
        (*this).RecordStatValueInternal(value);
    }//if//
}



class RealStatViewIterator {
public:
    RealStatViewIterator(const RealStatViewIterator& source);
    void operator=(const RealStatViewIterator& right);

    ~RealStatViewIterator();

    bool operator==(const RealStatViewIterator& right) const;
    bool operator!=(const RealStatViewIterator& right) const { return (!(*this == right)); }

    void operator++();

    double BinValueMin() const;
    double BinValueMax() const;
    double BinValueAverage() const;
    int BinNumberDatapoints() const;

    SimTime TimeForBinEnd() const;

private:
    friend class RealStatView;
    RealStatViewIterator();

    class Implementation;
    unique_ptr<Implementation> implPtr;

};//RealStatViewIterator//


class RealStatView: public enable_shared_from_this<RealStatView> {
public:
    typedef RealStatViewIterator const_iterator;


    NodeId GetNodeId() const;
    string GetStatName() const;
    bool NeedDbConversion() const;

    void SetCollectionStartTime(const SimTime& startTime);
    void SetCollectionEndTime(const SimTime& endTime);
    void SetAggregationInterval(const SimTime& timeBucketDuration);
    void SetLastValueViewMode();

    SimTime GetCollectionStartTime() const;
    SimTime GetCollectionEndTime() const;
    SimTime GetAggregationInterval() const;
    SimTime GetCurrentBinEndTime() const;

    bool IsEqualToEnd(const const_iterator& anIterator) const;

    bool IsEmpty() const;
    const_iterator begin() const;
    const_iterator end() const;

    bool IsLastValueViewMode() const;

    double GetLastRawValue() const;
    double GetAverageOfAllValues() const;
    const bool GetAverageOfLastBin(
        const SimTime& currentTime,
        double& value) const;
    void GetPeriodEvents(
        const SimTime& startTime,
        vector<pair<SimTime, double> >& result) const;

private:
    friend class RuntimeStatisticsSystem;
    friend class RealStatistic;
    friend class RealStatViewIterator;

    RealStatView(
        const NodeId& theNodeId,
        const string& statName,
        const bool needDbConversion);

    class Implementation;
    unique_ptr<Implementation> implPtr;

};//RealStatView//



////---------------------------------------------------------------------------------------------------
//class SparseArrayRealStatistic {
//public:
//    bool IsEnabled() const { return statIsEnabled; }
//    void RecordStatValue(const int sparse_index, const double& value);
//
//    ~SparseArrayRealArrayStatistic();
//private:
//    friend class RuntimeStatisticsSystem;
//    SparseArrayRealStatistic();
//
//    // For reducing the unused stat overhead to a inlined compare (statIsEnabled) and branch.
//    void RecordStatValueInternal(const double& value);
//    void AddView(const shared_ptr<RealStatView>& statViewPtr);
//
//    bool statIsEnabled;
//
//    class Implementation;
//    unique_ptr<Implementation> implPtr;
//
//    //Disabling:
//    SparseArrayRealStatistic(const SparseArrayRealStatistic&);
//    void operator=(const SparseArrayRealStatistic&);
//
//};//SparseArrayRealStatistic//

//---------------------------------------------------------------------------------------------------



class StatNameIterator {
public:
    StatNameIterator();
    ~StatNameIterator();

    StatNameIterator(const StatNameIterator& source);
    void operator=(const StatNameIterator& right);

    bool operator==(const StatNameIterator& right) const;
    bool operator!=(const StatNameIterator& right) const { return (!(*this == right)); }


    bool IsCounterStat() const;
    bool IsRealStat() const;
    string GetName() const;
    NodeId GetNodeId() const;

    void operator++();

private:
    friend class RuntimeStatisticsSystem;

    // Abbreviation: impl = implementation
    class Implementation;
    unique_ptr<Implementation> implPtr;

};//StatNameIterator//



class SimulationEngineInterface;
struct StatViewCollection;

class RuntimeStatisticsSystem: public enable_shared_from_this<RuntimeStatisticsSystem> {
public:
    RuntimeStatisticsSystem(const bool allowStatReconnection = false);
    ~RuntimeStatisticsSystem();

    void SetAllowStatReconnectionMode();

    // Simulation Model routines

    void CreateCounterStat(
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
        const NodeId& theNodeId,
        const string& statName,
        //const bool useBigInts,
        shared_ptr<CounterStatistic>& counterStatPtr);

    void CreateRealStat(
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
        const NodeId& theNodeId,
        const string& statName,
        //const bool useBigReals,
        const bool needDbConversion,
        shared_ptr<RealStatistic>& realStatPtr);

    void DeleteAllStatsFor(const NodeId& theNodeId);

    // Stat Controller Interface.
    // Runtime access to values.

    StatNameIterator begin() const;
    StatNameIterator end() const;

    StatNameIterator FindFirst(const string& statNamePrefix) const;

    void CreateCounterStatView(
        const NodeId& theNodeId,
        const string& statName,
        //const bool useBigCounter,
        shared_ptr<CounterStatView>& newStatViewPtr);

    void CreateRealStatView(
        const NodeId& theNodeId,
        const string& statName,
        //const bool useBigReal,
        shared_ptr<RealStatView>& newStatViewPtr);

    void CreateCounterStatViewFromCachedStatConfig(
        const NodeId& theNodeId,
        const string& statName,
        shared_ptr<CounterStatistic>& counterStatPtr);

    void CreateRealStatViewFromCachedStatConfig(
        const NodeId& theNodeId,
        const string& statName,
        shared_ptr<RealStatistic>& realStatPtr);

    void CacheStatConfig(
        const NodeId& theNodeId,
        const string& statNameWithWildcards,
        const SimTime& startTime,
        const SimTime& endTime,
        const SimTime& aggregationInterval,
        const bool viewLastValueMode,
        StatViewCollection& theStatViewCollection);

private:
    friend class StatNameIterator;
    //friend class StatNameIterator::Implementation;

    // Abbreviation: impl = implementation

    class Implementation;
    unique_ptr<Implementation> implPtr;

    // Disable:

    RuntimeStatisticsSystem(RuntimeStatisticsSystem&);
    void operator=(RuntimeStatisticsSystem&);

};//RuntimeStatisticsSystem//


//-----------------------------------------------------------------------------

struct StatViewCollection {
    struct StatViewIdType {
        NodeId theNodeId;
        string statName;
        unsigned int instanceId;

        bool operator<(const StatViewIdType& right) const
        {
            if (IsEqualCaseInsensitive(statName, right.statName)) {
                return
                    ((theNodeId < right.theNodeId) ||
                     ((theNodeId == right.theNodeId) && (instanceId < right.instanceId)));
            }
            else {
                return (IsLessThanCaseInsensitive(statName, right.statName));

            }//if//
        }

        StatViewIdType(
            const string& initStatName,
            const NodeId& initNodeId,
            const unsigned int initInstanceId = 0)
            : theNodeId(initNodeId), statName(initStatName), instanceId(initInstanceId)
        { assert(instanceId == 0); }
    };//StatViewIdType//


    map<StatViewIdType, shared_ptr<CounterStatView> > counterStatViewPtrs;
    map<StatViewIdType, shared_ptr<RealStatView> > realStatViewPtrs;

};//StatViewCollection//


void CreateStatViews(
    RuntimeStatisticsSystem& statRuntime,
    const NodeId& theNodeId,
    const string& statNameWithWildcards,
    const SimTime& startTime,
    const SimTime& endTime,
    const SimTime& aggregationInterval,
    const bool viewLastValueMode,
    StatViewCollection& theStatViewCollection);

void ReadStatConfigFile(
    RuntimeStatisticsSystem& statRuntime,
    const string& statConfigFilename,
    StatViewCollection& theStatViewCollection);


void OutputAStatToStream(
    const StatViewCollection& theStatViewCollection,
    const NodeId& theNodeId,
    const string& statName,
    const bool outputFullStats,
    const SimTime& startTime,
    const SimTime& endTime,
    std::ostream& aStream);


void OutputStatsToFile(
    const string& outputFileName,
    StatViewCollection& theStatViewCollection,
    const bool noDataOutputIsEnabled,
    const SimTime& startTime = ZERO_TIME,
    const SimTime& endTime = INFINITE_TIME);


void OutputLastStatValueToStream(
    const StatViewCollection& theStatViewCollection,
    const NodeId& theNodeId,
    const string& statName,
    const SimTime& currentTime,
    std::ostream& aStream);

void OutputLastStatValuesForAListOfNodesToStream(
    const StatViewCollection& guiStatViewCollection,
    const vector<NodeId>& nodeIds,
    const string& statNameString,
    const SimTime& currentTime,
    std::ostream& outStream);


void OutputPeriodStatEventsToStream(
    const StatViewCollection& theStatViewCollection,
    const NodeId& theNodeId,
    const string& statName,
    const SimTime& periodStartTime,
    std::ostream& aStream);

void OutputPeriodStatEventsForAListOfNodesToStream(
    const StatViewCollection& guiStatViewCollection,
    const vector<NodeId>& nodeIds,
    const string& statNameString,
    const SimTime& periodStartTime,
    std::ostream& outStream);


void OutputStatNamesToStream(
    const RuntimeStatisticsSystem& statRuntime,
    const NodeId& theNodeId,
    const string& statName,
    std::ostream& aStream);

}//namespace//



#endif
