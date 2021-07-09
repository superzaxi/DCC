// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11AD_SCHEDULER_H
#define DOT11AD_SCHEDULER_H

#include "scensim_engine.h"
#include "dot11ad_common.h"

namespace Dot11ad {

using ScenSim::SimulationEngineInterface;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeId;
using ScenSim::InterfaceOrInstanceId;
using ScenSim::SimTime;
using ScenSim::ZERO_TIME;

class Dot11ApManagementController;
class Dot11Mac;


// 802.11ad Acronym: DMG = Directional Multi-Gigabit


class DmgScheduler {
public:
    virtual void CalcScheduleForNextBeaconPeriod() = 0;
    virtual SimTime GetBeaconSuperframeIntervalDuration() const = 0;
    virtual SimTime GetBeaconStartTime() const = 0;
    virtual SimTime GetBeaconStartTimeMaxJitter() const = 0;
    virtual SimTime GetDmgBeaconTransmissionIntervalAkaBtiDuration() const = 0;
    virtual SimTime GetAssociationBeamformingTrainingAkaAbftDuration() const = 0;
    virtual SimTime GetDataTransferIntervalAkaDtiDuration() const = 0;
    virtual bool GetShouldDoReceiveSectorSweepInAbft() const = 0;
    virtual bool IsDistributedBeaconTransmissionIntervalAllowed() const = 0;
    virtual unsigned int GetNumSectorSweepSlots() const = 0;
    virtual unsigned int GetNumSectorSweepFrames() const = 0;

    virtual SimTime GetAssociationBeamformingTrainingAkaAbftSlotTime() const = 0;

    virtual unsigned int GetNumberDataTransferIntervalPeriods() const = 0;
    virtual NodeId GetDtiPeriodSourceNodeId(const unsigned int dtiPeriodIndex) const = 0;
    virtual NodeId GetDtiPeriodDestinationNodeId(const unsigned int dtiPeriodIndex) const = 0;
    virtual SimTime GetDtiPeriodRelativeStartTime(const unsigned int dtiPeriodIndex) const = 0;
    virtual SimTime GetDtiPeriodDuration(const unsigned int dtiPeriodIndex) const = 0;
    virtual bool DtiPeriodIsContentionBased(const unsigned int dtiPeriodIndex) const = 0;
    virtual bool MustDoRxssBeamformingTrainingInDtiPeriod(const unsigned int dtiPeriodIndex) const = 0;

};//DmgScheduler//


class StaticScheduleScheduler: public DmgScheduler {
public:
    StaticScheduleScheduler(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId,
        const shared_ptr<Dot11Mac>& macLayerPtr);

    virtual SimTime GetBeaconSuperframeIntervalDuration() const override
        { return (beaconSuperframeIntervalDuration); }

    virtual SimTime GetBeaconStartTime() const override
        { return (beaconStartTime); }

    virtual SimTime GetBeaconStartTimeMaxJitter() const override
        { return (beaconStartTimeMaxJitter); }

    virtual SimTime GetDmgBeaconTransmissionIntervalAkaBtiDuration() const override
        { return (dmgBeaconTransmissionIntervalAkaBtiDuration); }

    virtual SimTime GetAssociationBeamformingTrainingAkaAbftDuration() const override;

    virtual SimTime GetDataTransferIntervalAkaDtiDuration() const override
        { return (dataTransferIntervalAkaDtiDuration); }

    virtual void CalcScheduleForNextBeaconPeriod() override;
    virtual bool GetShouldDoReceiveSectorSweepInAbft() const override
        { return (shouldDoReceiveSectorSweepInAbft); }
    virtual bool IsDistributedBeaconTransmissionIntervalAllowed() const override
        { return (isDistributedBeaconTransmissionIntervalAllowed); }

    virtual unsigned int GetNumSectorSweepSlots() const override;
    virtual unsigned int GetNumSectorSweepFrames() const override;

    virtual SimTime GetAssociationBeamformingTrainingAkaAbftSlotTime() const override;

    virtual unsigned int GetNumberDataTransferIntervalPeriods() const override
        { return (static_cast<unsigned int>(dmgDataTransferIntervalList.size())); }

    virtual NodeId GetDtiPeriodSourceNodeId(const unsigned int dtiPeriodIndex) const override;
    virtual NodeId GetDtiPeriodDestinationNodeId(const unsigned int dtiPeriodIndex) const override;
    virtual SimTime GetDtiPeriodRelativeStartTime(const unsigned int dtiPeriodIndex) const override;
    virtual SimTime GetDtiPeriodDuration(const unsigned int dtiPeriodIndex) const override;
    virtual bool DtiPeriodIsContentionBased(const unsigned int dtiPeriodIndex) const override;
    virtual bool MustDoRxssBeamformingTrainingInDtiPeriod(const unsigned int dtiPeriodIndex) const override;

private:
    shared_ptr<Dot11Mac> macLayerPtr;

    NodeId theNodeId;

    unsigned int numberAntennaSectors;

    SimTime beaconSuperframeIntervalDuration;
    SimTime beaconStartTime;
    SimTime beaconStartTimeMaxJitter;
    SimTime dmgBeaconTransmissionIntervalAkaBtiDuration;
    SimTime dataTransferIntervalAkaDtiDuration;

    //Future SimTime announcementTransmissionIntervalAkaAtiDuration;

    SimTime txssAssociationBeamformingTrainingAkaAbftDuration;
    SimTime rxssAssociationBeamformingTrainingAkaAbftDuration;
    unsigned int associationBeamformingTrainingAkaAbftNumTxssSlots;
    unsigned int associationBeamformingTrainingAkaAbftNumRxssSlots;
    unsigned int maxNumResponderTransmitSectorSweepFrames;

    SimTime associationBeamformingTrainingAkaAbftNumTxssSlotTime;
    SimTime associationBeamformingTrainingAkaAbftNumRxssSlotTime;

    bool isDistributedBeaconTransmissionIntervalAllowed;

    //Future:
    bool shouldDoReceiveSectorSweepInAbft;

    struct DataTransferIntervalListElementType {
        bool isAContentionBasedAccessPeriodAkaCbap;

        // Converted to Association IDs when needed.
        NodeId sourceNodeId;
        NodeId destinationNodeId;

        SimTime relativeStartTime;
        SimTime duration;

        DataTransferIntervalListElementType(
            const bool initIsAContentionBasedAccessPeriodAkaCbap,
            const SimTime& initRelativeStartTime,
            const SimTime& initDuration,
            const NodeId& initSourceNodeId,
            const NodeId& initDestinationNodeId)
        :
            isAContentionBasedAccessPeriodAkaCbap(initIsAContentionBasedAccessPeriodAkaCbap),
            relativeStartTime(initRelativeStartTime),
            duration(initDuration),
            sourceNodeId(initSourceNodeId),
            destinationNodeId(initDestinationNodeId)
        {}
    };

    // Note: Time intervals can overlap (concurrent transmissions / spatial reuse).

    vector<DataTransferIntervalListElementType> dmgDataTransferIntervalList;

    unsigned int beaconCount;
    unsigned int apReceiveSectorSweepInterval;
    unsigned int staReceiveSectorSweepInterval;

};//StaticScheduleScheduler//

inline
void StaticScheduleScheduler::CalcScheduleForNextBeaconPeriod()
{
    (*this).beaconCount++;
    if (apReceiveSectorSweepInterval != 0) {
        (*this).shouldDoReceiveSectorSweepInAbft = ((beaconCount % apReceiveSectorSweepInterval) == 0);
    }//if//
}

inline
SimTime StaticScheduleScheduler::GetAssociationBeamformingTrainingAkaAbftSlotTime() const
{
    if (!shouldDoReceiveSectorSweepInAbft) {
        return associationBeamformingTrainingAkaAbftNumTxssSlotTime;
    }
    else {
        return associationBeamformingTrainingAkaAbftNumRxssSlotTime;
    }//if//
}//GetAssociationBeamformingTrainingAkaAbftSlotTime//

inline
unsigned int StaticScheduleScheduler::GetNumSectorSweepSlots() const
{
    if (!shouldDoReceiveSectorSweepInAbft) {
        return (associationBeamformingTrainingAkaAbftNumTxssSlots);
    }
    else {
        return (associationBeamformingTrainingAkaAbftNumRxssSlots);
    }
}


inline
unsigned int StaticScheduleScheduler::GetNumSectorSweepFrames() const
{
    if (!shouldDoReceiveSectorSweepInAbft) {
        return (maxNumResponderTransmitSectorSweepFrames);
    }
    else {
        return (numberAntennaSectors);
    }//if//
}


inline
SimTime StaticScheduleScheduler::GetAssociationBeamformingTrainingAkaAbftDuration() const
{
    if (!shouldDoReceiveSectorSweepInAbft) {
        return (txssAssociationBeamformingTrainingAkaAbftDuration);
    }
    else {
        return (rxssAssociationBeamformingTrainingAkaAbftDuration);
    }//if//
}



inline
NodeId StaticScheduleScheduler::GetDtiPeriodSourceNodeId(
    const unsigned int dtiPeriodIndex) const
{
    return (dmgDataTransferIntervalList.at(dtiPeriodIndex).sourceNodeId);
}


inline
NodeId StaticScheduleScheduler::GetDtiPeriodDestinationNodeId(
    const unsigned int dtiPeriodIndex) const
{
    return (dmgDataTransferIntervalList.at(dtiPeriodIndex).destinationNodeId);
}


inline
SimTime StaticScheduleScheduler::GetDtiPeriodRelativeStartTime(
    const unsigned int dtiPeriodIndex) const
{
    return (dmgDataTransferIntervalList.at(dtiPeriodIndex).relativeStartTime);
}


inline
SimTime StaticScheduleScheduler::GetDtiPeriodDuration(
    const unsigned int dtiPeriodIndex) const
{
    return (dmgDataTransferIntervalList.at(dtiPeriodIndex).duration);

}


inline
bool StaticScheduleScheduler::DtiPeriodIsContentionBased(
    const unsigned int dtiPeriodIndex) const
{
    return (dmgDataTransferIntervalList.at(dtiPeriodIndex).isAContentionBasedAccessPeriodAkaCbap);
}


inline
bool StaticScheduleScheduler::MustDoRxssBeamformingTrainingInDtiPeriod(
    const unsigned int dtiPeriodIndex) const
{
    const DataTransferIntervalListElementType& intervalInfo =
        dmgDataTransferIntervalList.at(dtiPeriodIndex);

    if ((intervalInfo.isAContentionBasedAccessPeriodAkaCbap) ||
        (intervalInfo.sourceNodeId != theNodeId) ||
        (staReceiveSectorSweepInterval == 0)) {

        return false;
    }
    else {
        return ((beaconCount % staReceiveSectorSweepInterval) == 0);
    }//if//

}//DoRxssBeamformingTrainingInDtiPeriod//


}//namespace//

#endif

