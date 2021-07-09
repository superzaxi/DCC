// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT11AD_SECTORSELECTOR_H
#define DOT11AD_SECTORSELECTOR_H

#include "dot11ad_common.h"
#include <map>

namespace Dot11ad {

using ScenSim::SimTime;

using std::map;

class BeamformingSectorSelector {
public:
    virtual void StartNewBeaconInterval() = 0;
    virtual void ReceiveSectorSweepCollisionNotification() = 0;

    virtual void ReceiveSectorMetrics(
        const SimTime& time,
        const unsigned int transmittedSectorId,
        const unsigned int receiveSectorId,
        const double& rssiDb,
        const double& sinrDb) = 0;

    virtual void ReceiveTxSectorFeedback(
        const SimTime& time,
        const unsigned int bestTxSectorId) = 0;

    virtual unsigned int GetBestTransmissionSectorId() const = 0;
    virtual unsigned int GetBestReceiveSectorId() const = 0;
    virtual unsigned int GetBestTransmissionSectorIdToThisNode() const = 0;

    virtual double GetBestTransmissionSectorMetric() const = 0;
    virtual double GetBestReceiveSectorMetric() const = 0;

    //virtual double GetRssi(const NodeId& theNodeId) const = 0;
    //virtual double GetSinr(const NodeId& theNodeId) const = 0;

};//BeamformingSectorSelector//



class BeamformingSectorSelectorFactory {
public:
    BeamformingSectorSelectorFactory(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId);

    unique_ptr<BeamformingSectorSelector> CreateSectorSelector();
private:
    string sectorSelectorSchemeName;
};


class RssiBeamformingSectorSelector: public BeamformingSectorSelector {
public:
    RssiBeamformingSectorSelector() { (*this).StartNewBeaconInterval(); }

    virtual void StartNewBeaconInterval() override;

    virtual void ReceiveSectorSweepCollisionNotification() override {}

    virtual void ReceiveSectorMetrics(
        const SimTime& time,
        const unsigned int transmissionSectorId,
        const unsigned int receiveSectorId,
        const double& rssiDb,
        const double& sinrDb) override;

    virtual void ReceiveTxSectorFeedback(
        const SimTime& time,
        const unsigned int bestTxSectorId) override;

    virtual unsigned int GetBestTransmissionSectorId() const override { return (bestTxSectorId); }
    virtual unsigned int GetBestReceiveSectorId() const override { return (bestRxSectorId); }
    virtual unsigned int GetBestTransmissionSectorIdToThisNode() const override
        { return (bestTxSectorIdToThisNode); }

    virtual double GetBestTransmissionSectorMetric() const override { return (bestRssiDb); }
    virtual double GetBestReceiveSectorMetric() const override { assert(false); return 0.0; }

private:
    unsigned int bestTxSectorId;
    unsigned int bestRxSectorId;
    unsigned int bestTxSectorIdToThisNode;
    double bestRssiDb;

};//RssiBeamformingSectorSelector//

inline
void RssiBeamformingSectorSelector::StartNewBeaconInterval()
{
    bestRssiDb = -DBL_MAX;
    bestRxSectorId = QuasiOmniSectorId;
    bestTxSectorId = QuasiOmniSectorId;
    bestTxSectorIdToThisNode = QuasiOmniSectorId;

}//StartNewBeaconInterval//


inline
void RssiBeamformingSectorSelector::ReceiveSectorMetrics(
    const SimTime& time,
    const unsigned int transmissionSectorId,
    const unsigned int receiveSectorId,
    const double& rssiDb,
    const double& sinrDb)
{
    if (rssiDb > bestRssiDb) {
        bestTxSectorIdToThisNode = transmissionSectorId;
        bestRxSectorId = receiveSectorId;
        bestRssiDb = rssiDb;
    }//if//

}//ReceiveSectorMetrics//

inline
void RssiBeamformingSectorSelector::ReceiveTxSectorFeedback(
    const SimTime& time,
    const unsigned int abestTxSectorId)
{
    (*this).bestTxSectorId = abestTxSectorId;
}

//--------------------------------------------------------------------------------------------------

// This is a too simple example:  Sinr should probably be filtered.

class SinrBeamformingSectorSelector: public BeamformingSectorSelector {
public:
    SinrBeamformingSectorSelector() { (*this).StartNewBeaconInterval(); }

    virtual void StartNewBeaconInterval() override;

    // SINR is likely effected by Colliding sector sweeps.
    virtual void ReceiveSectorSweepCollisionNotification() override {}

    virtual void ReceiveSectorMetrics(
        const SimTime& time,
        const unsigned int transmissionSectorId,
        const unsigned int receiveSectorId,
        const double& rssiDb,
        const double& sinrDb) override;

    virtual void ReceiveTxSectorFeedback(
        const SimTime& time,
        const unsigned int bestTxSectorId) override;

    virtual unsigned int GetBestTransmissionSectorId() const override { return (bestTxSectorId); }
    virtual unsigned int GetBestReceiveSectorId() const override { return (bestRxSectorId); }
    virtual unsigned int GetBestTransmissionSectorIdToThisNode() const override
        { return (bestTxSectorIdToThisNode); }

    virtual double GetBestTransmissionSectorMetric() const override { return (bestSinrDb); }
    virtual double GetBestReceiveSectorMetric() const override { assert(false); return 0.0; }

private:
    unsigned int bestTxSectorId;
    unsigned int bestRxSectorId;
    unsigned int bestTxSectorIdToThisNode;
    double bestSinrDb;

};//SinrBeamformingSectorSelector//

inline
void SinrBeamformingSectorSelector::StartNewBeaconInterval()
{
    bestSinrDb = -DBL_MAX;
    bestRxSectorId = QuasiOmniSectorId;
    bestTxSectorId = QuasiOmniSectorId;
    bestTxSectorIdToThisNode = QuasiOmniSectorId;

}//StartNewBeaconInterval//


inline
void SinrBeamformingSectorSelector::ReceiveSectorMetrics(
    const SimTime& time,
    const unsigned int transmissionSectorId,
    const unsigned int receiveSectorId,
    const double& rssiDb,
    const double& sinrDb)
{
    if (sinrDb > bestSinrDb) {
        bestTxSectorIdToThisNode = transmissionSectorId;
        bestRxSectorId = receiveSectorId;
        bestSinrDb = sinrDb;
    }//if//

}//ReceiveSectorMetrics//

inline
void SinrBeamformingSectorSelector::ReceiveTxSectorFeedback(
    const SimTime& time,
    const unsigned int abestTxSectorId)
{
    (*this).bestTxSectorId = abestTxSectorId;
}



//--------------------------------------------------------------------------------------------------

inline
BeamformingSectorSelectorFactory::BeamformingSectorSelectorFactory(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId)
{
    sectorSelectorSchemeName = "rssi";
    if (theParameterDatabaseReader.ParameterExists(
        (adParamNamePrefix + "beamforming-sector-selector-scheme-name"), theNodeId, theInterfaceId)) {

        sectorSelectorSchemeName =
            theParameterDatabaseReader.ReadString(
                (adParamNamePrefix + "beamforming-sector-selector-scheme-name"), theNodeId, theInterfaceId);

        ConvertStringToLowerCase(sectorSelectorSchemeName);

    }//if//

}//BeamformingSectorSelectorFactory//



inline
unique_ptr<BeamformingSectorSelector> BeamformingSectorSelectorFactory::CreateSectorSelector()
{
    if (sectorSelectorSchemeName == "rssi") {
        return (unique_ptr<BeamformingSectorSelector>(new RssiBeamformingSectorSelector()));
    }
    else if (sectorSelectorSchemeName == "sinr") {
        return (unique_ptr<BeamformingSectorSelector>(new SinrBeamformingSectorSelector()));
    }
    else {
        cerr << "Error: Sector Selector Scheme: \"" << sectorSelectorSchemeName << "\" is invalid." << endl;
        exit(1);
        return (unique_ptr<BeamformingSectorSelector>());
    }//if//

}//CreateBeamformingSectorSelector//


}//namespace//

#endif

