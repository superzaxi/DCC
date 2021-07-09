// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_MOBILITY_H
#define SCENSIM_MOBILITY_H

#include "scensim_proploss.h"
#include "scensim_parmio.h"
#include "scensim_route_search.h"
#include "scensim_gis.h"

#include <vector>
#include <string>
#include <cstdlib>

namespace ScenSim {

using std::vector;
using std::string;

const SimTime DEFAULT_WP_PAUSE_TIME = ZERO_TIME;
const double DEFAULT_WP_MIN_SPEED_METER_PER_SEC = 0.0;
const double DEFAULT_WP_MAX_SPEED_METER_PER_SEC = 5.0;

inline
double CalcStraightDistance(
    const double& x1, const double& y1, const double& z1,
    const double& x2, const double& y2, const double& z2)
{
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    const double dz = z1 - z2;

    return (std::sqrt(dx*dx + dy*dy + dz*dz));
}


inline
double CalcDistanceMeters(
    const ObjectMobilityPosition& position1,
    const ObjectMobilityPosition& position2)
{
    return (
        CalcStraightDistance(
            position1.X_PositionMeters(),
            position1.Y_PositionMeters(),
            position1.HeightFromGroundMeters(),
            position2.X_PositionMeters(),
            position2.Y_PositionMeters(),
            position2.HeightFromGroundMeters()));
}


inline
void CalculateVelocity(
    const double& x1,
    const double& y1,
    const double& z1,
    const double& x2,
    const double& y2,
    const double& z2,
    const SimTime& deltaT,
    double& velocity,
    double& velocityAzimuthDegrees,
    double& velocityElevationDegrees)
{

    assert(deltaT != ZERO_TIME);

    const double deltaX = x2 - x1;
    const double deltaY = y2 - y1;
    const double deltaZ = z2 - z1;

    velocity =
        sqrt((deltaX * deltaX) + (deltaY * deltaY) + (deltaZ * deltaZ)) / ConvertTimeToDoubleSecs(deltaT);


    if((deltaX == 0.0) && (deltaY == 0.0)) {
        velocityAzimuthDegrees = 0.0;
    }
    else {
        velocityAzimuthDegrees =
            NormalizeAngleToNeg180To180(
                -(180.0 / PI) * atan2(deltaY, deltaX) + 90.0);
    }//if//


    const double xyDistanceSquared = sqrt((deltaX * deltaX) + (deltaY * deltaY));

    velocityElevationDegrees =
        NormalizeAngleToNeg180To180(
            (180.0 / PI) * atan2(deltaZ, sqrt(xyDistanceSquared)));

}


inline
void GetWaypointParameters(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId,
    SimTime& startTime,
    SimTime& stopTime,
    SimTime& pauseTime,
    double& minSpeedMeterPerSec,
    double& maxSpeedMeterPerSec,
    string& initPositionFileName)
{
    pauseTime = DEFAULT_WP_PAUSE_TIME;
    minSpeedMeterPerSec = DEFAULT_WP_MIN_SPEED_METER_PER_SEC;
    maxSpeedMeterPerSec = DEFAULT_WP_MAX_SPEED_METER_PER_SEC;

    startTime = ZERO_TIME;
    assert(theParameterDatabaseReader.ParameterExists("simulation-time"));
    stopTime = theParameterDatabaseReader.ReadTime("simulation-time");

    if (theParameterDatabaseReader.ParameterExists(
           "mobility-wp-start-time",
           theNodeId, theInterfaceId)) {
        startTime = theParameterDatabaseReader.ReadTime(
            "mobility-wp-start-time",
            theNodeId, theInterfaceId);
    }
    if (theParameterDatabaseReader.ParameterExists(
           "mobility-wp-pause-time",
           theNodeId, theInterfaceId)) {
        pauseTime = theParameterDatabaseReader.ReadTime(
            "mobility-wp-pause-time",
            theNodeId, theInterfaceId);
    }
    if (theParameterDatabaseReader.ParameterExists(
           "mobility-wp-min-speed-meter-per-sec",
           theNodeId, theInterfaceId)) {
        minSpeedMeterPerSec = theParameterDatabaseReader.ReadDouble(
            "mobility-wp-min-speed-meter-per-sec",
            theNodeId, theInterfaceId);
    }
    if (theParameterDatabaseReader.ParameterExists(
           "mobility-wp-max-speed-meter-per-sec",
           theNodeId, theInterfaceId)) {
        maxSpeedMeterPerSec = theParameterDatabaseReader.ReadDouble(
            "mobility-wp-max-speed-meter-per-sec",
            theNodeId, theInterfaceId);
    }

    assert(theParameterDatabaseReader.ParameterExists(
               "mobility-init-positions-file",
               theNodeId, theInterfaceId));

    initPositionFileName =
        theParameterDatabaseReader.ReadString(
            "mobility-init-positions-file",
            theNodeId, theInterfaceId);

    //error check
    if ((minSpeedMeterPerSec != 0.0) || (maxSpeedMeterPerSec != 0.0)) {

        if (minSpeedMeterPerSec > maxSpeedMeterPerSec) {
            cerr
                << "Error: waypoint mobility, "
                << "The minimum speed is greater than the maximum speed." << endl;
            exit(1);
        }
        if (maxSpeedMeterPerSec <= 0) {
            cerr
                <<"Error: waypoint mobility, "
                << "The maximum speed is lower than zero." << endl;
            exit(1);
        }

    }//if//

}

//--------------------------------------------------------
// Trace
//--------------------------------------------------------


class TraceFileMobilityModel: public ObjectMobilityModel {
public:

    TraceFileMobilityModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId,
        InorderFileCache& fileCache,
        const string& fileName,
        const MobilityObjectId objectId,
        const double& initDistanceGranularityMeters,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        const bool initSupportsCreationAndDeletion = false);

    virtual void GetUnadjustedPositionForTime(
        const SimTime& snapshotTime,
        ObjectMobilityPosition& position) override;

    SimTime GetCreationTime() const override {
        if (supportsCreationAndDeletion &&
            !waypoints.empty()) {
            return waypoints.front().time;
        }
        return ZERO_TIME;
    }

    SimTime GetDeletionTime() const override {
        if (supportsCreationAndDeletion &&
            waypoints.size() > 1) {
            return waypoints.back().time;
        }
        return INFINITE_TIME;
    }

    void SetNeedToAddGroundHeight(const bool newNeedToAddGroundHeight) { needToAddGroundHeight = newNeedToAddGroundHeight; }

private:

    void SetTimeTo(const SimTime& snapshotTime);
    SimTime EarliestNextMoveTime() const { return nextTimeToMove; }
    void ParseLine(const string& aLine, MobilityObjectId& lineObjectId);

    MobilityObjectId objectId;
    bool isStationary;
    bool supportsCreationAndDeletion;

    double distanceGranularityMeters;

    shared_ptr<GisSubsystem> theGisSubsystemPtr;
    bool needToAddGroundHeight;

    SimTime lastMoveTime;
    SimTime nextTimeToMove;
    SimTime currentTimeGranularity;

    double currentXPositionMeters;
    double currentYPositionMeters;
    double currentHeightMeters;
    double currentAttitudeAzimuthDegrees;
    double currentAttitudeElevationDegrees;
    double currentVelocityMetersPerSecond;
    double currentVelocityAzimuthDegrees;
    double currentVelocityElevationDegrees;

    struct TimeAndPositionType {
        SimTime time;
        double xPositionMeters;
        double yPositionMeters;
        double heightMeters;
        double attitudeAzimuthDegrees;
        double attitudeElevationDegrees;

        TimeAndPositionType() : time(INFINITE_TIME) { }
        TimeAndPositionType(
            const SimTime& initTime,
            const double& initXPositionMeters,
            const double& initYPositionMeters,
            const double& initHeightMeters,
            const double& initAttitudeAzimuthDegrees,
            const double& initAttitudeElevationDegrees)
            :
            time(initTime),
            xPositionMeters(initXPositionMeters),
            yPositionMeters(initYPositionMeters),
            heightMeters(initHeightMeters),
            attitudeAzimuthDegrees(initAttitudeAzimuthDegrees),
            attitudeElevationDegrees(initAttitudeElevationDegrees)
        {}

    };//TimeAndPositionType//

    unsigned int currentWaypointIndex;
    vector<TimeAndPositionType> waypoints;

    static SimTime CalculateTimeGranularity(
        const double& distanceGranularityMeters,
        const TimeAndPositionType& waypoint1,
        const TimeAndPositionType& waypoint2);

};//TraceFileMobilityModel//


inline
void TraceFileMobilityModel::GetUnadjustedPositionForTime(
    const SimTime& snapshotTime,
    ObjectMobilityPosition& position)
{
    (*this).SetTimeTo(snapshotTime);

    double heightMeters = currentHeightMeters;

    if (needToAddGroundHeight) {
        heightMeters +=
            theGisSubsystemPtr->GetGroundElevationMetersAt(currentXPositionMeters, currentYPositionMeters);
    }

    position =
        ObjectMobilityPosition(
            lastMoveTime,
            nextTimeToMove,
            currentXPositionMeters,
            currentYPositionMeters,
            heightMeters,
            needToAddGroundHeight/*= theHeightContainsGroundHeightMeters*/,
            currentAttitudeAzimuthDegrees,
            currentAttitudeElevationDegrees,
            currentVelocityMetersPerSecond,
            currentVelocityAzimuthDegrees,
            currentVelocityElevationDegrees);

}//GetPositionForTime//

//--------------------------------------------------------
// Waypoint
//--------------------------------------------------------

class WaypointProperty {
public:
    WaypointProperty(const double& initDistanceGranularityMeters)
        :
        distanceGranularityMeters(initDistanceGranularityMeters),
        isStationary(true),
        currentTimeGranularityForNextMoving(ZERO_TIME),
        currentMobilityPosition(
            ZERO_TIME, ZERO_TIME,
            0.0, 0.0, 0.0, false/*theHeightContainsGroundHeightMeters*/,
            0.0, 0.0, 0.0, 0.0, 0.0),
        currentWaypointIndex(0)
    {}

    void SetInitialWaypoint(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId,
        const string& initialPositionFilePathString,
        InorderFileCache& mobilityFileCache,
        const double& initDistanceGranularityMeters,
        const SimTime& startMovingTime,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr);

    void PushAWaypointIfNecessaryForWaitingUntil(const SimTime& time);

    void PushAWaypointWithSameHeightAndAttitude(
        const double& speedMeterPerSec,
        const double& xPositionMeters,
        const double& yPositionMeters,
        const SimTime& pauseTime);

    void PushAWaypoint(
        const double& speedMeterPerSec,
        const double& xPositionMeters,
        const double& yPositionMeters,
        const double& heightFromGroundMeters,
        const double& attitudeAzimuthDegrees,
        const double& attitudeElevationDegrees,
        const SimTime& pauseTime);

    void ForwardTimeTo(const SimTime& currentTime);

    const ObjectMobilityPosition& GetCurrentMobilityPosition()
        const { return currentMobilityPosition; }

    SimTime GetMovingFinishTime() const {
       assert(!waypoints.empty());
       const TimeAndPosition& lastWaypoint = waypoints.back();
       return lastWaypoint.time;
    }

    bool HasWaypoint() const { return !waypoints.empty(); }
    Vertex GetLastPoint() const {
        assert((*this).HasWaypoint());
        const TimeAndPosition& timeAndPosition = waypoints.back();

        return Vertex(
            timeAndPosition.xPositionMeters,
            timeAndPosition.yPositionMeters,
            timeAndPosition.heightFromGroundMeters);
    }

private:
    struct TimeAndPosition {
        SimTime time;
        double xPositionMeters;
        double yPositionMeters;
        double heightFromGroundMeters;
        double attitudeAzimuthDegrees;
        double attitudeElevationDegrees;

        TimeAndPosition() : time(INFINITE_TIME) {}

        TimeAndPosition(
            const SimTime& initTime,
            const double initXPositionMeters,
            const double initYPositionMeters,
            const double initHeightFromGroundMeters,
            const double initAttitudeAzimuthDegrees,
            const double initAttitudeElevationDegrees)
            :
            time(initTime),
            xPositionMeters(initXPositionMeters),
            yPositionMeters(initYPositionMeters),
            heightFromGroundMeters(initHeightFromGroundMeters),
            attitudeAzimuthDegrees(initAttitudeAzimuthDegrees),
            attitudeElevationDegrees(initAttitudeElevationDegrees)
        {}
    };

    void GetHalfwayNextMobilityPosition(
        const TimeAndPosition& prevWaypoint,
        const TimeAndPosition& nextWaypoint,
        const SimTime& lastMoveTime,
        ObjectMobilityPosition& mobilityPosition) const;

    SimTime CalcTimeGranularity(
        const TimeAndPosition& prevWaypoint,
        const TimeAndPosition& nextWaypoint) const;

private:
    const double distanceGranularityMeters;
    bool isStationary;

    SimTime currentTimeGranularityForNextMoving;

    ObjectMobilityPosition currentMobilityPosition;
    size_t currentWaypointIndex;
    vector<TimeAndPosition> waypoints;
};


inline
void WaypointProperty::SetInitialWaypoint(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId,
    const string& initialPositionFilePathString,
    InorderFileCache& mobilityFileCache,
    const double& initDistanceGranularityMeters,
    const SimTime& startMovingTime,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr)
{
    ObjectMobilityModel::MobilityObjectId mobilityObjectId(theNodeId);

    if (theParameterDatabaseReader.ParameterExists(
            "mobility-trace-file-object-id", theNodeId, theInterfaceId)) {
        mobilityObjectId =
            theParameterDatabaseReader.ReadInt(
                "mobility-trace-file-object-id", theNodeId, theInterfaceId);
    }

    TraceFileMobilityModel traceMobilityForReadingMobilityFile(
        theParameterDatabaseReader,
        theNodeId,
        theInterfaceId,
        mobilityFileCache,
        initialPositionFilePathString,
        mobilityObjectId,
        initDistanceGranularityMeters,
        initGisSubsystemPtr);


    // Ground height will be completed by RandomWaypointMobilityModel::GetUnadjustedPositionForTime()
    traceMobilityForReadingMobilityFile.SetNeedToAddGroundHeight(false/*newNeedToAddGroundHeight*/);


    traceMobilityForReadingMobilityFile.GetUnadjustedPositionForTime(ZERO_TIME, currentMobilityPosition);

    currentMobilityPosition.SetEarliestNextMoveTime(startMovingTime);

    waypoints.push_back(
        TimeAndPosition(
            ZERO_TIME,
            currentMobilityPosition.X_PositionMeters(),
            currentMobilityPosition.Y_PositionMeters(),
            currentMobilityPosition.HeightFromGroundMeters(),
            currentMobilityPosition.AttitudeAzimuthFromNorthClockwiseDegrees(),
            currentMobilityPosition.AttitudeElevationFromHorizonDegrees()));
}


inline
void WaypointProperty::PushAWaypointIfNecessaryForWaitingUntil(
    const SimTime& time)
{
    assert(!waypoints.empty());

    const TimeAndPosition& lastWaypoint = waypoints.back();
    if (time <= lastWaypoint.time) {
        return;
    }

    waypoints.push_back(
        TimeAndPosition(
            time,
            lastWaypoint.xPositionMeters,
            lastWaypoint.yPositionMeters,
            lastWaypoint.heightFromGroundMeters,
            lastWaypoint.attitudeAzimuthDegrees,
            lastWaypoint.attitudeElevationDegrees));
}

inline
void WaypointProperty::PushAWaypointWithSameHeightAndAttitude(
    const double& speedMeterPerSec,
    const double& xPositionMeters,
    const double& yPositionMeters,
    const SimTime& pauseTime)
{
    assert(!waypoints.empty());
    const TimeAndPosition& lastWaypoint = waypoints.back();

    (*this).PushAWaypoint(
        speedMeterPerSec,
        xPositionMeters,
        yPositionMeters,
        lastWaypoint.heightFromGroundMeters,
        lastWaypoint.attitudeAzimuthDegrees,
        lastWaypoint.attitudeElevationDegrees,
        pauseTime);
}

inline
void WaypointProperty::PushAWaypoint(
    const double& speedMeterPerSec,
    const double& xPositionMeters,
    const double& yPositionMeters,
    const double& heightFromGroundMeters,
    const double& attitudeAzimuthDegrees,
    const double& attitudeElevationDegrees,
    const SimTime& pauseTime)
{
    assert(!waypoints.empty());
    const TimeAndPosition lastWaypoint = waypoints.back();

    assert(speedMeterPerSec > 0.0);
    const double distanceMeters =
        CalcStraightDistance(
            lastWaypoint.xPositionMeters,
            lastWaypoint.yPositionMeters,
            lastWaypoint.heightFromGroundMeters,
            xPositionMeters,
            yPositionMeters,
            heightFromGroundMeters);
    const SimTime nextTimeDuration = std::max<SimTime>(
        NANO_SECOND,
        SimTime((distanceMeters / speedMeterPerSec) * SECOND));
    const SimTime nextTime =
        lastWaypoint.time + nextTimeDuration;

    waypoints.push_back(
        TimeAndPosition(
            nextTime,
            xPositionMeters,
            yPositionMeters,
            heightFromGroundMeters,
            attitudeAzimuthDegrees,
            attitudeElevationDegrees));

    if (pauseTime > ZERO_TIME) {
        waypoints.push_back(
            TimeAndPosition(
                nextTime + pauseTime,
                xPositionMeters,
                yPositionMeters,
                heightFromGroundMeters,
                attitudeAzimuthDegrees,
                attitudeElevationDegrees));
    }

    if (isStationary) {
        if (!(lastWaypoint.xPositionMeters == xPositionMeters &&
              lastWaypoint.yPositionMeters == yPositionMeters &&
              lastWaypoint.heightFromGroundMeters == heightFromGroundMeters)) {
            isStationary = false;

            currentTimeGranularityForNextMoving =
                (*this).CalcTimeGranularity(
                    lastWaypoint,
                    waypoints.back());
        }
    }
}


inline
void WaypointProperty::GetHalfwayNextMobilityPosition(
    const TimeAndPosition& prevWaypoint,
    const TimeAndPosition& nextWaypoint,
    const SimTime& snapshotTime,
    ObjectMobilityPosition& mobilityPosition) const
{

    assert(currentTimeGranularityForNextMoving > ZERO_TIME);

    const SimTime timeToGetPosition =
        prevWaypoint.time +
        static_cast<unsigned int>(floor((double)(snapshotTime - prevWaypoint.time) / (double)currentTimeGranularityForNextMoving)) *
        currentTimeGranularityForNextMoving;

    const SimTime nextTimeToMove =
        std::min<SimTime>(
            timeToGetPosition + currentTimeGranularityForNextMoving,
            nextWaypoint.time);

    const double completionRatio =
        double(timeToGetPosition - prevWaypoint.time) /
        double(nextWaypoint.time - prevWaypoint.time);

    const double currentXPositionMeters =
        prevWaypoint.xPositionMeters +
        (completionRatio *
         (nextWaypoint.xPositionMeters - prevWaypoint.xPositionMeters));
    const double currentYPositionMeters =
        prevWaypoint.yPositionMeters +
        (completionRatio *
         (nextWaypoint.yPositionMeters - prevWaypoint.yPositionMeters));
    const double currentHeightFromGroundMeters =
        prevWaypoint.heightFromGroundMeters +
        (completionRatio *
         (nextWaypoint.heightFromGroundMeters - prevWaypoint.heightFromGroundMeters));

    const double currentAzimuth =
        prevWaypoint.attitudeAzimuthDegrees +
        (completionRatio *
         (nextWaypoint.attitudeAzimuthDegrees - prevWaypoint.attitudeAzimuthDegrees));
    const double currentElevation =
        prevWaypoint.attitudeElevationDegrees +
        (completionRatio *
         (nextWaypoint.attitudeElevationDegrees - prevWaypoint.attitudeElevationDegrees));

    mobilityPosition.SetLastMoveTime(timeToGetPosition);
    mobilityPosition.SetEarliestNextMoveTime(nextTimeToMove);

    mobilityPosition.SetX_PositionMeters(currentXPositionMeters);
    mobilityPosition.SetY_PositionMeters(currentYPositionMeters);
    mobilityPosition.SetHeightFromGroundMeters(currentHeightFromGroundMeters);
    mobilityPosition.
        SetAttitudeFromNorthClockwiseDegrees(currentAzimuth);
    mobilityPosition.
        SetAttitudeElevationFromHorizonDegrees(currentElevation);

    //update velocity
    double currentVelocityMetersPerSecond;
    double currentVelocityAzimuthDegrees;
    double currentVelocityElevationDegrees;

    CalculateVelocity(
        prevWaypoint.xPositionMeters,
        prevWaypoint.yPositionMeters,
        prevWaypoint.heightFromGroundMeters,
        nextWaypoint.xPositionMeters,
        nextWaypoint.yPositionMeters,
        nextWaypoint.heightFromGroundMeters,
        nextWaypoint.time - prevWaypoint.time,
        currentVelocityMetersPerSecond,
        currentVelocityAzimuthDegrees,
        currentVelocityElevationDegrees);

    mobilityPosition.SetVelocityMetersPerSecond(currentVelocityMetersPerSecond);
    mobilityPosition.SetVelocityFromNorthClockwiseDegrees(currentVelocityAzimuthDegrees);
    mobilityPosition.SetVelocityElevationFromHorizonDegrees(currentVelocityElevationDegrees);

}

inline
void WaypointProperty::ForwardTimeTo(const SimTime& snapshotTime)
{
    const size_t destWaypointIndex = waypoints.size() - 1;
    if (currentWaypointIndex == destWaypointIndex) {
        return;
    }
    if (snapshotTime < currentMobilityPosition.EarliestNextMoveTime()) {
        return;
    }

    assert(currentWaypointIndex <= destWaypointIndex);
    const TimeAndPosition& lastWaypoint = waypoints.at(currentWaypointIndex);
    const TimeAndPosition& nextWaypoint = waypoints.at(currentWaypointIndex + 1);

    assert(snapshotTime >= lastWaypoint.time);
    assert(nextWaypoint.time > lastWaypoint.time);

    if ((lastWaypoint.time <= snapshotTime) && (snapshotTime < nextWaypoint.time)) {

        (*this).GetHalfwayNextMobilityPosition(
            lastWaypoint,
            nextWaypoint,
            snapshotTime,
            currentMobilityPosition);

    } else {

        while ((currentWaypointIndex < destWaypointIndex) &&
               (snapshotTime > waypoints[currentWaypointIndex + 1].time)) {
            currentWaypointIndex++;
        }
        if (currentWaypointIndex < destWaypointIndex) {
            currentMobilityPosition.SetEarliestNextMoveTime(snapshotTime);

            currentTimeGranularityForNextMoving =
                (*this).CalcTimeGranularity(
                    waypoints[currentWaypointIndex],
                    waypoints[currentWaypointIndex + 1]);

            (*this).ForwardTimeTo(snapshotTime);

        } else {
            currentWaypointIndex = destWaypointIndex;

            const TimeAndPosition& destWaypoint =
                waypoints[destWaypointIndex];

            currentMobilityPosition =
                ObjectMobilityPosition(
                    INFINITE_TIME, INFINITE_TIME,
                    destWaypoint.xPositionMeters,
                    destWaypoint.yPositionMeters,
                    destWaypoint.heightFromGroundMeters,
                    currentMobilityPosition.TheHeightContainsGroundHeightMeters(),
                    destWaypoint.attitudeAzimuthDegrees,
                    destWaypoint.attitudeElevationDegrees,
                    0.0,
                    0.0,
                    0.0);
        }
    }
}

inline
SimTime WaypointProperty::CalcTimeGranularity(
    const TimeAndPosition& prevWaypoint,
    const TimeAndPosition& nextWaypoint) const
{
    const double epsilonTime = (1.0 / INFINITE_TIME);

    const double distance =
        CalcStraightDistance(
            prevWaypoint.xPositionMeters,
            prevWaypoint.yPositionMeters,
            prevWaypoint.heightFromGroundMeters,
            nextWaypoint.xPositionMeters,
            nextWaypoint.yPositionMeters,
            nextWaypoint.heightFromGroundMeters);

    assert(nextWaypoint.time > prevWaypoint.time);
    const SimTime dt = nextWaypoint.time - prevWaypoint.time;
    const double speed = distance / dt;

    if (speed < epsilonTime) {
        return dt;
    }

    return std::min<SimTime>(
        SimTime(distanceGranularityMeters / speed),
        dt);
}

//--------------------------------------------------------------------
// Random waypoint
//--------------------------------------------------------------------

class RandomWaypointMobilityModel : public ObjectMobilityModel {
public:
    RandomWaypointMobilityModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId,
        const RandomNumberGeneratorSeed& runSeed,
        InorderFileCache& mobilityFileCache,
        const double initDistanceGranularityMeters,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr);

    virtual void GetUnadjustedPositionForTime(
        const SimTime& snapshotTime,
        ObjectMobilityPosition& position)
    {
        if (isStationary) {
            position = stationaryPosition;
        }
        else {
            theWaypointProperty.ForwardTimeTo(snapshotTime);
            position = theWaypointProperty.GetCurrentMobilityPosition();
        }//if//

        position.SetTheHeightContainsGroundHeightMeters(needToAddGroundHeight);

        if (needToAddGroundHeight) {
            position.SetHeightFromGroundMeters(// ==> height above sea level
                theGisSubsystemPtr->GetGroundElevationMetersAt(
                    position.X_PositionMeters(),
                    position.Y_PositionMeters()) +
                position.HeightFromGroundMeters());
        }//if//
    }


private:
    void GetAreaParameter(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId,
        vector<Vertex>& movableAreaPolygon) const;

    static const int SEED_HASHING_INPUT = 35620163;

    shared_ptr<GisSubsystem> theGisSubsystemPtr;
    bool needToAddGroundHeight;

    WaypointProperty theWaypointProperty;

    bool isStationary;
    ObjectMobilityPosition stationaryPosition;
    void SetStationaryPosition(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId,
        const ObjectMobilityModel::MobilityObjectId& mobilityObjectId,
        const string& initialPositionFileName,
        const double& initDistanceGranularityMeters,
        InorderFileCache& mobilityFileCache);
};

static inline
Vertex GetRandomPositionInPolygon(
    const vector<Vertex>& polygon,
    RandomNumberGenerator& aRandomNumberGenerator)
{
    double minX = DBL_MAX;
    double minY = DBL_MAX;
    double maxX = -DBL_MAX;
    double maxY = -DBL_MAX;

    for(size_t i = 0; i < polygon.size(); i++) {
        const double x = polygon[i].x;
        const double y = polygon[i].y;

        minX = std::min(minX, x);
        minY = std::min(minY, y);
        maxX = std::max(maxX, x);
        maxY = std::max(maxY, y);
    }

    const Rectangle minRect(minX, minY, maxX, maxY);
    const int maxTryCount = 10;

    Vertex randomPosition;

    for(int i = 0; i < maxTryCount; i++) {
        randomPosition.x = minRect.minX + (minRect.maxX - minRect.minX)*aRandomNumberGenerator.GenerateRandomDouble();
        randomPosition.y = minRect.minY + (minRect.maxY - minRect.minY)*aRandomNumberGenerator.GenerateRandomDouble();

        if (PolygonContainsPoint(polygon, randomPosition)) {
            break;
        }
    }

    if (!PolygonContainsPoint(polygon, randomPosition)) {
        randomPosition = polygon[aRandomNumberGenerator.GenerateRandomInt(0, static_cast<int32_t>(polygon.size() - 1))];
    }

    // Set z value to 0
    // Ground height will be adjusted at GetUnadjustedPositionForTime()

    randomPosition.z = 0.;

    return randomPosition;
}

static inline
void ReplaceChar(string& aString, const char before, const char after)
{
    for(size_t i = 0; i < aString.length(); i++) {
        if (aString[i] == before) {
            aString[i] = after;
        }
    }
}

inline
void RandomWaypointMobilityModel::GetAreaParameter(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId,
    vector<Vertex>& movableAreaPolygon) const
{
    movableAreaPolygon.clear();

    bool success;
    vector<double> doubleValues;

    if (theParameterDatabaseReader.ParameterExists(
            "mobility-rwp-movable-area-gis-object-name",
            theNodeId, theInterfaceId)) {

        const string areaString = theParameterDatabaseReader.ReadString(
                "mobility-rwp-movable-area-gis-object-name", theNodeId, theInterfaceId);

        if (!theGisSubsystemPtr->ContainsPosition(areaString)) {
            cerr << "Error: There is no position : " << areaString << endl
                 << "       Specify available gis object name for random waypoint movable area." << endl;
             exit(1);
        }

        const GisPositionIdType positionId =
            theGisSubsystemPtr->GetPosition(areaString);

        switch (positionId.type) {
        case GIS_BUILDING:
            movableAreaPolygon = theGisSubsystemPtr->GetBuilding(positionId.id).GetBuildingPolygon();
            break;
        case GIS_PARK:
            movableAreaPolygon = theGisSubsystemPtr->GetPark(positionId.id).GetPolygon();
            break;
        case GIS_AREA:
            movableAreaPolygon = theGisSubsystemPtr->GetArea(positionId.id).GetPolygon();
            break;
        default:
            cerr << "MOBILITY-RWP-MOVABLE-AREA must be a polygon gis object." << endl;
            exit(1);
            break;
        }

    } else {
        string areaString =
            theParameterDatabaseReader.ReadString(
                "mobility-rwp-movable-area-min-xy-max-xy-meters",
                theNodeId, theInterfaceId);

        ReplaceChar(areaString, ',', ' ');

        ConvertAStringSequenceOfANumericTypeIntoAVector<double>(
            areaString, success, doubleValues);

        if ((!success) || (doubleValues.size() != 4)) {
            cerr
                << "mobility-rwp-movable-area-min-xy-max-xy-meters is "
                << "movable extent CSV(minx, miny, maxx, maxy) string" << endl;
            return;
        }

        const double minX = doubleValues[0];
        const double minY = doubleValues[1];
        const double maxX = doubleValues[2];
        const double maxY = doubleValues[3];

        movableAreaPolygon.push_back(Vertex(minX, minY));
        movableAreaPolygon.push_back(Vertex(minX, maxY));
        movableAreaPolygon.push_back(Vertex(maxX, maxY));
        movableAreaPolygon.push_back(Vertex(maxX, minY));
        movableAreaPolygon.push_back(Vertex(minX, minY));
    }
}


inline
RandomWaypointMobilityModel::RandomWaypointMobilityModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId,
    const RandomNumberGeneratorSeed& runSeed,
    InorderFileCache& mobilityFileCache,
    const double initDistanceGranularityMeters,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr)
    :
    ObjectMobilityModel(theParameterDatabaseReader, theNodeId, theInterfaceId),
    theGisSubsystemPtr(initGisSubsystemPtr),
    needToAddGroundHeight(true),
    theWaypointProperty(initDistanceGranularityMeters),
    isStationary(false)
{
   if (theParameterDatabaseReader.ParameterExists(
           "mobility-need-to-add-ground-height", theNodeId, theInterfaceId)) {
       needToAddGroundHeight =
            theParameterDatabaseReader.ReadBool(
                "mobility-need-to-add-ground-height", theNodeId, theInterfaceId);
    }//if//

    SimTime startMovingTime;
    SimTime stopMovingTime;
    SimTime pauseTime;
    double minSpeedMeterPerSec;
    double maxSpeedMeterPerSec;
    string initPositionFileName;
    GetWaypointParameters(
        theParameterDatabaseReader, theNodeId, theInterfaceId,
        startMovingTime, stopMovingTime, pauseTime,
        minSpeedMeterPerSec, maxSpeedMeterPerSec,
        initPositionFileName);

    if ((minSpeedMeterPerSec == 0.0) && (maxSpeedMeterPerSec == 0.0)) {

        isStationary = true;

        SetStationaryPosition(
            theParameterDatabaseReader,
            theNodeId,
            theInterfaceId,
            theNodeId,
            initPositionFileName,
            initDistanceGranularityMeters,
            mobilityFileCache);

        stationaryPosition.SetEarliestNextMoveTime(stopMovingTime);

        return;
    }

    theWaypointProperty.SetInitialWaypoint(
        theParameterDatabaseReader,
        theNodeId,
        theInterfaceId,
        initPositionFileName,
        mobilityFileCache,
        initDistanceGranularityMeters,
        startMovingTime,
        theGisSubsystemPtr);

    vector<Vertex> movableAreaPolygon;
    (*this).GetAreaParameter(
        theParameterDatabaseReader,
        theNodeId,
        theInterfaceId,
        movableAreaPolygon);

    if (movableAreaPolygon.size() > 3 && movableAreaPolygon.front() == movableAreaPolygon.back()) {
        const RandomNumberGeneratorSeed nodeSeed =
            HashInputsToMakeSeed(runSeed, theNodeId);
        RandomNumberGenerator aRandomNumberGenerator(
            HashInputsToMakeSeed(nodeSeed, theInterfaceId, SEED_HASHING_INPUT));

        const double deltaSpeedMeterPerSec =
            maxSpeedMeterPerSec - minSpeedMeterPerSec;

        assert(deltaSpeedMeterPerSec >= 0);

        // push random points
        while (theWaypointProperty.GetMovingFinishTime() < stopMovingTime) {

            Vertex nextPosition =
                GetRandomPositionInPolygon(movableAreaPolygon, aRandomNumberGenerator);

            if (theWaypointProperty.HasWaypoint()) {
                const Vertex& lastPoint = theWaypointProperty.GetLastPoint();
                vector<Vertex> intersectionPoints;

                GetIntersectionPositionBetweenLineAndPolygon(
                    lastPoint, nextPosition, movableAreaPolygon, intersectionPoints);

                if (intersectionPoints.size() >= 3) {
                    continue;
                }

                double minDistance = DBL_MAX;

                for(size_t i = 0; i < intersectionPoints.size(); i++) {
                    const Vertex& intersectionPoint = intersectionPoints[i];
                    const double distance = lastPoint.DistanceTo(intersectionPoint);

                    if (distance < minDistance && distance > initDistanceGranularityMeters) {
                        minDistance = distance;
                        nextPosition = intersectionPoint;
                    }
                }
            }

            double nextSpeedMeterPerSec;
            do {
                nextSpeedMeterPerSec =
                    minSpeedMeterPerSec +
                    deltaSpeedMeterPerSec * aRandomNumberGenerator.GenerateRandomDouble();
            } while (nextSpeedMeterPerSec <= 0);

            theWaypointProperty.PushAWaypointWithSameHeightAndAttitude(
                nextSpeedMeterPerSec,
                nextPosition.x,
                nextPosition.y,
                pauseTime);
        }

    } else {
        cerr
            << "Note : mobility-rwp-movable-area-min-xy-max-xy-meters is "
            << "invalid for Random waypoint mobility."
            << "Assume Node(" << theNodeId << ")" << "as a static node." << endl;
    }
}


inline
void RandomWaypointMobilityModel::SetStationaryPosition(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId,
    const ObjectMobilityModel::MobilityObjectId& mobilityObjectId,
    const string& initialPositionFileName,
    const double& initDistanceGranularityMeters,
    InorderFileCache& mobilityFileCache)
{

    TraceFileMobilityModel initialPositionMobilityFile(
        theParameterDatabaseReader,
        theNodeId,
        theInterfaceId,
        mobilityFileCache,
        initialPositionFileName,
        mobilityObjectId,
        initDistanceGranularityMeters,
        theGisSubsystemPtr);

    initialPositionMobilityFile.GetUnadjustedPositionForTime(ZERO_TIME, stationaryPosition);

}//SetInitialWaypoint//


enum TransportationType {
    TRANSPORT_PAUSE,
    TRANSPORT_FREESPACE,
    TRANSPORT_ROAD,
    TRANSPORT_RAILROAD
};


struct GisBasedWaypoint {
    GisObjectIdType objectId;
    Vertex position;
    TransportationType transportationType;
    double speed;

    GisBasedWaypoint()
        :
        objectId(INVALID_GIS_OBJECT_ID),
        position(0,0,0),
        transportationType(TRANSPORT_PAUSE),
        speed(0.0)
    {}

    GisBasedWaypoint(
        const GisObjectIdType& initObjectId,
        const Vertex& initPosition,
        const TransportationType& initTransportationType,
        const double& initSpeed)
        :
        objectId(initObjectId),
        position(initPosition),
        transportationType(initTransportationType),
        speed(initSpeed)
    {}

};

//--------------------------------------------------------------------
// GIS-based random waypoint mobility model
//--------------------------------------------------------------------

class GisBasedRandomWaypointMobilityModel : public ObjectMobilityModel {
public:
    GisBasedRandomWaypointMobilityModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ObjectMobilityModel::MobilityObjectId& mobilityObjectId,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId,
        const RandomNumberGeneratorSeed& runSeed,
        InorderFileCache& mobilityFileCache,
        const double& initDistanceGranularityMeters,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr);


    virtual void GetUnadjustedPositionForTime(
        const SimTime& snapshotTime,
        ObjectMobilityPosition& position) override
    {
        // velocityAzimuth and velocityElevation are same as
        // attitudeAzimuth and attitudeElevation in this model.

        if (isStationary || (snapshotTime == ZERO_TIME)) {

            // no position change
        }
        else {
            //regular
            SetTimeTo(snapshotTime);

        }//if//

        position = ObjectMobilityPosition(
            lastMoveTime,
            nextTimeToMove,
            currentPosition.positionX,
            currentPosition.positionY,
            currentPosition.positionZ,
            true/*theHeightContainsGroundHeightMeters*/,
            currentAttitudeAzimuthDegrees,
            currentAttitudeElevationDegrees,
            currentVelocityMetersPerSecond,
            currentAttitudeAzimuthDegrees,
            currentAttitudeElevationDegrees);

    }//GetAttachedAntennaMobilityModelPositionForTime//

private:
    shared_ptr<GisSubsystem> gisSubsystemPtr;

    map<SimTime, GisBasedWaypoint> gisBasedWaypoints;

    static const int SEED_HASH = 35620335;
    RandomNumberGenerator aRandomNumberGenerator;

    bool isStationary;

    GisObjectType gisObjectType;

    double distanceGranularityMeters;

    double minSpeedMeterPerSec;
    double maxSpeedMeterPerSec;

    MobilityObjectId objectId;

    SimTime lastMoveTime;
    SimTime nextTimeToMove;

    Vertex currentPosition;
    double currentAttitudeAzimuthDegrees;
    double currentAttitudeElevationDegrees;
    double currentVelocityMetersPerSecond;
    double additionalFixedHeightFromGroundMeters;

    double offsetMeters;

    void SetTimeTo(const SimTime& snapshotTime);

    void SetInitialPosition(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId,
        const ObjectMobilityModel::MobilityObjectId& mobilityObjectId,
        const string& initialPositionFileName,
        InorderFileCache& mobilityFileCache);

    void SetInitialWaypoint(
        const SimTime& startTime,
        const SimTime& pauseTime,
        const double& minSpeedMeterPerSec,
        const double& maxSpeedMeterPerSec);

    void GenerateRandomWaypoints(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const SimTime& stopMovingTime,
        const SimTime& pauseTime,
        const double& minSpeedMeterPerSec,
        const double& maxSpeedMeterPerSec,
        const bool routeSearchBasedAlgorithm);

    void InsertRoadIntersectionWaypoint(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GisObjectIdType currentGisObjectId,
        const GisObjectIdType nextGisObjectId,
        const double& currentSpeedMeterPerSec,
        const SimTime& pauseTime,
        const SimTime& stopMovingTime,
        SimTime& currentTime);

    void InsertRoadIntersectionWaypointWithOneHop(
        const GisObjectIdType nextGisObjectId,
        const RoadIdType& roadId,
        const double& currentSpeedMeterPerSec,
        const SimTime& pauseTime,
        const SimTime& stopMovingTime,
        SimTime& currentTime);

    const double GetARandomSpeedMeterPerSec() {

        const double deltaSpeed = maxSpeedMeterPerSec - minSpeedMeterPerSec;

        return (minSpeedMeterPerSec + deltaSpeed * aRandomNumberGenerator.GenerateRandomDouble());
    }

    //debug
    void OutputGisBasedDatabase() const;

};


inline
void GisBasedRandomWaypointMobilityModel::OutputGisBasedDatabase() const
{

    map<SimTime, GisBasedWaypoint>::const_iterator iter =
        gisBasedWaypoints.begin();

    for (; iter != gisBasedWaypoints.end(); ++iter) {

        cout << ConvertTimeToStringSecs(iter->first)
             << " id:" << (iter->second).objectId
             << " (" << (iter->second).position.positionX
             << "," << (iter->second).position.positionY
             << "," << (iter->second).position.positionZ
             << ") Speed:" << (iter->second).speed;

        switch((iter->second).transportationType) {
        case TRANSPORT_PAUSE: {
            cout << " PAUSE" << endl;
            break;
        }
        case TRANSPORT_FREESPACE: {
            cout << " FREESPACE" << endl;
            break;
        }
        case TRANSPORT_ROAD: {
            cout << " ROAD" << endl;
            break;
        }
        case TRANSPORT_RAILROAD: {
            cout << " RAILROAD" << endl;
            break;
        }
        default:
            assert(false);
            exit(1);
        }//switch//

    }//for//

}//OutputGisBasedDatabase//


inline
GisBasedRandomWaypointMobilityModel::GisBasedRandomWaypointMobilityModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ObjectMobilityModel::MobilityObjectId& mobilityObjectId,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId,
    const RandomNumberGeneratorSeed& runSeed,
    InorderFileCache& mobilityFileCache,
    const double& initDistanceGranularityMeters,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr)
    :
    ObjectMobilityModel(theParameterDatabaseReader, theNodeId, theInterfaceId),
    gisSubsystemPtr(initGisSubsystemPtr),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(HashInputsToMakeSeed(runSeed, theNodeId), theInterfaceId, SEED_HASH)),
    gisObjectType(GIS_ROAD),
    isStationary(false),
    objectId(mobilityObjectId),
    distanceGranularityMeters(initDistanceGranularityMeters),
    minSpeedMeterPerSec(0.0),
    maxSpeedMeterPerSec(0.0),
    lastMoveTime(ZERO_TIME),
    nextTimeToMove(ZERO_TIME),
    currentPosition(),
    currentAttitudeAzimuthDegrees(0.0),
    currentAttitudeElevationDegrees(0.0),
    currentVelocityMetersPerSecond(0.0),
    additionalFixedHeightFromGroundMeters(0.0),
    offsetMeters(0.0)
{
    SimTime startMovingTime;
    SimTime stopMovingTime;
    SimTime pauseTime;
    string initPositionFileName;

    GetWaypointParameters(
        theParameterDatabaseReader,
        theNodeId,
        theInterfaceId,
        startMovingTime,
        stopMovingTime,
        pauseTime,
        minSpeedMeterPerSec,
        maxSpeedMeterPerSec,
        initPositionFileName);

    string groundObjectTypeName =
        theParameterDatabaseReader.ReadString("mobility-gis-ground-object-type", theNodeId, theInterfaceId);
    ConvertStringToLowerCase(groundObjectTypeName);

    if (groundObjectTypeName.find(GIS_RAILROAD_STRING) != string::npos) {
        //Note: should this statement at top because "railroad" includes "road"
        //railroad
        gisObjectType = GIS_RAILROAD;
        assert(false && "invalid gis type");
        exit(1);
    }
    else if (groundObjectTypeName.find(GIS_ROAD_STRING) != string::npos) {
        //road
        gisObjectType = GIS_ROAD;

        if (theParameterDatabaseReader.ParameterExists("gis-road-set-intersection-margin")) {
            if (theParameterDatabaseReader.ReadBool("gis-road-set-intersection-margin")) {
                cerr << "Error in GIS-Based-Random-Waypoint: Set gis-road-set-intersection-margin = false" << endl;
                exit(1);
            }
        }
    }
    else {
        cerr
            << "Error: MOBILITY-GIS-OBJECT-TYPE "
            << groundObjectTypeName
            << " is not supported." << endl;
        exit(1);
    }//if//

    bool routeSearchBasedAlgorithm = false;
    if (theParameterDatabaseReader.ParameterExists("mobility-route-search-based-algorithm", theNodeId, theInterfaceId)) {
        routeSearchBasedAlgorithm =
            theParameterDatabaseReader.ReadBool("mobility-route-search-based-algorithm", theNodeId, theInterfaceId);
    }

    if (theParameterDatabaseReader.ParameterExists("mobility-lane-offset-meters", theNodeId, theInterfaceId)) {
        const double signedOffsetMeters =
            theParameterDatabaseReader.ReadDouble("mobility-lane-offset-meters", theNodeId, theInterfaceId);
        offsetMeters = std::abs(signedOffsetMeters);//for 1.4 compatibility
        if (!gisSubsystemPtr->IsRightHandTrafficRoad()) {
            offsetMeters *= -1;
        }
    }

    SetInitialPosition(
        theParameterDatabaseReader,
        theNodeId,
        theInterfaceId,
        mobilityObjectId,
        initPositionFileName,
        mobilityFileCache);

    if ((minSpeedMeterPerSec == 0.0) && (maxSpeedMeterPerSec == 0.0)) {

        isStationary = true;
        nextTimeToMove = stopMovingTime;
        return;
    }//if//

    SetInitialWaypoint(
        startMovingTime,
        pauseTime,
        minSpeedMeterPerSec,
        maxSpeedMeterPerSec);

    GenerateRandomWaypoints(
        theParameterDatabaseReader,
        stopMovingTime,
        pauseTime,
        minSpeedMeterPerSec,
        maxSpeedMeterPerSec,
        routeSearchBasedAlgorithm);

    //OutputGisBasedDatabase();

}


inline
void GisBasedRandomWaypointMobilityModel::SetTimeTo(const SimTime& snapshotTime)
{
    //optimization
    if (snapshotTime < nextTimeToMove) {
        return;
    }

    assert(gisBasedWaypoints.size() >= 2);

    map<SimTime, GisBasedWaypoint>::const_iterator nextIter =
        gisBasedWaypoints.lower_bound(snapshotTime);

    //set to the last position
    if (nextIter == gisBasedWaypoints.end()) {
        assert(nextIter != gisBasedWaypoints.begin());
        nextIter--;

        const GisBasedWaypoint currentGisWaypoint = (*nextIter).second;

        lastMoveTime = (*nextIter).first;

        assert(nextIter != gisBasedWaypoints.begin());
        nextIter--;

        const GisBasedWaypoint prevGisWaypoint = (*nextIter).second;


        CalculateAzimuthAndElevationDegrees(
            prevGisWaypoint.position,
            currentGisWaypoint.position,
            currentAttitudeAzimuthDegrees,
            currentAttitudeElevationDegrees);

        currentPosition = currentGisWaypoint.position;
        currentVelocityMetersPerSecond = 0;
        nextTimeToMove = INFINITE_TIME;

        return;
    }//if//

    if (nextIter->first == snapshotTime) {

        //nextIter is current

        const GisBasedWaypoint& currentGisWaypoint = nextIter->second;

        //update move time
        lastMoveTime = snapshotTime;

        map<SimTime, GisBasedWaypoint>::const_iterator continuingIter = nextIter;
        continuingIter++;

        if (continuingIter != gisBasedWaypoints.end()) {

            const SimTime& nextPointTime = continuingIter->first;

            if (currentGisWaypoint.transportationType == TRANSPORT_PAUSE) {
                //update next move time
                nextTimeToMove = nextPointTime;

                //update velocity
                currentVelocityMetersPerSecond = 0.0;

            }
            else {

                assert(currentGisWaypoint.speed != 0.0);
                const SimTime nextDeltaTime =
                    ConvertDoubleSecsToTime(distanceGranularityMeters / currentGisWaypoint.speed);
                //update next move time
                nextTimeToMove = std::min(nextPointTime, (snapshotTime + nextDeltaTime));

                //update velocity
                currentVelocityMetersPerSecond = currentGisWaypoint.speed;

            }//if//

        }//if//

        //update position
        currentPosition = currentGisWaypoint.position;
    }
    else {

        assert(nextIter != gisBasedWaypoints.begin());

        map<SimTime, GisBasedWaypoint>::const_iterator currentIter = nextIter;
        currentIter--;

        const GisBasedWaypoint& currentGisWaypoint = currentIter->second;
        const GisBasedWaypoint& nextGisWaypoint = nextIter->second;

        const SimTime previousPointTime = currentIter->first;
        const SimTime nextPointTime = nextIter->first;

        if (currentGisWaypoint.transportationType == TRANSPORT_PAUSE) {

            //update move time
            lastMoveTime = previousPointTime;
            //update next move time
            nextTimeToMove = nextPointTime;

            //update position
            currentPosition = currentGisWaypoint.position;
            //update velocity
            currentVelocityMetersPerSecond = 0.0;
        }
        else {

            assert(currentGisWaypoint.speed != 0.0);

            const SimTime deltaTime =
                ConvertDoubleSecsToTime(distanceGranularityMeters / currentGisWaypoint.speed);

            const SimTime timeToGetPosition =
                previousPointTime +
                static_cast<unsigned int>(floor((double)(snapshotTime - previousPointTime) / (double)deltaTime)) *
                deltaTime;

            //update move time
            lastMoveTime = timeToGetPosition;
            //update next move time
            nextTimeToMove = std::min(nextPointTime, (timeToGetPosition + deltaTime));

            assert(previousPointTime != nextPointTime);

            const double completionRatio =
                double(timeToGetPosition - previousPointTime) /
                double(nextPointTime - previousPointTime);

            Vertex startVertex;
            Vertex endVertex;

            switch(currentGisWaypoint.transportationType) {
            case TRANSPORT_FREESPACE: {

                startVertex = currentGisWaypoint.position;
                endVertex = nextGisWaypoint.position;

                //update position
                currentPosition.positionX =
                    startVertex.positionX + (endVertex.positionX - startVertex.positionX) * completionRatio;
                currentPosition.positionY =
                    startVertex.positionY + (endVertex.positionY - startVertex.positionY) * completionRatio;
                break;
            }
            case TRANSPORT_ROAD: {

                //update position
                const Intersection& startIntersection =
                    gisSubsystemPtr->GetIntersectionObject(currentGisWaypoint.objectId);
                const Intersection& endIntersection =
                    gisSubsystemPtr->GetIntersectionObject(nextGisWaypoint.objectId);

                const IntersectionIdType startIntersecionId =
                    startIntersection.GetIntersectionId();

                const Road& road = gisSubsystemPtr->GetRoad(
                    startIntersection.GetRoadIdTo(endIntersection.GetIntersectionId()));

                const double completeDistance =
                    road.GetArcDistanceMeters(false/*calculate3dArcDistance*/) * completionRatio;

                const ConstReverseAccess<Vertex> vertices(
                    road.GetCompleteVertices(),
                    (startIntersecionId != road.GetStartIntersectionId()));

                const size_t numberOfVertices = vertices.size();

                double restOfDistance = completeDistance;

                for (size_t i = 0; i < (numberOfVertices - 1); i++) {
                    startVertex = vertices[i];
                    endVertex = vertices[i+1];

                    const double pointToPointDistance =
                        sqrt(SquaredXYDistanceBetweenVertices(startVertex, endVertex));

                    if (pointToPointDistance < restOfDistance) {
                        restOfDistance -= pointToPointDistance;
                    }
                    else {
                        const double restCompletionRatio  =
                            restOfDistance / pointToPointDistance;

                        currentPosition.positionX =
                            startVertex.positionX + (endVertex.positionX - startVertex.positionX) * restCompletionRatio;
                        currentPosition.positionY =
                            startVertex.positionY + (endVertex.positionY - startVertex.positionY) * restCompletionRatio;
                        break;

                    }//if//

                }//for//

                break;
            }
            case TRANSPORT_RAILROAD: {
                //TBD
            }
            default:
                assert(false && "invalid transportation type");
                exit(1);
            }//switch//

            //update azimuth and elevation
            CalculateAzimuthAndElevationDegrees(
                startVertex,
                endVertex,
                currentAttitudeAzimuthDegrees,
                currentAttitudeElevationDegrees);

            //update velocity
            currentVelocityMetersPerSecond = currentGisWaypoint.speed;

        }//if//

    }//if//

    const GroundLayer& groundLayer = *gisSubsystemPtr->GetGroundLayerPtr();

    //calculate lane offset
    const double theta = currentAttitudeAzimuthDegrees * PI / 180.0;
    const double deltaX = offsetMeters * cos(theta);
    const double deltaY = - offsetMeters * sin(theta);

    currentPosition.positionX += deltaX;
    currentPosition.positionY += deltaY;

    // forceby adjust position z
    currentPosition.positionZ =
        groundLayer.GetElevationMetersAt(Vertex(currentPosition.positionX, currentPosition.positionY)) +
        additionalFixedHeightFromGroundMeters;
}//SetTimeTo//


inline
void GisBasedRandomWaypointMobilityModel::SetInitialPosition(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId,
    const ObjectMobilityModel::MobilityObjectId& mobilityObjectId,
    const string& initialPositionFileName,
    InorderFileCache& mobilityFileCache)
{
    const GroundLayer& groundLayer = *gisSubsystemPtr->GetGroundLayerPtr();

    TraceFileMobilityModel initialPositionMobilityFile(
        theParameterDatabaseReader,
        theNodeId,
        theInterfaceId,
        mobilityFileCache,
        initialPositionFileName,
        mobilityObjectId,
        distanceGranularityMeters,
        gisSubsystemPtr);

    ObjectMobilityPosition initialPosition;

    // Ground height will be completed by GisBasedRandomWaypointMobilityModel::SetTimeTo()
    initialPositionMobilityFile.SetNeedToAddGroundHeight(false/*newNeedToAddGroundHeight*/);

    initialPositionMobilityFile.GetUnadjustedPositionForTime(ZERO_TIME, initialPosition);

    currentPosition.positionX = initialPosition.X_PositionMeters();
    currentPosition.positionY = initialPosition.Y_PositionMeters();
    additionalFixedHeightFromGroundMeters = initialPosition.HeightFromGroundMeters();

    // forceby adjust position z
    currentPosition.positionZ =
        groundLayer.GetElevationMetersAt(Vertex(currentPosition.positionX, currentPosition.positionY)) +
        additionalFixedHeightFromGroundMeters;

    currentAttitudeAzimuthDegrees =
        initialPosition.AttitudeAzimuthFromNorthClockwiseDegrees();
    currentAttitudeElevationDegrees =
        initialPosition.AttitudeElevationFromHorizonDegrees();
    currentVelocityMetersPerSecond = 0.0;

}//SetInitialPosition//


inline
void GisBasedRandomWaypointMobilityModel::SetInitialWaypoint(
    const SimTime& startTime,
    const SimTime& pauseTime,
    const double& minSpeedMeterPerSec_NotUsed,
    const double& maxSpeedMeterPerSec_NotUsed)
{

    //current position is initial point at ZERO_TIME
    const Vertex initialPoint(
        currentPosition.positionX,
        currentPosition.positionY,
        0.0/*currentPosition.positionZ is not used*/);

    if (startTime != ZERO_TIME) {
        gisBasedWaypoints[ZERO_TIME] = GisBasedWaypoint(INVALID_GIS_OBJECT_ID, initialPoint, TRANSPORT_PAUSE, 0.0);
    }

    const double newSpeedMeterPerSec = GetARandomSpeedMeterPerSec();
    assert(newSpeedMeterPerSec != 0.0);

    gisBasedWaypoints[startTime] = GisBasedWaypoint(INVALID_GIS_OBJECT_ID, initialPoint, TRANSPORT_FREESPACE, newSpeedMeterPerSec);


    //get nearest intersection
    double intersectionSearchRadiusMeters = 500;
    double maxIntersectionSearchRadiusMeters = 100000;
    Vertex nextPoint;
    GisObjectIdType nextGisObjectId;

    switch (gisObjectType) {
    case GIS_ROAD: {

        set<RoadType> targetType; //all type
        shared_ptr<Intersection> nearestRoadIntersectionPtr;
        shared_ptr<const RoadLayer> roadLayerPtr = gisSubsystemPtr->GetRoadLayerPtr();

        IntersectionIdType nearestIntersectoinId;
        bool success = false;

        while (intersectionSearchRadiusMeters <= maxIntersectionSearchRadiusMeters) {

            roadLayerPtr->SearchNearestIntersectionId(
                initialPoint,
                Rectangle(initialPoint, intersectionSearchRadiusMeters),
                targetType,
                nearestIntersectoinId,
                success);
            if (success) {
                break;
            }

            intersectionSearchRadiusMeters += intersectionSearchRadiusMeters;

        }//while//

        if (!success) {
            cerr << "GIS based random waypoint: Cannot find a mobility start intersection." << endl
                 << "Place roads or intersections in the surroundings of object " << objectId << "." << endl;
            exit(1);
        }

        const Intersection& nearestIntersection =
            gisSubsystemPtr->GetIntersection(nearestIntersectoinId);

        //add waypoint
        nextPoint = nearestIntersection.GetVertex().XYPoint(); //set to XY point with Z = 0.
        nextGisObjectId = nearestIntersection.GetObjectId();

        const double elapsedTimeSecs = initialPoint.DistanceTo(nextPoint) / newSpeedMeterPerSec;
        const SimTime nextTime = ConvertDoubleSecsToTime(elapsedTimeSecs);

        gisBasedWaypoints[nextTime] = GisBasedWaypoint(nextGisObjectId, nextPoint, TRANSPORT_ROAD, newSpeedMeterPerSec);

        break;

    }
    case GIS_RAILROAD: {
        //TBD
    }
    default:
        cerr << "Invalid Gis Type: " << gisObjectType << endl;
        assert(false);
        exit(1);
        break;
    }//switch//

}//SetInitialWaypoint//


inline
void GisBasedRandomWaypointMobilityModel::InsertRoadIntersectionWaypoint(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GisObjectIdType currentGisObjectId,
    const GisObjectIdType nextGisObjectId,
    const double& currentSpeedMeterPerSec,
    const SimTime& pauseTime,
    const SimTime& stopMovingTime,
    SimTime& currentTime)
{
    const Intersection& sourceIntersection =
        gisSubsystemPtr->GetIntersectionObject(currentGisObjectId);
    const Intersection& destinationIntersection =
        gisSubsystemPtr->GetIntersectionObject(nextGisObjectId);

    shared_ptr<const RoadLayer> roadLayerPtr = gisSubsystemPtr->GetRoadLayerPtr();

    set<RoadType> targetType; //all type
    double searchMarginMeters = 1000; //1km

    vector<IntersectionIdType> targetIntersectionIds;
    vector<IntersectionIdType> resultIntersectionIds;

    roadLayerPtr->GetIntersectionIds(
        Rectangle(
            sourceIntersection.GetVertex(),
            destinationIntersection.GetVertex()).Expanded(searchMarginMeters),
        targetType,
        targetIntersectionIds);

    bool success = false;
    AStarAlgorithm astar(theParameterDatabaseReader);

    astar.RouteSearch(
        *gisSubsystemPtr,
        sourceIntersection.GetIntersectionId(),
        destinationIntersection.GetIntersectionId(),
        targetIntersectionIds,
        resultIntersectionIds,
        success);

    if (!success) {
        return;
    }

    assert(resultIntersectionIds.size() >= 2);

    const IntersectionIdType destinationIntersectionId = destinationIntersection.GetIntersectionId();

    typedef vector<IntersectionIdType>::const_iterator IterType;

    IterType startIntersectionIter = resultIntersectionIds.begin();
    IterType endIntersectionIter = startIntersectionIter;
    endIntersectionIter++;

    while ((currentTime < stopMovingTime) &&
           (endIntersectionIter != resultIntersectionIds.end())) {

        const Intersection& startIntersection =
            gisSubsystemPtr->GetIntersection(*startIntersectionIter);

        const Intersection& endIntersection =
            gisSubsystemPtr->GetIntersection(*endIntersectionIter);

        const Road& road = gisSubsystemPtr->GetRoad(
            startIntersection.GetRoadIdTo(*endIntersectionIter));

        const SimTime elapsedTime =
            ConvertDoubleSecsToTime(road.GetArcDistanceMeters(false/*calculate3dArcDistance*/) / currentSpeedMeterPerSec);

        currentTime += elapsedTime;

        if (*endIntersectionIter != destinationIntersectionId) {
            gisBasedWaypoints[currentTime] =
                GisBasedWaypoint(
                    endIntersection.GetObjectId(),
                    endIntersection.GetVertex().XYPoint(),
                    TRANSPORT_ROAD,
                    currentSpeedMeterPerSec);
        }
        else {

            //last one
            const double newSpeedMeterPerSec = GetARandomSpeedMeterPerSec();

            if (pauseTime == ZERO_TIME) {
                gisBasedWaypoints[currentTime] =
                    GisBasedWaypoint(
                        endIntersection.GetObjectId(),
                        endIntersection.GetVertex().XYPoint(),
                        TRANSPORT_ROAD,
                        newSpeedMeterPerSec);
            }
            else {
                gisBasedWaypoints[currentTime] =
                    GisBasedWaypoint(
                        endIntersection.GetObjectId(),
                        endIntersection.GetVertex().XYPoint(),
                        TRANSPORT_PAUSE,
                        0.0);

                //add pause time
                currentTime += pauseTime;
                gisBasedWaypoints[currentTime] =
                    GisBasedWaypoint(
                        endIntersection.GetObjectId(),
                        endIntersection.GetVertex().XYPoint(),
                        TRANSPORT_ROAD,
                        newSpeedMeterPerSec);
            }//if//

        }//if//

        startIntersectionIter = endIntersectionIter;
        ++endIntersectionIter;

    }//while//

}//InsertRoadIntersectionWaypoint//

inline
void GisBasedRandomWaypointMobilityModel::InsertRoadIntersectionWaypointWithOneHop(
    const GisObjectIdType nextGisObjectId,
    const RoadIdType& roadId,
    const double& currentSpeedMeterPerSec,
    const SimTime& pauseTime,
    const SimTime& stopMovingTime,
    SimTime& currentTime)
{
    const Road& road = gisSubsystemPtr->GetRoad(roadId);

    const Intersection& nextIntersection =
        gisSubsystemPtr->GetIntersectionObject(nextGisObjectId);

    const SimTime elapsedTime =
        ConvertDoubleSecsToTime(road.GetArcDistanceMeters(false/*calculate3dArcDistance*/) / currentSpeedMeterPerSec);

    currentTime += elapsedTime;

    const double newSpeedMeterPerSec = GetARandomSpeedMeterPerSec();

    if (pauseTime == ZERO_TIME) {
        gisBasedWaypoints[currentTime] =
            GisBasedWaypoint(nextIntersection.GetObjectId(), nextIntersection.GetVertex().XYPoint(), TRANSPORT_ROAD, newSpeedMeterPerSec);
    }
    else {
        gisBasedWaypoints[currentTime] =
            GisBasedWaypoint(nextIntersection.GetObjectId(), nextIntersection.GetVertex().XYPoint(), TRANSPORT_PAUSE, 0.0);
        //add pause time
        currentTime += pauseTime;
        gisBasedWaypoints[currentTime] =
            GisBasedWaypoint(nextIntersection.GetObjectId(), nextIntersection.GetVertex().XYPoint(), TRANSPORT_ROAD, newSpeedMeterPerSec);
    }//if//

}//InsertRoadIntersectionWaypointWithOneHop//


inline
void GisBasedRandomWaypointMobilityModel::GenerateRandomWaypoints(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const SimTime& stopMovingTime,
    const SimTime& pauseTime,
    const double& minSpeedMeterPerSec_NotUsed,
    const double& maxSpeedMeterPerSec_NotUsed,
    const bool routeSearchBasedAlgorithm)
{

    assert(!gisBasedWaypoints.empty());

    SimTime currentTime = gisBasedWaypoints.rbegin()->first;

    while (currentTime <= stopMovingTime) {

        //currently support road
        assert(gisObjectType == GIS_ROAD);

        map<SimTime, GisBasedWaypoint>::const_reverse_iterator iter =
            gisBasedWaypoints.rbegin();

        const GisBasedWaypoint& currentWaypoint = iter->second;
        const GisObjectIdType currentGisObjectId = currentWaypoint.objectId;

        GisObjectIdType nextGisObjectId = currentGisObjectId;

        const double currentSpeedMeterPerSec =
            currentWaypoint.speed;
        assert(currentSpeedMeterPerSec != 0.0);

        if(routeSearchBasedAlgorithm) {

            //pick a random intersection
            while (nextGisObjectId == currentGisObjectId) {
                nextGisObjectId = gisSubsystemPtr->GetARandomGisObjectId(
                    GIS_INTERSECTION, aRandomNumberGenerator);
            }

            InsertRoadIntersectionWaypoint(
                theParameterDatabaseReader,
                currentGisObjectId,
                nextGisObjectId,
                currentSpeedMeterPerSec,
                pauseTime,
                stopMovingTime,
                currentTime);

        }
        else {

            //pick a connected intersection

            //get previous gis object id
            GisObjectIdType previousGisObjectId = INVALID_GIS_OBJECT_ID;

            for(; iter != gisBasedWaypoints.rend(); iter++) {
                const GisBasedWaypoint& previousWaypoint = iter->second;

                if (previousWaypoint.objectId != currentGisObjectId) {
                    previousGisObjectId = previousWaypoint.objectId;
                    break;
                }//if//

            }//for//

            const Intersection& intersection =
                gisSubsystemPtr->GetIntersectionObject(currentGisObjectId);

            const vector<RoadIdType>& roadIds = intersection.GetConnectedRoadIds();

            vector<RoadIdType> movableRoadIds;

            for(size_t i = 0; i < roadIds.size(); i++) {
                const RoadIdType& roadId = roadIds[i];
                if (!gisSubsystemPtr->GetRoad(roadId).IsParking()) {
                    const Road& nextRoad = gisSubsystemPtr->GetRoad(roadId);
                    const IntersectionIdType nextIntersectionId =
                        nextRoad.GetOtherSideIntersectionId(intersection.GetIntersectionId());
                    nextGisObjectId =
                        gisSubsystemPtr->GetIntersection(nextIntersectionId).GetObjectId();
                    if (currentGisObjectId != nextGisObjectId) {
                        movableRoadIds.push_back(roadId);
                    }
                }
            }

            RoadIdType nextRoadId;

            do {

                const size_t randomIndex =
                    aRandomNumberGenerator.GenerateRandomInt(0, static_cast<int32_t>(movableRoadIds.size() - 1));

                nextRoadId = movableRoadIds[randomIndex];

                const Road& nextRoad = gisSubsystemPtr->GetRoad(nextRoadId);
                const IntersectionIdType nextIntersectionId =
                    nextRoad.GetOtherSideIntersectionId(intersection.GetIntersectionId());

                nextGisObjectId =
                    gisSubsystemPtr->GetIntersection(nextIntersectionId).GetObjectId();

                assert(currentGisObjectId != nextGisObjectId);

            } while ((movableRoadIds.size() > 1) &&
                     (previousGisObjectId != INVALID_GIS_OBJECT_ID) &&
                     (nextGisObjectId == previousGisObjectId));


            InsertRoadIntersectionWaypointWithOneHop(
                nextGisObjectId,
                nextRoadId,
                currentSpeedMeterPerSec,
                pauseTime,
                stopMovingTime,
                currentTime);

        }//if//

    }//while//

}//GenerateRandomWaypoints//


class AttachedAntennaMobilityModel : public ObjectMobilityModel {
public:
    AttachedAntennaMobilityModel(const shared_ptr<ObjectMobilityModel>& initMobilityModelPtr);

    AttachedAntennaMobilityModel(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const NodeId& initNodeId,
        const InterfaceId& initInterfaceId,
        const shared_ptr<ObjectMobilityModel>& initMobilityModelPtr);


    virtual void GetUnadjustedPositionForTime(
        const SimTime& snapshotTime,
        ObjectMobilityPosition& position) override;

    static bool AntennaIsAttachedAntenna(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId) {

        return (theParameterDatabaseReader.ParameterExists(
                    "antenna-height-meters", theNodeId, theInterfaceId) ||
                theParameterDatabaseReader.ParameterExists(
                    "antenna-azimuth-degrees", theNodeId, theInterfaceId) ||
                theParameterDatabaseReader.ParameterExists(
                    "antenna-elevation-degrees", theNodeId, theInterfaceId) ||
                theParameterDatabaseReader.ParameterExists(
                    "antenna-offset-meters", theNodeId, theInterfaceId) ||
                theParameterDatabaseReader.ParameterExists(
                    "antenna-offset-degrees", theNodeId, theInterfaceId) ||
                theParameterDatabaseReader.ParameterExists(
                    "antenna-offset-degreess", theNodeId, theInterfaceId));
    }

    void SetPlatformMobility(const shared_ptr<ObjectMobilityModel>& initMobilityModelPtr) { platformMobilityModelPtr = initMobilityModelPtr; }

    shared_ptr<ObjectMobilityModel> GetPlatformMobility() const { return platformMobilityModelPtr; }

private:
    static double NormalizedNeg360To360Degrees(const double degrees) {
        return degrees - 360 * (static_cast<int>(degrees)/360);
    }

    bool isWrapper;
    shared_ptr<ObjectMobilityModel> platformMobilityModelPtr;
    double heightFromPlatformMeters;
    double azimuthFromPlatformDegrees;
    double elevationFromPlatformDegrees;
    double offsetFromPlatformMeters;
    double offsetFromPlatformDegrees;

};//AttachedAntennaMobilityModel//


inline
AttachedAntennaMobilityModel::AttachedAntennaMobilityModel(
    const shared_ptr<ObjectMobilityModel>& initMobilityModelPtr)
    :
    isWrapper(true),
    platformMobilityModelPtr(initMobilityModelPtr),
    heightFromPlatformMeters(0),
    azimuthFromPlatformDegrees(0),
    elevationFromPlatformDegrees(0),
    offsetFromPlatformMeters(0),
    offsetFromPlatformDegrees(0)
{}


inline
AttachedAntennaMobilityModel::AttachedAntennaMobilityModel(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const NodeId& initNodeId,
    const InterfaceId& initInterfaceId,
    const shared_ptr<ObjectMobilityModel>& initMobilityModelPtr)
    :
    isWrapper(false),
    platformMobilityModelPtr(initMobilityModelPtr),
    heightFromPlatformMeters(0),
    azimuthFromPlatformDegrees(0),
    elevationFromPlatformDegrees(0),
    offsetFromPlatformMeters(0),
    offsetFromPlatformDegrees(0)
{
    if (parameterDatabaseReader.ParameterExists(
            "antenna-height-meters", initNodeId, initInterfaceId)) {
        heightFromPlatformMeters =
            parameterDatabaseReader.ReadDouble(
                "antenna-height-meters", initNodeId, initInterfaceId);
    }

    if (parameterDatabaseReader.ParameterExists(
            "antenna-azimuth-degrees", initNodeId, initInterfaceId)) {
        azimuthFromPlatformDegrees =
            NormalizedNeg360To360Degrees(
                parameterDatabaseReader.ReadDouble(
                    "antenna-azimuth-degrees", initNodeId, initInterfaceId));
    }

    if (parameterDatabaseReader.ParameterExists(
            "antenna-elevation-degrees", initNodeId, initInterfaceId)) {
        elevationFromPlatformDegrees =
            NormalizedNeg360To360Degrees(
                parameterDatabaseReader.ReadDouble(
                    "antenna-elevation-degrees", initNodeId, initInterfaceId));
    }

    if (parameterDatabaseReader.ParameterExists(
            "antenna-offset-meters", initNodeId, initInterfaceId)) {
        offsetFromPlatformMeters = parameterDatabaseReader.ReadDouble(
            "antenna-offset-meters", initNodeId, initInterfaceId);
    }

    if (parameterDatabaseReader.ParameterExists(
            "antenna-offset-degrees", initNodeId, initInterfaceId)) {
        offsetFromPlatformDegrees =
            NormalizedNeg360To360Degrees(
                parameterDatabaseReader.ReadDouble(
                    "antenna-offset-degrees", initNodeId, initInterfaceId));
    }
    else if (parameterDatabaseReader.ParameterExists(
        "antenna-offset-degreess", initNodeId, initInterfaceId)) {
        //backward compatibility since 2.1 (will be removed in the future)
        offsetFromPlatformDegrees =
            NormalizedNeg360To360Degrees(
            parameterDatabaseReader.ReadDouble(
            "antenna-offset-degreess", initNodeId, initInterfaceId));
    }


}//AttachedAntennaMobilityModel()//



inline
void AttachedAntennaMobilityModel::GetUnadjustedPositionForTime(
    const SimTime& snapshotTime,
    ObjectMobilityPosition& position)
{
    if (isWrapper) {
        platformMobilityModelPtr->GetUnadjustedPositionForTime(snapshotTime, position);
        return;
    }//if//

    platformMobilityModelPtr->GetUnadjustedPositionForTime(snapshotTime, position);

    const double platformAzimuthDegrees = position.AttitudeAzimuthFromNorthClockwiseDegrees();
    const double platformElevationDegrees = position.AttitudeElevationFromHorizonDegrees();

    const double offsetDegrees = NormalizeAngleToNeg180To180(90.0 - (platformAzimuthDegrees + offsetFromPlatformDegrees));
    const double offsetRad = offsetDegrees * PI / 180.;
    const double offsetXMeters = offsetFromPlatformMeters * cos(offsetRad);
    const double offsetYMeters = offsetFromPlatformMeters * sin(offsetRad);

    position.SetX_PositionMeters(position.X_PositionMeters() + offsetXMeters);
    position.SetY_PositionMeters(position.Y_PositionMeters() + offsetYMeters);
    position.SetHeightFromGroundMeters(position.HeightFromGroundMeters() + heightFromPlatformMeters);

    double azimuthDegrees = platformAzimuthDegrees + azimuthFromPlatformDegrees;
    double elevationDegrees = platformElevationDegrees + elevationFromPlatformDegrees;

    NormalizeAzimuthAndElevation(azimuthDegrees, elevationDegrees);

    position.SetAttitudeFromNorthClockwiseDegrees(azimuthDegrees);
    position.SetAttitudeElevationFromHorizonDegrees(elevationDegrees);

}//GetUnadjustedPositionForTime//



//--------------------------------------------------------------------
// Momentum Mobility Model
//--------------------------------------------------------------------

// Object's position keeps moving in the direction of the current velocity
// until set again.

class MomentumMobilityModel : public ObjectMobilityModel {
public:
    MomentumMobilityModel(
        const double& initDistanceGranularityMeters)
        : distanceGranularityMeters(initDistanceGranularityMeters), positionIsSet(false) { }

    void SetPositionAndVelicity(
        const SimTime& time,
        const ObjectMobilityPosition& positionAndVelocity);

    virtual void GetUnadjustedPositionForTime(
        const SimTime& snapshotTime,
        ObjectMobilityPosition& position) override;

private:
    double distanceGranularityMeters;
    SimTime timeGranularity;

    bool positionIsSet;
    ObjectMobilityPosition aPositionAndVelocity;
    SimTime timeForPosition;

};//MomentumMobilityModel//


inline
void MomentumMobilityModel::SetPositionAndVelicity(
    const SimTime& time,
    const ObjectMobilityPosition& positionAndVelocity)
{
    (*this).positionIsSet = true;
    (*this).timeForPosition = time;
    (*this).aPositionAndVelocity = positionAndVelocity;

    (*this).timeGranularity = INFINITE_TIME;
    if (positionAndVelocity.VelocityMetersPerSecond() > DBL_EPSILON) {
        (*this).timeGranularity =
            ConvertDoubleSecsToTime(distanceGranularityMeters / positionAndVelocity.VelocityMetersPerSecond());
    }//if//

}//SetPositionAndVelicity//


inline
void MomentumMobilityModel::GetUnadjustedPositionForTime(
    const SimTime& snapshotTime,
    ObjectMobilityPosition& position)
{
    assert(positionIsSet);
    assert(timeForPosition <= snapshotTime);
    const double deltaTimeSecs = ConvertTimeToDoubleSecs(snapshotTime - timeForPosition);

    position = aPositionAndVelocity;
    assert(position.VelocityElevationFromHorizonDegrees() == 0.0);

    const double velocityAngleRadians =
        (PI / 180.0) *
        NormalizeAngleToNeg180To180(90.0 - position.VelocityAzimuthFromNorthClockwiseDegrees());

    position.SetX_PositionMeters(
        (position.X_PositionMeters()  +
         (deltaTimeSecs * position.VelocityMetersPerSecond() * cos(velocityAngleRadians))));

    position.SetY_PositionMeters(
        (position.Y_PositionMeters()  +
         (deltaTimeSecs * position.VelocityMetersPerSecond() * sin(velocityAngleRadians))));

    position.SetLastMoveTime(snapshotTime);
    position.SetEarliestNextMoveTime(snapshotTime + timeGranularity);

}//GetUnadjustedPositionForTime//



//--------------------------------------------------------------------
// Stationary Mobility Model
//--------------------------------------------------------------------

class StationaryMobilityModel: public ObjectMobilityModel {
public:
    StationaryMobilityModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId,
        InorderFileCache& mobilityFileCache,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr);

    ~StationaryMobilityModel() {}

    virtual void GetUnadjustedPositionForTime(
        const SimTime& snapshotTime,
        ObjectMobilityPosition& position) override
    {
        position = stationaryPosition;
    }

private:
    ObjectMobilityPosition stationaryPosition;

};//StationaryMobilityModel//


inline
StationaryMobilityModel::StationaryMobilityModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId,
    InorderFileCache& mobilityFileCache,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr)
    :
    ObjectMobilityModel(theParameterDatabaseReader, theNodeId, theInterfaceId)
{
    string initPositionFileName =
        theParameterDatabaseReader.ReadString(
            "mobility-init-positions-file", theNodeId, theInterfaceId);

    ObjectMobilityModel::MobilityObjectId mobilityObjectId(theNodeId);

    const double notUsedDistanceGranularityMeters = 0.0;

    TraceFileMobilityModel traceFileMobility(
        theParameterDatabaseReader,
        theNodeId,
        theInterfaceId,
        mobilityFileCache,
        initPositionFileName,
        mobilityObjectId,
        notUsedDistanceGranularityMeters,
        initGisSubsystemPtr);

    traceFileMobility.GetUnadjustedPositionForTime(ZERO_TIME, stationaryPosition);

}//StationaryMobilityModel//

inline
ObjectMobilityModel::ObjectMobilityModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId)
    :
    relativeAttitudeAzimuthDegrees(0.0)
{
}

static inline
bool InterfaceIsWired(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId)
{
    if (theParameterDatabaseReader.ParameterExists("mac-protocol", theNodeId, theInterfaceId)) {
        string macProtocol = theParameterDatabaseReader.ReadString("mac-protocol", theNodeId, theInterfaceId);
        ConvertStringToLowerCase(macProtocol);

        if (macProtocol == "wired" || macProtocol == "abstract-network") {
            return true;
        }//if//
    }//if//

    return false;
}

static inline
bool IsNoInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId)
{
    if (theParameterDatabaseReader.ParameterExists("mac-protocol", theNodeId, theInterfaceId)) {
        string macProtocol = theParameterDatabaseReader.ReadString("mac-protocol", theNodeId, theInterfaceId);
        ConvertStringToLowerCase(macProtocol);

        if (macProtocol == "no") {
            return true;
        }//if//
    }//if//

    return false;
}

shared_ptr<ObjectMobilityModel> CreateAntennaMobilityModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId,
    const RandomNumberGeneratorSeed& mobilitySeed,
    InorderFileCache& mobilityFileCache,
    const shared_ptr<GisSubsystem>& theGisSubsystemPtr);

}; // namespace ScenSim

#endif
