// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include <sstream>

#include <boost/filesystem.hpp>

#include "s_hull.h"

#include "scensim_gis.h"
#include "scensim_gis_shape.h"
#include "scensim_tracedefs.h"

namespace ScenSim {

using std::istringstream;


PointIdType Point::GetPointId() const
{
    return commonImplPtr->variantId;
}


EntranceIdType Entrance::GetEntranceId() const
{
    return commonImplPtr->variantId;
}


const Vertex& Entrance::GetVertex() const
{
    return (*this).GisObject::GetVertex(0);
}


VertexIdType Entrance::GetVertexId() const
{
    return (*this).GisObject::GetVertexId(0);
}

bool Entrance::IntersectsWith(const Rectangle& rect) const
{
    return rect.Contains((*this).GetVertex());
}


PoiIdType Poi::GetPoiId() const
{
    return commonImplPtr->variantId;
}


const Vertex& Poi::GetVertex() const
{
    return (*this).GisObject::GetVertex(0);
}


VertexIdType Poi::GetVertexId() const
{
    return (*this).GisObject::GetVertexId(0);
}


RoadIdType Poi::GetNearestEntranceRoadId(const Vertex& position) const
{
    if ((*this).IsAPartOfObject()) {
        GisSubsystem* subsystemPtr = commonImplPtr->subsystemPtr;

        if (parentPositionId.type == GIS_BUILDING) {
            return subsystemPtr->GetBuilding(parentPositionId.id).GetNearestEntranceRoadId(position);
        } else if (parentPositionId.type == GIS_PARK) {
            return subsystemPtr->GetPark(parentPositionId.id).GetNearestEntranceRoadId(position);
        }
    }

    const GisVertex& poiGisVertex =
        commonImplPtr->subsystemPtr->GetGisVertex((*this).GetVertexId());
    const vector<RoadIdType>& roadIds = poiGisVertex.GetConnectedObjectIds(GIS_ROAD);

    for(size_t i = 0; i < roadIds.size(); i++) {
        const RoadIdType& roadId = roadIds[i];

        if (commonImplPtr->subsystemPtr->GetRoad(roadId).IsParking()) {
            return roadId;
        }
    }

    cerr << "Error: No entrance road at " << commonImplPtr->objectName << endl;
    exit(1);

    return INVALID_VARIANT_ID;
}

bool Poi::IntersectsWith(const Rectangle& rect) const
{
    return rect.Contains((*this).GetVertex());
}


PedestrianPath::PedestrianPath(
    GisSubsystem* initSubsystemPtr,
    const GisObjectIdType& initObjectId,
    const PedestrianPathIdType& initPathId)
    :
    GisObject(initSubsystemPtr, GIS_PEDESTRIAN_PATH, initObjectId, initPathId)
{}


Road::Road(
    GisSubsystem* initSubsystemPtr,
    const GisObjectIdType& initObjectId,
    const RoadIdType& initRoadId)
    :
    GisObject(initSubsystemPtr, GIS_ROAD, initObjectId, initRoadId),
    isRightHandTraffic(false),
    type(ROAD_UNCLASSIFIED),
    numberStartToEndLanes(1),
    numberEndToStartLanes(1),
    widthMeters(DEFAULT_ROAD_WIDTH_METERS),
    isBaseGroundLevel(true),
    humanCapacity(INT_MAX),
    capacity(1000.),
    speedLimitMetersPerSec(200.)
{
}



double Road::GetDirectionRadians() const
{
    const Vertex& startPosition = (*this).GetStartVertex();
    const Vertex& endPosition = (*this).GetEndVertex();

    return std::atan((endPosition.y - startPosition.y) /
                     (endPosition.x - startPosition.x));
}


double Road::GetArcDistanceMeters(const bool calculate3dArcDistance) const
{
    if (calculate3dArcDistance) {
        return CalculateArcDistance(vertices);
    }

    if (vertices.size() < 2) {
        return 0;
    }

    double totalDistance = 0;

    for(size_t i = 0; i < vertices.size() - 1; i++) {
        totalDistance += vertices[i].XYDistanceTo(vertices[i+1]);
    }

    return totalDistance;
}



double Road::GetArcDistanceMeters(
    const VertexIdType& vertexId1,
    const VertexIdType& vertexId2) const
{
    const vector<VertexIdType>& vertexIds = commonImplPtr->vertexIds;

    double distanceMeters = 0;
    bool found = false;

    for(size_t i = 0; i < vertexIds.size() - 1; i++) {
        const VertexIdType& vertexId = vertexIds[i];

        if (vertexId1 == vertexId || vertexId2 == vertexId) {
            if (!found) {
                found = true;
            } else {
                return distanceMeters;
            }
        }

        if (found) {
            const Vertex& v1 = (*this).GetVertex(i);
            const Vertex& v2 = (*this).GetVertex(i+1);

            distanceMeters += v1.DistanceTo(v2);
        }
    }

    if (vertexId1 == vertexIds.back() || vertexId2 == vertexIds.back()) {
        return distanceMeters;
    }


    cerr << "Error: Road " << (*this).GetRoadId() << " doesn't contain vertex " << vertexId1 << " and " << vertexId2 << endl;
    assert(false);

    return 0;
}



bool Road::Contains(const Vertex& point) const
{
    return PolygonContainsPoint(polygon, point);
}



double Road::DistanceTo(const Vertex& point) const
{
    if ((*this).IsParking()) {
        return (*this).GetVertex(0).DistanceTo(point);
    }

    double minDistance = DBL_MAX;

    assert(vertices.size() > 1);

    for(size_t i = 0; i < vertices.size() - 1; i++) {
        minDistance = std::min(minDistance, CalculatePointToLineDistance(point, vertices[i], vertices[+1]));
    }

    return minDistance;
}



RoadIdType Road::GetRoadId() const
{
    return commonImplPtr->variantId;
}



IntersectionIdType Road::GetOtherSideIntersectionId(const IntersectionIdType& intersectionId) const
{
    const IntersectionIdType startIntersectionId = (*this).GetStartIntersectionId();

    if (intersectionId != startIntersectionId) {
        return startIntersectionId;
    }

    return (*this).GetEndIntersectionId();
}



IntersectionIdType Road::GetStartIntersectionId() const
{
    const vector<IntersectionIdType> intersectionIds =
        (*this).GetGisVertex(0).GetConnectedObjectIds(GIS_INTERSECTION);

    assert(!intersectionIds.empty());

    return intersectionIds.front();
}



IntersectionIdType Road::GetEndIntersectionId() const
{
    if ((*this).IsParking()) {
        return (*this).GetStartIntersectionId();
    }

    const vector<IntersectionIdType> intersectionIds =
        (*this).GetGisVertex((*this).NumberOfVertices() - 1).GetConnectedObjectIds(GIS_INTERSECTION);

    assert(!intersectionIds.empty());

    return intersectionIds.front();
}



const Vertex& Road::GetNeighborVertex(
    const IntersectionIdType& intersectionId) const
{
    if ((*this).IsParking()) {
        return (*this).GetVertex(0);
    }

    if (intersectionId == (*this).GetStartIntersectionId()) {
        return vertices.at(1);
    } else {
        return vertices.at(vertices.size() - 2);
    }
}



Vertex Road::GetNearestPosition(const Vertex& position) const
{
    assert((*this).NumberOfVertices() > 0);

    double minDistance = DBL_MAX;
    Vertex nearestPosition;

    for(size_t i = 0; i < (*this).NumberOfVertices() - 1; i++) {

        const Vertex& edge1 = (*this).GetVertex(i);
        const Vertex& edge2 = (*this).GetVertex(i+1);

        const Vertex roadPosition = CalculatePointToLineNearestPosition(
            position.XYPoint(), edge1.XYPoint(), edge2.XYPoint());

        const double distance = roadPosition.XYDistanceTo(position);//use XY distance

        if (distance < minDistance) {
            minDistance = distance;
            nearestPosition = roadPosition;
        }
    }
    assert(minDistance != DBL_MAX);

    return nearestPosition;
}



IntersectionIdType Road::GetNearestIntersectionId(const Vertex& position) const
{
    const double distanceToStart = (*this).GetVertex(0).DistanceTo(position);
    const double distanceToEnd = (*this).GetVertex((*this).NumberOfVertices() - 1).DistanceTo(position);

    if (distanceToStart < distanceToEnd) {
        return (*this).GetStartIntersectionId();
    }

    return (*this).GetEndIntersectionId();
}



inline
Vertex CalculateInterpolatedLineIntersection(
    const Vertex& lineEdge11,
    const Vertex& lineEdge12,
    const Vertex& lineEdge21,
    const Vertex& lineEdge22)
{
    const Vertex a = lineEdge12 - lineEdge11;
    const Vertex b = lineEdge21 - lineEdge22;
    const Vertex c = lineEdge11 - lineEdge21;

    const double denominator = a.y * b.x - a.x * b.y;

    if (denominator == 0 || denominator >= DBL_MAX) {
        return (lineEdge12 + lineEdge21) * 0.5;
    }

    const double reciprocal = 1. / denominator;
    const double na = (b.y * c.x - b.x * c.y) * reciprocal;

    return lineEdge11 + a * na;
}



inline
void GetLinePolygon(
    const deque<Vertex>& vertices,
    const double widthMeters,
    vector<Vertex>& polygon)
{
    polygon.clear();

    if (vertices.size() < 2) {
        return;
    }

    typedef pair<Vertex, Vertex> LineType;

    const size_t numberPoints = vertices.size();
    const double halfWidth = widthMeters * 0.5;

    polygon.resize(numberPoints*2 + 1);
    vector<LineType> topLines(numberPoints - 1);
    vector<LineType> bottomLines(numberPoints - 1);

    const double expandLength = halfWidth;

    for(size_t i = 0; i < numberPoints - 1; i++) {
        Vertex p1 = vertices[i];
        Vertex p2 = vertices[i+1];

        const double distance = p1.DistanceTo(p2);

        if (distance == 0) {
            topLines[i].first = p1;
            topLines[i].second = p1;
            bottomLines[i].first = p1;
            bottomLines[i].second = p1;
            continue;
        }

        if (i == 0) {
            p1 = p1 + (p1 - p2) * (expandLength / distance);
        }
        if (i+1 == numberPoints - 1) {
            p2 = p2 + (p2 - p1) * (expandLength / distance);
        }

        const LineType aLine(p1, p2);
        const LineType normalVector(
            aLine.first,
            aLine.first + Vertex(aLine.second.y - aLine.first.y, aLine.first.x - aLine.second.x));

        Vertex offset;

        if (normalVector.second != normalVector.first) {
            offset =
                (normalVector.second - normalVector.first) *
                (halfWidth / normalVector.first.DistanceTo(normalVector.second));
        }

        topLines[i].first = aLine.first + offset;
        topLines[i].second = aLine.second + offset;

        bottomLines[i].first = aLine.first - offset;
        bottomLines[i].second = aLine.second - offset;
    }

    const size_t topLineStart = 0;
    const size_t bottomLineStart = numberPoints;

    polygon[topLineStart] = topLines.front().first;
    polygon[topLineStart + numberPoints - 1] = topLines.back().second;
    polygon[bottomLineStart] = bottomLines.back().second;
    polygon[bottomLineStart + numberPoints - 1] = bottomLines.front().first;

    const int numberLines = static_cast<int>(topLines.size());

    for(int i = 0; i < numberLines - 1; i++) {
        const LineType& topLine1 = topLines[i];
        const LineType& topLine2 = topLines[i+1];

        polygon[topLineStart + i + 1] = CalculateInterpolatedLineIntersection(
            topLine1.first, topLine1.second, topLine2.first, topLine2.second);

        const LineType& bottomLine1 = bottomLines[numberLines - i - 2];
        const LineType& bottomLine2 = bottomLines[numberLines - i - 1];

        polygon[bottomLineStart + i + 1] = CalculateInterpolatedLineIntersection(
            bottomLine1.first, bottomLine1.second, bottomLine2.first, bottomLine2.second);
    }

    polygon.back() = polygon.front();
}



void Road::UpdatePolygon()
{
    if (vertices.size() < 2) {
        return;
    }

    GetLinePolygon(vertices, widthMeters, polygon);
}



void Road::UpdateMinRectangle() const
{
    if (vertices.size() < 2) {
        commonImplPtr->minRectangle = GetPointsRect(vertices);
    } else {
        assert(polygon.size() > 2);
        commonImplPtr->minRectangle = GetPointsRect(polygon);
    }
}



bool Road::IntersectsWith(const Rectangle& rect) const
{
    return RectIsIntersectsWithPolygon(rect, polygon);
}



void Road::SetIntersectionMargin(
    const IntersectionIdType& intersectionId,
    const double marginLength)
{
    GisSubsystem* subsystemPtr = commonImplPtr->subsystemPtr;

    assert(!vertices.empty());

    const bool isReverse = ((*this).GetStartIntersectionId() != intersectionId);
    const Vertex& intersectionPos =
        subsystemPtr->GetIntersection(intersectionId).GetVertex();

    ReverseAccess<Vertex> verticesAccess(vertices, isReverse);

    const double distanceToIntersection =
        verticesAccess.front().DistanceTo(intersectionPos);

    if (marginLength > distanceToIntersection) {

        double marginLengthFromFront = marginLength - distanceToIntersection;

        while (true) {
            assert(verticesAccess.size() > 1);

            const Vertex& p1 = verticesAccess[0];
            const Vertex& p2 = verticesAccess[1];
            const double length = p1.DistanceTo(p2);

            if (marginLengthFromFront <= length) {
                verticesAccess.front() = (p2-p1)*(marginLengthFromFront/length) + p1;
                break;
            }
            marginLengthFromFront -= length;
            verticesAccess.pop_front();
        }

    } else {
        verticesAccess.front() =
            (intersectionPos - verticesAccess.front())*
            ((distanceToIntersection - marginLength)/distanceToIntersection) +
            verticesAccess.front();
    }
}



size_t Road::GetRandomOutgoingLaneNumber(
    const VertexIdType& startVertexId,
    HighQualityRandomNumberGenerator& aRandomNumberGenerator) const
{
    if (startVertexId == (*this).GetStartVertexId()) {

        return aRandomNumberGenerator.GenerateRandomInt(
            0, static_cast<int32_t>(numberStartToEndLanes - 1));
    }

    return numberStartToEndLanes +
        aRandomNumberGenerator.GenerateRandomInt(0, static_cast<int32_t>(numberEndToStartLanes - 1));
}



size_t Road::GetNearestOutgoingLaneNumber(
    const VertexIdType& startVertexId,
    const Vertex& position) const
{
    vector<size_t> laneNumberCandidates;

    if (startVertexId == (*this).GetStartVertexId()) {
        for(size_t i = 0; i < numberStartToEndLanes; i++) {
            laneNumberCandidates.push_back(i);
        }
    } else {
        for(size_t i = numberStartToEndLanes; i < numberStartToEndLanes + numberEndToStartLanes; i++) {
            laneNumberCandidates.push_back(i);
        }
    }

    size_t nearestLaneNumber = laneNumberCandidates.front();
    double nearestDistance = DBL_MAX;

    for(size_t i = 0; i < laneNumberCandidates.size(); i++) {
        const size_t laneNumber = laneNumberCandidates[i];
        deque<Vertex> laneVertices;

        (*this).GetLaneVertices(laneNumber, true/*waypointFromAdditionalStartPosition*/, position, laneVertices);

        const double distance = laneVertices.front().DistanceTo(position);

        if (distance < nearestDistance) {
            nearestDistance = distance;
            nearestLaneNumber = laneNumber;
        }
    }

    return nearestLaneNumber;
}



size_t Road::GetOutsideOutgoingLaneNumber(const VertexIdType& startVertexId) const
{
    if (startVertexId == (*this).GetStartVertexId()) {
        return numberStartToEndLanes - 1;
    }

    return numberStartToEndLanes + numberEndToStartLanes - 1;
}



bool Road::IsParking() const
{
    return (commonImplPtr->vertexIds.size() == 1);
}



size_t Road::GetParkingLaneNumber() const
{
    assert((*this).IsParking());
    return 0;
}



RoadDirectionType Road::GetRoadDirection(const size_t laneNumber) const
{
    if (laneNumber < numberStartToEndLanes) {
        return ROAD_DIRECTION_UP;
    }
    return ROAD_DIRECTION_DOWN;
}



bool Road::HasPassingLane(const RoadDirectionType& directionType, const size_t laneNumer) const
{
    if (directionType == ROAD_DIRECTION_UP) {
        if (isRightHandTraffic) {
            return (laneNumer > 0);
        } else {
            return (laneNumer + 1 < numberStartToEndLanes);
        }
    } else {
        if (isRightHandTraffic) {
            return (laneNumer + 1 < numberStartToEndLanes + numberEndToStartLanes);
        } else {
            return (laneNumer > numberStartToEndLanes);
        }
    }
}



bool Road::HasNonPassingLane(const RoadDirectionType& directionType, const size_t laneNumer) const
{
    if (directionType == ROAD_DIRECTION_UP) {
        if (isRightHandTraffic) {
            return (laneNumer + 1 < numberStartToEndLanes);
        } else {
            return (laneNumer > 0);
        }
    } else {
        if (isRightHandTraffic) {
            return (laneNumer > numberStartToEndLanes);
        } else {
            return (laneNumer + 1 < numberStartToEndLanes + numberEndToStartLanes);
        }
    }
}



size_t Road::GetPassingLaneNumber(
    const RoadDirectionType& directionType,
    const size_t laneNumer) const
{
    assert((*this).HasPassingLane(directionType, laneNumer));

    if (directionType == ROAD_DIRECTION_UP) {
        if (isRightHandTraffic) {
            return (laneNumer - 1);
        } else {
            return (laneNumer + 1);
        }
    } else {
        if (isRightHandTraffic) {
            return (laneNumer + 1);
        } else {
            return (laneNumer - 1);
        }
    }
}



size_t Road::GetNonPassingLaneNumber(
    const RoadDirectionType& directionType,
    const size_t laneNumer) const
{
    assert((*this).HasNonPassingLane(directionType, laneNumer));

    if (directionType == ROAD_DIRECTION_UP) {
        if (isRightHandTraffic) {
            return (laneNumer + 1);
        } else {
            return (laneNumer - 1);
        }
    } else {
        if (isRightHandTraffic) {
            return (laneNumer - 1);
        } else {
            return (laneNumer + 1);
        }
    }
}



bool Road::CanApproach(const size_t laneNumber, const RoadIdType& roadId) const
{
    const map<RoadIdType, vector<pair<RoadTurnDirectionType, size_t> > >& laneConnection =
        laneConnections.at(laneNumber);

    return (laneConnection.find(roadId) != laneConnection.end());
}



size_t Road::GetNextLaneNumber(const size_t laneNumber, const RoadIdType& roadId) const
{
    typedef map<RoadIdType, vector<pair<RoadTurnDirectionType, size_t> > >::const_iterator IterType;

    const map<RoadIdType, vector<pair<RoadTurnDirectionType, size_t> > >& laneConnection =
        laneConnections.at(laneNumber);

    IterType iter = laneConnection.find(roadId);

    assert(iter != laneConnection.end());

    const vector<pair<RoadTurnDirectionType, size_t> >& laneNumbers = (*iter).second;

    assert(!laneNumbers.empty());

    return laneNumbers.front().second;
}



size_t Road::GetApproachLaneNumber(const RoadIdType& roadId) const
{
    for(size_t i = 0; i < laneConnections.size(); i++) {
        const map<RoadIdType, vector<pair<RoadTurnDirectionType, size_t> > >& laneConnection = laneConnections[i];

        if (laneConnection.find(roadId) != laneConnection.end()) {
            return i;
        }
    }

    assert(false);

    return BAD_SIZE_T;
}



size_t Road::GetNeighborLaneNumberToApproach(
    const size_t laneNumber,
    const RoadIdType& roadId) const
{
    const size_t approachLaneNumber = (*this).GetApproachLaneNumber(roadId);

    if (approachLaneNumber < laneNumber) {
        return laneNumber - 1;
    } else if (approachLaneNumber > laneNumber) {
        return laneNumber + 1;
    }

    return laneNumber;
}



void Road::GetLaneVertices(
    const size_t laneNumber,
    const bool waypointFromAdditionalStartPosition,
    const Vertex& startPosition,
    deque<Vertex>& laneVertices) const
{
    const bool isReverse = (laneNumber >= numberStartToEndLanes);

    double offset;
    bool replaceFirstEntryWithStartPosition = false;

    if ((*this).IsExtraPath()) {

        offset = 0.;

        if (vertices.size() >= 2) {
            replaceFirstEntryWithStartPosition = true;
        }

    } else {

        size_t numberOffsetLanes;
        if (isReverse) {
            numberOffsetLanes = laneNumber - numberStartToEndLanes;
        } else {
            numberOffsetLanes = numberStartToEndLanes - laneNumber - 1;
        }

        const double laneWidthMeters = (*this).GetLaneWidthMeters();

        if (numberStartToEndLanes == 0 || numberEndToStartLanes == 0) {
            offset = (numberOffsetLanes + 0.5) * laneWidthMeters - widthMeters*0.5;
        } else {
            offset = (numberOffsetLanes + 0.5) * laneWidthMeters;
        }

        if (!isRightHandTraffic) {
            offset *= -1;
        }
    }

    // calculate offset points from center line
    (*this).GetOffsetWaypoints(
        offset,
        isReverse,
        waypointFromAdditionalStartPosition,
        startPosition,
        replaceFirstEntryWithStartPosition,
        laneVertices);
}



pair<Vertex, Vertex> Road::GetSideLineToIntersection(
    const IntersectionIdType& intersectionId,
    const double averageWidthMeters) const
{
    const bool isReverse = ((*this).GetStartIntersectionId() != intersectionId);

    pair<Vertex, Vertex> sideLine;
    deque<Vertex> laneVertices;

    (*this).GetOffsetWaypoints(
        averageWidthMeters/2.,
        isReverse,
        false/*waypointFromAdditionalStartPosition*/,
        Vertex(),
        false/*replaceFirstEntryWithStartPosition*/,
        laneVertices);


    if (laneVertices.size() >= 2) {
        sideLine.first = laneVertices[0];
        sideLine.second = laneVertices[1];
    }

    return sideLine;
}



Vertex Road::GetInternalPoint(
    const IntersectionIdType& intersectionId,
    const double internalLengthMeters) const
{
    GisSubsystem* subsystemPtr = commonImplPtr->subsystemPtr;

    const bool isReverse = ((*this).GetStartIntersectionId() != intersectionId);

    ConstReverseAccess<Vertex> verticesAccess(vertices, isReverse);

    const Vertex& intersectionPos =
        subsystemPtr->GetIntersection(intersectionId).GetVertex();

    const double distanceToIntersection =
        verticesAccess.front().DistanceTo(intersectionPos);

    if (internalLengthMeters > distanceToIntersection) {

        double marginLengthFromFront = internalLengthMeters - distanceToIntersection;

        for(size_t i = 0; i < verticesAccess.size() - 1; i++) {
            const Vertex& p1 = verticesAccess[i];
            const Vertex& p2 = verticesAccess[i+1];
            const double length = p1.DistanceTo(p2);

            if (marginLengthFromFront <= length) {
                return (p2-p1)*(marginLengthFromFront/length) + p1;
            }
            marginLengthFromFront -= length;
        }

        return verticesAccess.back();
    } else {
        return (intersectionPos - verticesAccess.front())*
            ((distanceToIntersection - internalLengthMeters)/distanceToIntersection) +
            verticesAccess.front();
    }
}



void Road::GetPedestrianVertices(
    const VertexIdType& lastVertexId,
    const VertexIdType& startVertexId,
    const bool walkLeftSide,
    const double maxOffset,
    const Vertex& startPosition,
    deque<Vertex>& waypoints) const
{
    waypoints.clear();

    if (lastVertexId == startVertexId) {
        return;
    }

    const bool isReverse = (startVertexId != (*this).GetStartVertexId());

    double offset;
    bool replaceFirstEntryWithStartPosition = false;

    if ((*this).IsExtraPath()) {

        offset = 0.;

        if (vertices.size() >= 2) {
            replaceFirstEntryWithStartPosition = true;
        }

    } else {

        offset = std::min((*this).GetRoadWidthMeters()/2, maxOffset);

        if (walkLeftSide) {
            offset = -offset;
        }
    }

    (*this).GetOffsetWaypoints(
        offset,
        isReverse,
        true/*waypointFromAdditionalStartPosition*/,
        startPosition,
        replaceFirstEntryWithStartPosition,
        waypoints);
}



void Road::GetPedestrianVertices(
    const VertexIdType& lastVertexId,
    const VertexIdType& startVertexId,
    const bool walkLeftSide,
    const Vertex& startPosition,
    deque<Vertex>& waypoints) const
{
    waypoints.clear();

    if (lastVertexId == startVertexId) {
        return;
    }

    const bool isReverse = (startVertexId != (*this).GetStartVertexId());

    double offset;
    bool replaceFirstEntryWithStartPosition = false;

    if ((*this).IsExtraPath()) {

        offset = 0.;

        if (vertices.size() >= 2) {
            replaceFirstEntryWithStartPosition = true;
        }

    } else {

        ConstReverseAccess<Vertex> verticesAccess(vertices, isReverse);

        const Vertex& p1 = verticesAccess[0];
        const Vertex& p2 = verticesAccess[1];

        assert(p1 != p2);

        const Vertex lastVeretx = p1.NormalVector(
            commonImplPtr->subsystemPtr->GetVertex(lastVertexId));

        if (CalculateRadiansBetweenVector(p2, p1, lastVeretx) < PI/2) {
            offset = numberStartToEndLanes*(*this).GetLaneWidthMeters();
        } else {
            offset = numberEndToStartLanes*(*this).GetLaneWidthMeters();
        }

        if (walkLeftSide) {
            offset = -offset;
        }

    }

    //TBD// if (length < offset) {
    //TBD//     waypoints.push_back(verticesAccess.front());
    //TBD//     waypoints.push_back(verticesAccess.back());
    //TBD//     return;
    //TBD// }

    (*this).GetOffsetWaypoints(
        offset,
        isReverse,
        true/*waypointFromAdditionalStartPosition*/,
        startPosition,
        replaceFirstEntryWithStartPosition,
        waypoints);
}



void Road::GetOffsetWaypoints(
    const double offset,
    const bool isReverse,
    const bool waypointFromAdditionalStartPosition,
    const Vertex& startPosition,
    const bool replaceFirstEntryWithStartPosition,
    deque<Vertex>& waypoints) const
{
    waypoints.clear();

    if ((*this).IsParking()) {
        waypoints.push_back((*this).GetVertex(0));
        return;
    }

    ConstReverseAccess<Vertex> verticesAccess(vertices, isReverse);

    vector<Vertex> srcWaypoints;

    for(size_t i = 0; i < verticesAccess.size(); i++) {
        srcWaypoints.push_back(verticesAccess[i]);
    }

    commonImplPtr->subsystemPtr->GetOffsetWaypoints(
        offset,
        srcWaypoints,
        waypointFromAdditionalStartPosition,
        startPosition,
        replaceFirstEntryWithStartPosition,
        isBaseGroundLevel,
        waypoints);
}


vector<size_t> inline
MakeLaneTable(
    const vector<size_t>& outLaneNumbers,
    const vector<size_t>& inLaneNumbers)
{
    vector<size_t> laneNumbers(outLaneNumbers.size());

    if (!inLaneNumbers.empty() && !outLaneNumbers.empty()) {
        const double laneCompressionRate = double(inLaneNumbers.size() + 1) / outLaneNumbers.size();

        for(size_t i = 0; i < outLaneNumbers.size(); i++) {
            const size_t inLaneNumber =
                std::min(static_cast<size_t>(i*laneCompressionRate), inLaneNumbers.size() - 1);
            laneNumbers[i] = inLaneNumbers[inLaneNumber];
        }
    }

    return laneNumbers;
}



void Road::MakeTurnaroundLaneConnection()
{
    vector<size_t> laneNumbers1;
    vector<size_t> laneNumbers2;

    for(size_t i = 0; i < numberStartToEndLanes; i++) {
        laneNumbers1.push_back(i);
    }
    for(size_t i = 0; i < numberEndToStartLanes; i++) {
        laneNumbers2.push_back(numberStartToEndLanes + numberEndToStartLanes - i - 1);
    }

    const vector<size_t> laneNumbers12 = MakeLaneTable(laneNumbers1, laneNumbers2);
    const vector<size_t> laneNumbers21 = MakeLaneTable(laneNumbers2, laneNumbers1);

    assert(laneNumbers12.size() == laneNumbers1.size());
    assert(laneNumbers21.size() == laneNumbers2.size());

    const RoadIdType roadId = (*this).GetRoadId();

    for(size_t i = 0; i < laneNumbers1.size(); i++) {
        laneConnections[laneNumbers1[i]][roadId].
            push_back(make_pair(ROAD_TURN_BACK, laneNumbers12[i]));
    }
    for(size_t i = 0; i < laneNumbers2.size(); i++) {
        laneConnections[laneNumbers2[i]][roadId].
            push_back(make_pair(ROAD_TURN_BACK, laneNumbers21[i]));
    }
}



void Road::MakeLaneConnection(
    const Road& otherRoad,
    const VertexIdType& vertexId,
    const RoadTurnDirectionType& direction)
{
    const RoadIdType otherRoadId = otherRoad.GetRoadId();

    if (direction == ROAD_TURN_RIGHT) {

        if ((*this).HasOutgoingLane(vertexId) &&
            otherRoad.HasIncomingLane(vertexId)) {

            laneConnections[(*this).GetOutgoingRightLaneNumber(vertexId)][otherRoadId].
                push_back(make_pair(direction, otherRoad.GetIncomingRightLaneNumber(vertexId)));
        }

    } else if (direction == ROAD_TURN_LEFT) {

        if ((*this).HasOutgoingLane(vertexId) &&
            otherRoad.HasIncomingLane(vertexId)) {

            laneConnections[(*this).GetOutgoingLeftLaneNumber(vertexId)][otherRoadId].
                push_back(make_pair(direction, otherRoad.GetIncomingLeftLaneNumber(vertexId)));
        }

    } else {
        const vector<size_t> outgoingLaneNumbers = (*this).GetOutgoingLaneNumbers(vertexId);
        const vector<size_t> incomingLaneNumbers = otherRoad.GetIncomingLaneNumbers(vertexId);
        const vector<size_t> laneNumbers =
            MakeLaneTable(outgoingLaneNumbers, incomingLaneNumbers);

        assert(laneNumbers.size() == outgoingLaneNumbers.size());

        for(size_t i = 0; i < outgoingLaneNumbers.size(); i++) {
            laneConnections[outgoingLaneNumbers[i]][otherRoadId].
                push_back(make_pair(direction, laneNumbers[i]));
        }
    }
}



size_t Road::GetOutgoingRightLaneNumber(const VertexIdType& vertexId) const
{
    if (vertexId == (*this).GetStartVertexId()) {
        return numberStartToEndLanes;
    } else {
        return numberStartToEndLanes - 1;
    }
}



size_t Road::GetIncomingRightLaneNumber(const VertexIdType& vertexId) const
{
    if (vertexId == (*this).GetStartVertexId()) {
        return numberStartToEndLanes - 1;
    } else {
        return numberStartToEndLanes;
    }
}



size_t Road::GetOutgoingLeftLaneNumber(const VertexIdType& vertexId) const
{
    if (vertexId == (*this).GetStartVertexId()) {
        return numberStartToEndLanes + numberEndToStartLanes - 1;
    } else {
        return 0;
    }
}



size_t Road::GetIncomingLeftLaneNumber(const VertexIdType& vertexId) const
{
    if (vertexId == (*this).GetStartVertexId()) {
        return 0;
    } else {
        return numberStartToEndLanes + numberEndToStartLanes - 1;
    }
}



vector<size_t> Road::GetOutgoingLaneNumbers(const VertexIdType& vertexId) const
{
    vector<size_t> laneNumbers;

    if (vertexId == (*this).GetStartVertexId()) {
        for(size_t i = 0; i < numberEndToStartLanes; i++) {
            laneNumbers.push_back(numberStartToEndLanes + numberEndToStartLanes - i - 1);
        }
    } else {
        for(size_t i = 0; i < numberStartToEndLanes; i++) {
            laneNumbers.push_back(i);
        }
    }

    return laneNumbers;
}


vector<size_t> Road::GetIncomingLaneNumbers(const VertexIdType& vertexId) const
{
    vector<size_t> laneNumbers;

    if (vertexId == (*this).GetStartVertexId()) {
        for(size_t i = 0; i < numberStartToEndLanes; i++) {
            laneNumbers.push_back(i);
        }
    } else {
        for(size_t i = 0; i < numberEndToStartLanes; i++) {
            laneNumbers.push_back(numberStartToEndLanes + numberEndToStartLanes - i - 1);
        }
    }

    return laneNumbers;
}



bool Road::HasOutgoingLane(const VertexIdType& vertexId) const
{
    if (vertexId == (*this).GetStartVertexId()) {
        return (numberEndToStartLanes > 0);
    } else {
        return (numberStartToEndLanes > 0);
    }
}



bool Road::HasIncomingLane(const VertexIdType& vertexId) const
{
    if (vertexId == (*this).GetStartVertexId()) {
        return (numberStartToEndLanes > 0);
    } else {
        return (numberEndToStartLanes > 0);
    }
}



void TrafficLight::SyncTrafficLight(const SimTime& currentTime)
{
    while (!trafficLightIdsPerTime.empty() &&
           currentTime > trafficLightIdsPerTime.front().first) {
        trafficLightIdsPerTime.pop_front();
    }
}



TrafficLightType TrafficLight::GetTrafficLight(const SimTime& currentTime) const
{
    if (!trafficLightIdsPerTime.empty()) {
        return trafficLightIdsPerTime.front().second;
    }

    const SimTime cycleDuration =
        greenDuration + yellowDuration + redDuration;

    const SimTime timeOffset =
        (currentTime + startOffset) % cycleDuration;

    if (timeOffset < greenDuration) {
        return TRAFFIC_LIGHT_GREEN;
    } else if (timeOffset < greenDuration + yellowDuration) {
        return TRAFFIC_LIGHT_YELLOW;
    }

    return TRAFFIC_LIGHT_RED;
}

bool TrafficLight::IntersectsWith(const Rectangle& rect) const
{
    return rect.Contains((*this).GisObject::GetVertex(0));
}



TrafficLightType Intersection::GetTrafficLight(
    const SimTime& currentTime,
    const RoadIdType& incomingRoadId) const
{
    if (!(*this).IsEnabled()) {
        return TRAFFIC_LIGHT_GREEN;
    }

    typedef map<RoadIdType, TrafficLightIdType>::const_iterator IterType;

    IterType iter = trafficLightIds.find(incomingRoadId);

    if (iter == trafficLightIds.end()) {
        return TRAFFIC_LIGHT_GREEN;
    }

    const TrafficLight& trafficLight = roadLayerPtr->GetTrafficLight((*iter).second);
    return trafficLight.GetTrafficLight(currentTime);
}



SimTime Intersection::CalculateCrossingStartTime(
    const SimTime& currentTime,
    const RoadIdType& roadId,
    const bool leftCrossing,
    const SimTime minWalkDuration) const
{
    if (!(*this).IsEnabled()) {
        return currentTime;
    }

    typedef map<RoadIdType, TrafficLightIdType>::const_iterator IterType;

    IterType iter = trafficLightIds.find(roadId);

    if (iter == trafficLightIds.end()) {
        return currentTime;
    }

    const TrafficLight& trafficLight = roadLayerPtr->GetTrafficLight((*iter).second);

    const SimTime cycleDuration =
        trafficLight.greenDuration + trafficLight.yellowDuration + trafficLight.redDuration;

    const SimTime timeOffset =
        (currentTime + trafficLight.startOffset) % cycleDuration;

    if (timeOffset < trafficLight.greenDuration + trafficLight.yellowDuration) {
        return currentTime + (trafficLight.greenDuration + trafficLight.yellowDuration - timeOffset);
    }

    if (timeOffset + minWalkDuration > cycleDuration) {
        return currentTime + (cycleDuration - timeOffset) + trafficLight.greenDuration + trafficLight.yellowDuration;
    }
    return currentTime;
}



IntersectionIdType Intersection::GetIntersectionId() const
{
    return commonImplPtr->variantId;
}



vector<RoadIdType> Intersection::GetConnectedRoadIds() const
{
    return (*this).GetGisVertex().GetConnectedObjectIds(GIS_ROAD);
}



RoadIdType Intersection::GetRoadIdTo(const IntersectionIdType& endIntersectionId) const
{
    const vector<RoadIdType> roadIds = (*this).GetConnectedRoadIds();
    const IntersectionIdType intersectionId = (*this).GetIntersectionId();

    for(size_t i = 0; i < roadIds.size(); i++) {
        const RoadIdType& roadId = roadIds[i];
        const Road& road = commonImplPtr->subsystemPtr->GetRoad(roadId);

        if (road.GetOtherSideIntersectionId(intersectionId) == endIntersectionId) {
            return roadId;
        }
    }

    assert(false && "No road connection");
    return INVALID_VARIANT_ID;
}



const Vertex& Intersection::GetVertex() const
{
    return (*this).GisObject::GetVertex(0);
}



VertexIdType Intersection::GetVertexId() const
{
    return (*this).GisObject::GetVertexId(0);
}



const GisVertex& Intersection::GetGisVertex() const
{
    return (*this).GisObject::GetGisVertex(0);
}



bool Intersection::IsTerminated() const
{
    return ((*this).GetConnections().size() == 1);
}



size_t Intersection::GetRoadsideNumber(const RoadIdType& roadId) const
{
    typedef map<RoadIdType, size_t>::const_iterator IterType;

    IterType iter = roadsideNumberPerRoadId.find(roadId);

    if (iter == roadsideNumberPerRoadId.end()) {
        cerr << "Error: invalid road id " << roadId << " for " << (*this).GetIntersectionId() << endl << "valid road id ";

        for(IterType validIter = roadsideNumberPerRoadId.begin();
            validIter != roadsideNumberPerRoadId.end(); validIter++) {
            cerr << "," << (*validIter).first;
        }
        cerr << endl;

        exit(1);
    }

    return (*iter).second;
}



bool Intersection::ContainsRoadside(const RoadIdType& roadId) const
{
    typedef map<RoadIdType, size_t>::const_iterator IterType;

    return (roadsideNumberPerRoadId.find(roadId) != roadsideNumberPerRoadId.end());
}



bool Intersection::CanPassRoad(
    const RoadIdType& incomingRoadId,
    const RoadIdType& outgoingRoadId) const
{
    if (incomingRoadId == INVALID_VARIANT_ID) {
        return true;
    }

    typedef map<pair<RoadIdType, RoadIdType>, RoadTurnType>::const_iterator IterType;

    IterType iter = roadTurnTypes.find(make_pair(incomingRoadId, outgoingRoadId));

    if (iter == roadTurnTypes.end()) {
        return false;
    }

    return (*iter).second.hasLanes;
}



const RoadTurnType& Intersection::GetRoadTurnType(
    const RoadIdType& incomingRoadId,
    const RoadIdType& outgoingRoadId) const
{
    typedef map<pair<RoadIdType, RoadIdType>, RoadTurnType>::const_iterator IterType;

    IterType iter = roadTurnTypes.find(make_pair(incomingRoadId, outgoingRoadId));

    if (iter == roadTurnTypes.end()) {
        cerr << "No road connection from Road" << incomingRoadId << " to Road" << outgoingRoadId << endl;
        exit(1);
    }

    return (*iter).second;
}



bool Intersection::PedestrianCanPass() const
{
    const vector<RoadIdType> roadIds = (*this).GetConnectedRoadIds();

    for(size_t i = 0; i < roadIds.size(); i++) {
        const RoadIdType& roadId = roadIds[i];
        const Road& road = commonImplPtr->subsystemPtr->GetRoad(roadId);

        if (!(road.IsPedestrianRoad() || road.IsExtraPath())) {
            return false;
        }
    }

    return true;
}



const map<GisObjectType, vector<VertexConnection> >& Intersection::GetConnections() const
{
    return (*this).GetGisVertex().connections;
}


bool Intersection::IntersectsWith(const Rectangle& rect) const
{
    return rect.Contains((*this).GetVertex());
}


VertexIdType BusStop::GetLineVertexId(
    const BusLineIdType& lineId,
    const RouteIdType& routeId) const
{
    return (*this).GetVertexId();
}



bool BusStop::IsBusStopVertex(const VertexIdType& vertexId) const
{
    return ((*this).GetVertexId() == vertexId);
}



VertexIdType BusStop::GetVertexId() const
{
    return (*this).GisObject::GetVertexId(0);
}



VertexIdType BusStop::GetNearestEntranceVertexId(const Vertex& targetPosition) const
{
    typedef set<EntranceIdType>::const_iterator IterType;

    double minDisntace = DBL_MAX;

    assert(!entranceIds.empty());

    VertexIdType entranceVertexId = INVALID_VERTEX_ID;

    for(IterType iter = entranceIds.begin(); iter != entranceIds.end(); iter++) {
        const Entrance& entrance = commonImplPtr->subsystemPtr->GetEntrance(*iter);
        const VertexIdType vertexId = entrance.GetVertexId();
        const Vertex& entrancePosition = commonImplPtr->subsystemPtr->GetVertex(vertexId);

        const double distance = entrancePosition.DistanceTo(targetPosition);

        if (distance < minDisntace) {
            entranceVertexId = vertexId;
            minDisntace = distance;
        }
    }

    return entranceVertexId;
}



void BusStop::UpdateMinRectangle() const
{
    const double margin = 0.00001;

    commonImplPtr->minRectangle = Rectangle(position, margin);
}



bool BusStop::IntersectsWith(const Rectangle& rect) const
{
    return rect.Contains(position);
}



void BusStop::CompleteEntrances(const size_t defaultNumberEntrances)
{
    if (!entranceIds.empty()) {
        return;
    }

    entranceIds.insert(commonImplPtr->subsystemPtr->CreateEntrance((*this).GetVertex()));
}


Vertex BusStop::AddEntrance(const EntranceIdType& entranceId, const Vertex& point)
{
    entranceIds.insert(entranceId);

    return Vertex(
        point.x,
        point.y,
        (*this).GetVertex().z);
}



RailRoadIdType RailRoad::GetRailRoadId() const
{
    return commonImplPtr->variantId;
}



void RailRoad::UpdateMinRectangle() const
{
    Rectangle minRectangle;

    minRectangle.minX = DBL_MAX;
    minRectangle.minY = DBL_MAX;
    minRectangle.maxX = -DBL_MAX;
    minRectangle.maxY = -DBL_MAX;

    assert(!vertices.empty());

    for(size_t i = 0; i < vertices.size(); i++) {
        const Vertex& vertex = railRoadLayerPtr->railVertices.at(vertices[i].first).vertex;

        minRectangle.minX = std::min(minRectangle.minX, vertex.x);
        minRectangle.minY = std::min(minRectangle.minY, vertex.y);
        minRectangle.maxX = std::max(minRectangle.maxX, vertex.x);
        minRectangle.maxY = std::max(minRectangle.maxY, vertex.y);
    }

    commonImplPtr->minRectangle = minRectangle;
}



bool RailRoad::IntersectsWith(const Rectangle& rect) const
{
    for(size_t i = 0; i < vertices.size(); i++) {
        const Vertex& vertex = railRoadLayerPtr->railVertices.at(vertices[i].first).vertex;

        if (rect.Contains(vertex)) {
            return true;
        }
    }

    const Vertex topLeft(rect.minX, rect.maxY);
    const Vertex bottomLeft(rect.minX, rect.minY);
    const Vertex topRight(rect.maxX,rect.maxY);
    const Vertex bottomRight(rect.maxX, rect.minY);

    for(size_t i = 0; i < vertices.size() - 1; i++) {
        const Vertex& p1 = railRoadLayerPtr->railVertices.at(vertices[i].first).vertex;
        const Vertex& p2 = railRoadLayerPtr->railVertices.at(vertices[i+1].first).vertex;

        if (HorizontalLinesAreIntersection(p1, p2, topLeft, bottomLeft) ||
            HorizontalLinesAreIntersection(p1, p2, bottomLeft, bottomRight) ||
            HorizontalLinesAreIntersection(p1, p2, bottomRight, topRight) ||
            HorizontalLinesAreIntersection(p1, p2, topRight, topLeft)) {
            return true;
        }
    }

    return false;
}



void RailRoad::SetEnabled(const bool enable)
{
    GisObject::SetEnabled(enable);

    typedef set<RailRoadLineIdType>::const_iterator IterType;

    for(IterType iter = railRoadLineIds.begin(); iter != railRoadLineIds.end(); iter++) {
        railRoadLayerPtr->UpdateRailRoadLineAvailability(*iter);
    }
}



RailRoadStationIdType RailRoadStation::GetStationId() const
{
    return commonImplPtr->variantId;
}



void RailRoadStation::AddRailRoadConnection(
    const RailRoadLineIdType& lineId,
    const RouteIdType& routedId,
    const Vertex& railRoadVertex)
{
    GisSubsystem* subsystemPtr = commonImplPtr->subsystemPtr;

    const VertexIdType railRoadVertexId =
        subsystemPtr->GetVertexId(railRoadVertex);

    const vector<VertexIdType>& entranceVertexIds =
        commonImplPtr->vertexIds;

    for(size_t i = 0; i < entranceVertexIds.size(); i++) {
        const VertexIdType& entranceVertexId = entranceVertexIds[i];

        if (entranceVertexId != railRoadVertexId) {
            subsystemPtr->GetRoadLayer().AddSimplePath(
                entranceVertexId,
                railRoadVertexId,
                ROAD_PATH);
        }
    }

    vertexIdPerLine[make_pair(lineId, routedId)] = railRoadVertexId;
    lineVertexIds.insert(railRoadVertexId);
}



bool RailRoadStation::HasLineVertex(
    const RailRoadLineIdType& lineId,
    const RouteIdType& routeId) const
{
    typedef map<pair<RailRoadLineIdType, RailRoadIdType>, VertexIdType>::const_iterator IterType;

    IterType iter = vertexIdPerLine.find(make_pair(lineId, routeId));

    return (iter != vertexIdPerLine.end());
}



VertexIdType RailRoadStation::GetLineVertexId(
    const RailRoadLineIdType& lineId,
    const RouteIdType& routeId) const
{
    typedef map<pair<RailRoadLineIdType, RailRoadIdType>, VertexIdType>::const_iterator IterType;

    IterType iter = vertexIdPerLine.find(make_pair(lineId, routeId));

    assert(iter != vertexIdPerLine.end());

    return (*iter).second;
}



VertexIdType RailRoadStation::GetNearestEntranceVertexId(const Vertex& position) const
{
    double minSqrtDistance = DBL_MAX;
    VertexIdType nearestVertexId = INVALID_VERTEX_ID;

    const vector<VertexIdType>& entranceVertexIds = commonImplPtr->vertexIds;

    if (entranceVertexIds.empty()) {
        cerr << "Error: No entrance at " << commonImplPtr->objectName << endl;
        exit(1);
    }

    for(size_t i = 0; i < entranceVertexIds.size(); i++) {

        const VertexIdType& entranceVertexId = entranceVertexIds[i];

        if (!commonImplPtr->subsystemPtr->IsVertexOf(GIS_ROAD, entranceVertexId)) {
            continue;
        }

        const Vertex& entrancePos =
            commonImplPtr->subsystemPtr->GetVertex(entranceVertexId);

        const double sqrtDistance = SquaredXYZDistanceBetweenVertices(
            position, entrancePos);

        if (sqrtDistance < minSqrtDistance) {
            minSqrtDistance = sqrtDistance;
            nearestVertexId = entranceVertexId;
        }
    }
    assert(nearestVertexId != INVALID_VERTEX_ID);

    return nearestVertexId;
}



void RailRoadStation::CompleteEntrances(const size_t defaultNumberEntrances)
{
    if (!entranceIds.empty()) {
        return;
    }

    const vector<Vertex> middlePoints = GetMiddlePointsOfPolygon(polygon);

    for(size_t i = 0; i < std::min(defaultNumberEntrances, middlePoints.size()); i++) {
        entranceIds.insert(commonImplPtr->subsystemPtr->CreateEntrance(middlePoints[i]));
    }
}



void RailRoadStation::UpdateMinRectangle() const
{
    commonImplPtr->minRectangle = GetPointsRect(polygon);
}



bool RailRoadStation::IntersectsWith(const Rectangle& rect) const
{
    return RectIsIntersectsWithPolygon(rect, polygon);
}



Vertex RailRoadStation::AddEntrance(const EntranceIdType& entranceId, const Vertex& point)
{
    entranceIds.insert(entranceId);

    return Vertex(
        point.x,
        point.y,
        (*this).GetVertex().z);
}



void Wall::UpdateMinRectangle() const
{
    commonImplPtr->minRectangle = GetPointsRect(vertices);
}



bool Wall::IntersectsWith(const Rectangle& rect) const
{
    for(size_t i = 0; i < vertices.size(); i++) {
        if (rect.Contains(vertices[i])) {
            return true;
        }
    }

    const Vertex topLeft(rect.minX, rect.maxY);
    const Vertex bottomLeft(rect.minX, rect.minY);
    const Vertex topRight(rect.maxX,rect.maxY);
    const Vertex bottomRight(rect.maxX, rect.minY);

    for(size_t i = 0; i < vertices.size() - 1; i++) {
        const Vertex& p1 = vertices[i];
        const Vertex& p2 = vertices[i+1];

        if (HorizontalLinesAreIntersection(p1, p2, topLeft, bottomLeft) ||
            HorizontalLinesAreIntersection(p1, p2, bottomLeft, bottomRight) ||
            HorizontalLinesAreIntersection(p1, p2, bottomRight, topRight) ||
            HorizontalLinesAreIntersection(p1, p2, topRight, topLeft)) {
            return true;
        }
    }

    return false;
}



const Material& Wall::GetMaterial() const
{
    return commonImplPtr->subsystemPtr->GetMaterial(materialId);
}



vector<Vertex> Wall::MakeWallPolygon() const
{
    vector<Vertex> polygon;

    const deque<Vertex> dequeVertices(vertices.begin(), vertices.end());

    GetLinePolygon(dequeVertices, widthMeters, polygon);

    return polygon;
}



BuildingIdType Building::GetBuildingId() const
{
    return commonImplPtr->variantId;
}



const Material& Building::GetOuterWallMaterial() const
{
    return commonImplPtr->subsystemPtr->GetMaterial(outerWallMaterialId);
}



const Material& Building::GetRoofMaterial() const
{
    return commonImplPtr->subsystemPtr->GetMaterial(roofWallMaterialId);
}



const Material& Building::GetFloorMaterial() const
{
    return commonImplPtr->subsystemPtr->GetMaterial(floorWallMaterialId);
}



void Building::CompleteEntrances(const size_t defaultNumberEntrances)
{
    if (!entranceIds.empty()) {
        return;
    }

    const vector<Vertex> middlePoints = GetMiddlePointsOfPolygon(polygon);

    for(size_t i = 0; i < std::min(defaultNumberEntrances, middlePoints.size()); i++) {
        entranceIds.insert(commonImplPtr->subsystemPtr->CreateEntrance(middlePoints[i]));
    }
}



Vertex Building::GetRandomPosition(HighQualityRandomNumberGenerator& aRandomNumberGenerator) const
{
    const Rectangle& minRect = (*this).GetMinRectangle();
    const int maxTryCount = 5;

    Vertex randomPosition;

    for(int i = 0; i < maxTryCount; i++) {
        randomPosition.x = minRect.minX + (minRect.maxX - minRect.minX)*aRandomNumberGenerator.GenerateRandomDouble();
        randomPosition.y = minRect.minY + (minRect.maxY - minRect.minY)*aRandomNumberGenerator.GenerateRandomDouble();

        if (PolygonContainsPoint(polygon, randomPosition)) {
            break;
        }
    }

    if (PolygonContainsPoint(polygon, randomPosition)) {
        randomPosition.z = polygon.front().z;
    } else {
        randomPosition = polygon[aRandomNumberGenerator.GenerateRandomInt(0, static_cast<int32_t>(polygon.size() - 1))];
    }

    return randomPosition;
}




VertexIdType Building::GetNearestEntranceVertexId(const Vertex& position) const
{
    double minSqrtDistance = DBL_MAX;
    VertexIdType nearestVertexId = 0;

    const vector<VertexIdType>& entranceVertexIds = commonImplPtr->vertexIds;

    if (entranceVertexIds.empty()) {
        cerr << "Error: No entrance at " << commonImplPtr->objectName << endl;
        exit(1);
    }

    for(size_t i = 0; i < entranceVertexIds.size(); i++) {

        const VertexIdType& entranceVertexId = entranceVertexIds[i];
        const Vertex& entrancePos =
            commonImplPtr->subsystemPtr->GetVertex(entranceVertexId);

        const double sqrtDistance = SquaredXYZDistanceBetweenVertices(
            position, entrancePos);

        if (sqrtDistance < minSqrtDistance) {
            minSqrtDistance = sqrtDistance;
            nearestVertexId = entranceVertexId;
        }
    }

    return nearestVertexId;
}



void Building::GetNearEntranceVertexIds(const Vertex& position, vector<VertexIdType>& vertexIds) const
{
    const vector<VertexIdType>& entranceVertexIds = commonImplPtr->vertexIds;

    if (entranceVertexIds.empty()) {
        cerr << "Error: No entrance at " << commonImplPtr->objectName << endl;
        exit(1);
    }

    multimap<double, VertexIdType> nearVertexIds;

    for(size_t i = 0; i < entranceVertexIds.size(); i++) {

        const VertexIdType& entranceVertexId = entranceVertexIds[i];
        const Vertex& entrancePos =
            commonImplPtr->subsystemPtr->GetVertex(entranceVertexId);

        const double sqrtDistance = SquaredXYZDistanceBetweenVertices(
            position, entrancePos);

        nearVertexIds.insert(make_pair(sqrtDistance, entranceVertexId));
    }

    typedef multimap<double, VertexIdType>::const_iterator IterType;

    vertexIds.clear();

    for(IterType iter = nearVertexIds.begin();
        iter != nearVertexIds.end(); iter++) {

        vertexIds.push_back((*iter).second);
    }
}



double Building::CalculateSize() const
{
    return CalculatePolygonSize(polygon);
}



Vertex Building::AddEntrance(
    const EntranceIdType& entranceId,
    const Vertex& point)
{
    entranceIds.insert(entranceId);

    Vertex entrancePosition(
        point.x,
        point.y);

    if (!polygon.empty()) {
        entrancePosition.z = polygon.front().z;
    }

    return entrancePosition;
}



Vertex Building::GetEntranceVertex(const EntranceIdType& entranceId) const
{
    return (commonImplPtr->subsystemPtr->GetEntrance(entranceId).GetVertex());
}



EntranceIdType Building::GetNearestEntranceId(const Vertex& position) const
{

    if (entranceIds.empty()) {
        cerr << "Error: No entrance at " << commonImplPtr->objectName << endl;
        exit(1);
    }

    double minSqrtDistance = DBL_MAX;
    EntranceIdType nearestEntranceId = InvalidVariantId;

    typedef set<EntranceIdType>::const_iterator IterType;

    for(IterType iter = entranceIds.begin(); (iter != entranceIds.end()); ++iter) {
        const EntranceIdType& entranceId = *iter;
        const Vertex& entranceVertex = GetEntranceVertex(entranceId);

        const double sqrtDistance =
            SquaredXYZDistanceBetweenVertices(position, entranceVertex);

        if (sqrtDistance < minSqrtDistance) {
            minSqrtDistance = sqrtDistance;
            nearestEntranceId = entranceId;
        }//if//
    }//for//

    assert(nearestEntranceId != InvalidVariantId);

    return (nearestEntranceId);
}




RoadIdType Building::GetNearestEntranceRoadId(const Vertex& position) const
{
    const VertexIdType nearestVertexId = (*this).GetNearestEntranceVertexId(position);
    const GisVertex& poiGisVertex = commonImplPtr->subsystemPtr->GetGisVertex(nearestVertexId);
    const vector<RoadIdType>& roadIds = poiGisVertex.GetConnectedObjectIds(GIS_ROAD);

    for(size_t i = 0; i < roadIds.size(); i++) {
        const RoadIdType& roadId = roadIds[i];

        if (commonImplPtr->subsystemPtr->GetRoad(roadId).IsParking()) {
            return roadId;
        }
    }

    cerr << "Error: No entrance road at " << commonImplPtr->objectName << endl;
    exit(1);

    return INVALID_VARIANT_ID;
}





void Building::UpdateMinRectangle() const
{
    commonImplPtr->minRectangle = GetPointsRect(polygon);
}



bool Building::IntersectsWith(const Rectangle& rect) const
{
    return RectIsIntersectsWithPolygon(rect, polygon);
}



void Area::UpdateMinRectangle() const
{
    commonImplPtr->minRectangle = GetPointsRect(polygon);
}



double Area::CalculateSize() const
{
    return CalculatePolygonSize(polygon);
}



bool Area::IntersectsWith(const Rectangle& rect) const
{
    return RectIsIntersectsWithPolygon(rect, polygon);
}



ParkIdType Park::GetParkId() const
{
    return commonImplPtr->variantId;
}



void Park::CompleteEntrances(const size_t defaultNumberEntrances)
{
    if (!entranceIds.empty()) {
        return;
    }

    const vector<Vertex> middlePoints = GetMiddlePointsOfPolygon(polygon);

    for(size_t i = 0; i < std::min(defaultNumberEntrances, middlePoints.size()); i++) {
        entranceIds.insert(commonImplPtr->subsystemPtr->CreateEntrance(middlePoints[i]));
    }
}



void Park::UpdateMinRectangle() const
{
    commonImplPtr->minRectangle = GetPointsRect(polygon);
}



bool Park::IntersectsWith(const Rectangle& rect) const
{
    return RectIsIntersectsWithPolygon(rect, polygon);
}



Vertex Park::GetRandomPosition(HighQualityRandomNumberGenerator& aRandomNumberGenerator) const
{
    const Rectangle& minRect = (*this).GetMinRectangle();
    const int maxTryCount = 5;

    Vertex randomPosition;

    for(int i = 0; i < maxTryCount; i++) {
        randomPosition.x = minRect.minX + (minRect.maxX - minRect.minX)*aRandomNumberGenerator.GenerateRandomDouble();
        randomPosition.y = minRect.minY + (minRect.maxY - minRect.minY)*aRandomNumberGenerator.GenerateRandomDouble();

        if (PolygonContainsPoint(polygon, randomPosition)) {
            break;
        }
    }

    if (PolygonContainsPoint(polygon, randomPosition)) {
        randomPosition.z = polygon.front().z;
    } else {
        randomPosition = polygon[aRandomNumberGenerator.GenerateRandomInt(0, static_cast<int32_t>(polygon.size() - 1))];
    }

    return randomPosition;
}



VertexIdType Park::GetNearestEntranceVertexId(const Vertex& position) const
{
    double minSqrtDistance = DBL_MAX;
    VertexIdType nearestVertexId = 0;

    const vector<VertexIdType>& entranceVertexIds = commonImplPtr->vertexIds;
    assert(!entranceVertexIds.empty());

    for(size_t i = 0; i < entranceVertexIds.size(); i++) {

        const VertexIdType& entranceVertexId = entranceVertexIds[i];
        const Vertex& entrancePos =
            commonImplPtr->subsystemPtr->GetVertex(entranceVertexId);

        const double sqrtDistance = SquaredXYZDistanceBetweenVertices(
            position, entrancePos);

        if (sqrtDistance < minSqrtDistance) {
            minSqrtDistance = sqrtDistance;
            nearestVertexId = entranceVertexId;
        }
    }

    return nearestVertexId;
}

EntranceIdType Park::GetNearestEntranceId(const Vertex& position) const
{
    double minSqrtDistance = DBL_MAX;
    EntranceIdType nearestEntranceId = InvalidVariantId;

    typedef set<EntranceIdType>::const_iterator IterType;

    for(IterType iter = entranceIds.begin(); (iter != entranceIds.end()); ++iter) {
        const EntranceIdType& entranceId = *iter;
        const Vertex& entranceVertex = commonImplPtr->subsystemPtr->GetEntrance(entranceId).GetVertex();

        const double sqrtDistance =
            SquaredXYZDistanceBetweenVertices(position, entranceVertex);

        if (sqrtDistance < minSqrtDistance) {
            minSqrtDistance = sqrtDistance;
            nearestEntranceId = entranceId;
        }//if//
    }//for//

    assert(nearestEntranceId != InvalidVariantId);

    return (nearestEntranceId);
}


void Park::GetNearEntranceVertexIds(const Vertex& position, vector<VertexIdType>& vertexIds) const
{
    const vector<VertexIdType>& entranceVertexIds = commonImplPtr->vertexIds;

    if (entranceVertexIds.empty()) {
        cerr << "Error: No entrance at " << commonImplPtr->objectName << endl;
        exit(1);
    }

    multimap<double, VertexIdType> nearVertexIds;

    for(size_t i = 0; i < entranceVertexIds.size(); i++) {

        const VertexIdType& entranceVertexId = entranceVertexIds[i];
        const Vertex& entrancePos =
            commonImplPtr->subsystemPtr->GetVertex(entranceVertexId);

        const double sqrtDistance = SquaredXYZDistanceBetweenVertices(
            position, entrancePos);

        nearVertexIds.insert(make_pair(sqrtDistance, entranceVertexId));
    }

    typedef multimap<double, VertexIdType>::const_iterator IterType;

    vertexIds.clear();

    for(IterType iter = nearVertexIds.begin();
        iter != nearVertexIds.end(); iter++) {

        vertexIds.push_back((*iter).second);
    }
}



RoadIdType Park::GetNearestEntranceRoadId(const Vertex& position) const
{
    const VertexIdType nearestVertexId = (*this).GetNearestEntranceVertexId(position);
    const GisVertex& poiGisVertex = commonImplPtr->subsystemPtr->GetGisVertex(nearestVertexId);
    const vector<RoadIdType>& roadIds = poiGisVertex.GetConnectedObjectIds(GIS_ROAD);

    for(size_t i = 0; i < roadIds.size(); i++) {
        const RoadIdType& roadId = roadIds[i];

        if (commonImplPtr->subsystemPtr->GetRoad(roadId).IsParking()) {
            return roadId;
        }
    }

    cerr << "Error: No entrance road at " << commonImplPtr->objectName << endl;
    exit(1);

    return INVALID_VARIANT_ID;
}



double Park::CalculateSize() const
{
    return CalculatePolygonSize(polygon);
}



Vertex Park::AddEntrance(
    const EntranceIdType& entranceId,
    const Vertex& point)
{
    entranceIds.insert(entranceId);

    Vertex entrancePosition(
        point.x,
        point.y);

    if (!polygon.empty()) {
        entrancePosition.z = polygon.front().z;
    }

    return entrancePosition;
}


void GenericPolygon::UpdateMinRectangle() const
{
    commonImplPtr->minRectangle = GetPointsRect(vertices);
}

bool GenericPolygon::IntersectsWith(const Rectangle& rect) const
{
    return RectIsIntersectsWithPolygon(rect, vertices);
}


//------------------------------------------------------------------------------
// Layer
//------------------------------------------------------------------------------

RoadLayer::RoadLayer(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    GisSubsystem* initSubsystemPtr)
    :
    subsystemPtr(initSubsystemPtr),
    isRightHandTraffic(false),
    breakDownCurvedRoads(false),
    numberEntrancesToBuilding(0),
    numberEntrancesToStation(0),
    numberEntrancesToBusStop(0),
    numberEntrancesToPark(0),
    setIntersectionMargin(false),
    maxRoadWidthMeters(0.01) //1cm
{
    if (theParameterDatabaseReader.ParameterExists("gis-road-driving-side")) {

        const string drivingSide =
            MakeLowerCaseString(theParameterDatabaseReader.ReadString("gis-road-driving-side"));

        if (drivingSide == "right") {
            isRightHandTraffic = true;
        }
        else if (drivingSide == "left") {
            isRightHandTraffic = false;
        }
        else {
            cerr << "Unknown Driving Side (right|left): " << drivingSide << endl;
            exit(1);
        }//if//

    }//if//

    if (theParameterDatabaseReader.ParameterExists("gis-los-break-down-cureved-road-into-straight-roads")) {
        // cost match memory
        breakDownCurvedRoads = theParameterDatabaseReader.ReadBool("gis-los-break-down-cureved-road-into-straight-roads");
    }

    if (theParameterDatabaseReader.ParameterExists("gis-number-entrances-to-building")) {
        numberEntrancesToBuilding = theParameterDatabaseReader.ReadNonNegativeInt("gis-number-entrances-to-building");
    }
    if (theParameterDatabaseReader.ParameterExists("gis-number-entrances-to-station")) {
        numberEntrancesToStation = theParameterDatabaseReader.ReadNonNegativeInt("gis-number-entrances-to-station");
    }
    if (theParameterDatabaseReader.ParameterExists("gis-number-entrances-to-busstop")) {
        numberEntrancesToBusStop = theParameterDatabaseReader.ReadNonNegativeInt("gis-number-entrances-to-busstop");
    }
    if (theParameterDatabaseReader.ParameterExists("gis-number-entrances-to-park")) {
        numberEntrancesToPark = theParameterDatabaseReader.ReadNonNegativeInt("gis-number-entrances-to-park");
    }

    if (theParameterDatabaseReader.ParameterExists("gis-road-set-intersection-margin")) {
        setIntersectionMargin = theParameterDatabaseReader.ReadBool("gis-road-set-intersection-margin");
    }
}



void RoadLayer::ImportIntersection(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder radiusFinder(hDBF, GIS_DBF_RADIUS_STRING);
    const AttributeFinder generationVolumeFinder(hDBF, GIS_DBF_GENERATION_VOLUME_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);

    map<SimTime, vector<TrafficLightIdType> > trafficLightIdsPerTime;

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);
        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);

        assert(shpObjPtr->nParts == 0);
        assert(shpObjPtr->nVertices == 1);

        const double x = shpObjPtr->padfX[0];
        const double y = shpObjPtr->padfY[0];
        double z = shpObjPtr->padfZ[0];

        if (ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId)) {
            z += subsystemPtr->GetGroundElevationMetersAt(x, y);
        }

        const Vertex point(x, y, z);

        string name;
        double radiusMeters = 7.5;
        double generationVolume = 0.0;

        if (nameFinder.IsAvailable()) {
            name = nameFinder.GetLowerString(entryId);
        }
        if (radiusFinder.IsAvailable()) {
            radiusMeters = radiusFinder.GetDouble(entryId);
        }
        if (generationVolumeFinder.IsAvailable()) {
            generationVolume = generationVolumeFinder.GetDouble(entryId);
        }

        (*this).CreateIntersectionIfNecessary(point, name, radiusMeters, generationVolume);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}



void RoadLayer::ImportRoad(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder widthFinder(hDBF, GIS_DBF_WIDTH_STRING);
    const AttributeFinder startToEndLaneFinder(hDBF, GIS_DBF_LANE12_STRING);
    const AttributeFinder endToStartLaneFinder(hDBF, GIS_DBF_LANE21_STRING);
    const AttributeFinder typeFinder(hDBF, GIS_DBF_TYPE_STRING);
    const AttributeFinder capacityFinder(hDBF, GIS_DBF_CAPACITY_STRING);
    const AttributeFinder speedLimitFinder(hDBF, GIS_DBF_SPEED_LIMIT_STRING);

    set<Vertex> vertices;

    // register intersection
    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        Vertex startPoint(
            shpObjPtr->padfX[0],
            shpObjPtr->padfY[0],
            shpObjPtr->padfZ[0]);

        Vertex endPoint(
            shpObjPtr->padfX[shpObjPtr->nVertices-1],
            shpObjPtr->padfY[shpObjPtr->nVertices-1],
            shpObjPtr->padfZ[shpObjPtr->nVertices-1]);

        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        if (isBaseGroundLevel) {
            startPoint.z += subsystemPtr->GetGroundElevationMetersAt(startPoint);
            endPoint.z += subsystemPtr->GetGroundElevationMetersAt(endPoint);
        }

        (*this).CreateIntersectionIfNecessary(startPoint);
        (*this).CreateIntersectionIfNecessary(endPoint);

        vertices.insert(startPoint);
        vertices.insert(endPoint);

        if (breakDownCurvedRoads) {
            for(int i = 1; i < shpObjPtr->nVertices - 1; i++) {
                const double x = shpObjPtr->padfX[i];
                const double y = shpObjPtr->padfY[i];
                double z = shpObjPtr->padfZ[i];

                if (isBaseGroundLevel) {
                    z += subsystemPtr->GetGroundElevationMetersAt(x, y);
                }

                const Vertex point(x, y, z);

                if (vertices.find(point) == vertices.end()) {
                    (*this).CreateIntersectionIfNecessary(point);
                    subsystemPtr->GetVertexId(point);
                    vertices.insert(point);
                }
            }
        }

        SHPDestroyObject(shpObjPtr);
    }

    // register roads
    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        assert(shpObjPtr->nParts == 1);
        assert(shpObjPtr->nVertices > 0);

        vector<VertexIdType> vertexIds;
        deque<Vertex> roadVertices;

        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        for(int i = 0; i < shpObjPtr->nVertices; i++) {
            const double x = shpObjPtr->padfX[i];
            const double y = shpObjPtr->padfY[i];
            double z = shpObjPtr->padfZ[i];

            if (isBaseGroundLevel) {
                z += subsystemPtr->GetGroundElevationMetersAt(x, y);
            }

            const Vertex point(x, y, z);

            roadVertices.push_back(point);

            const VertexIdType vertexId = subsystemPtr->GetVertexId(point);
            vertexIds.push_back(vertexId);

            if (!breakDownCurvedRoads) {
                if (!subsystemPtr->IsVertexPoint(point)) {
                    continue;
                }
                if (!subsystemPtr->IsIntersectionVertex(vertexId)) {
                    continue;
                }
            }

            if (vertexIds.size() >= 2) {
                const RoadIdType roadId = static_cast<RoadIdType>(roadPtrs.size());
                roadPtrs.push_back(make_shared<Road>(subsystemPtr, objectId, roadId));

                Road& road = *roadPtrs.back();

                road.LoadParameters(theParameterDatabaseReader);
                road.isBaseGroundLevel = isBaseGroundLevel;

                if (nameFinder.IsAvailable()) {
                    road.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
                }
                if (startToEndLaneFinder.IsAvailable()) {
                    const int numberStartToEndLanes =
                        startToEndLaneFinder.GetInt(entryId);

                    if (numberStartToEndLanes < 0) {
                        cerr << "Error: Number of lanes is negative value: " << numberStartToEndLanes << " for " << road.commonImplPtr->objectName << endl;
                        exit(1);
                    }

                    road.numberStartToEndLanes = static_cast<size_t>(numberStartToEndLanes);
                }
                if (endToStartLaneFinder.IsAvailable()) {
                    const int numberEndToStartLanes =
                        endToStartLaneFinder.GetInt(entryId);

                    if (numberEndToStartLanes < 0) {
                        cerr << "Error: Number of lanes is negative value: " << numberEndToStartLanes << " for " << road.commonImplPtr->objectName << endl;
                        exit(1);
                    }

                    road.numberEndToStartLanes = static_cast<size_t>(numberEndToStartLanes);
                }

                if (road.numberStartToEndLanes == 0 &&
                    road.numberEndToStartLanes == 0) {
                    cerr << "Error: Total number of lanes is 0 for " << road.commonImplPtr->objectName << endl
                         << "  Total number of lanes must be greater than 0." << endl;
                    exit(1);
                }

                if (widthFinder.IsAvailable()) {
                    road.widthMeters = widthFinder.GetDouble(entryId);
                } else {
                    const double defaultRoadWidthMeters = 6;
                    road.widthMeters =
                        defaultRoadWidthMeters*
                        (road.numberStartToEndLanes + road.numberEndToStartLanes);
                }

                if (typeFinder.IsAvailable()) {
                    const string typeName = typeFinder.GetLowerString(entryId);

                    if (typeName == "road" || typeName == "vehicleandpedestrian") {
                        road.type = ROAD_ROAD;
                    } else if (typeName == "pedestrian" || typeName == "pedestianonly") {
                        road.type = ROAD_PEDESTRIAN;
                    } else if (typeName == "motorway" || typeName == "vehicleonly") {
                        road.type = ROAD_MOTORWAY;
                    }
                    //Road import/export feature
                    //future// } else if (typeName == "extrapedestrianpath") {
                    //future//     road.type = ROAD_EXTRA_ROAD;
                    //future// } else if (typeName == "railroadstationwalkway") {
                    //future//     road.type = ROAD_PATH;
                    else {
                        road.type = ROAD_ROAD;
                    }
                }

                road.isRightHandTraffic = isRightHandTraffic;
                road.vertices = roadVertices;
                road.commonImplPtr->vertexIds = vertexIds;
                road.UpdatePolygon();

                assert(road.polygon.size() > 2);

                if (capacityFinder.IsAvailable()) {
                    road.capacity = capacityFinder.GetDouble(entryId);
                    road.humanCapacity =
                        static_cast<int>(
                            std::min<double>(INT_MAX, std::ceil(road.capacity*CalculatePolygonSize(road.polygon))));
                    assert(road.humanCapacity >= 0);
                }

                if (speedLimitFinder.IsAvailable()) {
                    road.speedLimitMetersPerSec =
                        speedLimitFinder.GetDouble(entryId) / 3.6; //convert km/h -> m/s

                    if (road.speedLimitMetersPerSec <= 0.) {
                        cerr << "Error: Speed limit is " << speedLimitFinder.GetDouble(entryId) << " for " << road.commonImplPtr->objectName << endl
                             << "  Speed limit must be greater than 0." << endl;
                        exit(1);
                    }
                }

                const VertexIdType startVertexId = vertexIds.front();
                const VertexIdType endVertexId = vertexIds.back();

                if (vertexIds.size() == 2) {
                    const SimplePathKey simplePathKey(startVertexId, endVertexId);

                    simplePathIdMap[simplePathKey] = roadId;
                }//if//

                subsystemPtr->ConnectBidirectionalGisObject(
                    startVertexId, GIS_ROAD, endVertexId, roadId);

                subsystemPtr->RegisterGisObject(road, roadId);

                maxRoadWidthMeters = std::max(maxRoadWidthMeters, road.widthMeters);

                roadVertices.clear();
                vertexIds.clear();

                roadVertices.push_back(point);
                vertexIds.push_back(vertexId);
            }
        }

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}


void RoadLayer::ExportRoad(const string& filePath)
{
    const int shapeType = SHPT_ARCZ;

    SHPHandle hSHP = SHPCreate(filePath.c_str(), shapeType);
    DBFHandle hDBF = DBFCreate(filePath.c_str());

    if ((hSHP == nullptr) || (hDBF == nullptr)) {
        cerr << "Error: Failed to open " << filePath << endl;
        exit(1);
    }//if//

    const int idFieldId = 0;
    const int widthFieldId = 1;
    const int typeFieldId = 2;
    const int numberLane12FieldId = 3;
    const int numberLane21FieldId = 4;
    const int nameFieldId = 5;
    const int capacityFieldId = 6;
    const int speedLimitFieldId = 7;

    const int idFieldLength = 10;
    const int widthFieldLength = 10;
    const int numberDigits = 10;
    const int typeFieldLength = 32;
    const int numberLane12FieldLength = 2;
    const int numberLane21FieldLength = 2;
    const int nameFieldLength = 128;
    const int capacityFieldLength = 10;
    const int speedLimitFieldLength = 10;


    if (DBFAddField(hDBF, GIS_DBF_ID_STRING.c_str(), FTInteger, idFieldLength, 0) == -1) {
        cerr << "Error: Failed to add atrribute \"" << GIS_DBF_ID_STRING << "\"" << endl;
        exit(1);
    }//if//

    if (DBFAddField(hDBF, GIS_DBF_WIDTH_STRING.c_str(), FTDouble, widthFieldLength, numberDigits) == -1) {
        cerr << "Error: Failed to add atrribute \"" << GIS_DBF_WIDTH_STRING << "\"" << endl;
        exit(1);
    }//if//

    if (DBFAddField(hDBF, GIS_DBF_TYPE_STRING.c_str(), FTString, typeFieldLength, 0) == -1) {
        cerr << "Error: Failed to add atrribute \"" << GIS_DBF_TYPE_STRING << "\"" << endl;
        exit(1);
    }//if//

    if (DBFAddField(hDBF, GIS_DBF_LANE12_STRING.c_str(), FTInteger, numberLane12FieldLength, 0) == -1) {
        cerr << "Error: Failed to add atrribute \"" << GIS_DBF_LANE12_STRING << "\"" << endl;
        exit(1);
    }//if//

    if (DBFAddField(hDBF, GIS_DBF_LANE21_STRING.c_str(), FTInteger, numberLane21FieldLength, 0) == -1) {
        cerr << "Error: Failed to add atrribute \"" << GIS_DBF_LANE21_STRING << "\"" << endl;
        exit(1);
    }//if//

    if (DBFAddField(hDBF, GIS_DBF_NAME_STRING.c_str(), FTString, nameFieldLength, 0) == -1) {
        cerr << "Error: Failed to add atrribute \"" << GIS_DBF_NAME_STRING << "\"" << endl;
        exit(1);
    }//if//

    if (DBFAddField(hDBF, GIS_DBF_CAPACITY_STRING.c_str(), FTDouble, capacityFieldLength, numberDigits) == -1) {
        cerr << "Error: Failed to add atrribute \"" << GIS_DBF_CAPACITY_STRING << "\"" << endl;
        exit(1);
    }//if//

    if (DBFAddField(hDBF, GIS_DBF_SPEED_LIMIT_STRING.c_str(), FTDouble, speedLimitFieldLength, numberDigits) == -1) {
        cerr << "Error: Failed to add atrribute \"" << GIS_DBF_SPEED_LIMIT_STRING << "\"" << endl;
        exit(1);
    }//if//

    const GisObjectIdType GISOBJECT_ROAD_START_ID = 100000001;

    int roadNumber = 0;

    for(size_t i = 0; i < roadPtrs.size(); i++) {
        const Road& road = *roadPtrs[i];

        if (road.IsParking()) {
            continue;
        }//if//

        const int numberOfVertices = static_cast<int>(road.NumberOfVertices());

        //add arc points
        shared_array<double> padfX(new double[numberOfVertices]);
        shared_array<double> padfY(new double[numberOfVertices]);
        shared_array<double> padfZ(new double[numberOfVertices]);

        for(int j = 0; j < numberOfVertices; j++) {
            const Vertex& vertex = road.GetVertex(j);

            padfX[j] = vertex.x;
            padfY[j] = vertex.y;
            padfZ[j] = vertex.z;
        }//for//

        SHPObject* shpObj = SHPCreateSimpleObject(
            shapeType, numberOfVertices, padfX.get(), padfY.get(), padfZ.get());

        assert(shpObj != nullptr);

        //add road id
        DBFWriteIntegerAttribute(hDBF, roadNumber, idFieldId, roadNumber + GISOBJECT_ROAD_START_ID);
        DBFWriteDoubleAttribute(hDBF, roadNumber, widthFieldId, road.GetWidthMeters());

        const RoadType roadType = road.GetRoadType();

        string roadTypeString = "vehicleandpedestrian";

        if (roadType == ROAD_ROAD) {
            roadTypeString = "vehicleandpedestrian";
        }
        else if (roadType == ROAD_PEDESTRIAN) {
            roadTypeString = "pedestianonly";
        }
        //Road import/export feature
        //future// else if (roadType == ROAD_EXTRA_ROAD) {
        //future//     roadTypeString = "extrapedestrianpath";
        //future// }
        //future// else if (roadType == ROAD_PATH) {
        //future//     roadTypeString = "railroadstationwalkway";
        //future// }
        else if (roadType == ROAD_MOTORWAY) {
            roadTypeString = "vehicleonly";
        }//if//

        DBFWriteStringAttribute(hDBF, roadNumber, typeFieldId, roadTypeString.c_str());

        DBFWriteIntegerAttribute(hDBF, roadNumber, numberLane12FieldId, static_cast<int>(road.GetNumberOfStartToEndLanes()));
        DBFWriteIntegerAttribute(hDBF, roadNumber, numberLane21FieldId, static_cast<int>(road.GetNumberOfEndToStartLanes()));
        DBFWriteStringAttribute(hDBF, roadNumber, nameFieldId, road.GetObjectName().c_str());
        DBFWriteDoubleAttribute(hDBF, roadNumber, capacityFieldId, road.GetCapacity());
        DBFWriteDoubleAttribute(hDBF, roadNumber, speedLimitFieldId, road.GetSpeedLimitMetersPerSec()*3.6); //convert m/s -> km/h

        SHPWriteObject(hSHP, -1, shpObj);
        SHPDestroyObject(shpObj);

        roadNumber++;
    }//for//

    SHPClose(hSHP);
    DBFClose(hDBF);

}//OvtputCurrentRoadShapeFile//

void RoadLayer::ExportIntersection(const string& filePath)
{
    const int shapeType = SHPT_POINTZ;

    SHPHandle hSHP = SHPCreate(filePath.c_str(), shapeType);
    DBFHandle hDBF = DBFCreate(filePath.c_str());

    if ((hSHP == nullptr) || (hDBF == nullptr)) {
        cerr << "Error: Failed to open " << filePath << endl;
        exit(1);
    }//if//

    const int idFieldId = 0;
    const int radiusFieldId = 1;
    const int generationVolumeFieldId = 2;
    const int nameFieldId = 3;


    const int idFieldLength = 10;
    const int radiusFieldLength = 10;
    const int numberDigits = 10;
    const int generationVolumeFieldLength = 10;
    const int nameFieldLength = 128;


    if (DBFAddField(hDBF, GIS_DBF_ID_STRING.c_str(), FTInteger, idFieldLength, 0) == -1) {
        cerr << "Error: Failed to add atrribute \"" << GIS_DBF_ID_STRING << "\"" << endl;
        exit(1);
    }//if//

    if (DBFAddField(hDBF, GIS_DBF_TYPE_STRING.c_str(), FTDouble, radiusFieldLength, numberDigits) == -1) {
        cerr << "Error: Failed to add atrribute \"" << GIS_DBF_TYPE_STRING << "\"" << endl;
        exit(1);
    }//if//

    if (DBFAddField(hDBF, GIS_DBF_TYPE_STRING.c_str(), FTDouble, generationVolumeFieldLength, numberDigits) == -1) {
        cerr << "Error: Failed to add atrribute \"" << GIS_DBF_TYPE_STRING << "\"" << endl;
        exit(1);
    }//if//

    if (DBFAddField(hDBF, GIS_DBF_NAME_STRING.c_str(), FTString, nameFieldLength, 0) == -1) {
        cerr << "Error: Failed to add atrribute \"" << GIS_DBF_NAME_STRING << "\"" << endl;
        exit(1);
    }//if//

    const NodeId GISOBJECT_INTERSECTION_START_ID = 101000001;

    int intersectionNumber = 0;

    for(size_t i = 0; i < intersections.size(); i++) {
        const Intersection& intersection = intersections[i];

        Vertex vertex = intersection.GetVertex();

        SHPObject* shpObj = SHPCreateSimpleObject(
            shapeType, 1,
            &vertex.x,
            &vertex.y,
            &vertex.z);

        assert(shpObj != nullptr);

        DBFWriteIntegerAttribute(hDBF, intersectionNumber, idFieldId, intersectionNumber + GISOBJECT_INTERSECTION_START_ID);
        DBFWriteDoubleAttribute(hDBF, intersectionNumber, radiusFieldId, intersection.GetRadiusMeters());
        DBFWriteDoubleAttribute(hDBF, intersectionNumber, generationVolumeFieldId, intersection.GetGenerationVolumen());
        DBFWriteStringAttribute(hDBF, intersectionNumber, nameFieldId, intersection.GetObjectName().c_str());

        SHPWriteObject(hSHP, -1, shpObj);
        SHPDestroyObject(shpObj);

        intersectionNumber++;
    }//for//

    SHPClose(hSHP);
    DBFClose(hDBF);


}//ExportIntersection//


inline
bool IsStraightDirection(const double radians)
{
    const double directionRadians =
        std::max<double>(0, ((radians) - (PI/4.)) / (PI/2.));

    return (1. <= directionRadians && directionRadians < 2.);
}



void RoadLayer::ImportTrafficLight(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    map<string, PeriodicTrafficLightPatternData> periodicTrafficLightPatterns;
    map<string, DistributedTrafficLightPatternData> distributedTrafficLightPatterns;

    (*this).ReadTrafficLightPatternFileIfNecessary(
        theParameterDatabaseReader,
        periodicTrafficLightPatterns,
        distributedTrafficLightPatterns);

    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder offsetFinder(hDBF, GIS_DBF_OFFSET_STRING);
    const AttributeFinder greenFinder(hDBF, GIS_DBF_GREEN_STRING);
    const AttributeFinder yellowFinder(hDBF, GIS_DBF_YELLOW_STRING);
    const AttributeFinder redFinder(hDBF, GIS_DBF_RED_STRING);
    const AttributeFinder patternFinder(hDBF, GIS_DBF_PATTERN_STRING);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder roadIdFinder(hDBF, GIS_DBF_ROADID_STRING);

    map<SimTime, vector<TrafficLightIdType> > trafficLightIdsPerTime;

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);
        GisObjectIdType objectId = INVALID_GIS_OBJECT_ID;

        if (idFinder.IsAvailable()) {
            objectId = idFinder.GetGisObjectId(entryId);
        }

        assert(shpObjPtr->nParts == 0);
        assert(shpObjPtr->nVertices == 1);

        const double x = shpObjPtr->padfX[0];
        const double y = shpObjPtr->padfY[0];
        double z = shpObjPtr->padfZ[0];

        if (ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId)) {
            z += subsystemPtr->GetGroundElevationMetersAt(x, y);
        }

        const Vertex point(x, y, z);

        string name;
        SimTime startOffset = ZERO_TIME;
        SimTime greenDuration = 60*SECOND;
        SimTime yellowDuration = 0*SECOND;
        SimTime redDuration = 60*SECOND;
        string patternName;
        map<SimTime, TrafficLightType> trafficLightPerTime;

        GisPositionIdType intersectionPositionId;
        GisPositionIdType roadPositionId;

        if (nameFinder.IsAvailable()) {
            name = nameFinder.GetLowerString(entryId);
        }


        if (patternFinder.IsAvailable()) {
            patternName = patternFinder.GetLowerString(entryId);
        }

        if (!patternName.empty()) {
            if (periodicTrafficLightPatterns.find(patternName) != periodicTrafficLightPatterns.end()) {
                const PeriodicTrafficLightPatternData& patternData =
                    periodicTrafficLightPatterns[patternName];

                greenDuration = patternData.greenDuration;
                yellowDuration = patternData.yellowDuration;
                redDuration = patternData.redDuration;

            } else if (distributedTrafficLightPatterns.find(patternName) != distributedTrafficLightPatterns.end()) {
                const DistributedTrafficLightPatternData& patternData =
                    distributedTrafficLightPatterns[patternName];

                trafficLightPerTime = patternData.trafficLightPerTime;
            }
        } else {
            if (offsetFinder.IsAvailable()) {
                startOffset = static_cast<SimTime>(SECOND*offsetFinder.GetDouble(entryId));
            }
            if (greenFinder.IsAvailable()) {
                greenDuration = static_cast<SimTime>(SECOND*greenFinder.GetDouble(entryId));
            }
            if (yellowFinder.IsAvailable()) {
                yellowDuration = static_cast<SimTime>(SECOND*yellowFinder.GetDouble(entryId));
            }
            if (redFinder.IsAvailable()) {
                redDuration = static_cast<SimTime>(SECOND*redFinder.GetDouble(entryId));
            }
        }

        if (redDuration == ZERO_TIME) {
            cerr << "Error: Set non-zero red duration for trafficlight " << name << endl;
            exit(1);
        }

        const VertexIdType vertexId = subsystemPtr->GetVertexId(point);

        if (subsystemPtr->IsIntersectionVertex(vertexId)) {
            if (greenDuration + yellowDuration > redDuration) {
                cerr << "Error: Set red duration to be larger than sum of green duration and yellow duration for trafficlight " << name << endl;
                exit(1);
            }

            const IntersectionIdType intersectionId =
                subsystemPtr->GetIntersectionId(vertexId);

            Intersection& intersection = intersections[intersectionId];

            const Vertex& origVertex = intersection.GetVertex();
            const vector<RoadIdType>& connectedRoadIds = intersection.GetConnectedRoadIds();

            set<RoadIdType> alreadyGroupedRoadIds;
            vector<vector<RoadIdType> > groupedRoadIds;

            for(size_t i = 0; i < connectedRoadIds.size(); i++) {
                const RoadIdType roadId1 = connectedRoadIds[i];

                if (alreadyGroupedRoadIds.find(roadId1) != alreadyGroupedRoadIds.end()) {
                    continue;
                }

                groupedRoadIds.push_back(vector<RoadIdType>());
                vector<RoadIdType>& roadIds = groupedRoadIds.back();

                const Road& road1 = *roadPtrs[roadId1];

                roadIds.push_back(roadId1);
                alreadyGroupedRoadIds.insert(roadId1);

                for(size_t j = i+1; j < connectedRoadIds.size(); j++) {
                    const RoadIdType roadId2 = connectedRoadIds[j];

                    if (alreadyGroupedRoadIds.find(roadId2) != alreadyGroupedRoadIds.end()) {
                        continue;
                    }

                    const Road& road2 = *roadPtrs[roadId2];
                    const Vertex& vertex1 = road1.GetNeighborVertex(intersectionId);
                    const Vertex& vertex2 = road2.GetNeighborVertex(intersectionId);

                    if (IsStraightDirection(CalculateFullRadiansBetweenVector(vertex1, origVertex, vertex2))) {

                        assert(IsStraightDirection(CalculateFullRadiansBetweenVector(vertex2, origVertex, vertex1)));
                        roadIds.push_back(roadId2);
                        alreadyGroupedRoadIds.insert(roadId2);
                    }
                }
            }

            if (groupedRoadIds.size() > 0) {
                const SimTime cycleDuration = greenDuration + yellowDuration;

                if (cycleDuration == ZERO_TIME) {
                    redDuration = INFINITE_TIME;
                } else {
                    redDuration = redDuration * (groupedRoadIds.size() - 1);
                }

                for(size_t i = 0; i < groupedRoadIds.size(); i++) {
                    const vector<RoadIdType>& roadIds = groupedRoadIds[i];

                    const TrafficLightIdType trafficLightId = static_cast<TrafficLightIdType>(trafficLights.size());

                    (*this).AddTrafficLight(
                        theParameterDatabaseReader,
                        objectId,
                        intersection.GetVertexId(),
                        trafficLightPerTime,
                        startOffset + cycleDuration*i,
                        greenDuration,
                        yellowDuration,
                        redDuration,
                        trafficLightIdsPerTime);

                    for(size_t j = 0; j < roadIds.size(); j++) {
                        intersection.trafficLightIds.insert(make_pair(roadIds[j], trafficLightId));
                    }
                }
            }

        } else if (roadIdFinder.IsAvailable()) {
            const GisObjectIdType roadObjectId = roadIdFinder.GetGisObjectId(entryId);

            if (!subsystemPtr->ContainsObject(roadObjectId)) {
                cerr << "Error: trafficlight [" << name << "] is located at invalid road." << endl;
                exit(1);
            }

            roadPositionId = subsystemPtr->GetPositionId(roadObjectId);

            assert(roadPositionId.type == GIS_ROAD);

            const Road& road = *roadPtrs[roadPositionId.id];

            const IntersectionIdType intersectionid = road.GetNearestIntersectionId(point);

            Intersection& intersection = intersections[intersectionid];

            intersection.trafficLightIds.insert(
                make_pair(
                    roadPositionId.id,
                    (TrafficLightIdType)(trafficLights.size())));

            (*this).AddTrafficLight(
                theParameterDatabaseReader,
                objectId,
                intersection.GetVertexId(),
                trafficLightPerTime,
                startOffset,
                greenDuration,
                yellowDuration,
                redDuration,
                trafficLightIdsPerTime);

        } else {
        }

        SHPDestroyObject(shpObjPtr);
    }

    trafficLightChanges.assign(trafficLightIdsPerTime.begin(), trafficLightIdsPerTime.end());

    SHPClose(hSHP);
    DBFClose(hDBF);
}



void RoadLayer::AddTrafficLight(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GisObjectIdType& objectId,
    const VertexIdType& vertexId,
    const map<SimTime, TrafficLightType>& trafficLightPerTime,
    const SimTime& startOffset,
    const SimTime& greenDuration,
    const SimTime& yellowDuration,
    const SimTime& redDuration,
    map<SimTime, vector<TrafficLightIdType> >& trafficLightIdsPerTime)
{
    const TrafficLightIdType trafficLightId = static_cast<TrafficLightIdType>(trafficLights.size());

    if (trafficLightPerTime.empty()) {
        trafficLights.push_back(
            TrafficLight(
                subsystemPtr,
                objectId,
                trafficLightId,
                vertexId,
                startOffset,
                greenDuration,
                yellowDuration,
                redDuration));
    } else {

        trafficLights.push_back(
            TrafficLight(
                subsystemPtr,
                objectId,
                trafficLightId,
                vertexId,
                trafficLightPerTime));

        typedef map<SimTime, TrafficLightType>::const_iterator IterType;

        for(IterType iter = trafficLightPerTime.begin();
            iter != trafficLightPerTime.end(); iter++) {

            const SimTime& time = (*iter).first;

            trafficLightIdsPerTime[time].push_back(trafficLightId);
        }
    }

    trafficLights.back().LoadParameters(theParameterDatabaseReader);
}



void RoadLayer::SyncTrafficLight(const SimTime& currentTime)
{
    while (!trafficLightChanges.empty() &&
           currentTime >= trafficLightChanges.front().first) {

        const vector<TrafficLightIdType>& changedTrafficLightIds = trafficLightChanges.front().second;

        typedef vector<TrafficLightIdType>::const_iterator IterType;

        for(IterType iter = changedTrafficLightIds.begin();
            iter != changedTrafficLightIds.end(); iter++) {

            const TrafficLightIdType& trafficLightId = (*iter);

            trafficLights[trafficLightId].SyncTrafficLight(currentTime);
        }

        trafficLightChanges.pop_front();
    }
}



void RoadLayer::ReadTrafficLightPatternFileIfNecessary(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    map<string, PeriodicTrafficLightPatternData>& periodicTrafficLightPatterns,
    map<string, DistributedTrafficLightPatternData>& distributedTrafficLightPattern)
{
    SimTime synchronizationStep = 1 * NANO_SECOND;

    if (theParameterDatabaseReader.ParameterExists("time-step-event-synchronization-step")) {
        synchronizationStep = theParameterDatabaseReader.ReadTime("time-step-event-synchronization-step");
    }

    if (theParameterDatabaseReader.ParameterExists("gis-trafficlight-pattern-definition-file")) {
        const string patternFilePath =
            theParameterDatabaseReader.ReadString("gis-trafficlight-pattern-definition-file");

        ifstream inStream(patternFilePath.c_str());

        if (!inStream.good()) {
            cerr << "Error: Couldn't open trafficlight pattern file: " << patternFilePath << endl;
            exit(1);
        }//if//

        while(!inStream.eof()) {
            string aLine;
            getline(inStream, aLine);

            DeleteTrailingSpaces(aLine);

            if (IsAConfigFileCommentLine(aLine)) {
                continue;
            }

            deque<string> tokens;
            TokenizeToTrimmedLowerString(aLine, " ", tokens);

            if (tokens.size() < 2) {
                continue;
            }

            const string patternName = tokens[0];
            tokens.pop_front();

            if (aLine.find(":") != string::npos) {

                DistributedTrafficLightPatternData& patternData = distributedTrafficLightPattern[patternName];

                for(size_t i = 0; i < tokens.size(); i++) {
                    deque<string> timeAndLightType;

                    TokenizeToTrimmedLowerString(tokens[i], ":", timeAndLightType);

                    if (timeAndLightType.size() != 2) {
                        cerr << "Error: trafficlight pattern file: " << patternFilePath << " " << aLine << endl
                             << "PatternName [Time:LightType]+ " << endl;
                        exit(1);
                    }

                    SimTime time;
                    bool success;

                    ConvertStringToTime(timeAndLightType[0], time, success);

                    if (!success) {
                        cerr << "Error: trafficlight pattern file: " << patternFilePath << " " << aLine << endl
                             << "PatternName [Time:LightType]+ " << endl;
                        exit(1);
                    }

                    if (time % synchronizationStep != ZERO_TIME) {
                        cerr << "Error: trafficlight pattern file: " << patternFilePath << " " << aLine << endl
                             << "Time must be multiple number of time-step-event-synchronization-step." << endl;
                    }

                    TrafficLightType lightType;

                    if (timeAndLightType[1].find("blue") != string::npos ||
                        timeAndLightType[1].find("green") != string::npos) {
                        lightType = TRAFFIC_LIGHT_GREEN;
                    } else if (timeAndLightType[1].find("yellow") != string::npos) {
                        lightType = TRAFFIC_LIGHT_YELLOW;
                    } else if (timeAndLightType[1].find("red") != string::npos) {
                        lightType = TRAFFIC_LIGHT_RED;
                    } else {
                        cerr << "Error: trafficlight pattern file: " << patternFilePath << " " << aLine << endl
                             << "PatternName [Time:LightType]+ " << endl;
                        exit(1);
                    }

                    patternData.trafficLightPerTime[time] = lightType;
                }

            } else {

                if (tokens.size() < 3) {
                    cerr << "Error: trafficlight pattern file: " << patternFilePath << " " << aLine << endl
                         << "PatternName BlueDration YellowDuration RedDuration" << endl;
                    exit(1);
                }

                SimTime greenDuration;
                SimTime yellowDuration;
                SimTime redDuration;

                bool success1;
                bool success2;
                bool success3;

                ConvertStringToTime(tokens[0], greenDuration, success1);
                ConvertStringToTime(tokens[1], yellowDuration, success2);
                ConvertStringToTime(tokens[2], redDuration, success3);

                if (!success1 || !success2 || !success3) {
                    cerr << "Error: trafficlight pattern file: " << patternFilePath << " " << aLine << endl
                         << "PatternName BlueDration YellowDuration RedDuration" << endl;
                    exit(1);
                }

                periodicTrafficLightPatterns.insert(
                    make_pair(
                        patternName,
                        PeriodicTrafficLightPatternData(
                            greenDuration,
                            yellowDuration,
                            redDuration)));
            }
        }
    }
}

void RoadLayer::ImportBusStop(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder capacityFinder(hDBF, GIS_DBF_CAPACITY_STRING);

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);
        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);

        assert(shpObjPtr->nParts == 0);
        assert(shpObjPtr->nVertices == 1);

        const double x = shpObjPtr->padfX[0];
        const double y = shpObjPtr->padfY[0];
        double z = shpObjPtr->padfZ[0];

        if (ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId)) {
            z += subsystemPtr->GetGroundElevationMetersAt(x, y);
        }

        const Vertex point(x, y, z);

        bool foundRoad;
        RoadIdType roadId;

        (*this).FindRoadAt(point, foundRoad, roadId);

        if (!foundRoad) {
            cerr << "Error: No road is located arround bus stop." << endl;
            exit(1);
        }

        const BusStopIdType busStopId = static_cast<BusStopIdType>(busStops.size());

        busStops.push_back(BusStop(subsystemPtr, objectId, busStopId));
        BusStop& busStop = busStops.back();

        busStop.position = point;

        if (nameFinder.IsAvailable()) {
            busStop.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }
        if (capacityFinder.IsAvailable()) {
            busStop.humanCapacity = capacityFinder.GetInt(entryId);
        }

        subsystemPtr->RegisterGisObject(busStop, busStopId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}



void RoadLayer::CreateIntersectionIfNecessary(
    const Vertex& point,
    const string& name,
    const double radiusMeters,
    const double generationVolume)
{
    const VertexIdType& vertexId = subsystemPtr->GetVertexId(point);

    if (!subsystemPtr->IsIntersectionVertex(vertexId)) {
        const IntersectionIdType intersectionId = static_cast<IntersectionIdType>(intersections.size());

        intersections.push_back(Intersection(subsystemPtr, this, subsystemPtr->CreateNewObjectId(), intersectionId, vertexId));

        Intersection& intersection = intersections.back();

        intersection.commonImplPtr->objectName = name;
        intersection.radiusMeters = radiusMeters;
        intersection.generationVolume = generationVolume;

        subsystemPtr->ConnectGisObject(vertexId, GIS_INTERSECTION, intersectionId);
        subsystemPtr->RegisterGisObject(intersection, intersectionId);
    }
}



void RoadLayer::DivideRoad(
    const RoadIdType& srcRoadId,
    const VertexIdType& newVertexId)
{
    size_t vertexNumber = 0;
    Road& srcRoad = *roadPtrs.at(srcRoadId);

    if (newVertexId != srcRoad.GetStartVertexId() &&
        newVertexId != srcRoad.GetEndVertexId()) {

        const Vertex& position = subsystemPtr->GetVertex(newVertexId);
        const double capacity = srcRoad.capacity;

        subsystemPtr->DisconnectBidirectionalGisObject(
            srcRoad.GetStartVertexId(),
            GIS_ROAD,
            srcRoad.GetEndVertexId(),
            srcRoadId);

        subsystemPtr->UnregisterGisObject(
            srcRoad, srcRoadId);

        const SimplePathKey srcSimplePathKey(
            srcRoad.GetStartVertexId(),
            srcRoad.GetEndVertexId());

        simplePathIdMap.erase(srcSimplePathKey);

        const RoadIdType tailRoadId = static_cast<RoadIdType>(roadPtrs.size());

        const IntersectionIdType& startIntersectionId = srcRoad.GetStartIntersectionId();
        const IntersectionIdType& endIntersectionId = srcRoad.GetEndIntersectionId();

        map<RoadIdType, TrafficLightIdType>& startTrafficLightIds = intersections[startIntersectionId].trafficLightIds;
        map<RoadIdType, TrafficLightIdType>& endTrafficLightIds = intersections[endIntersectionId].trafficLightIds;

        if (startTrafficLightIds.find(srcRoadId) != startTrafficLightIds.end()) {
            startTrafficLightIds[tailRoadId] = startTrafficLightIds[srcRoadId];
        }
        if (endTrafficLightIds.find(srcRoadId) != endTrafficLightIds.end()) {
            endTrafficLightIds[tailRoadId] = endTrafficLightIds[srcRoadId];
        }

        const GisObjectIdType srcObjectId = srcRoad.GetObjectId();

        roadPtrs.push_back(make_shared<Road>(subsystemPtr, srcObjectId, tailRoadId));

        Road& headRoad = *roadPtrs.at(srcRoadId);
        Road& tailRoad = *roadPtrs.back();

        tailRoad.commonImplPtr->objectName = headRoad.commonImplPtr->objectName;
        tailRoad.isRightHandTraffic = isRightHandTraffic;
        tailRoad.widthMeters = headRoad.widthMeters;
        tailRoad.isBaseGroundLevel = headRoad.isBaseGroundLevel;
        tailRoad.numberStartToEndLanes = headRoad.numberEndToStartLanes;
        tailRoad.numberEndToStartLanes = headRoad.numberStartToEndLanes;
        tailRoad.speedLimitMetersPerSec = headRoad.speedLimitMetersPerSec;
        tailRoad.type = headRoad.type;

        double minDistance = DBL_MAX;
        vertexNumber = 0;

        for(size_t i = 0; i < headRoad.NumberOfVertices() - 1; i++) {

            const Vertex& edge1 = headRoad.GetVertex(i);
            const Vertex& edge2 = headRoad.GetVertex(i+1);

            const Vertex roadPosition = CalculatePointToLineNearestPosition(
                position, edge1, edge2);

            const double distance = roadPosition.DistanceTo(position);

            if (distance < minDistance) {
                minDistance = distance;
                vertexNumber = i;
            }
        }

        while (headRoad.vertices.size() > vertexNumber + 1) {
            tailRoad.vertices.push_back(headRoad.vertices.back());
            tailRoad.commonImplPtr->vertexIds.push_back(headRoad.commonImplPtr->vertexIds.back());
            headRoad.vertices.pop_back();
            headRoad.commonImplPtr->vertexIds.pop_back();
        }

        assert(headRoad.commonImplPtr->vertexIds.size() >= 1);
        assert(tailRoad.commonImplPtr->vertexIds.size() >= 1);
        assert(headRoad.vertices.size() >= 1);
        assert(tailRoad.vertices.size() >= 1);

        if (tailRoad.commonImplPtr->vertexIds.back() != newVertexId) {
            tailRoad.commonImplPtr->vertexIds.push_back(newVertexId);
        }
        if (headRoad.commonImplPtr->vertexIds.back() != newVertexId) {
            headRoad.commonImplPtr->vertexIds.push_back(newVertexId);
        }
        if (headRoad.vertices.back() != position) {
            headRoad.vertices.push_back(position);
        }
        if (tailRoad.vertices.back() != position) {
            tailRoad.vertices.push_back(position);
        }

        assert(headRoad.vertices.size() >= 2);
        assert(tailRoad.vertices.size() >= 2);

        subsystemPtr->ConnectBidirectionalGisObject(
            headRoad.commonImplPtr->vertexIds.front(), GIS_ROAD,
            headRoad.commonImplPtr->vertexIds.back(), srcRoadId);

        subsystemPtr->ConnectBidirectionalGisObject(
            tailRoad.commonImplPtr->vertexIds.front(), GIS_ROAD,
            tailRoad.commonImplPtr->vertexIds.back(), tailRoadId);

        headRoad.UpdatePolygon();
        tailRoad.UpdatePolygon();

        subsystemPtr->RegisterGisObject(headRoad, srcRoadId);
        subsystemPtr->RegisterGisObject(tailRoad, tailRoadId);

        if (headRoad.vertices.size() == 2) {
            const SimplePathKey smplePathKey(
                headRoad.commonImplPtr->vertexIds.front(),
                headRoad.commonImplPtr->vertexIds.back());

            simplePathIdMap[smplePathKey] = srcRoadId;
        }//if//
        if (tailRoad.vertices.size() == 2) {
            const SimplePathKey smplePathKey(
                tailRoad.commonImplPtr->vertexIds.front(),
                tailRoad.commonImplPtr->vertexIds.back());

            simplePathIdMap[smplePathKey] = tailRoadId;
        }//if//


        headRoad.capacity = capacity;
        headRoad.humanCapacity =
            static_cast<int>(
                std::min<double>(INT_MAX, std::ceil(capacity*CalculatePolygonSize(headRoad.polygon))));

        tailRoad.capacity = capacity;
        tailRoad.humanCapacity =
            static_cast<int>(
                std::min<double>(INT_MAX, std::ceil(capacity*CalculatePolygonSize(tailRoad.polygon))));

        assert(headRoad.humanCapacity >= 0);

        assert(tailRoad.humanCapacity >= 0);

        (*this).CreateIntersectionIfNecessary(position);
    }
}



void RoadLayer::MakeDirectPathToPoi(
    const RoadIdType& srcRoadId,
    const GisPositionIdType& destinationPositionId,
    const Vertex& position,
    RoadIdType& pathRoadId,
    VertexIdType& intersectionVertexId)
{
    Vertex nearestPosition;
    size_t vertexNumber = 0;

    Road& srcRoad = *roadPtrs.at(srcRoadId);

    CalculatePointToHorizontalArcNearestPosition(
        srcRoad.vertices, position, nearestPosition, vertexNumber);

    const double srcRoadWidth = srcRoad.widthMeters;
    const double srcCapacity = srcRoad.capacity;

    bool found;
    IntersectionIdType intersectionId;

    (*this).FindIntersectionAt(nearestPosition, srcRoadWidth/2, found, intersectionId);

    if (found) {

        intersectionVertexId = intersections.at(intersectionId).GetVertexId();

    } else {

        intersectionVertexId = subsystemPtr->GetVertexId(nearestPosition);

        (*this).DivideRoad(srcRoadId, intersectionVertexId);
    }

    const VertexIdType parkingVertexId = subsystemPtr->GetVertexId(position);

    pathRoadId = (*this).AddSimplePath(intersectionVertexId, parkingVertexId, ROAD_EXTRA_ROAD, srcRoadWidth, srcCapacity);

    (*this).CreateParking(destinationPositionId, parkingVertexId, roadPtrs.at(srcRoadId)->GetObjectId());

    (*this).CreateIntersectionIfNecessary(position);
}



void RoadLayer::AddBusStopVertex(
    const BusStopIdType& busStopId,
    const VertexIdType& vertexId)
{
    const BusStop& busStop = busStops.at(busStopId);

    if (!(*this).IsParking(vertexId)) {
        (*this).CreateParking(
            GisPositionIdType(GIS_BUSSTOP, busStopId),
            vertexId,
            subsystemPtr->CreateNewObjectId());
    }

    subsystemPtr->ConnectGisObject(vertexId, GIS_BUSSTOP, busStopId);

    busStop.commonImplPtr->vertexIds.push_back(vertexId);
}



bool RoadLayer::IsParking(const VertexIdType& vertexId) const
{
    const GisVertex& parkingGisVertex = subsystemPtr->GetGisVertex(vertexId);
    const vector<RoadIdType>& roadIds = parkingGisVertex.GetConnectedObjectIds(GIS_ROAD);

    for(size_t i = 0; i < roadIds.size(); i++) {

        if (roadPtrs[roadIds[i]]->IsParking()) {
            return true;
        }
    }

    return false;
}



void RoadLayer::CreateParking(
    const GisPositionIdType& destinationPositionId,
    const VertexIdType& vertexId,
    const GisObjectIdType& objectId)
{
    if ((*this).IsParking(vertexId)) {
        return;
    }

    const RoadIdType tailRoadId = static_cast<RoadIdType>(roadPtrs.size());

    roadPtrs.push_back(make_shared<Road>(subsystemPtr, objectId, tailRoadId));

    Road& parkingRoad = *roadPtrs.back();

    parkingRoad.commonImplPtr->vertexIds.push_back(vertexId);
    parkingRoad.type = ROAD_ROAD;

    // Specific to building, but could be made general.

    if (destinationPositionId.type == GIS_BUILDING) {
        parkingRoad.SetBuildingId(destinationPositionId.id);
    }//if//

    subsystemPtr->ConnectGisObject(vertexId, GIS_ROAD, tailRoadId);
}



RoadIdType RoadLayer::AddSimplePath(
    const VertexIdType& vertexId1,
    const VertexIdType& vertexId2,
    const RoadType& roadType,
    const double widthMeters,
    const double capacity)
{
    if (vertexId1 == vertexId2) {
        return INVALID_VARIANT_ID;
    }

    const SimplePathKey simplePathKey(vertexId1, vertexId2);

    typedef map<SimplePathKey, RoadIdType>::const_iterator IterType;

    IterType iter = simplePathIdMap.find(simplePathKey);

    if (iter != simplePathIdMap.end()) {
        return (*iter).second;
    }//if//

    const Vertex& position1 = subsystemPtr->GetVertex(vertexId1);
    const Vertex& position2 = subsystemPtr->GetVertex(vertexId2);

    const RoadIdType roadId = static_cast<RoadIdType>(roadPtrs.size());

    roadPtrs.push_back(make_shared<Road>(subsystemPtr, subsystemPtr->CreateNewObjectId(), roadId));

    Road& road = *roadPtrs.back();

    road.commonImplPtr->vertexIds.push_back(vertexId1);
    road.commonImplPtr->vertexIds.push_back(vertexId2);

    road.vertices.push_back(position1);
    road.vertices.push_back(position2);
    road.type = roadType;
    road.widthMeters = widthMeters;

    road.UpdatePolygon();

    road.capacity = capacity;
    road.humanCapacity =
        static_cast<int>(
            std::min<double>(INT_MAX, std::ceil(capacity*CalculatePolygonSize(road.polygon))));

    assert(road.humanCapacity >= 0);

    subsystemPtr->ConnectBidirectionalGisObject(
        vertexId1, GIS_ROAD, vertexId2, roadId);

    subsystemPtr->RegisterGisObject(road, roadId);

    (*this).CreateIntersectionIfNecessary(position1);
    (*this).CreateIntersectionIfNecessary(position2);

    simplePathIdMap[simplePathKey] = roadId;

    return roadId;
}



bool RoadLayer::ContainsBusStop(const string& name) const
{
    for(size_t i = 0; i < busStops.size(); i++) {
        if (busStops[i].GetObjectName() == name) {
            return true;
        }
    }

    return false;
}



BusLineIdType RoadLayer::GetBusLineId(const string& lineName) const
{
    typedef map<string, BusLineIdType>::const_iterator IterType;

    IterType iter = busLineIds.find(lineName);

    if (iter == busLineIds.end()) {
        cerr << "Error: Couldn't find bus line: " << lineName << endl;
        exit(1);
    }

    return (*iter).second;
}



BusStopIdType RoadLayer::GetBusStopId(const string& name) const
{
    for(size_t i = 0; i < busStops.size(); i++) {

        if (busStops[i].GetObjectName() == name) {
            return BusStopIdType(i);
        }
    }

    cerr << "Couldn't find bus stop " << name << endl;
    assert(false);
    return INVALID_VARIANT_ID;
}



void RoadLayer::GetRoadIdsAt(
    const Vertex& position,
    vector<RoadIdType>& roadIds) const
{
    roadIds.clear();

    const Rectangle searchRect(
        position.x - maxRoadWidthMeters,
        position.y - maxRoadWidthMeters,
        position.x + maxRoadWidthMeters,
        position.y + maxRoadWidthMeters);

    vector<RoadIdType> searchedRoadIds;

    subsystemPtr->GetSpatialIntersectedGisObjectIds(searchRect, GIS_ROAD, searchedRoadIds);

    typedef vector<RoadIdType>::const_iterator IterType;

    for(IterType iter = searchedRoadIds.begin(); iter != searchedRoadIds.end(); iter++) {

        const Road& aRoad = *roadPtrs[*iter];

        if (aRoad.Contains(position)) {
            roadIds.push_back(*iter);
        }
    }
}



void RoadLayer::SearchNearestIntersectionId(
    const Vertex& position,
    const Rectangle& searchRect,
    const set<RoadType>& availableRoadTypes,
    IntersectionIdType& nearestIntersectionId,
    bool& success) const
{
    vector<IntersectionIdType> intersectionIds;

    subsystemPtr->GetSpatialIntersectedGisObjectIds(searchRect, GIS_INTERSECTION, intersectionIds);

    typedef vector<IntersectionIdType>::const_iterator IterType;

    success = false;

    double minDistance = DBL_MAX;

    for(IterType iter = intersectionIds.begin(); iter != intersectionIds.end(); iter++) {

        const IntersectionIdType& intersectionId = (*iter);
        const Intersection& intersection = intersections[intersectionId];

        if (!searchRect.Contains(intersection.GetVertex())) {
            continue;
        }

        const vector<RoadIdType> roadIds = intersection.GetConnectedRoadIds();

        for(size_t i = 0; i < roadIds.size(); i++) {
            const Road& road = *roadPtrs[roadIds[i]];

            if (availableRoadTypes.empty() ||
                availableRoadTypes.find(road.GetRoadType()) != availableRoadTypes.end()) {

                const double distance = intersection.GetVertex().DistanceTo(position);

                if (distance < minDistance) {
                    minDistance = distance;
                    nearestIntersectionId = intersectionId;
                    success = true;
                }
            }
        }
    }
}



void RoadLayer::FindRoadAt(
    const Vertex& position,
    bool& found,
    RoadIdType& roadId) const
{
    const double findLength = 0.1;
    const double expandLength = 100;
    const int maxRetryCount = 10;

    found = false;

    for(int i = 0; i < maxRetryCount; i++) {
        vector<RoadIdType> roadIds;

        subsystemPtr->GetSpatialIntersectedGisObjectIds(
            Rectangle(position, findLength + i*expandLength), GIS_ROAD, roadIds);

        typedef vector<RoadIdType>::const_iterator IterType;

        if (roadIds.empty()) {
            continue;
        }

        double mindistance = DBL_MAX;

        for(IterType iter = roadIds.begin(); iter != roadIds.end(); iter++) {

            const Road& aRoad = *roadPtrs[(*iter)];
            double distance = aRoad.DistanceTo(position);

            if (distance < mindistance) {
                mindistance = distance;
                roadId = (*iter);
                found = true;
            }
        }
    }
}



void RoadLayer::FindIntersectionAt(
    const Vertex& position,
    const double radius,
    bool& found,
    IntersectionIdType& foundIntersectionId) const
{
    const Rectangle searchRect(position, radius);

    vector<IntersectionIdType> intersectionIds;

    typedef vector<IntersectionIdType>::const_iterator IterType;

    subsystemPtr->GetSpatialIntersectedGisObjectIds(searchRect, GIS_INTERSECTION, intersectionIds);

    double minDistance = DBL_MAX;

    found = false;

    for(IterType iter = intersectionIds.begin(); iter != intersectionIds.end(); iter++) {

        const IntersectionIdType& intersectionId = (*iter);
        const Intersection& intersection = intersections[intersectionId];

        const double distance = position.DistanceTo(intersection.GetVertex());

        if (distance <= radius &&
            distance < minDistance) {
            minDistance = distance;

            found = true;
            foundIntersectionId = intersectionId;
        }
    }
}



void RoadLayer::SetIntersectionMarginAndMakeLaneConnection()
{
    if (!setIntersectionMargin) {
        return;
    }

    for(size_t i = 0; i < roadPtrs.size(); i++) {
        Road& road = *roadPtrs[i];

        road.laneConnections.resize(road.numberStartToEndLanes + road.numberEndToStartLanes);
    }

    for(IntersectionIdType intersectionId = 0;
        intersectionId < IntersectionIdType(intersections.size()); intersectionId++) {

        const Intersection& intersection = intersections[intersectionId];

        const Vertex& basePoint = intersection.GetVertex();
        const vector<RoadIdType> roadIds = intersection.GetConnectedRoadIds();

        if (roadIds.size() <= 1) {
            continue;
        }

        //TBD// if (roadIds.size() <= 2) {
        //TBD//     continue;
        //TBD// }

        map<double, vector<RoadIdType> > roadIdsPerRadians;

        for(size_t j = 0; j < roadIds.size(); j++) {
            const RoadIdType roadId = roadIds[j];
            const Road& road = *roadPtrs[roadId];

            const double directionRadians =
                (road.GetNeighborVertex(intersectionId) - basePoint).DirectionRadians();

            if (road.IsParking()) {
                continue;
            }

            const vector<VertexIdType>& vertexIds = road.commonImplPtr->vertexIds;

            if (vertexIds.size() == 2 &&
                vertexIds.front() == vertexIds.back()) {
                continue;
            }

            roadIdsPerRadians[directionRadians].push_back(roadId);
        }

        (*this).SetIntersectionMargin(intersectionId, roadIdsPerRadians);

        (*this).MakeLaneConnection(intersectionId);
    }
}



// -PI to PI
inline
double CalculateFullRadiansBetweenVector(
    const Vertex& vertex1,
    const Vertex& vertex2)
{
    return (std::atan2(vertex2.y - vertex1.y, vertex2.x - vertex1.x));
}



struct RoadConnectionInfo {
    RoadIdType roadId;
    int prevMainRoadNumber;
    int nextMainRoadNumber;

    bool isExtraPath;

    RoadConnectionInfo(
        const RoadIdType& initRoadId,
        const int initPrevMainRoadNumber,
        const int initNextMainRoadNumber,
        const bool initIsExtraPath)
        :
        roadId(initRoadId),
        prevMainRoadNumber(initPrevMainRoadNumber),
        nextMainRoadNumber(initNextMainRoadNumber),
        isExtraPath(initIsExtraPath)
    {}
};




void RoadLayer::SetIntersectionMargin(
    const IntersectionIdType& intersectionId,
    const map<double, vector<RoadIdType> >& roadIdsPerRadians)
{
    Intersection& intersection = intersections[intersectionId];

    intersection.hasIntersectionPolygon = true;
    intersection.antiClockwiseRoadsides.clear();
    intersection.roadsideNumberPerRoadId.clear();

    if (roadIdsPerRadians.size() >= 2) {
        typedef map<double, vector<RoadIdType> >::const_iterator IterType;

        const Vertex& vertex = intersection.GetVertex();

        size_t i = 0;

        deque<RoadIdType> antiClockwiseMainRoadIds;
        deque<RoadConnectionInfo> antiClockwiseRoadInfos;

        for(IterType iter = roadIdsPerRadians.begin();
            iter != roadIdsPerRadians.end(); iter++, i++) {

            const vector<RoadIdType>& roadIds = (*iter).second;
            assert(!roadIds.empty());

            double maxRoadWidthMetersPerRadians = 0;
            RoadIdType wideRoadId = roadIds.front();

            if (roadIds.size() > 2) {
                for(size_t j = 0; j < roadIds.size(); j++) {
                    const RoadIdType& roadId = roadIds[j];

                    const Road& road = *roadPtrs[roadId];
                    const double roadWidthMeters = road.GetRoadWidthMeters();

                    if (roadWidthMeters > maxRoadWidthMetersPerRadians) {
                        maxRoadWidthMetersPerRadians = roadWidthMeters;
                        wideRoadId = roadId;
                    }
                }
            }

            const Road& road = *roadPtrs.at(wideRoadId);
            const int prevRoadNumber = int(antiClockwiseMainRoadIds.size()) - 1;
            const bool isExtraPath = road.IsExtraPath();

            if (!isExtraPath) {

                antiClockwiseMainRoadIds.push_back(wideRoadId);
                antiClockwiseRoadInfos.push_back(RoadConnectionInfo(wideRoadId, prevRoadNumber, (int)(antiClockwiseMainRoadIds.size()), isExtraPath));

            } else {
                antiClockwiseRoadInfos.push_back(RoadConnectionInfo(wideRoadId, prevRoadNumber, (int)(antiClockwiseMainRoadIds.size()), isExtraPath));

            }
        }

        if (antiClockwiseMainRoadIds.empty()) {
            return;
        }

        vector<double> margins;

        i = 0;
        for(IterType iter = roadIdsPerRadians.begin();
            iter != roadIdsPerRadians.end(); iter++, i++) {

            const vector<RoadIdType>& roadIds = (*iter).second;
            const RoadConnectionInfo& roadInfo = antiClockwiseRoadInfos[i];

            int prevMainRoadNumber = roadInfo.prevMainRoadNumber;
            if (prevMainRoadNumber < 0) {
                prevMainRoadNumber += (int)(antiClockwiseMainRoadIds.size());
            } else if (prevMainRoadNumber >= (int)(antiClockwiseMainRoadIds.size())) {
                prevMainRoadNumber -= (int)(antiClockwiseMainRoadIds.size());
            }
            int nextMainRoadNumber = roadInfo.nextMainRoadNumber;
            if (nextMainRoadNumber < 0) {
                nextMainRoadNumber += (int)(antiClockwiseMainRoadIds.size());
            } else if (nextMainRoadNumber >= (int)(antiClockwiseMainRoadIds.size())) {
                nextMainRoadNumber -= (int)(antiClockwiseMainRoadIds.size());
            }

            const RoadIdType& prevRoadId = antiClockwiseMainRoadIds.at(prevMainRoadNumber);
            const RoadIdType& nextRoadId = antiClockwiseMainRoadIds.at(nextMainRoadNumber);

            const Road& prevRoad = *roadPtrs.at(prevRoadId);
            const Road& road = *roadPtrs.at(roadInfo.roadId);
            const Road& nextRoad = *roadPtrs.at(nextRoadId);

            const double prevRoadWidth =
                (prevRoad.GetRoadWidthMeters() + road.GetRoadWidthMeters()) / 2.;

            const double nextRoadWidth =
                (nextRoad.GetRoadWidthMeters() + road.GetRoadWidthMeters()) / 2.;

            const pair<Vertex, Vertex> prevLine =
                prevRoad.GetSideLineToIntersection(intersectionId, -prevRoadWidth);

            const pair<Vertex, Vertex> line1 =
                road.GetSideLineToIntersection(intersectionId, prevRoadWidth);

            const pair<Vertex, Vertex> line2 =
                road.GetSideLineToIntersection(intersectionId, -nextRoadWidth);

            const pair<Vertex, Vertex> nextLine =
                nextRoad.GetSideLineToIntersection(intersectionId, nextRoadWidth);

            Vertex leftEdge;
            Vertex rightEdge;

            // vertex integration
            const double nearDistanceMeters = 0.1;

            if ((prevLine.first.DistanceTo(line1.first) > nearDistanceMeters) &&
                HorizontalLinesAreIntersection(prevLine.first, prevLine.second, line1.first, line1.second)) {

                leftEdge = CalculateIntersectionPositionBetweenLine(
                    prevLine.first, prevLine.second, line1.first, line1.second);
            } else {
                leftEdge = line1.first;
            }

            if ((nextLine.first.DistanceTo(line2.first) > nearDistanceMeters) &&
                HorizontalLinesAreIntersection(line2.first, line2.second, nextLine.first, nextLine.second)) {

                rightEdge = CalculateIntersectionPositionBetweenLine(
                    line2.first, line2.second, nextLine.first, nextLine.second);
            } else {
                rightEdge = line2.first;
            }

            if (leftEdge.DistanceTo(line1.first) > rightEdge.DistanceTo(line2.first)) {

                rightEdge = line2.first - (line1.first - leftEdge);

            } else {

                leftEdge = line1.first - (line2.first - rightEdge);
            }

            margins.push_back(leftEdge.DistanceTo(line1.first));

            for(size_t j = 0; j < roadIds.size(); j++) {
                const RoadIdType& roadId = roadIds[j];

                intersection.roadsideNumberPerRoadId[roadId] =
                    intersection.antiClockwiseRoadsides.size();
            }

            intersection.antiClockwiseRoadsides.push_back(
                Intersection::Roadside(leftEdge, rightEdge));
        }


        i = 0;
        for(IterType iter = roadIdsPerRadians.begin();
            iter != roadIdsPerRadians.end(); iter++, i++) {

            const double maxMargin = margins[i];
            const vector<RoadIdType>& roadIds = (*iter).second;

            for(size_t j = 0; j < roadIds.size(); j++) {
                const RoadIdType& roadId = roadIds[j];
                Road& road = *roadPtrs[roadId];

                const double marginLength =
                    std::min(maxMargin, std::max(0., (road.GetArcDistanceMeters() - maxMargin)));

                if (marginLength > 0) {
                    road.SetIntersectionMargin(intersectionId, marginLength);
                }
            }
        }
    }
}



inline
RoadTurnDirectionType CalculateDirectionFromRadians(const double radians)
{
    return static_cast<RoadTurnDirectionType>(
        std::max<double>(0, ((radians) - (PI/4.)) / (PI/2.)));
}



void RoadLayer::MakeLaneConnection(const IntersectionIdType& intersectionId)
{
    Intersection& intersection = intersections[intersectionId];

    const vector<RoadIdType> roadIds = intersection.GetConnectedRoadIds();

    const VertexIdType intersectionVertexId = intersection.GetVertexId();
    const Vertex& origVertex = intersection.GetVertex();

    map<pair<RoadIdType, RoadIdType>, RoadTurnType>& roadTurnTypes = intersection.roadTurnTypes;

    for(size_t i = 0; i < roadIds.size(); i++) {
        const RoadIdType roadId1 = roadIds[i];
        Road& road1 = *roadPtrs[roadId1];

        const bool hasRoad1OutgoingLane =
            road1.HasOutgoingLane(intersectionVertexId);

        const bool hasRoad1IncomingLane =
            road1.HasIncomingLane(intersectionVertexId);

        roadTurnTypes[make_pair(roadId1, roadId1)] =
            RoadTurnType(ROAD_TURN_BACK, road1.GetRoadWidthMeters()*2, hasRoad1OutgoingLane && hasRoad1IncomingLane);

        road1.MakeTurnaroundLaneConnection();

        if (roadIds.size() == 1) {
            continue;
        }

        const double roadWidth1 = road1.GetRoadWidthMeters();

        for(size_t j = i+1; j < roadIds.size(); j++) {
            const RoadIdType roadId2 = roadIds[j];

            Road& road2 = *roadPtrs[roadId2];
            const Vertex& vertex1 = road1.GetNeighborVertex(intersectionId);
            const Vertex& vertex2 = road2.GetNeighborVertex(intersectionId);
            const double totalRoadWidth = roadWidth1 + road2.GetRoadWidthMeters();

            const bool hasRoad2OutgoingLane =
                road2.HasOutgoingLane(intersectionVertexId);

            const bool hasRoad2IncomingLane =
                road2.HasIncomingLane(intersectionVertexId);

            const RoadTurnDirectionType direction12 =
                CalculateDirectionFromRadians(
                    CalculateFullRadiansBetweenVector(vertex1, origVertex, vertex2));

            const RoadTurnDirectionType direction21 =
                CalculateDirectionFromRadians(
                    CalculateFullRadiansBetweenVector(vertex2, origVertex, vertex1));

            road1.MakeLaneConnection(road2, intersectionVertexId, direction12);
            road2.MakeLaneConnection(road1, intersectionVertexId, direction21);

            roadTurnTypes[make_pair(roadId1, roadId2)] =
                RoadTurnType(direction12, totalRoadWidth, hasRoad1OutgoingLane && hasRoad2IncomingLane);
            roadTurnTypes[make_pair(roadId2, roadId1)] =
                RoadTurnType(direction21, totalRoadWidth, hasRoad2OutgoingLane && hasRoad1IncomingLane);
        }
    }
}



void RoadLayer::GetIntersectionIds(
    const Rectangle& searchRect,
    const set<RoadType>& availableRoadTypes, //throw empty to enable all road type.
    vector<IntersectionIdType>& foundIntersectionIds) const
{
    foundIntersectionIds.clear();

    vector<IntersectionIdType> intersectionIds;

    typedef vector<IntersectionIdType>::const_iterator IterType;

    subsystemPtr->GetSpatialIntersectedGisObjectIds(searchRect, GIS_INTERSECTION, intersectionIds);

    for(IterType iter = intersectionIds.begin(); iter != intersectionIds.end(); iter++) {

        const IntersectionIdType& intersectionId = (*iter);
        const Intersection& intersection = intersections[intersectionId];

        if (searchRect.Contains(intersection.GetVertex())) {
            foundIntersectionIds.push_back(intersectionId);
        }
    }
}



void RoadLayer::CreateIntersection(
    const VertexIdType& vertexId,
    const GisObjectIdType& objectId)
{
    const IntersectionIdType intersectionId =
        static_cast<IntersectionIdType>(intersections.size());

    intersections.push_back(Intersection(subsystemPtr, this, objectId, intersectionId, vertexId));
    subsystemPtr->ConnectGisObject(vertexId, GIS_INTERSECTION, intersectionId);
    subsystemPtr->RegisterGisObject(intersections.back(), intersectionId);
}


string RoadLayer::GetLineName(const RailRoadLineIdType& lineId) const
{
    typedef map<string, BusLineIdType>::const_iterator IterType;

    for(IterType iter = busLineIds.begin(); iter != busLineIds.end(); iter++) {
        if ((*iter).second == lineId) {
            return (*iter).first;
        }
    }

    return string();
}



void RoadLayer::AssignBusLine(
    const string& lineName,
    const deque<string>& busStopNames)
{
    if (busStopNames.empty()) {
        return;
    }

    if (busLineIds.find(lineName) == busLineIds.end()) {
        busLineIds[lineName] = (BusLineIdType)(routeInfosPerLine.size());
        routeInfosPerLine.push_back(vector<RouteInfo>());
    }

    RailRoadLineIdType lineId = busLineIds[lineName];
    deque<BusStopIdType> busStopIds;
    for(size_t i = 0; i < busStopNames.size(); i++) {
        busStopIds.push_back((*this).GetBusStopId(busStopNames[i]));
    }

    vector<RouteInfo>& routeInfos = routeInfosPerLine.at(lineId);

    for(size_t i = 0; i < routeInfos.size(); i++) {
        if (routeInfos[i].busStopIds == busStopIds) {
            cerr << "Error: Already assigned a bus route for line: " << endl
                 << (*this).GetLineName(lineId);

            for(size_t j = 0; j < busStopIds.size(); j++) {
                cerr << "," << (*this).GetBusStop(busStopIds[j]).GetObjectName();
            }
            cerr << endl;

            exit(1);
        }
    }

    routeInfos.push_back(RouteInfo());
    RouteInfo& routeInfo = routeInfos.back();
    routeInfo.busStopIds = busStopIds;
}



RouteIdType RoadLayer::GetRouteId(
    const BusLineIdType& lineId,
    const deque<BusStopIdType>& busStopIds) const
{
    const vector<RouteInfo>& routeInfos = routeInfosPerLine.at(lineId);
    bool found = false;
    RouteIdType routeId;

    for(routeId = 0; routeId < RouteIdType(routeInfos.size()); routeId++) {
        found = (routeInfos[routeId].busStopIds == busStopIds);

        if (found) {
            break;
        }
    }
    if (!found) {
        cerr << "Error: No bus route for line: " << (*this).GetLineName(lineId) << endl;
        exit(1);
    }

    return routeId;
}



const deque<BusStopIdType>& RoadLayer::GetRouteBusStopIds(
    const BusLineIdType& lineId,
    const RouteIdType& routeId) const
{
    return routeInfosPerLine.at(lineId).at(routeId).busStopIds;
}

//-------------------------------------------------------------

void RailRoadLayer::ImportRailRoad(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);

    map<VertexIdType, GisObjectIdType> intersectionObjectIds;
    map<GisObjectIdType, pair<string, set<VertexIdType> > > verticesPerStation;

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const RailRoadIdType railRoadId = static_cast<RailRoadIdType>(railRoads.size());
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        railRoads.push_back(RailRoad(subsystemPtr, this, objectId, railRoadId));
        RailRoad& railRoad = railRoads.back();

        railRoad.LoadParameters(theParameterDatabaseReader);

        if (nameFinder.IsAvailable()) {
            railRoad.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }

        assert(shpObjPtr->nParts == 1);
        assert(shpObjPtr->nVertices > 0);

        vector<VertexIdType>& vertexIds = railRoad.commonImplPtr->vertexIds;

        if (shpObjPtr->nVertices > 0) {
            railRoad.commonImplPtr->elevationFromGroundMeters = shpObjPtr->padfZ[0];
        }

        for(int i = 0; i < shpObjPtr->nVertices; i++) {
            const double x = shpObjPtr->padfX[i];
            const double y = shpObjPtr->padfY[i];
            double z = shpObjPtr->padfZ[i];

            if (isBaseGroundLevel) {
                z += subsystemPtr->GetGroundElevationMetersAt(x, y);
            }

            const Vertex point(x, y, z);
            const RailVertexIdType railVertexId = (*this).GetNewOrExistingRailVertexId(point);

            railRoad.vertices.push_back(make_pair(railVertexId, INVALID_VARIANT_ID));
        }

        const VertexIdType startVertexId =
            subsystemPtr->GetVertexId(railVertices.at(railRoad.vertices.front().first).vertex);

        const VertexIdType endVertexId =
            subsystemPtr->GetVertexId(railVertices.at(railRoad.vertices.back().first).vertex);

        vertexIds.push_back(startVertexId);
        vertexIds.push_back(endVertexId);

        // not linear connection

        subsystemPtr->ConnectBidirectionalGisObject(
            startVertexId, GIS_RAILROAD, endVertexId, railRoadId);

        subsystemPtr->RegisterGisObject(railRoad, railRoadId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}



RailVertexIdType RailRoadLayer::GetNewOrExistingRailVertexId(const Vertex& point)
{
    if (railVertexIds.find(point) == railVertexIds.end()) {
        railVertexIds[point] = (RailVertexIdType)(railVertices.size());
        railVertices.push_back(point);
    }

    return railVertexIds[point];
}



void RailRoadLayer::ImportStation(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder capacityFinder(hDBF, GIS_DBF_CAPACITY_STRING);

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        const RailRoadStationIdType stationId = static_cast<RailRoadStationIdType>(stations.size());
        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        stations.push_back(RailRoadStation(subsystemPtr, objectId, stationId));
        RailRoadStation& station = stations.back();
        station.LoadParameters(theParameterDatabaseReader);

        if (nameFinder.IsAvailable()) {
            station.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }
        if (capacityFinder.IsAvailable()) {
            station.humanCapacity = capacityFinder.GetInt(entryId);
        }

        assert(shpObjPtr->nParts == 1);
        assert(shpObjPtr->nVertices > 0);

        if (shpObjPtr->nVertices > 0) {
            station.commonImplPtr->elevationFromGroundMeters = shpObjPtr->padfZ[0];
        }

        for(int i = 0; i < shpObjPtr->nVertices; i++) {
            const double x = shpObjPtr->padfX[i];
            const double y = shpObjPtr->padfY[i];
            double z = shpObjPtr->padfZ[i];

            if (isBaseGroundLevel) {
                z += subsystemPtr->GetGroundElevationMetersAt(x, y);
            }

            station.polygon.push_back(Vertex(x, y, z));
        }

        station.UpdateMinRectangle();

        station.centerPosition = station.commonImplPtr->minRectangle.GetCenter();
        station.centerPosition.z =
            station.commonImplPtr->elevationFromGroundMeters +
            subsystemPtr->GetGroundElevationMetersAt(station.centerPosition);

        subsystemPtr->RegisterGisObject(station, stationId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}



void RailRoadLayer::GetShortestRailroadPath(
    const vector<vector<RailRoadStationIdType> >& stationIdsPerName,
    deque<RailRoadStationIdType>& bestStationIds,
    vector<pair<deque<RailLink>, double> >& bestSections,
    set<pair<RailRoadStationIdType, RailRoadStationIdType> >& noConnections,
    map<pair<RailRoadStationIdType, RailRoadStationIdType>, vector<pair<deque<RailLink>, double> > >& sectionCandidatesCache)
{
    size_t numberStationPatterns = 1;

    for(size_t i = 0; i < stationIdsPerName.size(); i++) {
        numberStationPatterns *= stationIdsPerName[i].size();
    }

    double minRunningDistance = DBL_MAX;

    vector<size_t> bestCandidateNumbers;
    vector<vector<pair<deque<RailLink>, double> > > bestSectionCandidatesPerSection;

    bestStationIds.clear();
    bestSectionCandidatesPerSection.clear();
    noConnections.clear();

    for(size_t stationPatternId = 0; stationPatternId < numberStationPatterns; stationPatternId++) {

        size_t remain = stationPatternId;

        deque<RailRoadStationIdType> stationIds;

        for(size_t i = 0; i < stationIdsPerName.size(); i++) {
            const vector<RailRoadStationIdType>& stationIdCandidates = stationIdsPerName[i];

            stationIds.push_back(stationIdCandidates[remain % stationIdCandidates.size()]);

            assert(!stationIdCandidates.empty());
            remain /= stationIdCandidates.size();
        }

        typedef vector<RailRoadIdType>::const_iterator RailIdIter;

        vector<vector<pair<deque<RailLink>, double> > > sectionCandidatesPerSection(stationIds.size() - 1);

        for(size_t i = 0; i < stationIds.size() - 1; i++) {

            const RailRoadStationIdType& stationId1 = stationIds[i];
            const RailRoadStationIdType& stationId2 = stationIds[i+1];

            const pair<RailRoadStationIdType, RailRoadStationIdType> stationIdPair(stationId1, stationId2);

            if (sectionCandidatesCache.find(stationIdPair) == sectionCandidatesCache.end()) {
                const RailRoadStation& station1 = stations[stationId1];
                const RailRoadStation& station2 = stations[stationId2];

                const Vertex& position1 = station1.GetVertex();
                const Vertex& position2 = station2.GetVertex();

                const set<RailVertexIdType>& railVertexIds1 = station1.railVertexIds;
                const set<RailVertexIdType>& railVertexIds2 = station2.railVertexIds;

                typedef set<RailVertexIdType>::const_iterator IterType;

                vector<pair<deque<RailLink>, double> >& sectionCandidates = sectionCandidatesCache[stationIdPair];

                for(IterType iter1 = railVertexIds1.begin(); iter1 != railVertexIds1.end(); iter1++) {

                    const RailVertexIdType& railVertexId1 = (*iter1);
                    const double distance1 = position1.DistanceTo(railVertices.at(railVertexId1).vertex);

                    for(IterType iter2 = railVertexIds2.begin(); iter2 != railVertexIds2.end(); iter2++) {

                        const RailVertexIdType& railVertexId2 = (*iter2);
                        const double distance2 = position2.DistanceTo(railVertices.at(railVertexId2).vertex);

                        deque<RailLink> routeLinks;

                        (*this).SearchRailRoadRoute(railVertexId1, railVertexId2, routeLinks);

                        if (!routeLinks.empty()) {
                            sectionCandidates.push_back(make_pair(routeLinks, distance1+distance2));
                        }
                    }
                }
            }

            sectionCandidatesPerSection[i] = sectionCandidatesCache[stationIdPair];
        }

        size_t minCandidatesSectionNumber = 0;
        size_t minNumberCandidates = INT_MAX;

        bool sectionsAreConnected = true;

        for(size_t i = 0; i < sectionCandidatesPerSection.size(); i++) {
            const vector<pair<deque<RailLink>, double> >& sectionCandidates =
                sectionCandidatesPerSection[i];

            const size_t numberCandidates = sectionCandidates.size();

            if (numberCandidates == 0) {
                sectionsAreConnected = false;
                noConnections.insert(
                    make_pair(
                        stationIdsPerName[i].front(),
                        stationIdsPerName[i+1].front()));
            }

            if (numberCandidates > 0 &&
                numberCandidates < minNumberCandidates) {
                minNumberCandidates = numberCandidates;
                minCandidatesSectionNumber = i;
            }
        }

        if (!sectionsAreConnected) {
            continue;
        }

        vector<size_t> currentCandidateNumbers(sectionCandidatesPerSection.size(), 0);

        const vector<pair<deque<RailLink>, double> >& mainSectionCandidates =
            sectionCandidatesPerSection[minCandidatesSectionNumber];

        for(size_t i = 0; i < mainSectionCandidates.size(); i++) {

            currentCandidateNumbers[minCandidatesSectionNumber] = i;

            for(int j = int(minCandidatesSectionNumber) - 1; j >= 0; j--) {
                const pair<deque<RailLink>, double>& lastSection =
                    sectionCandidatesPerSection.at(j+1).at(currentCandidateNumbers.at(j+1));
                const Vertex& frontVertex = railVertices.at(lastSection.first.front().railVertexId).vertex;

                const vector<pair<deque<RailLink>, double> >& sectionCandidates =
                    sectionCandidatesPerSection[j];

                double minDistanceBetweenSections = DBL_MAX;

                for(size_t k = 0; k < sectionCandidates.size(); k++) {
                    const pair<deque<RailLink>, double>& section = sectionCandidates[k];
                    const Vertex& lastVertex = railVertices.at(section.first.back().railVertexId).vertex;
                    const double distance = lastVertex.DistanceTo(frontVertex) + (*this).CalculateDistance(section);

                    const Vertex& lastVertex2 = railVertices.at(section.first.front().railVertexId).vertex;

                    if (distance < minDistanceBetweenSections) {
                        minDistanceBetweenSections = distance;
                        currentCandidateNumbers[j] = k;
                    }
                }
            }

            for(size_t j = minCandidatesSectionNumber + 1; j < sectionCandidatesPerSection.size(); j++) {
                const pair<deque<RailLink>, double>& lastSection =
                    sectionCandidatesPerSection.at(j-1).at(currentCandidateNumbers.at(j-1));
                const Vertex& lastVertex = railVertices.at(lastSection.first.back().railVertexId).vertex;

                const vector<pair<deque<RailLink>, double> >& sectionCandidates =
                    sectionCandidatesPerSection[j];

                double minDistanceBetweenSections = DBL_MAX;

                for(size_t k = 0; k < sectionCandidates.size(); k++) {
                    const pair<deque<RailLink>, double>& section = sectionCandidates[k];
                    const Vertex& frontVertex = railVertices.at(section.first.front().railVertexId).vertex;
                    const double distance = frontVertex.DistanceTo(lastVertex) + (*this).CalculateDistance(section);

                    if (distance < minDistanceBetweenSections) {
                        minDistanceBetweenSections = distance;
                        currentCandidateNumbers[j] = k;
                    }
                }
            }

            double totalRunnningDistance = 0;

            for(size_t j = 0; j < currentCandidateNumbers.size(); j++) {
                const size_t candidateNumber = currentCandidateNumbers[j];
                const pair<deque<RailLink>, double>& sectionCandidate =
                    sectionCandidatesPerSection[j].at(candidateNumber);
                const deque<RailLink>& railVertexIdsQueue = sectionCandidate.first;

                assert(railVertexIdsQueue.size() > 0);

                totalRunnningDistance += (*this).CalculateDistance(sectionCandidate) + sectionCandidate.second;
            }

            if (totalRunnningDistance < minRunningDistance) {
                minRunningDistance = totalRunnningDistance;
                bestCandidateNumbers = currentCandidateNumbers;
                bestSectionCandidatesPerSection = sectionCandidatesPerSection;
                bestStationIds = stationIds;
            }
        }
    }

    if (!bestCandidateNumbers.empty()) {
        noConnections.clear();

        for(size_t i = 0; i < bestCandidateNumbers.size(); i++) {
            const size_t bestCandidateNumber = bestCandidateNumbers[i];
            const vector<pair<deque<RailLink>, double> >& sectionCandidates =
                bestSectionCandidatesPerSection[i];

            const pair<deque<RailLink>, double>& bestSectionCandidate =
                sectionCandidates.at(bestCandidateNumber);

            bestSections.push_back(bestSectionCandidate);
        }
    }
}



void RailRoadLayer::AssignRailRoadLine(
    const string& lineName,
    const deque<string>& stationNames,
    map<pair<RailRoadStationIdType, RailRoadStationIdType>, vector<pair<deque<RailLink>, double> > >& sectionCandidatesCache,
    map<string, pair<deque<RailRoadStationIdType>, vector<pair<deque<RailLink>, double> > > >& stationIdsAndRailCache)
{
    if (stationNames.empty()) {
        return;
    }

    if (railRoadLineIds.find(lineName) == railRoadLineIds.end()) {
        railRoadLineIds[lineName] = (RailRoadLineIdType)(lineInfos.size());
        lineInfos.push_back(RailRoadLineInfo());
    }

    for(size_t i = 0; i < stationNames.size(); i++) {
        const string& stationName = stationNames[i];

        if (!subsystemPtr->ContainsPosition(stationName)) {
            cerr << "Warning: Skip line " << lineName << " which contains invalid station " << stationName << endl;
            return;
        }
    }


    RailRoadLineIdType lineId = railRoadLineIds[lineName];

    RailRoadLineInfo& lineInfo = lineInfos.at(lineId);
    vector<RouteInfo>& routeInfos = lineInfo.routeInfos;

    const RouteIdType routeId = RouteIdType(routeInfos.size());

    routeInfos.push_back(RouteInfo());
    RouteInfo& routeInfo = routeInfos.back();

    deque<RailRoadStationIdType> bestStationIds;
    vector<pair<deque<RailLink>, double> > bestSections;
    set<pair<RailRoadStationIdType, RailRoadStationIdType> > noConnections;

    ostringstream reverseStationNamesKeyStream;
    ostringstream stationNamesKeyStream;

    for(size_t i = 0; i < stationNames.size(); i++) {
        stationNamesKeyStream << stationNames[i];
        reverseStationNamesKeyStream << stationNames[stationNames.size() - i - 1];

        if (i < stationNames.size() - 1) {
            stationNamesKeyStream << ":";
            reverseStationNamesKeyStream << ":";
        }
    }

    const string stationNamesKey = stationNamesKeyStream.str();
    const string reverseStationNamesKey = reverseStationNamesKeyStream.str();

    bool isFromCache = false;

    if (stationIdsAndRailCache.find(stationNamesKey) != stationIdsAndRailCache.end()) {
        pair<deque<RailRoadStationIdType>, vector<pair<deque<RailLink>, double> > >& stationIdsAndRailLinks = stationIdsAndRailCache[stationNamesKey];

        bestStationIds = stationIdsAndRailLinks.first;
        bestSections = stationIdsAndRailLinks.second;
        isFromCache = true;

    } else {

        vector<vector<RailRoadStationIdType> > stationIdsPerName;
        vector<vector<vector<RailRoadStationIdType> > > stationIdsPerNames;

        size_t numberStationPatterns = 1;
        size_t dividedNumberStationPatterns = 1;
        size_t totalDividedNumberStationPatterns = 0;

        bool lastStationIsAmbiguous = false;

        for(size_t i = 0; i < stationNames.size(); i++) {
            const string& stationName = stationNames[i];

            stationIdsPerName.push_back(vector<RailRoadStationIdType>());

            vector<RailRoadStationIdType>& stationIds = stationIdsPerName.back();

            (*this).GetStationIds(stationName, stationIds);

            numberStationPatterns *= stationIds.size();
            dividedNumberStationPatterns *= stationIds.size();

            if (lastStationIsAmbiguous && stationIds.size() == 1) {
                stationIdsPerNames.push_back(stationIdsPerName);

                stationIdsPerName.clear();
                stationIdsPerName.push_back(stationIdsPerNames.back().back());

                totalDividedNumberStationPatterns += dividedNumberStationPatterns;

                lastStationIsAmbiguous = false;
                dividedNumberStationPatterns = 1;
            }

            if (stationIds.size() > 1) {
                lastStationIsAmbiguous = true;
            }
        }

        if (stationIdsPerName.size() > 1) {
            stationIdsPerNames.push_back(stationIdsPerName);

            totalDividedNumberStationPatterns += dividedNumberStationPatterns;
        }

        for(size_t i = 0; i < stationIdsPerNames.size(); i++) {
            const vector<vector<RailRoadStationIdType> >& stationIdsPerNameInAPart = stationIdsPerNames[i];

            deque<RailRoadStationIdType> bestStationIdsInAPart;
            vector<pair<deque<RailLink>, double> > bestSectionsInAPart;
            set<pair<RailRoadStationIdType, RailRoadStationIdType> > noConnectionsInAPart;

            (*this).GetShortestRailroadPath(
                stationIdsPerNameInAPart,
                bestStationIdsInAPart,
                bestSectionsInAPart,
                noConnectionsInAPart,
                sectionCandidatesCache);

            if (bestStationIds.empty()) {
                bestStationIds = bestStationIdsInAPart;
            } else {
                for(size_t j = 1; j < bestStationIdsInAPart.size(); j++) {
                    bestStationIds.push_back(bestStationIdsInAPart[j]);
                }
            }

            for(size_t j = 0; j < bestSectionsInAPart.size(); j++) {
                bestSections.push_back(bestSectionsInAPart[j]);
            }

            noConnections.insert(noConnectionsInAPart.begin(), noConnectionsInAPart.end());
        }
    }

    if (!noConnections.empty()) {
        cerr << "Warning: no railroad connection candidate for line [" << lineName << "]" << endl;

        typedef set<pair<RailRoadStationIdType, RailRoadStationIdType> >::const_iterator IterType;

        for(IterType iter = noConnections.begin();
            iter != noConnections.end(); iter++) {
            cerr << "    between "
                 << stations[(*iter).first].GetObjectName() << "(" << (*iter).first << ")"
                 << " and "
                 << stations[(*iter).second].GetObjectName() << "(" << (*iter).second << ")" << endl;
        }
        return;
    }

    pair<deque<RailRoadStationIdType>, vector<pair<deque<RailLink>, double> > >& reverseStationIdsAndRailLinks = stationIdsAndRailCache[reverseStationNamesKey];

    pair<deque<RailRoadStationIdType>, vector<pair<deque<RailLink>, double> > >& stationIdsAndRailLinks = stationIdsAndRailCache[stationNamesKey];

    routeInfo.stationIds = bestStationIds;

    assert(stationNames.size() == bestStationIds.size());

    for(size_t i = 0; i < bestStationIds.size(); i++) {
        lineInfo.railRoadStationIds[stationNames[i]] = bestStationIds[i];

        if (!isFromCache) {
            // cache
            stationIdsAndRailLinks.first.push_back(bestStationIds[i]);
            reverseStationIdsAndRailLinks.first.push_back(bestStationIds[bestStationIds.size() - i - 1]);
        }
    }

    assert(bestSections.size() == bestStationIds.size() - 1);

    for(size_t i = 0; i < bestSections.size(); i++) {
        const pair<deque<RailLink>, double>& bestSection = bestSections[i];

        if (!isFromCache) {
            // cache
            stationIdsAndRailLinks.second.push_back(bestSection);

            pair<deque<RailLink>, double> reverseSection = bestSections[bestSections.size() - i - 1];
            const deque<RailLink>& railLinks = bestSections[bestSections.size() - i - 1].first;

            reverseSection.first.clear();

            for(size_t j = 0; j < railLinks.size(); j++) {
                const RailLink& railLink = railLinks[j];

                reverseSection.first.push_front(railLink);
            }

            reverseStationIdsAndRailLinks.second.push_back(reverseSection);
        }

        deque<RailVertexIdType> railVertexIdsQueue;

        const deque<RailLink>& railLinks = bestSection.first;

        for(size_t j = 0; j < railLinks.size(); j++) {
            const RailLink& railLink = railLinks[j];

            railVertexIdsQueue.push_back(railLink.railVertexId);

            if (railLink.railId != INVALID_VARIANT_ID) {
                railRoads.at(railLink.railId).railRoadLineIds.insert(lineId);
                lineInfo.railIds.insert(railLink.railId);
            }
        }

        routeInfo.railVertexIdsPerSection.push_back(railVertexIdsQueue);

        assert(!railVertexIdsQueue.empty());
        const RailVertexIdType& frontVertexId = railVertexIdsQueue.front();
        const RailVertexIdType& backVertexId = railVertexIdsQueue.back();

        if (i == 0) {
            stations.at(bestStationIds[i]).AddRailRoadConnection(
                lineId, routeId, railVertices.at(frontVertexId).vertex);
        }

        stations.at(bestStationIds[i+1]).AddRailRoadConnection(
            lineId, routeId, railVertices.at(backVertexId).vertex);
    }
}



double RailRoadLayer::CalculateDistance(const pair<deque<RailLink>, double>& section) const
{
    const deque<RailLink>& railLinks = section.first;

    double pathDistance = 0;

    for(size_t i = 0; (i+1) < railLinks.size(); i++) {
        pathDistance +=
            railVertices.at(railLinks[i].railVertexId).vertex.
            DistanceTo(railVertices.at(railLinks[i+1].railVertexId).vertex);
    }

    return pathDistance;
}



void RailRoadLayer::ComplementLineInfo()
{
    for(RailRoadStationIdType stationId = 0; stationId < RailRoadStationIdType(stations.size()); stationId++) {
        RailRoadStation& station = stations[stationId];
        const Vertex& position = station.GetVertex();
        const vector<Vertex>& stationPolygon = station.GetPolygon();
        const Rectangle& rect = station.GetMinRectangle();
        const double nearRailRoadDistance = std::max(rect.GetWidth(), rect.GetHeight())*0.5;

        vector<RailRoadIdType> railIds;

        subsystemPtr->GetSpatialIntersectedGisObjectIds(rect, GIS_RAILROAD, railIds);

        typedef vector<RailRoadIdType>::const_iterator RailIdIter;

        for(RailIdIter railIdIter = railIds.begin(); railIdIter != railIds.end(); railIdIter++) {
            const RailRoadIdType& railId = (*railIdIter);

            RailRoad& rail = railRoads[railId];

            deque<pair<RailVertexIdType, RailRoadStationIdType> >& vertices = rail.vertices;

            typedef deque<pair<RailVertexIdType, RailRoadStationIdType> >::iterator VertexIter;

            VertexIter vertexIter = vertices.begin();

            Vertex nearestPoint;
            double nearestDistance = DBL_MAX;
            VertexIter nearestVertexIter = vertices.end();
            size_t nearestVertexNumber = 0;

            for(size_t i = 0; i < vertices.size() - 1; i++, vertexIter++) {
                const Vertex& vertex1 = railVertices.at(vertices[i].first).vertex;
                const Vertex& vertex2 = railVertices.at(vertices[i+1].first).vertex;

                const Vertex aNearestPoint =
                    CalculatePointToLineNearestPosition(position, vertex1, vertex2);

                const double distance = position.DistanceTo(aNearestPoint);

                if (distance <= nearestDistance) {
                    nearestDistance = distance;
                    nearestPoint = aNearestPoint;
                    nearestVertexIter = vertexIter;
                    nearestVertexNumber = i;
                }
            }

            if (nearestDistance <= nearRailRoadDistance) {
                assert(nearestVertexNumber < vertices.size());

                RailVertexIdType lastVertexId = vertices[nearestVertexNumber].first;
                RailVertexIdType nextVertexId = INVALID_VARIANT_ID;

                if (nearestVertexNumber < vertices.size() - 1) {
                    nextVertexId = vertices[nearestVertexNumber+1].first;
                }

                RailVertexIdType stationRailVertexId;

                const double nearStationDistance = 1; //1m

                if (railVertices.at(lastVertexId).vertex.DistanceTo(nearestPoint) <= nearStationDistance) {
                    stationRailVertexId = lastVertexId;

                    if (nextVertexId != INVALID_VARIANT_ID &&
                        railVertices.at(nextVertexId).vertex.DistanceTo(nearestPoint) < railVertices.at(lastVertexId).vertex.DistanceTo(nearestPoint)) {
                        stationRailVertexId = nextVertexId;
                    }

                } else if (nextVertexId != INVALID_VARIANT_ID &&
                           railVertices.at(nextVertexId).vertex.DistanceTo(nearestPoint) <= nearStationDistance) {
                    stationRailVertexId = nextVertexId;

                } else {

                    stationRailVertexId =
                        (*this).GetNewOrExistingRailVertexId(nearestPoint);

                    assert(nearestVertexIter != vertices.end());
                    nearestVertexIter++;

                    vertices.insert(nearestVertexIter, make_pair(stationRailVertexId, stationId));
                }

                station.railVertexIds.insert(stationRailVertexId);
            }
        }
    }

    for(RailRoadIdType railId = 0; railId < RailRoadIdType(railRoads.size()); railId++) {
        const RailRoad& rail = railRoads[railId];
        const deque<pair<RailVertexIdType, RailRoadStationIdType> >& vertices = rail.vertices;

        for(size_t i = 0; i < vertices.size() - 1; i++) {
            const pair<RailVertexIdType, RailRoadStationIdType>& vertex1 = vertices[i];
            const pair<RailVertexIdType, RailRoadStationIdType>& vertex2 = vertices[i+1];

            RailVertex& railVertex1 = railVertices.at(vertex1.first);
            RailVertex& railVertex2 = railVertices.at(vertex2.first);

            railVertex1.railLinks.push_back(RailLink(vertex2.first, railId));
            railVertex2.railLinks.push_back(RailLink(vertex1.first, railId));
        }
    }
}



class RailRoadLayer::RailNode {
public:
    RailNode()
        :
        railLink(),
        distance(),
        expectedDistance()
    {}

    RailNode(
        const RailLink& initRailLink,
        const double initDistance,
        const double initExpectedDistance)
        :
        railLink(initRailLink),
        distance(initDistance),
        expectedDistance(initExpectedDistance)
    {}

    bool operator<(const RailNode& right) const {
        return (expectedDistance > right.expectedDistance);
    }

    RailLink railLink;
    double distance;
    double expectedDistance;
};



void RailRoadLayer::SearchRailRoadRoute(
    const RailVertexIdType& startVertexId,
    const RailVertexIdType& destVertexId,
    deque<RailLink>& routeLinks)
{
    routeLinks.clear();

    for(size_t i = 0; i < railVertices.size(); i++) {
        railVertices[i].Initialize();
    }

    railVertices.at(startVertexId).distance = 0;

    double minDistanceToDest = DBL_MAX;
    Vertex destVertex = railVertices.at(destVertexId).vertex;
    Vertex startVertex = railVertices.at(startVertexId).vertex;

    std::priority_queue<RailNode> railNodes;

    railNodes.push(RailNode(RailLink(startVertexId), 0, startVertex.DistanceTo(destVertex)));

    while (!railNodes.empty()) {

        const RailNode railNode = railNodes.top();
        railNodes.pop();

        const RailVertex& railVertex = railVertices.at(railNode.railLink.railVertexId);

        if (railNode.distance > railVertex.distance) {
            continue;
        }

        const vector<RailLink>& railLinks = railVertex.railLinks;

        for(size_t i = 0; i < railLinks.size(); i++) {
            const RailLink& railLink = railLinks[i];
            const RailVertexIdType& linkedRailVertexId = railLink.railVertexId;
            RailVertex& linkedRailVertex = railVertices.at(linkedRailVertexId);

            const double distance =
                railVertex.distance + railVertex.vertex.DistanceTo(linkedRailVertex.vertex);

            if (linkedRailVertex.expectedMinDistanceToDest == DBL_MAX) {
                linkedRailVertex.expectedMinDistanceToDest =
                    linkedRailVertex.vertex.DistanceTo(destVertex);
            }

            if (distance + linkedRailVertex.expectedMinDistanceToDest < minDistanceToDest &&
                linkedRailVertex.IsFastRoute(distance)) {

                linkedRailVertex.SetBestRoute(RailLink(railNode.railLink.railVertexId, railLink.railId), distance);

                if (linkedRailVertexId == destVertexId) {

                    minDistanceToDest = distance;

                } else {

                    railNodes.push(
                        RailNode(
                            railLink,
                            distance,
                            distance + linkedRailVertex.expectedMinDistanceToDest));
                }
            }
        }
    }

    if (railVertices.at(destVertexId).FoundRoute()) {

        RailVertexIdType railVertexId = destVertexId;

        routeLinks.push_front(RailLink(railVertexId));

        while (railVertexId != startVertexId) {
            const RailVertex& railVertex = railVertices.at(railVertexId);

            railVertexId = railVertex.trackRailLink.railVertexId;

            routeLinks.push_front(railVertex.trackRailLink);
        }
    }
}



void RailRoadLayer::UpdateRailRoadLineAvailability(const RailRoadLineIdType& lineId)
{
    RailRoadLineInfo& lineInfo = lineInfos.at(lineId);

    const set<RailRoadIdType>& railIds = lineInfo.railIds;

    typedef set<RailRoadIdType>::const_iterator IterType;

    bool isAvailable = true;

    for(IterType iter = railIds.begin(); (isAvailable && iter != railIds.end()); iter++) {
        const RailRoadIdType& railId = (*iter);

        if (!railRoads[railId].IsEnabled()) {
            isAvailable = false;
        }
    }

    lineInfo.isAvailable = isAvailable;
}


RailRoadLineIdType RailRoadLayer::GetRailRoadLineId(const string& lineName) const
{
    typedef map<string, RailRoadLineIdType>::const_iterator IterType;

    IterType iter = railRoadLineIds.find(lineName);

    if (iter == railRoadLineIds.end()) {
        cerr << "Error: Couldn't find railroad line: " << lineName << endl;
        exit(1);
    }

    return (*iter).second;
}



bool RailRoadLayer::LineHasARoute(const string& lineName) const
{
    typedef map<string, RailRoadLineIdType>::const_iterator LineIter;

    LineIter lineIter = railRoadLineIds.find(lineName);

    if (lineIter == railRoadLineIds.end()) {
        return false;
    }

    return !lineInfos[(*lineIter).second].railRoadStationIds.empty();
}



RailRoadStationIdType RailRoadLayer::GetStationId(
    const string& lineName,
    const string& stationName) const
{
    typedef map<string, RailRoadLineIdType>::const_iterator LineIter;

    LineIter lineIter = railRoadLineIds.find(lineName);

    if (lineIter == railRoadLineIds.end()) {
        cerr << "Invalid line " << lineName << endl;
        exit(1);
    }

    const RailRoadLineIdType lineId = (*lineIter).second;
    const RailRoadLineInfo& lineInfo = lineInfos[lineId];

    typedef map<string, RailRoadStationIdType>::const_iterator StationIter;

    const map<string, RailRoadStationIdType>& railRoadStationIds = lineInfo.railRoadStationIds;

    StationIter stationIter = railRoadStationIds.find(stationName);

    if (stationIter == railRoadStationIds.end()) {
        cerr << "Line " << lineName << " does not contain " << stationName << endl;
        exit(1);
    }

    return (*stationIter).second;
}



void RailRoadLayer::GetStationIds(const string& stationName, vector<RailRoadStationIdType>& stationIds) const
{
    vector<GisPositionIdType> positionIds;

    subsystemPtr->GetPositions(stationName, positionIds, GIS_RAILROAD_STATION);

    stationIds.clear();

    for(size_t i = 0; i < positionIds.size(); i++) {
        stationIds.push_back(positionIds[i].id);
    }
}



RouteIdType RailRoadLayer::GetRouteId(
    const RailRoadLineIdType& lineId,
    const deque<RailRoadStationIdType>& stationIds) const
{
    const vector<RouteInfo>& routeInfos = lineInfos.at(lineId).routeInfos;
    bool found = false;
    RouteIdType routeId;

    for(routeId = 0; routeId < RouteIdType(routeInfos.size()); routeId++) {
        found = (routeInfos[routeId].stationIds == stationIds);

        if (found) {
            break;
        }
    }
    if (!found) {
        cerr << "Error: No railroad route for line: " << (*this).GetLineName(lineId) << endl;
        exit(1);
    }

    return routeId;
}



bool RailRoadLayer::ContainsRouteId(
    const RailRoadLineIdType& lineId,
    const deque<RailRoadStationIdType>& stationIds) const
{
    const vector<RouteInfo>& routeInfos = lineInfos.at(lineId).routeInfos;
    RouteIdType routeId;

    for(routeId = 0; routeId < RouteIdType(routeInfos.size()); routeId++) {

        if (routeInfos[routeId].stationIds == stationIds) {

            vector<vector<Vertex> > lineVertices;

            (*this).GetLineVertices(lineId, routeId, lineVertices);

            if (!lineVertices.empty()) {
                return true;
            }
        }
    }

    return false;
}



const deque<RailRoadStationIdType>& RailRoadLayer::GetRouteStationIds(
    const RailRoadLineIdType& lineId,
    const RouteIdType& routeId) const
{
    return lineInfos.at(lineId).routeInfos.at(routeId).stationIds;
}



bool RailRoadLayer::IsRailRoadLineAvailable(const RailRoadLineIdType& lineId) const
{
    return lineInfos.at(lineId).isAvailable;
}



void RailRoadLayer::GetLineVertices(
    const RailRoadLineIdType& lineId,
    const RouteIdType& routeId,
    vector<vector<Vertex> >& lineVertices) const
{
    lineVertices.clear();

    const vector<RouteInfo>& routeInfos = lineInfos.at(lineId).routeInfos;
    const RouteInfo& routeInfo = routeInfos.at(routeId);

    const deque<deque<RailVertexIdType> >& railVertexIdsPerSection = routeInfo.railVertexIdsPerSection;

    lineVertices.resize(railVertexIdsPerSection.size());

    for(size_t i = 0; i < railVertexIdsPerSection.size(); i++) {
        const deque<RailVertexIdType>& railVertexIdsQueue = railVertexIdsPerSection[i];
        vector<Vertex>& vertices = lineVertices[i];

        for(size_t j = 0; j < railVertexIdsQueue.size(); j++) {
            vertices.push_back(railVertices.at(railVertexIdsQueue[j]).vertex);
        }

        assert(!vertices.empty());
    }

    if (lineVertices.size() > 0) {

        for(size_t i = 0; i < lineVertices.size() - 1; i++) {
            vector<Vertex>& vertices1 = lineVertices[i];
            vector<Vertex>& vertices2 = lineVertices[i+1];

            if (vertices1.back() != vertices2.front()) {
                vertices1.push_back(vertices2.front());
            }
        }
    }
}



void RailRoadLayer::GetNearestStationId(
    const Vertex& position,
    bool& found,
    RailRoadStationIdType& nearStationId) const
{
    if (stations.empty()) {
        found = false;
        return;
    }

    const double lengthUnit = 500;

    found = false;

    for(double searchLength = lengthUnit; (!found); searchLength += lengthUnit) {

        vector<RailRoadStationIdType> stationIds;

        subsystemPtr->GetSpatialIntersectedGisObjectIds(
            Rectangle(position, searchLength),
            GIS_RAILROAD_STATION,
            stationIds);

        typedef vector<RailRoadStationIdType>::const_iterator IterType;

        double minDistanceToStop = DBL_MAX;

        for(IterType iter = stationIds.begin(); iter != stationIds.end(); iter++) {

            const RailRoadStationIdType& stationId = *iter;
            const double distance = position.DistanceTo(stations.at(stationId).GetVertex());

            if (distance < minDistanceToStop) {
                minDistanceToStop = distance;
                nearStationId = stationId;
                found = true;
            }
        }
    }
}



string RailRoadLayer::GetLineName(const RailRoadLineIdType& lineId) const
{
    typedef map<string, RailRoadLineIdType>::const_iterator IterType;

    // slow comparison
    for(IterType iter = railRoadLineIds.begin();
        iter != railRoadLineIds.end(); iter++) {

        if ((*iter).second == lineId) {
            return (*iter).first;
        }
    }

    return string();
}



void RailRoadLayer::MakeEntrance(
    const RailRoadStationIdType& stationId,
    const Vertex& position)
{
    RailRoadStation& station = stations[stationId];

    const VertexIdType entranceVertexId =
        station.commonImplPtr->subsystemPtr->GetVertexId(position);

    station.commonImplPtr->vertexIds.push_back(entranceVertexId);

    subsystemPtr->ConnectGisObject(entranceVertexId, GIS_RAILROAD_STATION, stationId);
}



void AreaLayer::ImportArea(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const AreaIdType areaId = static_cast<AreaIdType>(areas.size());
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        areas.push_back(Area(subsystemPtr, objectId, areaId));
        Area& area = areas.back();
        area.LoadParameters(theParameterDatabaseReader);

        if (nameFinder.IsAvailable()) {
            area.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }

        assert(shpObjPtr->nParts == 1);
        assert(shpObjPtr->nVertices > 0);

        if (shpObjPtr->nVertices > 0) {
            area.commonImplPtr->elevationFromGroundMeters = shpObjPtr->padfZ[0];
        }

        for(int i = 0; i < shpObjPtr->nVertices; i++) {
            const double x = shpObjPtr->padfX[i];
            const double y = shpObjPtr->padfY[i];
            double z = shpObjPtr->padfZ[i];

            if (isBaseGroundLevel) {
                z += subsystemPtr->GetGroundElevationMetersAt(x, y);
            }

            area.polygon.push_back(Vertex(x, y, z));
        }

        subsystemPtr->RegisterGisObject(area, areaId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}



void ParkLayer::ImportPark(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder capacityFinder(hDBF, GIS_DBF_CAPACITY_STRING);

    // register parks
    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const ParkIdType parkId = static_cast<ParkIdType>(parks.size());
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        parks.push_back(Park(subsystemPtr, objectId, parkId));
        Park& park = parks.back();

        park.LoadParameters(theParameterDatabaseReader);

        if (nameFinder.IsAvailable()) {
            park.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }
        if (capacityFinder.IsAvailable()) {
            park.humanCapacity = capacityFinder.GetInt(entryId);
        }

        assert(shpObjPtr->nParts == 1);
        assert(shpObjPtr->nVertices > 0);

        if (shpObjPtr->nVertices > 0) {
            park.commonImplPtr->elevationFromGroundMeters = shpObjPtr->padfZ[0];
        }

        for(int i = 0; i < shpObjPtr->nVertices; i++) {
            const double x = shpObjPtr->padfX[i];
            const double y = shpObjPtr->padfY[i];
            double z = shpObjPtr->padfZ[i];

            if (isBaseGroundLevel) {
                z += subsystemPtr->GetGroundElevationMetersAt(x, y);
            }

            park.polygon.push_back(Vertex(x, y, z));
        }

        subsystemPtr->RegisterGisObject(park, parkId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}



BuildingLayer::BuildingLayer(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    GisSubsystem* initSubsystemPtr)
    :
    subsystemPtr(initSubsystemPtr),
    enabledLineOfSightCalculation(true),
    totalHeightMeters(0),
    averageHeightMeters(0),
    minHeightMeters(DBL_MAX),
    maxHeightMeters(0),
    heightMetersVariance(0),
    theQuadTreePtr(new LosQuadTree(Rectangle(), 0/*depth*/))
{
    (*this).LoadMovingObjectShapeDefinition(theParameterDatabaseReader);
}



void BuildingLayer::GetBuildingIdAt(
    const Vertex& position,
    bool& found,
    BuildingIdType& foundBuildingId) const
{
    found = false;

    vector<BuildingIdType> buildingIds;

    subsystemPtr->GetSpatialIntersectedGisObjectIds(position, GIS_BUILDING, buildingIds);

    typedef vector<BuildingIdType>::const_iterator IterType;

    for(IterType iter = buildingIds.begin(); iter != buildingIds.end(); iter++) {

        const BuildingIdType& buildingId = (*iter);
        const Building& building = buildings[buildingId];

        if (PolygonContainsPoint(building.polygon, position)) {
            found = true;
            foundBuildingId = buildingId;
            return;
        }
    }
}



void BuildingLayer::ImportBuilding(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder heightFinder(hDBF, GIS_DBF_HEIGHT_STRING);
    const AttributeFinder outerWallMaterialFinder(hDBF, GIS_DBF_MATERIAL_STRING);
    const AttributeFinder roofMaterialFinder(hDBF, GIS_DBF_ROOF_MATERIAL_STRING);
    const AttributeFinder floorMaterialFinder(hDBF, GIS_DBF_FLOOR_MATERIAL_STRING);

    const AttributeFinder meshKindFinder(hDBF, GIS_DBF_MESH_KIND_STRING);
    const AttributeFinder numberOfRoofFacesFinder(hDBF, GIS_DBF_NUMBER_OF_ROOF_FACES_STRING);
    const AttributeFinder numberOfWallFacesFinder(hDBF, GIS_DBF_NUMBER_OF_WALL_FACES_STRING);
    const AttributeFinder numberOfFloorFacesFinder(hDBF, GIS_DBF_NUMBER_OF_FLOOR_FACES_STRING);

    const AttributeFinder capacityFinder(hDBF, GIS_DBF_CAPACITY_STRING);
    const AttributeFinder vehicleCapacityFinder(hDBF, GIS_DBF_VEHICLE_CAPACITY_STRING);

    // register buildings
    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        const BuildingIdType buildingId = static_cast<BuildingIdType>(buildings.size());
        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        buildings.push_back(Building(subsystemPtr, objectId, buildingId));
        Building& building = buildings.back();

        building.LoadParameters(theParameterDatabaseReader);

        if (nameFinder.IsAvailable()) {
            building.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }
        if (heightFinder.IsAvailable()) {
            building.heightMeters = heightFinder.GetDouble(entryId);
        }
        if (outerWallMaterialFinder.IsAvailable()) {
            building.outerWallMaterialId =
                subsystemPtr->GetMaterialId(outerWallMaterialFinder.GetString(entryId));
        }
        if (roofMaterialFinder.IsAvailable()) {
            building.roofWallMaterialId =
                subsystemPtr->GetMaterialId(roofMaterialFinder.GetString(entryId));
        }
        if (floorMaterialFinder.IsAvailable()) {
            building.floorWallMaterialId =
                subsystemPtr->GetMaterialId(floorMaterialFinder.GetString(entryId));
        }
        if (capacityFinder.IsAvailable()) {
            building.humanCapacity = capacityFinder.GetInt(entryId);
        }

        if (vehicleCapacityFinder.IsAvailable()) {
            building.vehicleCapacity = vehicleCapacityFinder.GetInt(entryId);
        }

        assert(shpObjPtr->nParts == 1);
        assert(shpObjPtr->nVertices > 0);

        const int meshKind2D = 0;
        const int meshKind3D = 1;
        int meshKind = meshKind2D;
        if (meshKindFinder.IsAvailable()) {
            meshKind = meshKindFinder.GetInt(entryId);
        }

        double groundElevationMeters = 0.;

        // Use the lower ground level between vertices
        if (shpObjPtr->nVertices > 0) {
            groundElevationMeters = DBL_MAX;

            for(int i = 0; i < shpObjPtr->nVertices; i++) {
                const double x = shpObjPtr->padfX[i];
                const double y = shpObjPtr->padfY[i];

                groundElevationMeters = std::min(groundElevationMeters, subsystemPtr->GetGroundElevationMetersAt(x, y));
            }
        }

        if (meshKind == meshKind3D) {
            double minX = DBL_MAX;
            double minY = DBL_MAX;
            double maxX = -DBL_MAX;
            double maxY = -DBL_MAX;
            double maxZ = -DBL_MAX;

            for(int i = 0; i < shpObjPtr->nVertices; i += 3) {
                const double x1 = shpObjPtr->padfX[i];
                const double y1 = shpObjPtr->padfY[i];
                double z1 = shpObjPtr->padfZ[i];

                const double x2 = shpObjPtr->padfX[i+1];
                const double y2 = shpObjPtr->padfY[i+1];
                double z2 = shpObjPtr->padfZ[i+1];

                const double x3 = shpObjPtr->padfX[i+2];
                const double y3 = shpObjPtr->padfY[i+2];
                double z3 = shpObjPtr->padfZ[i+2];

                if (isBaseGroundLevel) {
                    z1 += groundElevationMeters;
                    z2 += groundElevationMeters;
                    z3 += groundElevationMeters;
                }

                const Vertex p1 = Vertex(x1, y1, z1);
                const Vertex p2 = Vertex(x2, y2, z2);
                const Vertex p3 = Vertex(x3, y3, z3);
                const Triangle face(p1, p2, p3);
                building.faces.push_back(face);

                minX = std::min(minX, p1.x);
                minX = std::min(minX, p2.x);
                minX = std::min(minX, p3.x);

                minY = std::min(minY, p1.y);
                minY = std::min(minY, p2.y);
                minY = std::min(minY, p3.y);

                maxX = std::max(maxX, p1.x);
                maxX = std::max(maxX, p2.x);
                maxX = std::max(maxX, p3.x);

                maxY = std::max(maxY, p1.y);
                maxY = std::max(maxY, p2.y);
                maxY = std::max(maxY, p3.y);

                maxZ = std::max(maxZ, p1.z);
                maxZ = std::max(maxZ, p2.z);
                maxZ = std::max(maxZ, p3.z);
            }

            building.polygon.reserve(5);
            building.polygon.push_back(Vertex(minX, minY, maxZ));
            building.polygon.push_back(Vertex(minX, maxY, maxZ));
            building.polygon.push_back(Vertex(maxX, maxY, maxZ));
            building.polygon.push_back(Vertex(maxX, minY, maxZ));
            building.polygon.push_back(Vertex(minX, minY, maxZ));

            if (numberOfRoofFacesFinder.IsAvailable()) {
                building.numberOfRoofFaces = numberOfRoofFacesFinder.GetInt(entryId);
            }
            if (numberOfWallFacesFinder.IsAvailable()) {
                building.numberOfWallFaces = numberOfWallFacesFinder.GetInt(entryId);
            }
            if (numberOfFloorFacesFinder.IsAvailable()) {
                building.numberOfFloorFaces = numberOfFloorFacesFinder.GetInt(entryId);
            }
        } else {

            if (shpObjPtr->nVertices > 0) {
                building.commonImplPtr->elevationFromGroundMeters = shpObjPtr->padfZ[0];
            }

            for(int i = 0; i < shpObjPtr->nVertices; i++) {
                const double x = shpObjPtr->padfX[i];
                const double y = shpObjPtr->padfY[i];
                double z = shpObjPtr->padfZ[i];

                if (isBaseGroundLevel) {
                    z += groundElevationMeters;
                }

                building.polygon.push_back(Vertex(x, y, z));
            }
        }

        subsystemPtr->RegisterGisObject(building, buildingId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}



Rectangle BuildingLayer::GetMinRectangle() const
{
    if (buildings.empty()) {
        return Rectangle();
    }

    Rectangle rect = buildings.front().GetMinRectangle();

    for(size_t i = 1; i < buildings.size(); i++) {
        rect += buildings[i].GetMinRectangle();
    }

    return rect;
}



void BuildingLayer::RemakeLosTopology()
{
    if (!enabledLineOfSightCalculation) {
        return;
    }

    const Rectangle areaRect = (*this).GetMinRectangle();
    const double minAreaLength = 1000.;
    const double width = std::max(minAreaLength, areaRect.GetWidth());
    const double height = std::max(minAreaLength, areaRect.GetHeight());
    const Rectangle minRect = Rectangle(areaRect.GetCenter(), width, height);

    theQuadTreePtr.reset(new LosQuadTree(minRect, 0/*depth*/));
    totalHeightMeters = 0;
    minHeightMeters = DBL_MAX;
    maxHeightMeters = 0;

    for(BuildingIdType buildingId = 0;
        buildingId < BuildingIdType(buildings.size()); buildingId++) {
        const Building& building = buildings[buildingId];
        const double heightMeters = building.heightMeters;

        LosPolygon aPolygon;
        aPolygon.variantId = buildingId;
        aPolygon.theNodeId = building.GetObjectId();
        aPolygon.shieldingLossDb = building.GetOuterWallMaterial().transmissionLossDb;
        aPolygon.anObstructionType = LosPolygon::OuterWall;

        const vector<Vertex>& polygon = building.polygon;
        for(size_t i = 0; i < polygon.size() - 1; i++) {
            const Vertex& v1 = polygon[i];
            const Vertex& v2 = polygon[i+1];

            aPolygon.SetTriangle(
                Vertex(v1.x, v1.y, v1.z),
                Vertex(v1.x, v1.y, v1.z + heightMeters),
                Vertex(v2.x, v2.y, v2.z));

            theQuadTreePtr->PushLosPolygon(aPolygon);

            aPolygon.SetTriangle(
                Vertex(v2.x, v2.y, v2.z),
                Vertex(v2.x, v2.y, v2.z + heightMeters),
                Vertex(v1.x, v1.y, v1.z + heightMeters));

            theQuadTreePtr->PushLosPolygon(aPolygon);
        }
        double roofTopMeters = heightMeters;

        vector<Triangle> roofTriangles;
        PolygonToTriangles(polygon, roofTriangles);

        for(size_t i = 0; i < roofTriangles.size(); i++) {

            const Triangle& aRoofTraiangle = roofTriangles[i];
            const Vertex& v1 = aRoofTraiangle.GetP1();
            const Vertex& v2 = aRoofTraiangle.GetP2();
            const Vertex& v3 = aRoofTraiangle.GetP3();

            aPolygon.SetTriangle(
                Vertex(v1.x, v1.y, v1.z),
                Vertex(v2.x, v2.y, v2.z),
                Vertex(v3.x, v3.y, v3.z));

            aPolygon.shieldingLossDb = building.GetRoofMaterial().transmissionLossDb;
            aPolygon.anObstructionType = LosPolygon::Roof;

            theQuadTreePtr->PushLosPolygon(aPolygon);

            aPolygon.SetTriangle(
                Vertex(v1.x, v1.y, v1.z + heightMeters),
                Vertex(v2.x, v2.y, v2.z + heightMeters),
                Vertex(v3.x, v3.y, v3.z + heightMeters));

            aPolygon.shieldingLossDb = building.GetFloorMaterial().transmissionLossDb;
            aPolygon.anObstructionType = LosPolygon::Floor;

            theQuadTreePtr->PushLosPolygon(aPolygon);

            roofTopMeters = std::max(roofTopMeters, v1.z + heightMeters);
            roofTopMeters = std::max(roofTopMeters, v2.z + heightMeters);
            roofTopMeters = std::max(roofTopMeters, v3.z + heightMeters);
        }

        totalHeightMeters += roofTopMeters;
        minHeightMeters = std::min(minHeightMeters, heightMeters);
        maxHeightMeters = std::max(maxHeightMeters, heightMeters);
    }

    for(WallIdType wallId = 0;  wallId < WallIdType(walls.size()); wallId++) {
        const Wall& wall = walls[wallId];
        const double heightMeters = wall.heightMeters;

        LosPolygon aPolygon;
        aPolygon.variantId =
            static_cast<VariantIdType>(buildings.size() + wallId);
        aPolygon.theNodeId = wall.GetObjectId();
        aPolygon.shieldingLossDb = wall.GetMaterial().transmissionLossDb;
        aPolygon.anObstructionType = LosPolygon::InnerWall;

        const vector<Vertex>& vertices = wall.vertices;
        for(size_t i = 0; i < vertices.size() - 1; i++) {
            const Vertex& v1 = vertices[i];
            const Vertex& v2 = vertices[i+1];

            aPolygon.SetTriangle(
                Vertex(v1.x, v1.y, v1.z),
                Vertex(v1.x, v1.y, v1.z + heightMeters),
                Vertex(v2.x, v2.y, v2.z));

            theQuadTreePtr->PushLosPolygon(aPolygon);

            aPolygon.SetTriangle(
                Vertex(v2.x, v2.y, v2.z),
                Vertex(v2.x, v2.y, v2.z + heightMeters),
                Vertex(v1.x, v1.y, v1.z + heightMeters));

            theQuadTreePtr->PushLosPolygon(aPolygon);
        }

        totalHeightMeters += heightMeters;
        minHeightMeters = std::min(minHeightMeters, heightMeters);
        maxHeightMeters = std::max(maxHeightMeters, heightMeters);
    }

    // XXX
    // add wall polygons

    averageHeightMeters = totalHeightMeters / buildings.size();

    double squaredDifferenceSigma = 0;

    for(size_t i = 0; i < buildings.size(); i++) {
        const double heightDifference = buildings[i].heightMeters - averageHeightMeters;
        squaredDifferenceSigma += (heightDifference*heightDifference);
    }
    for(size_t i = 0; i < walls.size(); i++) {
        const double heightDifference = walls[i].heightMeters - averageHeightMeters;
        squaredDifferenceSigma += (heightDifference*heightDifference);
    }

    heightMetersVariance = squaredDifferenceSigma / (buildings.size() + walls.size());
}



void BuildingLayer::ImportWall(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder materialFinder(hDBF, GIS_DBF_MATERIAL_STRING);
    const AttributeFinder widthFinder(hDBF, GIS_DBF_WIDTH_STRING);
    const AttributeFinder heightFinder(hDBF, GIS_DBF_HEIGHT_STRING);

    // register walls
    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const WallIdType wallId = static_cast<WallIdType>(walls.size());
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        walls.push_back(Wall(subsystemPtr, objectId, wallId));
        Wall& wall = walls.back();

        if (nameFinder.IsAvailable()) {
            wall.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }
        if (materialFinder.IsAvailable()) {
            wall.materialId = subsystemPtr->GetMaterialId(materialFinder.GetString(entryId));
        }
        if (widthFinder.IsAvailable()) {
            wall.widthMeters = widthFinder.GetDouble(entryId);
        }
        if (heightFinder.IsAvailable()) {
            wall.heightMeters = heightFinder.GetDouble(entryId);
        }

        assert(shpObjPtr->nParts == 1);
        assert(shpObjPtr->nVertices > 0);

        double groundElevationMeters = 0.;

        // Use the lower ground level between vertices
        if (shpObjPtr->nVertices > 0) {
            groundElevationMeters = DBL_MAX;

            for(int i = 0; i < shpObjPtr->nVertices; i++) {
                const double x = shpObjPtr->padfX[i];
                const double y = shpObjPtr->padfY[i];

                groundElevationMeters = std::min(groundElevationMeters, subsystemPtr->GetGroundElevationMetersAt(x, y));
            }
        }

        if (shpObjPtr->nVertices > 0) {
            wall.commonImplPtr->elevationFromGroundMeters = shpObjPtr->padfZ[0];
        }

        for(int i = 0; i < shpObjPtr->nVertices; i++) {
            const double x = shpObjPtr->padfX[i];
            const double y = shpObjPtr->padfY[i];
            double z = shpObjPtr->padfZ[i];

            if (isBaseGroundLevel) {
                z += groundElevationMeters;
            }

            wall.vertices.push_back(Vertex(x, y, z));
        }

        subsystemPtr->RegisterGisObject(wall, wallId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}



bool BuildingLayer::PositionsAreLineOfSight(
    const Vertex& vector1,
    const Vertex& vector2,
    const set<NodeId>& ignoredNodeIds) const
{
    if (!enabledLineOfSightCalculation) {
        return false;
    }

    return (!theQuadTreePtr->HasCollision(LosRay(vector1, vector2), ignoredNodeIds));
}



void BuildingLayer::CalculateWallCollisionPoints(
    const Vertex& vector1,
    const Vertex& vector2,
    vector<pair<Vertex, VariantIdType> >& collisionPoints,
    const set<NodeId>& ignoredNodeIds) const
{
    double notUsedShieldingLossDb;

    (*this).CalculateWallCollisionPoints(
        vector1,
        vector2,
        collisionPoints,
        notUsedShieldingLossDb,
        ignoredNodeIds);
}



void BuildingLayer::CalculateWallCollisionPoints(
    const Vertex& vector1,
    const Vertex& vector2,
    vector<pair<Vertex, VariantIdType> >& collisionPoints,
    double& totalShieldingLossDb,
    const set<NodeId>& ignoredNodeIds) const
{
    collisionPoints.clear();
    totalShieldingLossDb = 0;

    //if (!(IsInRectangle(minRectangle, position1) &&
    //      IsInRectangle(minRectangle, position2))) {
    //    return;
    //}

    const LosRay ray(vector1, vector2);

    map<double, WallCollisionInfoType> collisions;

    theQuadTreePtr->CheckCollision(ray, ignoredNodeIds, false/*checkJustACollsion*/, false/*isJustHorizontalCheck*/, collisions);

    if (!collisions.empty()) {

        typedef map<double, WallCollisionInfoType>::iterator IterType;

        for(IterType iter = collisions.begin();
            iter != collisions.end(); iter++) {

            const double t = (*iter).first;
            const WallCollisionInfoType& collision = (*iter).second;

            collisionPoints.push_back(make_pair(ray.Position(t), (*iter).second.variantId));

            totalShieldingLossDb += collision.shieldingLossDb;
        }
    }
}



void BuildingLayer::CalculateNumberOfFloorsAndWallsTraversed(
    const Vertex& vector1,
    const Vertex& vector2,
    const set<NodeId>& ignoredNodeIds,
    unsigned int& numberOfFloors,
    unsigned int& numberOfWalls) const
{
    numberOfFloors = 0;
    numberOfWalls = 0;

    const LosRay ray(vector1, vector2);

    map<double, WallCollisionInfoType> collisions;

    theQuadTreePtr->CheckCollision(ray, ignoredNodeIds, false/*checkJustACollsion*/, false/*isJustHorizontalCheck*/, collisions);

    if (!collisions.empty()) {

        typedef map<double, WallCollisionInfoType>::iterator IterType;

        for(IterType iter = collisions.begin(); iter != collisions.end(); iter++) {
            const WallCollisionInfoType& collision = (*iter).second;

            switch (collision.anObstructionType) {
            case LosPolygon::Floor:
                numberOfFloors++;
                break;

            case LosPolygon::InnerWall:
            case LosPolygon::OuterWall:
                numberOfWalls++;
                break;

            default:
                assert(false); abort(); break;
            }//switch//
        }
    }

}//CalculateNumberOfFloorsAndWallsTraversed//



double BuildingLayer::CalculateTotalWallAndFloorLossDb(
    const Vertex& position1,
    const Vertex& position2) const
{
    vector<pair<Vertex, VariantIdType> > collisionPoints;
    double totalShieldingLossDb;

    (*this).CalculateWallCollisionPoints(
        position1, position2, collisionPoints, totalShieldingLossDb);

    return totalShieldingLossDb;
}



double BuildingLayer::GetCollisionPointHeight(
    pair<Vertex, VariantIdType>& collisionPoint) const
{
    const VariantIdType& variantId = collisionPoint.second;

    // check if moving object
    if (variantId == INVALID_GIS_OBJECT_ID) {
        return 0.;
    }

    if (collisionPoint.second < VariantIdType(buildings.size())) {
        const Building& building = buildings[collisionPoint.second];

        return building.GetHeightMeters();

    } else {
        const WallIdType wallId = static_cast<WallIdType>(collisionPoint.second - buildings.size());
        const Wall& wall = walls.at(wallId);

        return wall.GetHeightMeters();
    }
}



size_t BuildingLayer::CalculateNumberOfWallRoofFloorInteractions(
    const Vertex& position1,
    const Vertex& position2) const
{
    vector<pair<Vertex, VariantIdType> > collisionPoints;
    double totalShieldingLossDb;

    (*this).CalculateWallCollisionPoints(position1, position2, collisionPoints, totalShieldingLossDb);

    return collisionPoints.size();
}



void BuildingLayer::MakeEntrance(
    const BuildingIdType& buildingId,
    const Vertex& position)
{
    Building& building = buildings[buildingId];

    const VertexIdType entranceVertexId =
        building.commonImplPtr->subsystemPtr->GetVertexId(position);

    building.commonImplPtr->vertexIds.push_back(entranceVertexId);

    subsystemPtr->ConnectGisObject(entranceVertexId, GIS_BUILDING, buildingId);
}



void BuildingLayer::LoadMovingObjectShapeDefinition(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    if (!theParameterDatabaseReader.ParameterExists("moving-object-shape-file")) {
        return;
    }//if//

    const string filename = theParameterDatabaseReader.ReadString("moving-object-shape-file");

    ifstream inStream(filename.c_str());
    if (!inStream.good()) {
        cerr << "Error: Couldn't open Moving Object Shape File: " << filename << endl;
        exit(1);
    }//if//

    while (!inStream.eof()) {

        string aLine;
        getline(inStream, aLine);
        DeleteTrailingSpaces(aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }//if//

        istringstream lineStream(aLine);

        string shapeType;
        lineStream >> shapeType;
        ConvertStringToLowerCase(shapeType);

        if (movingShapes.find(shapeType) != movingShapes.end()) {
            cerr << "Error: Duplicated shape: " << shapeType << endl;
            cerr << aLine << endl;
            exit(1);
        }//if//

        MovingShape& movingShape = movingShapes[shapeType];

        lineStream >> movingShape.length;
        lineStream >> movingShape.width;
        lineStream >> movingShape.height;
        lineStream >> movingShape.materialName;
    }
}



void BuildingLayer::AddMovingObject(
    const MaterialSet& materials,
    const NodeId theNodeId,
    const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
    const string& shapeType)
{
    if (movingShapes.find(shapeType) == movingShapes.end()) {
        cerr << "Invalid object-shape-type: " << shapeType << endl;
        exit(1);
    }

    const MovingShape& movingShape = movingShapes[shapeType];
    const Material& material = materials.GetMaterial(movingShape.materialName);

    shared_ptr<LosMovingObject> movingObjectPtr(
        new LosMovingObject(
            theNodeId,
            mobilityModelPtr,
            movingShape.length,
            movingShape.width,
            movingShape.height,
            material.transmissionLossDb * 0.5));

    losMovingObjects.push_back(movingObjectPtr);

}//AddMovingObject//



void BuildingLayer::SyncMovingObjectTime(const SimTime& currentTime)
{
    typedef vector<shared_ptr<LosMovingObject> >::iterator IterType;
    const GroundLayer& groundLayer = *subsystemPtr->GetGroundLayerPtr();

    for(IterType iter = losMovingObjects.begin();
        (iter != losMovingObjects.end()); iter++) {

        LosMovingObject& movingObject = *(*iter);
        bool positionChanged;

        movingObject.UpdateMovingObjectMobilityPosition(
            groundLayer,
            currentTime,
            positionChanged);

        if (positionChanged) {
            (*this).UpdateMovingObjectPolygon(movingObject);
        }
    }
}



void BuildingLayer::DeleteMovingObjectPolygon(LosMovingObject& movingObject)
{
    LosQuadTree* lastTreePtr = movingObject.lastTreePtr;

    if (lastTreePtr != nullptr) {
        for(unsigned int i = 0; (i < lastTreePtr->losObbPtrs.size()); i++) {
            if (lastTreePtr->losObbPtrs[i]->theNodeId == movingObject.theNodeId) {
                lastTreePtr->losObbPtrs.erase(lastTreePtr->losObbPtrs.begin() + i);
                break;
            }//if//
        }//for//
    }//if//

    movingObject.lastTreePtr = nullptr;
}



void BuildingLayer::UpdateMovingObjectPolygon(LosMovingObject& movingObject)
{
    LosQuadTree* const lastTreePtr = movingObject.lastTreePtr;

    (*this).DeleteMovingObjectPolygon(movingObject);

    LosQuadTree* startTreePtr = theQuadTreePtr.get();

    if (lastTreePtr != nullptr) {
        if (lastTreePtr->place.Contains(movingObject.losObbPtr->GetRect())) {
            // if in same place, restart from current tree
            startTreePtr = lastTreePtr;
        }
    }

    startTreePtr->PushLosOrientedBoundingBox(
        movingObject.losObbPtr,
        movingObject.lastTreePtr);
}


void BuildingLayer::RemoveMovingObject(const NodeId theNodeId)
{
    typedef vector<shared_ptr<LosMovingObject> >::iterator IterType;

    IterType iter = losMovingObjects.begin();

    while (iter != losMovingObjects.end()) {
        LosMovingObject& movingObject = *(*iter);

        if (movingObject.theNodeId == theNodeId) {
            // delete polygons before removing
            (*this).DeleteMovingObjectPolygon(movingObject);

            iter = losMovingObjects.erase(iter);
        } else {
            iter++;
        }
    }
}



void ParkLayer::MakeEntrance(
    const ParkIdType& parkId,
    const Vertex& position)
{
    Park& park = parks[parkId];

    const VertexIdType entranceVertexId =
        park.commonImplPtr->subsystemPtr->GetVertexId(position);

    park.commonImplPtr->vertexIds.push_back(entranceVertexId);

    subsystemPtr->ConnectGisObject(entranceVertexId, GIS_PARK, parkId);
}



GroundLayer::GroundLayer(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    GisSubsystem* initSubsystemPtr)
    :
    subsystemPtr(initSubsystemPtr),
    numberVerticalMeshes(0),
    numberHorizontalMeshes(0),
    meshLengthMeters(DBL_MAX)
{
}



void GroundLayer::ImportGroundSurface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    using SHull::Shx;
    using SHull::s_hull_del_ray2;
    using SHull::Triad;

    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, rect);

    if (entities <= 0) {
        return;
    }

    const double minMeshLengthMeters = 10.;

    // assume ground surface as a square area for mesh length calculation.
    meshLengthMeters = std::max<double>(
        minMeshLengthMeters,
        std::min<double>(rect.GetWidth(), rect.GetHeight()) / (sqrt(static_cast<double>(entities))));

    typedef double ShxFloat;

    const int seed = 1;
    const ShxFloat maxOffset = static_cast<ShxFloat>(meshLengthMeters*0.1);

    RandomNumberGenerator aRandomNumberGenerator(seed);

    vector<Vertex> points;
    vector<Shx> shxs;

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        assert(shpObjPtr->nParts == 0);
        assert(shpObjPtr->nVertices == 1);

        const Vertex point(shpObjPtr->padfX[0], shpObjPtr->padfY[0], shpObjPtr->padfZ[0]);

        points.push_back(point);

        //Note:
        // Completely grid vertex inputs generate a redudant delauney triangulation.
        // Use non-grid vertex with small offset.

        Shx pt;
        pt.id = static_cast<int>(shxs.size());
        pt.r = static_cast<ShxFloat>(point.x) + static_cast<ShxFloat>(aRandomNumberGenerator.GenerateRandomInt(0, INT_MAX))/INT_MAX * maxOffset;
        pt.c = static_cast<ShxFloat>(point.y) + static_cast<ShxFloat>(aRandomNumberGenerator.GenerateRandomInt(0, INT_MAX))/INT_MAX * maxOffset;

        shxs.push_back(pt);

        SHPDestroyObject(shpObjPtr);
    }

    vector<Triad> triads;
    s_hull_del_ray2(shxs, triads);

    rect = rect.Expanded(meshLengthMeters*0.5);

    const double width = rect.GetWidth();
    const double height = rect.GetHeight();

    numberHorizontalMeshes = static_cast<MeshIdType>(ceil(width / meshLengthMeters));
    numberVerticalMeshes = static_cast<MeshIdType>(ceil(height / meshLengthMeters));

    assert((numberVerticalMeshes*numberHorizontalMeshes) < ULONG_MAX);

    const MeshIdType numberMeshes =
        static_cast<MeshIdType>(numberVerticalMeshes * numberHorizontalMeshes);

    grounMeshes.reset(new GroundMesh[numberMeshes]);

    for(size_t i = 0; i < triads.size(); i++) {
        const Triad& triad = triads[i];
        const Triangle triangle(points[triad.a], points[triad.b], points[triad.c]);
        const Rectangle triangleRect = triangle.GetRect();

        if (triangleRect.GetWidth() > 0 && triangleRect.GetHeight() > 0) {
            const MeshIdType minX = static_cast<MeshIdType>((triangleRect.minX - rect.minX) / meshLengthMeters);
            const MeshIdType minY = static_cast<MeshIdType>((triangleRect.minY - rect.minY) / meshLengthMeters);

            const MeshIdType maxX = static_cast<MeshIdType>((triangleRect.maxX - rect.minX) / meshLengthMeters);
            const MeshIdType maxY = static_cast<MeshIdType>((triangleRect.maxY - rect.minY) / meshLengthMeters);

            for(MeshIdType indexX = minX; indexX <= maxX; indexX++) {
                for(MeshIdType indexY = minY; indexY <= maxY; indexY++) {
                    const Rectangle meshRect(
                        indexX*meshLengthMeters + rect.minX,
                        indexY*meshLengthMeters + rect.minY,
                        (indexX+1)*meshLengthMeters + rect.minX,
                        (indexY+1)*meshLengthMeters + rect.minY);

                    if (!triangle.IntersectsWith(meshRect)) {
                        continue;
                    }

                    GroundMesh& groundMesh =
                        grounMeshes[(*this).CalculateMeshNumberAt(indexX, indexY)];

                    groundMesh.triangleIds.push_back(TriangleIdType(triangles.size()));
                }
            }
            triangles.push_back(triangle);
        }
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}



void GroundLayer::ImportTree(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;

    Rectangle treeRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, treeRect);

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        assert(shpObjPtr->nParts == 0);
        assert(shpObjPtr->nVertices == 1);

        const Vertex point(shpObjPtr->padfX[0], shpObjPtr->padfY[0], shpObjPtr->padfZ[0]);

        if (rect.Contains(point)) {
            const MeshIdType meshNumber = (*this).CalculateMeshNumberAt(point);
            GroundMesh& groundMesh = grounMeshes[meshNumber];

            groundMesh.obstructionHeightMetersFromGround = static_cast<float>(point.z);
            groundMesh.obstructionTypes = GROUND_OBSTRUCTION_TREE;
        }

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}



double GroundLayer::GetElevationMetersAt(const Vertex& pos, const bool isRoofTopElevation) const
{
    const TriangleIdType triangleId = (*this).GetTriangleIdAt(pos);

    if (triangleId != INVALID_VARIANT_ID) {
        const Triangle& triangle = triangles[triangleId];

        return triangle.GetSurfaceHeightAt(pos);
    }

    return 0.;
}



GroundLayer::TriangleIdType GroundLayer::GetTriangleIdAt(const Vertex& pos) const
{
    if (!rect.Contains(pos)) {
        return INVALID_VARIANT_ID;
    }

    const MeshIdType meshNumber = (*this).CalculateMeshNumberAt(pos);
    const GroundMesh& groundMesh = grounMeshes[meshNumber];

    typedef vector<TriangleIdType>::const_iterator IterType;

    for(IterType iter = groundMesh.triangleIds.begin(); iter != groundMesh.triangleIds.end(); iter++) {
        const TriangleIdType& triangleId = (*iter);
        const Triangle& triangle = triangles[triangleId];

        if (triangle.Contains(pos)) {
            return triangleId;
        }
    }

    return INVALID_VARIANT_ID;
}



void GroundLayer::GetSeriallyCompleteElevationPoints(
    const Vertex txPos,
    const Vertex rxPos,
    const int numberDivisions,
    const bool addBuildingHeightToElevation,
    const bool addTreeHeightToElevation,
    vector<Vertex>& points) const
{
    const Vertex directionVector = (rxPos - txPos);
    const Vertex normalVector = (rxPos - txPos).XYPoint().Normalized();
    const double distance = txPos.DistanceTo(rxPos);
    const double calculationPointDivisionLength = distance / numberDivisions;

    map<double, WallCollisionInfoType> collisions;

    const BuildingLayer& buildingLayer = *subsystemPtr->GetBuildingLayerPtr();
    const shared_ptr<LosQuadTree> theQuadTreePtr = buildingLayer.GetQuadTreePtr();

    if (addBuildingHeightToElevation) {
        const LosRay ray(txPos.XYPoint(), rxPos.XYPoint());

        set<NodeId> ignoredNodeIds;

        theQuadTreePtr->CheckCollision(ray, ignoredNodeIds, false/*checkJustACollsion*/, true/*isJustHorizontalCheck*/, collisions);
    }

    TriangleIdType triangleId = INVALID_VARIANT_ID;

    points.clear();

    typedef map<double, WallCollisionInfoType>::const_iterator IterType;

    IterType collisionIter = collisions.begin();
    IterType lastIter = collisionIter;

    for(int i = 0; i <= numberDivisions; i++) {
        const double t = double(i) / numberDivisions;

        Vertex point = txPos + directionVector*t;
        bool isElevationAdjustedToRoofTop = false;

        if ((i > 0) && (i < numberDivisions)) {

            // Consider building height only for intermediate points.

            if (collisionIter != collisions.end()) {
                while ((collisionIter != collisions.end()) &&
                       ((*collisionIter).first < t)) {

                    assert((*collisionIter).second.anObstructionType == LosPolygon::Roof);

                    lastIter = collisionIter;
                    collisionIter++;
                }
            }

            if (lastIter != collisions.end()) {
                const BuildingIdType buildingId = (*lastIter).second.variantId;
                const Building& building = buildingLayer.GetBuilding(buildingId);

                if (PolygonContainsPoint(building.GetBuildingPolygon(), point)) {
                    point.z = building.GetRoofTopHeightMeters();
                    isElevationAdjustedToRoofTop = true;
                }
            }
        }


        if (!isElevationAdjustedToRoofTop) {

            if (triangleId == INVALID_VARIANT_ID) {

                triangleId = (*this).GetTriangleIdAt(point);

            } else {

                // Continue from last triangle

                const Triangle& triangle = triangles[triangleId];

                if (!triangle.Contains(point)) {
                    triangleId = (*this).GetTriangleIdAt(point);
                }
            }

            if (triangleId != INVALID_VARIANT_ID) {
                const Triangle& triangle = triangles[triangleId];

                point.z = triangle.GetSurfaceHeightAt(point);

            } else {

                // no ground triangle ==> "z = 0.0"

            }

            if (addTreeHeightToElevation) {

                // Consider obstacle height only for intermediate points.

                if ((rect.Contains(point)) &&
                    (i > 0) &&
                    (i < numberDivisions)) {

                    const MeshIdType meshNumber = (*this).CalculateMeshNumberAt(point);
                    const GroundMesh& groundMesh = grounMeshes[meshNumber];

                    point.z += groundMesh.obstructionHeightMetersFromGround;
                }
            }

        }

        points.push_back(point);
    }
}



void GroundLayer::GetGroundElevationCompletedVertices(
    const Vertex& lineEdge1,
    const Vertex& lineEdge2,
    deque<Vertex>& vertices) const
{
    const Rectangle lineRect(lineEdge1, lineEdge2);

    const MeshIdType minX = std::min<MeshIdType>(static_cast<MeshIdType>(floor((lineRect.minX - rect.minX) / meshLengthMeters)), numberHorizontalMeshes -1);
    const MeshIdType maxX = std::max<MeshIdType>(0, static_cast<MeshIdType>(ceil((lineRect.maxX - rect.minX) / meshLengthMeters)));
    const MeshIdType minY = std::min<MeshIdType>(static_cast<MeshIdType>(floor((lineRect.minY - rect.minY) / meshLengthMeters)), numberVerticalMeshes -1);
    const MeshIdType maxY = std::max<MeshIdType>(0, static_cast<MeshIdType>(ceil((lineRect.maxY - rect.minY) / meshLengthMeters)));

    set<pair<Vertex, Vertex> > triangleEdges;
    set<TriangleIdType> triangleIds;

    for(MeshIdType indexX = minX; indexX <= maxX; indexX++) {
        for(MeshIdType indexY = minY; indexY <= maxY; indexY++) {
            const Rectangle meshRect(
                indexX*meshLengthMeters + rect.minX,
                indexY*meshLengthMeters + rect.minY,
                (indexX+1)*meshLengthMeters + rect.minX,
                (indexY+1)*meshLengthMeters + rect.minY);

            if (!meshRect.IntersectsWithLine(lineEdge1, lineEdge2)) {
                continue;
            }

            if ((0 <= indexX && indexX < numberHorizontalMeshes) &&
                (0 <= indexY && indexY < numberVerticalMeshes)) {

                const MeshIdType meshNumber = (*this).CalculateMeshNumberAt(indexX, indexY);
                const GroundMesh& groundMesh = grounMeshes[meshNumber];

                typedef vector<TriangleIdType>::const_iterator IterType;

                for(IterType iter = groundMesh.triangleIds.begin();
                    iter != groundMesh.triangleIds.end(); iter++) {

                    const TriangleIdType& triangleId = (*iter);
                    const Triangle& triangle = triangles[triangleId];

                    if (triangleIds.find(triangleId) == triangleIds.end()) {
                        triangleIds.insert(triangleId);

                        if (triangle.IntersectsWithLine(lineEdge1, lineEdge2)) {
                            const pair<Vertex, Vertex>& triangleEdge1 = triangle.GetEdge1();
                            const pair<Vertex, Vertex>& triangleEdge2 = triangle.GetEdge2();
                            const pair<Vertex, Vertex>& triangleEdge3 = triangle.GetEdge3();

                            if (HorizontalLinesAreIntersection(
                                    lineEdge1, lineEdge2, triangleEdge1.first, triangleEdge1.second)) {
                                triangleEdges.insert(triangleEdge1);
                            }
                            if (HorizontalLinesAreIntersection(
                                    lineEdge1, lineEdge2, triangleEdge2.first, triangleEdge2.second)) {
                                triangleEdges.insert(triangleEdge2);
                            }
                            if (HorizontalLinesAreIntersection(
                                    lineEdge1, lineEdge2, triangleEdge3.first, triangleEdge3.second)) {
                                triangleEdges.insert(triangleEdge3);
                            }
                        }
                    }
                }
            }
        }
    }

    typedef set<pair<Vertex, Vertex> >::const_iterator EdgeIter;

    map<double, Vertex> elevationVertices;

    const double origEdge1Z = (*this).GetElevationMetersAt(lineEdge1);
    const double origEdge2Z = (*this).GetElevationMetersAt(lineEdge2);

    const Vertex origEdge1(lineEdge1.x, lineEdge1.y, std::max(0., lineEdge1.z - origEdge1Z));
    const Vertex origEdge2(lineEdge2.x, lineEdge2.y, std::max(0., lineEdge2.z - origEdge2Z));

    elevationVertices[0.] = Vertex(origEdge1.x, origEdge1.y, origEdge1.z + origEdge1Z);
    elevationVertices[1.] = Vertex(origEdge2.x, origEdge2.y, origEdge2.z + origEdge2Z);

    const double distance = lineEdge1.XYPoint().DistanceTo(lineEdge2.XYPoint());

    if (distance > 0) {
        for(EdgeIter iter = triangleEdges.begin(); iter != triangleEdges.end(); iter++) {
            const pair<Vertex, Vertex>& edge = (*iter);

            const Vertex intersectoinPosOfBase =
                CalculateIntersectionPositionBetweenLine(
                    lineEdge1, lineEdge2,
                    edge.first, edge.second);

            const double errorCorrection = 1e-10;
            const double t = (intersectoinPosOfBase.XYPoint() - lineEdge1.XYPoint()).Distance() / distance - errorCorrection;

            const Vertex intersectionPosOfOrig = (origEdge2 - origEdge1)*t + origEdge1;

            assert(intersectionPosOfOrig.z >= 0);

            elevationVertices[t] =
                intersectionPosOfOrig + Vertex(0., 0., (*this).GetElevationMetersAt(intersectionPosOfOrig));
        }
    }

    typedef map<double, Vertex>::const_iterator VertexIter;

    for(VertexIter iter = elevationVertices.begin(); iter != elevationVertices.end(); iter++) {
        vertices.push_back((*iter).second);
    }
}



double GroundLayer::GetRoofTopHeightWithGroundElevationtMetersAt(const Vertex& pos) const
{
    return (*this).GetElevationMetersAt(pos, true/*isRoofTopElevation*/);
}



bool GroundLayer::HasTree(const Vertex& pos) const
{
    if (!rect.Contains(pos)) {
        return false;
    }

    const MeshIdType meshNumber = (*this).CalculateMeshNumberAt(pos);
    const GroundMesh& groundMesh = grounMeshes[meshNumber];

    return groundMesh.IsTree();
}



GroundLayer::MeshIdType GroundLayer::CalculateMeshNumberAt(const Vertex& pos) const
{
    const MeshIdType indexX = static_cast<MeshIdType>((pos.x - rect.minX) / meshLengthMeters);
    const MeshIdType indexY = static_cast<MeshIdType>((pos.y - rect.minY) / meshLengthMeters);

    return (*this).CalculateMeshNumberAt(indexX, indexY);
}



GroundLayer::MeshIdType GroundLayer::CalculateMeshNumberAt(const MeshIdType indexX, const MeshIdType indexY) const
{
    assert(0 <= indexX && indexX < numberHorizontalMeshes);
    assert(0 <= indexY && indexY < numberVerticalMeshes);

    const MeshIdType retValue =  (indexY*numberHorizontalMeshes + indexX);

    return retValue;
}

void GenericPolygonLayer::ImportPolygonsFromWavfrontObj(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{

    string objFileDirPath = "./";
    if (theParameterDatabaseReader.ParameterExists("gis-wavefront-obj-file-path")) {
        objFileDirPath = theParameterDatabaseReader.ReadString("gis-wavefront-obj-file-path");
        objFileDirPath += "/";
    }//if//

    Vertex basePosition;

    if ((theParameterDatabaseReader.ParameterExists("gis-wavefront-obj-base-position-x")) ||
        (theParameterDatabaseReader.ParameterExists("gis-wavefront-obj-base-position-y"))) {

        basePosition.x = theParameterDatabaseReader.ReadDouble("gis-wavefront-obj-base-position-x");
        basePosition.y = theParameterDatabaseReader.ReadDouble("gis-wavefront-obj-base-position-y");

        if (theParameterDatabaseReader.ParameterExists("gis-wavefront-obj-base-position-z")) {

            basePosition.z = theParameterDatabaseReader.ReadDouble("gis-wavefront-obj-base-position-z");

        }//if//

    }//if//

    bool offsetFileOutput = false;
    if (theParameterDatabaseReader.ParameterExists("gis-wavefront-obj-offsetted-files-output")) {
        offsetFileOutput = theParameterDatabaseReader.ReadBool("gis-wavefront-obj-offsetted-files-output");
    }//if//

    //terrain
    vector<string> terrainFilePaths;
    if (theParameterDatabaseReader.ParameterExists("gis-wavefront-obj-terrain-files")) {
        const string fileNames = theParameterDatabaseReader.ReadString("gis-wavefront-obj-terrain-files");
        std::istringstream inStream(fileNames);

        while (!inStream.eof()) {
            string fileName;
            inStream >> fileName;
            terrainFilePaths.push_back(objFileDirPath + fileName);
        }//while//

    }//if//

    //building
    vector<string> buildingFilePaths;
    if (theParameterDatabaseReader.ParameterExists("gis-wavefront-obj-building-files")) {
        const string fileNames = theParameterDatabaseReader.ReadString("gis-wavefront-obj-building-files");
        std::istringstream inStream(fileNames);

        while (!inStream.eof()) {
            string fileName;
            inStream >> fileName;
            buildingFilePaths.push_back(objFileDirPath + fileName);
        }//while//

    }
    else {
        //use all obj files if the paramter is not specified

        using boost::filesystem::directory_iterator;
        for (directory_iterator iter(objFileDirPath); iter != directory_iterator(); ++iter) {

            if (iter->path().extension() == ".obj") {

                bool includedInTerrain = false;
                for (size_t i = 0; i < terrainFilePaths.size(); i++) {

                    if (terrainFilePaths[i].find(iter->path().filename().string()) != string::npos) {
                        includedInTerrain = true;
                        break;
                    }//if//

                }//for//

                if (includedInTerrain) continue;

                buildingFilePaths.push_back(iter->path().string());

            }//if//

        }//for//

    }//if//

    if (!buildingFilePaths.empty()) {

        WavefrontObjReader objReader;
        vector<Vertex> vertices;
        vector<vector<int> > facesDefinedByIndex;

        objReader.ReadPolygon(buildingFilePaths, basePosition, offsetFileOutput, vertices, facesDefinedByIndex);

        //cunstruct building polygons
        (*this).ConstructPolygons(vertices, facesDefinedByIndex, (*this).buildingPolygons, (*this).spatialBuildingObjectMap);

    }//if//

    if (!terrainFilePaths.empty()) {

        WavefrontObjReader objReader;
        vector<Vertex> vertices;
        vector<vector<int> > facesDefinedByIndex;

        objReader.ReadPolygon(terrainFilePaths, basePosition, offsetFileOutput, vertices, facesDefinedByIndex);

        //cunstruct terrain polygons
        (*this).ConstructPolygons(vertices, facesDefinedByIndex, (*this).terrainPolygons, (*this).spatialTerrainObjectMap);

    }//if//

}//ImportPolygonsFromWavfrontObj//


void GenericPolygonLayer::ConstructPolygons(
    const vector<Vertex>& vertices,
    const vector<vector<int> >& facesDefinedByIndex,
    vector<GenericPolygon>& polygons,
    SpatialObjectMap& spatialObjectMap)
{

    //create faces with vertex values and insert
    Rectangle entireRectangle;

    for (size_t polygonIndex = 0; polygonIndex < facesDefinedByIndex.size(); polygonIndex++) {

        const size_t numberOfVertices = facesDefinedByIndex[polygonIndex].size();
        const vector<int>& face = facesDefinedByIndex[polygonIndex];

        const GisObjectIdType objectId =
            static_cast<GisObjectIdType>(GISOBJECT_GENERIC_POLYGON_START_NODEID + polygonIndex);
        const GenericPolygonIdType localPolygonId = static_cast<GenericPolygonIdType>(polygonIndex);

        GenericPolygon polygon(subsystemPtr, objectId, localPolygonId);

        for (size_t i = 0; i < numberOfVertices; i++) {

            const int vertexIndex = face[i];

            assert(vertexIndex >= 1);

            polygon.vertices.push_back(vertices[vertexIndex - 1]);

        }//for//

        polygon.UpdateMinRectangle();

        if (polygonIndex == 0) {
            entireRectangle = polygon.GetMinRectangle();
        }
        else {
            entireRectangle += polygon.GetMinRectangle();
        }//if//

        polygons.push_back(polygon);

        //subsystemPtr->RegisterGisObject(polygon, localPolygonId);

    }//for//

    spatialObjectMap.SetMesh(entireRectangle, 1.0);

    for (size_t polygonIndex = 0; polygonIndex < polygons.size(); polygonIndex++) {

        spatialObjectMap.InsertGisObject(polygons[polygonIndex], static_cast<GenericPolygonIdType>(polygonIndex));

    }//for//

}//ConstructPolygons//


static
bool Check2DCollisionForTwoLineSegment(
    const Vertex& vertex1,
    const Vertex& vertex2,
    const Vertex& vertex3,
    const Vertex& vertex4)
{

    return ((((((vertex1.x - vertex2.x) * (vertex3.y - vertex1.y)) + ((vertex1.y - vertex2.y) * (vertex1.x - vertex3.x)))
        * (((vertex1.x - vertex2.x) * (vertex4.y - vertex1.y)) + ((vertex1.y - vertex2.y) * (vertex1.x - vertex4.x)))) < 0.0)
        && (((((vertex3.x - vertex4.x) * (vertex1.y - vertex3.y)) + ((vertex3.y - vertex4.y) * (vertex3.x - vertex1.x)))
        * (((vertex3.x - vertex4.x) * (vertex2.y - vertex3.y)) + ((vertex3.y - vertex4.y) * (vertex3.x - vertex2.x)))) < 0.0));

}


void GenericPolygonLayer::GetPolygonsOnAnEllipse(
    const vector<Vertex>& firstFocalPoints,
    const vector<Vertex>& secondForcalPoints,
    const double& maxPropDistanceMeters,
    vector<GenericPolygon>& overlappedBuildingPolygons,
    vector<GenericPolygon>& overlappedTerrainPolygons) const
{

    set<size_t> finalOverlappedBuildingPolygonIds;
    set<size_t> finalOverlappedTerrainPolygonIds;

    for (size_t firstPointIndex = 0; firstPointIndex < firstFocalPoints.size(); firstPointIndex++) {

        const Vertex point1 = firstFocalPoints[firstPointIndex];

        for (size_t secondPointIndex = 0; secondPointIndex < secondForcalPoints.size(); secondPointIndex++) {

            const Vertex point2 = secondForcalPoints[secondPointIndex];

            const double xyzDistanceBetweenPoints = XYZDistanceBetweenVertices(point1, point2);

            if (xyzDistanceBetweenPoints > maxPropDistanceMeters) {
                //no ellipse area
                continue;
            }//if//

            const double minorAxisLength = sqrt((maxPropDistanceMeters * maxPropDistanceMeters) - (xyzDistanceBetweenPoints * xyzDistanceBetweenPoints));

            double adjustedMajorAxisLength;
            const double deltaZ = point1.z - point2.z;
            if (deltaZ == 0.0) {
                adjustedMajorAxisLength = maxPropDistanceMeters;
            }
            else {
                adjustedMajorAxisLength =
                    std::max(minorAxisLength, (sqrt((xyzDistanceBetweenPoints * xyzDistanceBetweenPoints) - (deltaZ * deltaZ)) / xyzDistanceBetweenPoints * maxPropDistanceMeters));
            }//if//

            const double a = (adjustedMajorAxisLength / 2);
            const double b = (minorAxisLength / 2);

            assert(a >= b);

            const Vertex middlePoint((point1 + point2) / 2);
            const Vertex centeredPoint1 = point1 - middlePoint;
            const double theta = centeredPoint1.DirectionRadians();

            double minX;
            double minY;
            double maxX;
            double maxY;

            vector<Vertex> centeredCornerPoint;
            vector<Vertex> cornerPointForEllipse;

            //const bool simpleMode = true;
            const bool simpleMode = false;

            if (simpleMode) {

                //simple mode: rectangle

                centeredCornerPoint.resize(4);
                cornerPointForEllipse.resize(4);

                centeredCornerPoint[0] = Vertex(a, b);
                centeredCornerPoint[1] = Vertex(-a, b);
                centeredCornerPoint[2] = Vertex(-a, -b);
                centeredCornerPoint[3] = Vertex(a, -b);

            }
            else {

                //complex mode: 12-sided polygon

                centeredCornerPoint.resize(12);
                cornerPointForEllipse.resize(12);

                const double radianFor45Degrees = PI / 4;
                const double crossPointX = a * cos(radianFor45Degrees);
                const double crossPointY = b * sin(radianFor45Degrees);


                centeredCornerPoint[0] = Vertex(a, crossPointY);
                centeredCornerPoint[1] = Vertex(crossPointX, crossPointY);
                centeredCornerPoint[2] = Vertex(crossPointX, b);
                centeredCornerPoint[3] = Vertex(-crossPointX, b);
                centeredCornerPoint[4] = Vertex(-crossPointX, crossPointY);
                centeredCornerPoint[5] = Vertex(-a, crossPointY);
                centeredCornerPoint[6] = Vertex(-a, -crossPointY);
                centeredCornerPoint[7] = Vertex(-crossPointX, -crossPointY);
                centeredCornerPoint[8] = Vertex(-crossPointX, -b);
                centeredCornerPoint[9] = Vertex(crossPointX, -b);
                centeredCornerPoint[10] = Vertex(crossPointX, -crossPointY);
                centeredCornerPoint[11] = Vertex(a, -crossPointY);

            }//if//


            for (size_t i = 0; i < centeredCornerPoint.size(); i++) {

                cornerPointForEllipse[i].x = (centeredCornerPoint[i].x * cos(theta) - centeredCornerPoint[i].y * sin(theta));
                cornerPointForEllipse[i].y = (centeredCornerPoint[i].x * sin(theta) + centeredCornerPoint[i].y * cos(theta));
                cornerPointForEllipse[i] += middlePoint;

                if (i == 0) {

                    minX = cornerPointForEllipse[i].x;
                    maxX = cornerPointForEllipse[i].x;
                    minY = cornerPointForEllipse[i].y;
                    maxY = cornerPointForEllipse[i].y;

                }
                else {

                    minX = std::min(minX, cornerPointForEllipse[i].x);
                    maxX = std::max(maxX, cornerPointForEllipse[i].x);
                    minY = std::min(minY, cornerPointForEllipse[i].y);
                    maxY = std::max(maxY, cornerPointForEllipse[i].y);

                }//if//

            }//for//
            cornerPointForEllipse.push_back(cornerPointForEllipse[0]);

            Rectangle targetArea(minX, minY, maxX, maxY);

            //debug outptu
            //cout << "#Original: (" << point1.x << "," << point1.y << ") (" << point2.x << "," << point2.y << ")"
            //    << " Distance: " << distance << endl;
            //cout << "#Middle: (" << middlePoint.x << "," << middlePoint.y << ")" << endl;
            //cout << "#Theta: " << theta << endl;
            //cout << "#(a,b): (" << a << "," << b << ")" << endl;
            //cout << "#Final: (" << cornerPointForEllipse[0].x << "," << cornerPointForEllipse[0].y << ") (" << cornerPointForEllipse[1].x << "," << cornerPointForEllipse[1].y << ")"
            //    << " (" << cornerPointForEllipse[2].x << "," << cornerPointForEllipse[2].y << ") (" << cornerPointForEllipse[3].x << "," << cornerPointForEllipse[3].y << ")" << endl;
            //cout << "#TargetArea: (" << targetArea.minX << "," << targetArea.minY << ") (" << targetArea.maxX << "," << targetArea.maxY << ")" << endl;


            //building: get overlaped polygon id (rectangle vs rectangle)
            vector<VariantIdType> overlappedBuildingPolygonIds;
            (*this).spatialBuildingObjectMap.GetGisObject(targetArea, overlappedBuildingPolygonIds);

            for (size_t i = 0; i < overlappedBuildingPolygonIds.size(); i++) {

                const size_t polygonId = overlappedBuildingPolygonIds[i];

                if (finalOverlappedBuildingPolygonIds.find(polygonId) != finalOverlappedBuildingPolygonIds.end()) continue;

                const GenericPolygon& targetPolygon = (*this).buildingPolygons[polygonId];
                const vector<Vertex>& targetVertices = targetPolygon.GetPolygon();

                bool overlapped = false;
                //check line cross
                for (size_t j = 0; (j < (targetVertices.size() - 1)) && (!overlapped); j++) {

                    for (size_t k = 0; (k < (cornerPointForEllipse.size() - 1)) && (!overlapped); k++) {
                        overlapped = Check2DCollisionForTwoLineSegment(
                            targetVertices[j], targetVertices[j + 1], cornerPointForEllipse[k], cornerPointForEllipse[k + 1]);
                    }//for//
                }//for//

                //check cover polygon
                for (size_t j = 0; (j < targetVertices.size()) && (!overlapped); j++) {

                    if (PolygonContainsPoint(cornerPointForEllipse, targetVertices[j])) {
                        overlapped = true;
                        break;
                    }//if//

                }//for//

                for (size_t j = 0; (j < cornerPointForEllipse.size()) && (!overlapped); j++) {

                    if (PolygonContainsPoint(targetVertices, cornerPointForEllipse[j])) {
                        overlapped = true;
                        break;
                    }//if//

                }//for//

                if (overlapped) {
                    overlappedBuildingPolygons.push_back(targetPolygon);
                    finalOverlappedBuildingPolygonIds.insert(polygonId);
                }//if//

            }//for//

            //terrain: get overlaped polygon id (rectangle vs rectangle)
            vector<VariantIdType> overlappedTerrainPolygonIds;
            (*this).spatialTerrainObjectMap.GetGisObject(targetArea, overlappedTerrainPolygonIds);

            for (size_t i = 0; i < overlappedTerrainPolygonIds.size(); i++) {

                const size_t polygonId = overlappedTerrainPolygonIds[i];

                if (finalOverlappedTerrainPolygonIds.find(polygonId) != finalOverlappedTerrainPolygonIds.end()) continue;

                const GenericPolygon& targetPolygon = (*this).terrainPolygons[polygonId];
                const vector<Vertex>& targetVertices = targetPolygon.GetPolygon();

                bool overlapped = false;
                //check line cross
                for (size_t j = 0; (j < (targetVertices.size() - 1)) && (!overlapped); j++) {

                    for (size_t k = 0; (k < (cornerPointForEllipse.size() - 1)) && (!overlapped); k++) {
                        overlapped = Check2DCollisionForTwoLineSegment(
                            targetVertices[j], targetVertices[j + 1], cornerPointForEllipse[k], cornerPointForEllipse[k + 1]);
                    }//for//
                }//for//

                //check cover polygon
                for (size_t j = 0; (j < targetVertices.size()) && (!overlapped); j++) {

                    if (PolygonContainsPoint(cornerPointForEllipse, targetVertices[j])) {
                        overlapped = true;
                        break;
                    }//if//

                }//for//

                for (size_t j = 0; (j < cornerPointForEllipse.size()) && (!overlapped); j++) {

                    if (PolygonContainsPoint(targetVertices, cornerPointForEllipse[j])) {
                        overlapped = true;
                        break;
                    }//if//

                }//for//

                if (overlapped) {
                    overlappedTerrainPolygons.push_back(targetPolygon);
                    finalOverlappedTerrainPolygonIds.insert(polygonId);
                }//if//

            }//for//

        }//for//

    }//for//

    cout << "#Building Total: " << (*this).buildingPolygons.size() << " Overlapped: " << overlappedBuildingPolygons.size() << endl;
    cout << "#Terrain  Total: " << (*this).terrainPolygons.size() << " Overlapped: " << overlappedTerrainPolygons.size() << endl;

    ////debug: output OBJ
    //cout << endl << "#Building" << endl;
    //cout << "#Vertices: " << overlappedBuildingPolygons.size() << endl;
    //for (size_t i = 0; i < overlappedBuildingPolygons.size(); i++) {

    //    const vector<Vertex>& vertices = overlappedBuildingPolygons[i].GetPolygon();

    //    for (size_t j = 0; j < vertices.size(); j++) {
    //        cout << "v " << vertices[j].x << " " << vertices[j].y << " " << vertices[j].z << endl;
    //    }//for//

    //}//for//

    //cout << endl << "#Faces: " << overlappedBuildingPolygons.size() << endl;
    //size_t buildinVertexIndex = 1;
    //for (size_t i = 0; i < overlappedBuildingPolygons.size(); i++) {

    //    const vector<Vertex>& vertices = overlappedBuildingPolygons[i].GetPolygon();

    //    cout << "f";
    //    for (size_t j = 0; j < vertices.size(); j++) {
    //        cout << " " << buildinVertexIndex;
    //        buildinVertexIndex++;
    //    }//for//
    //    cout << endl;

    //}//for//

    //cout << endl << "#Terrain" << endl;
    //cout << "#Vertices: " << overlappedTerrainPolygons.size() << endl;
    //for (size_t i = 0; i < overlappedTerrainPolygons.size(); i++) {

    //    const vector<Vertex>& vertices = overlappedTerrainPolygons[i].GetPolygon();

    //    for (size_t j = 0; j < vertices.size(); j++) {
    //        cout << "v " << vertices[j].x << " " << vertices[j].y << " " << vertices[j].z << endl;
    //    }//for//

    //}//for//

    //cout << endl << "#Faces: " << overlappedTerrainPolygons.size() << endl;
    //size_t terrainVertexIndex = 1;
    //for (size_t i = 0; i < overlappedTerrainPolygons.size(); i++) {

    //    const vector<Vertex>& vertices = overlappedTerrainPolygons[i].GetPolygon();

    //    cout << "f";
    //    for (size_t j = 0; j < vertices.size(); j++) {
    //        cout << " " << terrainVertexIndex;
    //        terrainVertexIndex++;
    //    }//for//
    //    cout << endl;

    //}//for//

}//GetPolygonsOnAnEllipse//


void PoiLayer::ImportPoi(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder capacityFinder(hDBF, GIS_DBF_CAPACITY_STRING);
    const AttributeFinder infoFinder(hDBF, GIS_DBF_INFO_STRING);

    const ParkLayer& parkLayer = *subsystemPtr->GetParkLayerPtr();
    const BuildingLayer& buildingLayer = *subsystemPtr->GetBuildingLayerPtr();

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        const PoiIdType poiId = static_cast<PoiIdType>(pois.size());
        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);

        const double x = shpObjPtr->padfX[0];
        const double y = shpObjPtr->padfY[0];
        double z = shpObjPtr->padfZ[0];

        if (ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId)) {
            z += subsystemPtr->GetGroundElevationMetersAt(x, y);
        }

        const Vertex point(x, y, z);
        const VertexIdType& vertexId = subsystemPtr->GetVertexId(point);

        pois.push_back(Poi(subsystemPtr, objectId, poiId, vertexId));
        Poi& poi = pois.back();
        poi.LoadParameters(theParameterDatabaseReader);

        if (nameFinder.IsAvailable()) {
            poi.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }
        if (infoFinder.IsAvailable()) {
            poi.information = infoFinder.GetString(entryId);
        }
        if (shpObjPtr->nVertices > 0) {
            poi.commonImplPtr->elevationFromGroundMeters = shpObjPtr->padfZ[0];
        }

        vector<ParkIdType> parkIds;
        subsystemPtr->GetSpatialIntersectedGisObjectIds(point, GIS_PARK, parkIds);

        const double integrationLength = 0.01;//1cm
        const double elevationFromGroundMeters = poi.commonImplPtr->elevationFromGroundMeters;

        typedef vector<VariantIdType>::const_iterator IterType;

        for(IterType iter = parkIds.begin(); iter != parkIds.end(); iter++) {
            const Park& park = parkLayer.GetPark(*iter);
            if (PolygonContainsPoint(park.GetPolygon(), point) &&
                std::fabs(park.GetElevationFromGroundMeters() - elevationFromGroundMeters) <= integrationLength) {
                poi.parentPositionId = GisPositionIdType(GIS_PARK, *iter);
                break;
            }
        }

        if (poi.parentPositionId.IsInvalid()) {
            vector<BuildingIdType> buildingIds;
            subsystemPtr->GetSpatialIntersectedGisObjectIds(point, GIS_BUILDING, buildingIds);

            for(IterType iter = buildingIds.begin(); iter != buildingIds.end(); iter++) {
                const Building& building = buildingLayer.GetBuilding(*iter);
                if (PolygonContainsPoint(building.GetBuildingPolygon(), point) &&
                std::fabs(building.GetElevationFromGroundMeters() - elevationFromGroundMeters) <= integrationLength) {
                    poi.parentPositionId = GisPositionIdType(GIS_BUILDING, *iter);
                    break;
                }
            }
        }

        if (poi.parentPositionId.IsInvalid()) {
            if (capacityFinder.IsAvailable()) {
                poi.humanCapacity = capacityFinder.GetInt(entryId);
            }
        }

        subsystemPtr->ConnectGisObject(vertexId, GIS_POI, poiId);

        subsystemPtr->RegisterGisObject(poi, poiId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}


}; //namespace ScenSim
