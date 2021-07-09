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

#include "scensim_gis.h"
#include "scensim_gis_shape.h"
#include "insiteglue_chooser.h"
#include "boost/filesystem.hpp"

#include <iostream>

namespace ScenSim {

using std::istringstream;
using std::cout;
using std::endl;

static const GisObjectIdType RESERVED__GIS_OBJECT_ID = 150000000;

void GisSubsystem::GetOffsetWaypoints(
    const double offset,
    const vector<Vertex>& srcWaypoints,
    const bool waypointFromAdditionalStartPosition,
    const Vertex& startPosition,
    const bool replaceFirstEntryWithStartPosition,
    const bool isBaseGroundLevel,
    deque<Vertex>& waypoints) const
{
    waypoints.clear();

    if (offset == 0.) {

        for(size_t i = 0; i < srcWaypoints.size(); i++) {
            waypoints.push_back(srcWaypoints[i]);
        }

    } else {

        vector<pair<Vertex, Vertex> > lines(srcWaypoints.size() - 1);

        for(size_t i = 0; i < srcWaypoints.size() - 1; i++) {
            const Vertex& p1 = srcWaypoints[i];
            const Vertex& p2 = srcWaypoints[i+1];

            if (p1 == p2) {
                cerr << "Error: Waypoints ";

                for(size_t j = 0; j < srcWaypoints.size(); j++) {
                    cerr << " v" << j << "(" << srcWaypoints[j].x << "," << srcWaypoints[j].y << "," << srcWaypoints[j].z << ")";
                }

                cerr << " contains continuous duplicated vertex (" << p1.x << "," << p1.y << "," << p1.z << ")" << endl;

                exit(1);
            }

            assert(p1 != p2);

            const Vertex normal = p1.NormalVector(p2);
            const double xyDistance = p1.XYDistanceTo(p2);

            Vertex offsetPoint;

            if (xyDistance > 0) {
                offsetPoint = normal * (offset/xyDistance);
            }

            lines[i].first = p1 + offsetPoint;
            lines[i].second = p2 + offsetPoint;
        }

        waypoints.push_back(lines.front().first);
        for(size_t i = 0; i < lines.size() - 1; i++) {
            const pair<Vertex, Vertex>& line1 = lines[i];
            const pair<Vertex, Vertex>& line2 = lines[i+1];

            if (HorizontalLinesAreIntersection(
                    line1.first, line1.second,
                    line2.first, line2.second)) {

                waypoints.push_back(
                    CalculateIntersectionPositionBetweenLine(
                        line1.first, line1.second,
                        line2.first, line2.second));
            } else {

                waypoints.push_back(line1.second);

                const int numberDivisions = static_cast<int>(
                    std::ceil(line1.second.DistanceTo(line2.first) / (offset*0.1)));

                if (numberDivisions > 0) {
                    const double absOffset = fabs(offset);
                    const double divisorRate = 2./numberDivisions;
                    const double minDivisionDistance = 0.01;// -> 1cm

                    for(int j = 1; j < numberDivisions; j++) {
                        const double ratio1 = 2. - j*divisorRate;
                        const double ratio2 = 2. - ratio1;

                        const Vertex& p1 = srcWaypoints[i+1];
                        const Vertex p2 = (line1.second*ratio1 + line2.first*ratio2)*0.5;
                        const double distance = p1.DistanceTo(p2);

                        if (distance > minDivisionDistance) {
                            const Vertex dividiedVertex = p1 + (p2 - p1)*(absOffset/distance);

                            waypoints.push_back(dividiedVertex);
                        }
                    }
                }

                waypoints.push_back(line2.first);
            }
        }

        waypoints.push_back(lines.back().second);
    }

    if (waypointFromAdditionalStartPosition) {

        if (replaceFirstEntryWithStartPosition) {
            waypoints[0] = startPosition;
        }
        else {
            waypoints.push_front(startPosition);
        }
    }

    if (isBaseGroundLevel) {
        (*this).CompleteGroundElevation(waypoints);
    }

    if ((waypointFromAdditionalStartPosition) &&
        (!replaceFirstEntryWithStartPosition)) {
        waypoints.pop_front();
    }

}//GetOffsetWaypoints//



void GisSubsystem::CompleteGroundElevation(deque<Vertex>& waypoints) const
{
    if (waypoints.empty()) {
        return;
    }

    const deque<Vertex> srcWaypoints = waypoints;

    waypoints.clear();

    for(size_t i = 0; i < srcWaypoints.size() - 1; i++) {
        deque<Vertex> completedVertices;

        groundLayerPtr->GetGroundElevationCompletedVertices(srcWaypoints[i], srcWaypoints[i+1], completedVertices);

        if (i == 0 && !completedVertices.empty()) {
            waypoints.push_back(completedVertices.front());
        }

        for(size_t j = 1; j < completedVertices.size(); j++) {
            waypoints.push_back(completedVertices[j]);
        }
    }
}


void GisSubsystem::ScheduleGisEvent(
    const shared_ptr<SimulationEvent>& gisEventPtr,
    const SimTime& eventTime)
{
    gisEventInfos.push(GisEventInfo(eventTime, gisEventPtr));
}



void GisSubsystem::LoadLineInfo(const string& fileName)
{
    ifstream inStream(fileName.c_str());

    if (!inStream.good()) {
        cerr << "Error: Failed to open " << fileName << endl;
        exit(1);
    }//if//

    bool isTrain = false;
    string lineName;

    map<pair<RailRoadStationIdType, RailRoadStationIdType>, vector<pair<deque<RailRoadLayer::RailLink>, double> > > sectionCandidatesCache;

    map<string, pair<deque<RailRoadStationIdType>, vector<pair<deque<RailRoadLayer::RailLink>, double> > > > stationIdsAndRailCache;

    const bool busStopIsAvailable =
        (importedGisObjectTypes.find(GIS_BUSSTOP) != importedGisObjectTypes.end());

    const bool stationIsAvailable =
        (importedGisObjectTypes.find(GIS_RAILROAD_STATION) != importedGisObjectTypes.end());


    while(!inStream.eof()) {
        string aLine;
        getline(inStream, aLine);

        DeleteTrailingSpaces(aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }

        deque<string> tokens;
        TokenizeToTrimmedLowerString(aLine, ",", tokens);

        if (tokens[0] == "line") {

            lineName.clear();
            isTrain = false;

            if (tokens.size() > 2) {
                lineName = tokens[1];
                isTrain = (tokens[2] == "train");
            }

        } else if (tokens[0] == "stop" && !lineName.empty()) {

            tokens.pop_front();

            if (isTrain) {

                if (stationIsAvailable) {
                    railRoadLayerPtr->AssignRailRoadLine(lineName, tokens, sectionCandidatesCache, stationIdsAndRailCache);
                }

            } else {

                if (busStopIsAvailable) {
                    roadLayerPtr->AssignBusLine(lineName, tokens);
                }
            }
        }
    }
}

//----------------------------------------------------
// GisSubsystem
//----------------------------------------------------

#pragma warning(disable:4355)

GisSubsystem::GisSubsystem(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr)
    :
    theSimulationEnginePtr(initSimulationEnginePtr.get()),
    isDebugMode(false),
    changedGisTopology(false),
    executeEventsManually(false),
    positionInLatlongDegree(false),
    latitudeOriginDegrees(LATITUDE_ORIGIN_DEGREES),
    longitudeOriginDegrees(LONGITUDE_ORIGIN_DEGREES),
    poiLayerPtr(new PoiLayer(this)),
    roadLayerPtr(new RoadLayer(theParameterDatabaseReader, this)),
    railRoadLayerPtr(new RailRoadLayer(this)),
    areaLayerPtr(new AreaLayer(this)),
    parkLayerPtr(new ParkLayer(this)),
    buildingLayerPtr(new BuildingLayer(theParameterDatabaseReader, this)),
    groundLayerPtr(new GroundLayer(theParameterDatabaseReader, this)),
    genericPolygonLayerPtr(new GenericPolygonLayer(this)),
    insiteGeometryPtr(new InsiteGeometry()),
    spatialObjectMaps(GIS_GENERIC_START),
    reservedObjectId(RESERVED__GIS_OBJECT_ID)
{
    if (theParameterDatabaseReader.ParameterExists("gis-debug-mode")) {
        isDebugMode = theParameterDatabaseReader.ReadBool("gis-debug-mode");
    }
    if (theParameterDatabaseReader.ParameterExists("gis-object-position-in-latlong-degree")) {
        positionInLatlongDegree =
            theParameterDatabaseReader.ReadBool("gis-object-position-in-latlong-degree");

        if (positionInLatlongDegree) {
            latitudeOriginDegrees =
                theParameterDatabaseReader.ReadDouble("gis-latitude-origin-degrees");
            longitudeOriginDegrees =
                theParameterDatabaseReader.ReadDouble("gis-longitude-origin-degrees");
        }
    }


    if (theParameterDatabaseReader.ParameterExists("material-file")) {
        (*this).LoadLocalMaterials(theParameterDatabaseReader.ReadString("material-file"));
    }

    string shapeFileDirPath;

    if (theParameterDatabaseReader.ParameterExists("gis-object-file-path")) {
        shapeFileDirPath = theParameterDatabaseReader.ReadString("gis-object-file-path");
    }

    if (theParameterDatabaseReader.ParameterExists("gis-object-files")) {
        const string fileNames = theParameterDatabaseReader.ReadString("gis-object-files");
        std::istringstream inStream(fileNames);

        typedef vector<pair<int, string> > ContainarType;
        typedef std::greater<pair<int, string> > PriorityType;

        std::priority_queue<pair<int, string>, ContainarType, PriorityType> fileNamesWithPriority;

        Vertex originPoint(0, 0);

        entireRect = Rectangle(originPoint, 1);

        while (!inStream.eof()) {
            string fileName;
            inStream >> fileName;

            int readingPriority = 0;

            if (fileName.find(GIS_RAILROAD_STRING) != string::npos) {
                readingPriority = 7;
            } else if (fileName.find(GIS_ROAD_STRING) != string::npos) {
                readingPriority = 1;
            } else if (fileName.find(GIS_INTERSECTION_STRING) != string::npos) {
                readingPriority = 0;
            } else if (fileName.find(GIS_AREA_STRING) != string::npos) {
                readingPriority = -1;
            } else if (fileName.find(GIS_PARK_STRING) != string::npos) {
                readingPriority = 2;
            } else if (fileName.find(GIS_BUILDING_STRING) != string::npos) {
                readingPriority = 3;
            } else if (fileName.find(GIS_WALL_STRING) != string::npos) {
                readingPriority = 5;
            } else if (fileName.find(GIS_STATION_STRING) != string::npos) {
                readingPriority = 6;
            } else if ((fileName.find(GIS_OLD_TRAFFIC_LIGHT_STRING) != string::npos) ||
                       (fileName.find(GIS_TRAFFIC_LIGHT_STRING) != string::npos)) {
                readingPriority = 8;
            } else if (fileName.find(GIS_BUSSTOP_STRING) != string::npos) {
                readingPriority = 9;
            } else if (fileName.find(GIS_ENTRANCE_STRING) != string::npos) {
                readingPriority = 10;
            } else if (fileName.find(GIS_POI_STRING) != string::npos) {
                readingPriority = 11;
            } else if (fileName.find(GIS_GROUND_STRING) != string::npos) {
                readingPriority = -2;
            } else if (fileName.find(GIS_TREE_STRING) != string::npos) {
                readingPriority = 12;
            } else if (!fileName.empty()) {
                readingPriority = 13;
            }

            fileNamesWithPriority.push(make_pair(readingPriority, fileName));

            entireRect += PeekLayerRectangle(shapeFileDirPath + fileName);
        }

        const double spatialMeshUnit = 100; //100m
        spatialVertexMap.SetMesh(
            entireRect, spatialMeshUnit,
            SpatialObjectMap::MAX_VERTEX_MESH_SIZE);
        for(size_t i = 0; i < spatialObjectMaps.size(); i++) {
            spatialObjectMaps[i].SetMesh(entireRect, spatialMeshUnit);
        }

        while (!fileNamesWithPriority.empty()) {
            const string& fileName = fileNamesWithPriority.top().second;
            const string filePath = shapeFileDirPath + fileName;

            if (fileName.find(GIS_RAILROAD_STRING) != string::npos) {
                railRoadLayerPtr->ImportRailRoad(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_RAILROAD);
            } else if (fileName.find(GIS_INTERSECTION_STRING) != string::npos) {
                roadLayerPtr->ImportIntersection(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_INTERSECTION);
            } else if (fileName.find(GIS_ROAD_STRING) != string::npos) {
                roadLayerPtr->ImportRoad(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_ROAD);
            } else if (fileName.find(GIS_AREA_STRING) != string::npos) {
                areaLayerPtr->ImportArea(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_AREA);
            } else if (fileName.find(GIS_PARK_STRING) != string::npos) {
                parkLayerPtr->ImportPark(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_PARK);
            } else if (fileName.find(GIS_BUILDING_STRING) != string::npos) {
                buildingLayerPtr->ImportBuilding(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_BUILDING);
            } else if (fileName.find(GIS_WALL_STRING) != string::npos) {
                buildingLayerPtr->ImportWall(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_WALL);
            } else if (fileName.find(GIS_STATION_STRING) != string::npos) {
                railRoadLayerPtr->ImportStation(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_RAILROAD_STATION);
            } else if ((fileName.find(GIS_OLD_TRAFFIC_LIGHT_STRING) != string::npos) ||
                       (fileName.find(GIS_TRAFFIC_LIGHT_STRING) != string::npos)) {
                roadLayerPtr->ImportTrafficLight(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_TRAFFICLIGHT);
            } else if (fileName.find(GIS_BUSSTOP_STRING) != string::npos) {
                roadLayerPtr->ImportBusStop(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_BUSSTOP);
            } else if (fileName.find(GIS_ENTRANCE_STRING) != string::npos) {
                (*this).ImportEntrance(theParameterDatabaseReader, filePath);
            } else if (fileName.find(GIS_POI_STRING) != string::npos) {
                poiLayerPtr->ImportPoi(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_POI);
            } else if (fileName.find(GIS_GROUND_STRING) != string::npos) {
                groundLayerPtr->ImportGroundSurface(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_GROUND);
            } else if (fileName.find(GIS_TREE_STRING) != string::npos) {
                groundLayerPtr->ImportTree(theParameterDatabaseReader, filePath);
            } else if (!fileName.empty()) {
                //(*this).ImportGenericLayer(theParameterDatabaseReader, filePath);
            }

            fileNamesWithPriority.pop();
        }
    }

    (*this).CompleteConnections(theParameterDatabaseReader);

    //for obj file (RF Planner)
    if ((theParameterDatabaseReader.ParameterExists("gis-wavefront-obj-file-path")) ||
        (theParameterDatabaseReader.ParameterExists("gis-wavefront-obj-building-files")) ||
        (theParameterDatabaseReader.ParameterExists("gis-wavefront-obj-terrain-files"))) {
        genericPolygonLayerPtr->ImportPolygonsFromWavfrontObj(theParameterDatabaseReader);
    }//if//

    insiteGeometryPtr->RegisterBuildingLayerPtr(buildingLayerPtr);

    //(*this).OutputVertexInformation();
    //(*this).OutputGisObjectVertexInformation();
}



#pragma warning(default:4355)


const GisObject& GisSubsystem::GetGisObject(const GisObjectIdType& objectId) const
{
    typedef map<GisObjectIdType, GisPositionIdType>::const_iterator IterType;

    IterType iter = objectIdMap.find(objectId);

    assert(iter != objectIdMap.end());

    return (*this).GetGisObject((*iter).second);
}



const GisObject& GisSubsystem::GetGisObject(const GisPositionIdType& positionId) const
{
    const GisObjectType& objectType = positionId.type;
    const VariantIdType& variantId = positionId.id;

    switch (objectType) {
    case GIS_AREA: return areaLayerPtr->GetArea(variantId);
    case GIS_POINT: return poiLayerPtr->GetPoint(variantId);
    case GIS_ROAD: return roadLayerPtr->GetRoad(variantId);
    case GIS_INTERSECTION: return roadLayerPtr->GetIntersection(variantId);
    case GIS_RAILROAD: return railRoadLayerPtr->GetRailRoad(variantId);
    case GIS_RAILROAD_INTERSECTION: return railRoadLayerPtr->GetRailRoadIntersection(variantId);
    case GIS_RAILROAD_STATION: return railRoadLayerPtr->GetStation(variantId);
    case GIS_PARK: return parkLayerPtr->GetPark(variantId);
    case GIS_WALL: return buildingLayerPtr->GetWall(variantId);
    case GIS_BUILDING: return buildingLayerPtr->GetBuilding(variantId);
    case GIS_POI: return poiLayerPtr->GetPoi(variantId);
    case GIS_BUSSTOP: return roadLayerPtr->GetBusStop(variantId);
    case GIS_TRAFFICLIGHT: return roadLayerPtr->GetTrafficLight(variantId);
    case GIS_ENTRANCE: return (*this).GetEntrance(variantId);
    default:
        break;
    }

    return genericGisLayerPtrs[objectType]->GetGenericGisObject(variantId);
}



const Vertex& GisSubsystem::GetVertex(const VertexIdType& vertexId) const
{
    return vertices[vertexId].vertex;
}


const GisVertex& GisSubsystem::GetGisVertex(const VertexIdType& vertexId) const
{
    return vertices[vertexId];
}


const Point& GisSubsystem::GetPoint(const PointIdType& pointId) const
{
    return poiLayerPtr->GetPoint(pointId);
}


const Intersection& GisSubsystem::GetIntersection(const IntersectionIdType& intersectionId) const
{
    return roadLayerPtr->GetIntersection(intersectionId);
}


const Road& GisSubsystem::GetRoad(const RoadIdType& roadId) const
{
    return roadLayerPtr->GetRoad(roadId);
}


const Poi& GisSubsystem::GetPoi(const PoiIdType& poiId) const
{
    return poiLayerPtr->GetPoi(poiId);
}


const RailRoad& GisSubsystem::GetRailRoad(const RailRoadIdType& railRoadId) const
{
    return railRoadLayerPtr->GetRailRoad(railRoadId);
}


const RailRoadIntersection& GisSubsystem::GetRailRoadIntersection(const RailRoadIntersectionIdType& railRoadIntersectionId) const
{
    return railRoadLayerPtr->GetRailRoadIntersection(railRoadIntersectionId);
}


const RailRoadStation& GisSubsystem::GetStation(const RailRoadStationIdType& stationId) const
{
    return railRoadLayerPtr->GetStation(stationId);
}


const Area& GisSubsystem::GetArea(const AreaIdType& areaId) const
{
    return areaLayerPtr->GetArea(areaId);
}


const Park& GisSubsystem::GetPark(const ParkIdType& parkId) const
{
    return parkLayerPtr->GetPark(parkId);
}


const Wall& GisSubsystem::GetWall(const WallIdType& wallId) const
{
    return buildingLayerPtr->GetWall(wallId);
}


const Building& GisSubsystem::GetBuilding(const BuildingIdType& buildingId) const
{
    return buildingLayerPtr->GetBuilding(buildingId);
}


const BusStop& GisSubsystem::GetBusStop(const BusStopIdType& busStopId) const
{
    return roadLayerPtr->GetBusStop(busStopId);
}


const TrafficLight& GisSubsystem::GetTrafficLight(const TrafficLightIdType& busStopId) const
{
    return roadLayerPtr->GetTrafficLight(busStopId);
}



bool GisSubsystem::ContainsObject(const GisObjectIdType& objectId) const
{
    return (objectIdMap.find(objectId) != objectIdMap.end());
}



VariantIdType GisSubsystem::GetVariantId(
    const GisObjectType& objectType,
    const GisObjectIdType& objectId) const
{
    typedef map<GisObjectIdType, GisPositionIdType>::const_iterator IterType;

    IterType iter = objectIdMap.find(objectId);

    assert(iter != objectIdMap.end());

    const GisPositionIdType& idPair = (*iter).second;

    assert(idPair.type == objectType);

    return idPair.id;
}



const Point& GisSubsystem::GetPointObject(const GisObjectIdType& objectId) const
{
    return poiLayerPtr->GetPoint((*this).GetVariantId(GIS_POINT, objectId));
}


const Intersection& GisSubsystem::GetIntersectionObject(const GisObjectIdType& objectId) const
{
    return roadLayerPtr->GetIntersection((*this).GetVariantId(GIS_INTERSECTION, objectId));
}


const Road& GisSubsystem::GetRoadObject(const GisObjectIdType& objectId) const
{
    return roadLayerPtr->GetRoad((*this).GetVariantId(GIS_ROAD, objectId));
}


const Poi& GisSubsystem::GetPoiObject(const GisObjectIdType& objectId) const
{
    return poiLayerPtr->GetPoi((*this).GetVariantId(GIS_POI, objectId));
}


const RailRoad& GisSubsystem::GetRailRoadObject(const GisObjectIdType& objectId) const
{
    return railRoadLayerPtr->GetRailRoad((*this).GetVariantId(GIS_RAILROAD, objectId));
}


const RailRoadIntersection& GisSubsystem::GetRailRoadIntersectionObject(const GisObjectIdType& objectId) const
{
    return railRoadLayerPtr->GetRailRoadIntersection((*this).GetVariantId(GIS_RAILROAD_INTERSECTION, objectId));
}


const RailRoadStation& GisSubsystem::GetStationObject(const GisObjectIdType& objectId) const
{
    return railRoadLayerPtr->GetStation((*this).GetVariantId(GIS_RAILROAD_STATION, objectId));
}


const Area& GisSubsystem::GetAreaObject(const GisObjectIdType& objectId) const
{
    return areaLayerPtr->GetArea((*this).GetVariantId(GIS_AREA, objectId));
}


const Park& GisSubsystem::GetParkObject(const GisObjectIdType& objectId) const
{
    return parkLayerPtr->GetPark((*this).GetVariantId(GIS_PARK, objectId));
}


const Wall& GisSubsystem::GetWallObject(const GisObjectIdType& objectId) const
{
    return buildingLayerPtr->GetWall((*this).GetVariantId(GIS_WALL, objectId));
}


const Building& GisSubsystem::GetBuildingObject(const GisObjectIdType& objectId) const
{
    return buildingLayerPtr->GetBuilding((*this).GetVariantId(GIS_BUILDING, objectId));
}

const vector<Point>& GisSubsystem::GetPoints() const
{
    return poiLayerPtr->GetPoints();
}


const vector<Intersection>& GisSubsystem::GetIntersections() const
{
    return roadLayerPtr->GetIntersections();
}


const vector<shared_ptr<const Road> > GisSubsystem::GetRoadPtrs() const
{
    return (
        vector<shared_ptr<const Road> >(
            roadLayerPtr->GetRoadPtrs().begin(),
            roadLayerPtr->GetRoadPtrs().end()));
}


const vector<BusStop>& GisSubsystem::GetBusStops() const
{
    return roadLayerPtr->GetBusStops();
}


const vector<Poi>& GisSubsystem::GetPois() const
{
    return poiLayerPtr->GetPois();
}


const vector<Building>& GisSubsystem::GetBuildings() const
{
    return buildingLayerPtr->GetBuildings();
}


const vector<Park>& GisSubsystem::GetParks() const
{
    return parkLayerPtr->GetParks();
}


const vector<Wall>& GisSubsystem::GetWalls() const
{
    return buildingLayerPtr->GetWalls();
}


const vector<RailRoadStation>& GisSubsystem::GetStations() const
{
    return railRoadLayerPtr->GetStations();
}


const vector<Area>& GisSubsystem::GetAreas() const
{
    return areaLayerPtr->GetAreas();
}



GisPositionIdType GisSubsystem::GetPosition(
    const string& name,
    const GisObjectType& objectType) const
{
    typedef map<string, map<GisObjectType, set<VariantIdType> > >::const_iterator NameIter;

    const string lowerName = MakeLowerCaseString(name);

    NameIter nameIter = positionsPerName.find(lowerName);

    if (nameIter == positionsPerName.end()) {
        cerr << "Error: There is no position: " << name << endl;
        assert(false);
        exit(1);
    }

    typedef map<GisObjectType, set<VariantIdType> >::const_iterator PositionIter;

    const map<GisObjectType, set<VariantIdType> >& positionIds = (*nameIter).second;

    PositionIter posIter;

    if (objectType == INVALID_OBJECT_TYPE) {
        posIter = positionIds.begin();
    } else {
        posIter = positionIds.find(objectType);
    }

    if (posIter == positionIds.end()) {
        cerr << "Error: Couldn't find gis object: " << name << endl;
        exit(1);
    }

    if ((*posIter).second.size() > 1) {
        cerr << "Error: Duplicated gis object name: " << name << endl;
        exit(1);
    }

    return GisPositionIdType((*posIter).first, *(*posIter).second.begin());
}



void GisSubsystem::GetPositions(
    const string& name,
    vector<GisPositionIdType>& foundPositionIds,
    const GisObjectType& objectType) const
{
    foundPositionIds.clear();

    typedef map<string, map<GisObjectType, set<VariantIdType> > >::const_iterator NameIter;

    const string lowerName = MakeLowerCaseString(name);

    NameIter nameIter = positionsPerName.find(lowerName);

    if (nameIter == positionsPerName.end()) {
        cerr << "Error: There is no position: " << name << endl;
        assert(false);
        exit(1);
    }

    typedef map<GisObjectType, set<VariantIdType> >::const_iterator PositionIter;

    const map<GisObjectType, set<VariantIdType> >& positionIds = (*nameIter).second;

    if (objectType != INVALID_OBJECT_TYPE) {

        if (positionIds.find(objectType) == positionIds.end()) {
            cerr << "Error: Couldn't find gis object: " << name << endl;
            exit(1);
        }
    }

    for(PositionIter posIter = positionIds.begin(); posIter != positionIds.end(); posIter++) {

        const GisObjectType& gisObjectType = (*posIter).first;

        if ((gisObjectType == objectType) ||
            (objectType == INVALID_OBJECT_TYPE/*target is any*/)) {

            const set<VariantIdType>& variantIds = (*posIter).second;

            typedef set<VariantIdType>::const_iterator VariantIdIter;

            for(VariantIdIter iter = variantIds.begin();
                iter != variantIds.end(); iter++) {

                foundPositionIds.push_back(GisPositionIdType((*posIter).first, *iter));
            }
        }
    }
}



bool GisSubsystem::ContainsPosition(const string& name) const
{
    typedef map<string, map<GisObjectType, set<VariantIdType> > >::const_iterator NameIter;

    const string lowerName = MakeLowerCaseString(name);

    return (positionsPerName.find(lowerName) != positionsPerName.end());
}



GisPositionIdType GisSubsystem::GetPositionId(
    const GisObjectIdType& objectId) const
{
    typedef map<GisObjectIdType, GisPositionIdType>::const_iterator IterType;

    IterType iter = objectIdMap.find(objectId);

    if (iter == objectIdMap.end()) {
        cerr << "Error: Couldn't find gis object: " << objectId << endl;
        exit(1);
    }

    return (*iter).second;
}



bool GisSubsystem::IntersectsWith(
    const GisPositionIdType& positionId,
    const Rectangle& rect) const
{
    switch (positionId.type) {
    case GIS_BUILDING:return buildingLayerPtr->GetBuilding(positionId.id).IntersectsWith(rect);
    case GIS_AREA: return areaLayerPtr->GetArea(positionId.id).IntersectsWith(rect);
    case GIS_POINT: return poiLayerPtr->GetPoint(positionId.id).IntersectsWith(rect);
    case GIS_ROAD: return roadLayerPtr->GetRoad(positionId.id).IntersectsWith(rect);
    case GIS_INTERSECTION: return roadLayerPtr->GetIntersection(positionId.id).IntersectsWith(rect);
    case GIS_POI: return poiLayerPtr->GetPoi(positionId.id).IntersectsWith(rect);
    case GIS_RAILROAD: return railRoadLayerPtr->GetRailRoad(positionId.id).IntersectsWith(rect);
    case GIS_RAILROAD_INTERSECTION: return railRoadLayerPtr->GetRailRoadIntersection(positionId.id).IntersectsWith(rect);
    case GIS_RAILROAD_STATION: return  railRoadLayerPtr->GetStation(positionId.id).IntersectsWith(rect);
    case GIS_PARK: return parkLayerPtr->GetPark(positionId.id).IntersectsWith(rect);
    case GIS_WALL: return buildingLayerPtr->GetWall(positionId.id).IntersectsWith(rect);
    case GIS_BUSSTOP: return roadLayerPtr->GetBusStop(positionId.id).IntersectsWith(rect);
    default:
        break;
    }

    return false;
}



void GisSubsystem::RemoveMovingObject(const NodeId theNodeId)
{
    buildingLayerPtr->RemoveMovingObject(theNodeId);

    insiteGeometryPtr->DeleteObject(theNodeId);

}



GisPositionIdType GisSubsystem::GetPositionId(
    const Vertex& position,
    const vector<GisObjectType>& prioritizedSearchObjectTypes,
    const double integrationLength) const
{
    const Rectangle searchRect(position, integrationLength);

    for(size_t i = 0 ; i < prioritizedSearchObjectTypes.size(); i++) {
        const GisObjectType& objectType = prioritizedSearchObjectTypes[i];

        vector<VariantIdType> variantIds;

        (*this).GetSpatialIntersectedGisObjectIds(position, objectType, variantIds);

        typedef vector<VariantIdType>::const_iterator IterType;

        for(IterType iter = variantIds.begin(); iter != variantIds.end(); iter++) {
            const VariantIdType& variantId = (*iter);

            if ((*this).IntersectsWith(GisPositionIdType(objectType, variantId), searchRect)) {
                return GisPositionIdType(objectType, variantId);
            }
        }
    }

    return GisPositionIdType();
}


GisPositionIdType GisSubsystem::GetPositionIdWithZValue(
    const Vertex& position,
    const vector<GisObjectType>& prioritizedSearchObjectTypes,
    const double integrationLength) const
{
    const Rectangle searchRect(position, integrationLength);

    for(size_t i = 0 ; i < prioritizedSearchObjectTypes.size(); i++) {
        const GisObjectType& objectType = prioritizedSearchObjectTypes[i];

        vector<VariantIdType> variantIds;

        (*this).GetSpatialIntersectedGisObjectIds(position, objectType, variantIds);

        map<double, VariantIdType> positionForZValue;

        for(size_t j = 0; j < variantIds.size(); j++) {
            const VariantIdType& variantId = variantIds[j];

            if ((*this).IntersectsWith(GisPositionIdType(objectType, variantId), searchRect)) {


                const double objectZValue =
                    (*this).GetPositionVertex(GisPositionIdType(objectType, variantId)).z;

                positionForZValue[objectZValue] = variantId;
            }//if//
        }//for//

        typedef map<double, VariantIdType>::const_iterator IterType;

        if (!positionForZValue.empty()) {

            IterType iter = positionForZValue.upper_bound(position.z);

            if (iter == positionForZValue.end()) {
                iter--;
            }//if//

            return GisPositionIdType(objectType, (*iter).second);
        }//if//

    }

    return GisPositionIdType();
}



//----------------------------------------------------

void GisSubsystem::GetGisObjectIds(
    const Rectangle& targetRect,
    const GisObjectType& objectType,
    vector<VariantIdType>& variantIds) const
{
    variantIds.clear();

    vector<VariantIdType> variantIdCandidates;

    (*this).GetSpatialIntersectedGisObjectIds(targetRect, objectType, variantIdCandidates);

    typedef vector<VariantIdType>::const_iterator IterType;

    for(IterType iter = variantIdCandidates.begin(); iter != variantIdCandidates.end(); iter++) {
        const VariantIdType& variantId = (*iter);

        if ((*this).IntersectsWith(GisPositionIdType(objectType, variantId), targetRect)) {
            variantIds.push_back(variantId);
        }
    }
}



void GisSubsystem::GetGisObjectIds(
    const Vertex& vertex,
    const GisObjectType& objectType,
    vector<VariantIdType>& variantIds) const
{
    const double integrationLength = 0.01;//1cm

    (*this).GetGisObjectIds(Rectangle(vertex, integrationLength), objectType, variantIds);
}



void GisSubsystem::GetGisObjectIds(
    const Rectangle& targetRect,
    const vector<GisObjectType>& searchObjectTypes,
    vector<GisPositionIdType>& positionIds) const
{
    typedef vector<GisObjectType>::const_iterator TypeIter;
    typedef vector<VariantIdType>::const_iterator IdIter;

    positionIds.clear();

    for(TypeIter typeIter = searchObjectTypes.begin();
        typeIter != searchObjectTypes.end(); typeIter++) {

        const GisObjectType& type = *typeIter;

        vector<VariantIdType> variantIds;

        (*this).GetGisObjectIds(targetRect, type, variantIds);

        for(IdIter idIter = variantIds.begin();
            (idIter != variantIds.end()); idIter++) {
            positionIds.push_back(GisPositionIdType(type, *idIter));
        }
    }
}



void GisSubsystem::GetGisObjectIds(
    const Vertex& vertex,
    const vector<GisObjectType>& searchObjectTypes,
    vector<GisPositionIdType>& positionIds) const
{
    const double integrationLength = 0.01;//1cm

    (*this).GetGisObjectIds(Rectangle(vertex, integrationLength), searchObjectTypes, positionIds);
}



void GisSubsystem::GetSpatialIntersectedGisObjectIds(
    const Vertex& vertex,
    const GisObjectType& objectType,
    vector<VariantIdType>& variantIds) const
{
    const double integrationLength = 0.01;//1cm

    spatialObjectMaps[objectType].GetGisObject(Rectangle(vertex, integrationLength), variantIds);
}



void GisSubsystem::GetSpatialIntersectedGisObjectIds(
    const Rectangle& targetRect,
    const GisObjectType& objectType,
    vector<VariantIdType>& variantIds) const
{
    spatialObjectMaps[objectType].GetGisObject(targetRect, variantIds);
}



void GisSubsystem::GetVertexIds(
    const Rectangle& targetRect,
    vector<VertexIdType>& variantIds) const
{
    spatialVertexMap.GetGisObject(targetRect, variantIds);
}



void GisSubsystem::ConnectGisObject(
    const VertexIdType& srcVertexId,
    const GisObjectType& objectType,
    const VariantIdType& destVariantId)
{
    (*this).ConnectGisObject(srcVertexId, objectType, srcVertexId, destVariantId);
}



void GisSubsystem::ConnectGisObject(
    const VertexIdType& srcVertexId,
    const GisObjectType& objectType,
    const VertexIdType& destVertexId,
    const VariantIdType& destVariantId)
{
    GisVertex& gisVertex = vertices[srcVertexId];

    gisVertex.connections[objectType].push_back(
        VertexConnection(destVertexId, destVariantId));

    gisVertex.connectionsPerVertex[destVertexId] =
        GisPositionIdType(objectType, destVariantId);

    assert(objectType != GIS_AREA &&
           objectType != GIS_POINT);
}



void GisSubsystem::ConnectBidirectionalGisObject(
    const VertexIdType& vertexId1,
    const GisObjectType& objectType,
    const VertexIdType& vertexId2,
    const VariantIdType& destVariantId)
{
    if (vertexId1 == vertexId2) {

        (*this).ConnectGisObject(
            vertexId1, objectType, destVariantId);

    } else {

        (*this).ConnectGisObject(
            vertexId1, objectType, vertexId2, destVariantId);

        (*this).ConnectGisObject(
            vertexId2, objectType, vertexId1, destVariantId);
    }
}



void GisSubsystem::DisconnectGisObject(
    const VertexIdType& srcVertexId,
    const GisObjectType& objectType,
    const VertexIdType& destVertexId,
    const VariantIdType& destVariantId)
{
    GisVertex& gisVertex = vertices[srcVertexId];
    vector<VertexConnection>& connections = gisVertex.connections[objectType];

    typedef vector<VertexConnection>::iterator IterType;

    const VertexConnection removeConnection(destVertexId, destVariantId);

    for(IterType iter = connections.begin(); iter != connections.end(); iter++) {
        if ((*iter) == removeConnection) {
            connections.erase(iter);
            break;
        }
    }

    gisVertex.connectionsPerVertex.erase(destVertexId);
}



const Entrance& GisSubsystem::GetEntrance(const EntranceIdType& entranceId) const
{
    return entrances.at(entranceId);
}



Entrance& GisSubsystem::GetEntrance(const EntranceIdType& entranceId)
{
    return entrances.at(entranceId);
}



void GisSubsystem::DisconnectBidirectionalGisObject(
    const VertexIdType& vertexId1,
    const GisObjectType& objectType,
    const VertexIdType& vertexId2,
    const VariantIdType& destVariantId)
{
    (*this).DisconnectGisObject(vertexId1, objectType, vertexId2, destVariantId);

    (*this).DisconnectGisObject(vertexId2, objectType, vertexId1, destVariantId);
}



void GisSubsystem::UnregisterGisObject(
    const GisObject& gisObject,
    const VariantIdType& variantId)
{
    const GisObjectType objectType = gisObject.GetObjectType();
    const GisObjectIdType objectId = gisObject.GetObjectId();
    const GisPositionIdType positionId(objectType, variantId);

    objectIdMap.erase(objectId);

    gisObject.UpdateMinRectangle();

    spatialObjectMaps[objectType].RemoveGisObject(gisObject, variantId);

    string name = gisObject.GetObjectName();

    if (!name.empty()) {
        positionsPerName[name][objectType].erase(variantId);
    }
}



void GisSubsystem::RegisterGisObject(
    const GisObject& gisObject,
    const VariantIdType& variantId)
{
    const GisObjectType objectType = gisObject.GetObjectType();
    const GisObjectIdType objectId = gisObject.GetObjectId();
    const GisPositionIdType positionId(objectType, variantId);

    objectIdMap[objectId] = positionId;

    gisObject.UpdateMinRectangle();

    spatialObjectMaps[objectType].InsertGisObject(gisObject, variantId);


    string name = gisObject.GetObjectName();

    if (!name.empty()) {
        map<GisObjectType, set<VariantIdType> >& positions = positionsPerName[name];

        if (objectType == GIS_BUSSTOP) {

            if (positions.find(objectType) != positions.end()) {
                if (objectType == GIS_RAILROAD_STATION) {
                    cerr << "Error: Station name [" << name << "] is duplicated." << endl;
                } else {
                    cerr << "Error: BusStop name [" << name << "] is duplicated." << endl;
                }
                exit(1);
            }
        }

        positions[objectType].insert(variantId);
    }
}



void GisSubsystem::CompleteConnections(const ParameterDatabaseReader& theParameterDatabaseReader)
{
    // create LoS topology before adding road.
    buildingLayerPtr->RemakeLosTopology();

    // Additional road connection
    vector<Building>& buildings = buildingLayerPtr->GetBuildings();
    vector<RailRoadStation>& stations = railRoadLayerPtr->GetStations();
    vector<BusStop>& busStops = roadLayerPtr->GetBusStops();
    vector<Park>& parks = parkLayerPtr->GetParks();
    vector<Poi>& pois = poiLayerPtr->GetPois();

    const size_t numberDefaultEntrancesToBuilding = roadLayerPtr->GetNumberOfEntrancesToBuilding();
    const size_t numberDefaultEntrancesToStation = roadLayerPtr->GetNumberOfEntrancesToStation();
    const size_t numberDefaultEntrancesToBusStop = roadLayerPtr->GetNumberOfEntrancesToBusStop();
    const size_t numberDefaultEntrancesToPark = roadLayerPtr->GetNumberOfEntrancesToPark();

    if (roadLayerPtr->GetRoadPtrs().empty()) {

        if ((!buildings.empty() && numberDefaultEntrancesToBuilding > 0) ||
            (!stations.empty() && numberDefaultEntrancesToStation > 0) ||
            (!busStops.empty() && numberDefaultEntrancesToBusStop > 0) ||
            (!parks.empty() && numberDefaultEntrancesToPark > 0) ||
            (!pois.empty()) ||
            (!entrances.empty())) {

            cerr << "Error: There is no road to connect an entrance. Place at least one road." << endl;
            exit(1);
        }

    } else {

        typedef set<EntranceIdType>::const_iterator IterType;

        set<RoadIdType> ignoreRoadIds;

        for(BuildingIdType buildingId = 0;
            buildingId < BuildingIdType(buildings.size()); buildingId++) {

            Building& building = buildings[buildingId];
            building.CompleteEntrances(numberDefaultEntrancesToBuilding);

            const set<EntranceIdType>& entranceIds = building.GetEntranceIds();

            for(IterType iter = entranceIds.begin(); iter != entranceIds.end(); iter++) {
                const EntranceIdType& entranceId = (*iter);
                const Vertex entrancePosition = entrances.at(entranceId).GetVertex();

                RoadIdType entranceRoadId;
                RoadIdType newRoadId;
                bool success;

                (*this).GetRoadEntranceCandidates(
                    building.GetMinRectangle(),
                    entrancePosition,
                    ignoreRoadIds,
                    success,
                    entranceRoadId);

                if (success) {
                    buildingLayerPtr->MakeEntrance(buildingId, entrancePosition);
                    const VertexIdType entranceVertexId = (*this).GetVertexId(entrancePosition);
                    const Road& entranceRoad = (*this).GetRoad(entranceRoadId);

                    if (entranceRoad.ContainsVertexId(entranceVertexId)) {
                        roadLayerPtr->CreateParking(
                            GisPositionIdType(GIS_BUILDING, buildingId),
                            entranceVertexId,
                            entranceRoad.GetObjectId());
                    }
                    else {
                        VertexIdType intersectionVertexId;

                        roadLayerPtr->MakeDirectPathToPoi(
                            entranceRoadId,
                            GisPositionIdType(GIS_BUILDING, buildingId),
                            entrancePosition,
                            newRoadId,
                            intersectionVertexId);
                        ignoreRoadIds.insert(newRoadId);
                    }//if//

                } else {

                    cerr << "Error: There is no road to connect entrance"
                         << (*this).GetEntrance(entranceId).GetObjectName()
                         << " for " << building.GetObjectName()
                         << ". Place at least one road." << endl;
                    exit(1);
                }
            }
        }

        for(RailRoadStationIdType stationId = 0;
            stationId < RailRoadStationIdType(stations.size()); stationId++) {

            RailRoadStation& station = stations[stationId];
            station.CompleteEntrances(numberDefaultEntrancesToStation);

            const set<EntranceIdType>& entranceIds = station.GetEntranceIds();

            for(IterType iter = entranceIds.begin(); iter != entranceIds.end(); iter++) {
                const EntranceIdType& entranceId = (*iter);
                const Vertex entrancePosition = entrances.at(entranceId).GetVertex();

                RoadIdType entranceRoadId;
                RoadIdType newRoadId;
                bool success;

                (*this).GetRoadEntranceCandidates(
                    station.GetMinRectangle(),
                    entrancePosition,
                    ignoreRoadIds,
                    success,
                    entranceRoadId);

                if (success) {
                    railRoadLayerPtr->MakeEntrance(stationId, entrancePosition);
                    const VertexIdType entranceVertexId = (*this).GetVertexId(entrancePosition);
                    const Road& entranceRoad = (*this).GetRoad(entranceRoadId);

                    if (entranceRoad.ContainsVertexId(entranceVertexId)) {
                        roadLayerPtr->CreateParking(
                            GisPositionIdType(GIS_RAILROAD_STATION, stationId),
                            entranceVertexId,
                            entranceRoad.GetObjectId());
                    }
                    else {
                        VertexIdType intersectionVertexId;

                        roadLayerPtr->MakeDirectPathToPoi(
                            entranceRoadId,
                            GisPositionIdType(GIS_RAILROAD_STATION, stationId),
                            entrancePosition,
                            newRoadId,
                            intersectionVertexId);
                        ignoreRoadIds.insert(newRoadId);
                    }//if//-+

                } else {

                    cerr << "Error: There is no road to connect entrance" << (*this).GetEntrance(entranceId).GetObjectName() << " for " << station.GetObjectName() << ". Place at least one road." << endl;
                    exit(1);
                }
            }
        }

        for(ParkIdType parkId = 0; parkId < ParkIdType(parks.size()); parkId++) {

            Park& park = parks[parkId];
            park.CompleteEntrances(numberDefaultEntrancesToPark);

            const set<EntranceIdType>& entranceIds = park.GetEntranceIds();

            for(IterType iter = entranceIds.begin(); iter != entranceIds.end(); iter++) {
                const EntranceIdType& entranceId = (*iter);
                const Vertex entrancePosition = entrances.at(entranceId).GetVertex();

                RoadIdType entranceRoadId;
                RoadIdType newRoadId;
                bool success;

                (*this).GetRoadEntranceCandidates(
                    park.GetMinRectangle(),
                    entrancePosition,
                    ignoreRoadIds,
                    success,
                    entranceRoadId);

                if (success) {
                    parkLayerPtr->MakeEntrance(parkId, entrancePosition);

                    const VertexIdType entranceVertexId = (*this).GetVertexId(entrancePosition);
                    const Road& entranceRoad = (*this).GetRoad(entranceRoadId);

                    if (entranceRoad.ContainsVertexId(entranceVertexId)) {
                        roadLayerPtr->CreateParking(
                            GisPositionIdType(GIS_PARK, parkId),
                            entranceVertexId,
                            entranceRoad.GetObjectId());
                    }
                    else {
                        VertexIdType intersectionVertexId;

                        roadLayerPtr->MakeDirectPathToPoi(
                            entranceRoadId,
                            GisPositionIdType(GIS_PARK, parkId),
                            entrancePosition,
                            newRoadId,
                            intersectionVertexId);
                        ignoreRoadIds.insert(newRoadId);
                    }//if//

                } else {

                    cerr << "Error: There is no road to connect entrance" << (*this).GetEntrance(entranceId).GetObjectName() << " for " << park.GetObjectName() << ". Place at least one road." << endl;
                    exit(1);
                }
            }
        }

        for(PoiIdType poiId = 0; poiId < PoiIdType(pois.size()); poiId++) {
            const Poi& poi = pois[poiId];

            if (poi.IsAPartOfObject()) {
                continue;
            }

            const Vertex entrancePosition = poi.GetVertex();

            RoadIdType entranceRoadId;
            RoadIdType newRoadId;
            bool success;

            (*this).GetRoadEntranceCandidates(
                poi.GetMinRectangle(),
                entrancePosition,
                ignoreRoadIds,
                success,
                entranceRoadId);

            if (success) {
                const VertexIdType entranceVertexId = (*this).GetVertexId(entrancePosition);
                const Road& entranceRoad = (*this).GetRoad(entranceRoadId);

                if (entranceRoad.ContainsVertexId(entranceVertexId)) {
                    roadLayerPtr->CreateParking(
                        GisPositionIdType(GIS_POI, poiId),
                        entranceVertexId,
                        entranceRoad.GetObjectId());
                }
                else {
                    VertexIdType intersectionVertexId;

                    roadLayerPtr->MakeDirectPathToPoi(
                        entranceRoadId,
                        GisPositionIdType(GIS_POI, poiId),
                        entrancePosition,
                        newRoadId,
                        intersectionVertexId);
                    ignoreRoadIds.insert(newRoadId);
                }//if//

            } else {

                cerr << "Error: There is no road to connect POI " << poi.GetObjectName() << ". Place at least one road." << endl;
                exit(1);
            }
        }

        // Bus stop calculation must be last step for import/export compatibity.

        for(BusStopIdType busStopId = 0; busStopId < BusStopIdType(busStops.size()); busStopId++) {

            BusStop& busStop = busStops[busStopId];
            busStop.CompleteEntrances(numberDefaultEntrancesToBusStop);
            busStop.LoadParameters(theParameterDatabaseReader);

            const set<EntranceIdType>& entranceIds = busStop.GetEntranceIds();

            for(IterType iter = entranceIds.begin(); iter != entranceIds.end(); iter++) {
                const EntranceIdType& entranceId = (*iter);
                const Entrance& entrance = entrances.at(entranceId);
                const Vertex entrancePosition = entrance.GetVertex();

                RoadIdType entranceRoadId;
                RoadIdType newRoadId;
                bool success;

                (*this).GetRoadEntranceCandidates(
                    Rectangle(entrancePosition, 1),
                    entrancePosition,
                    ignoreRoadIds,
                    success,
                    entranceRoadId);

                if (success) {
                    const VertexIdType entranceVertexId = (*this).GetVertexId(entrancePosition);
                    const GisVertex& gisVertex = (*this).GetGisVertex(entranceVertexId);
                    const Road& entranceRoad = (*this).GetRoad(entranceRoadId);

                    if ((!entranceRoad.ContainsVertexId(entranceVertexId)) ||
                        (!gisVertex.HasConnection(GIS_INTERSECTION))) {
                        VertexIdType intersectionVertexId;

                        roadLayerPtr->MakeDirectPathToPoi(
                            entranceRoadId,
                            GisPositionIdType(GIS_BUSSTOP, busStopId),
                            entrancePosition,
                            newRoadId,
                            intersectionVertexId);

                        // Add busstop parking on the road
                        roadLayerPtr->AddBusStopVertex(busStopId, intersectionVertexId);

                        ignoreRoadIds.insert(newRoadId);
                    }
                    else {
                        const IntersectionIdType intersectionId = gisVertex.GetConnectedObjectId(GIS_INTERSECTION);

                        const IntersectionIdType othersideIntersectionId =
                            entranceRoad.GetOtherSideIntersectionId(intersectionId);

                        // Actually this parking will be not used.
                        // Added only for import/export compability
                        roadLayerPtr->CreateParking(
                            GisPositionIdType(GIS_BUSSTOP, busStopId),
                            entranceVertexId,
                            entranceRoad.GetObjectId());

                        // Add busstop parking on the road
                        roadLayerPtr->AddBusStopVertex(busStopId, othersideIntersectionId);
                    }//if//

                } else {

                    cerr << "Error: There is no road to connect entrance" << (*this).GetEntrance(entranceId).GetObjectName() << " for " << busStop.GetObjectName() << ". Place at least one road." << endl;
                    exit(1);
                }
            }
        }
    }

    // Rail road completion
    railRoadLayerPtr->ComplementLineInfo();

    if (theParameterDatabaseReader.ParameterExists("gis-public-vehicle-file")) {
        (*this).LoadLineInfo(
            theParameterDatabaseReader.ReadString("gis-public-vehicle-file"));
    }

    roadLayerPtr->SetIntersectionMarginAndMakeLaneConnection();

    //railRoadLayerPtr->MakeRailRoadToStationConnection();

    // final check
    // for(size_t i = 0; i < vertices.size(); i++) {
    //     GisVertex& gisVertex = vertices[i];

    //     const vector<RoadIdType>& roadIds =
    //         gisVertex.GetConnectedObjectIds(GIS_ROAD);

    //     if (roadIds.size() >= 2) {

    //         const vector<IntersectionIdType> intersectionIds =
    //             gisVertex.GetConnectedObjectIds(GIS_INTERSECTION);

    //         if (intersectionIds.empty()) {
    //             roadLayerPtr->CreateIntersection(
    //                 i, (*this).CreateNewObjectId());
    //         }
    //     }
    // }
}




void GisSubsystem::GetRoadEntranceCandidates(
    const Rectangle& minSearchRectangle,
    const Vertex& entrancePosition,
    const set<RoadIdType>& ignoreRoadIds,
    bool& success,
    RoadIdType& entranceRoadId)
{
    if ((*this).IsVertexPos(entrancePosition)) {
        const VertexIdType entranceVertexId = (*this).GetVertexId(entrancePosition);
        const GisVertex& gisVertex = (*this).GetGisVertex(entranceVertexId);

        const vector<RoadIdType> roadIds = gisVertex.GetConnectedObjectIds(GIS_ROAD);

        for(size_t i = 0; i < roadIds.size(); i++) {
            const RoadIdType roadId = roadIds[i];
            const Road& road = (*this).GetRoad(roadId);

            entranceRoadId = roadId;
            success = true;
            break;
        }//for//
    }//if//

    const double maxSearchLength = 100 * 1000; //100km
    double searchLength = 500;
    Rectangle availableRect;
    vector<RoadIdType> roadIds;

    typedef vector<RoadIdType>::iterator RoadIdIter;

    for(; (roadIds.empty() && searchLength < maxSearchLength); searchLength += 50) {
        availableRect = minSearchRectangle.Expanded(searchLength);
        (*this).GetSpatialIntersectedGisObjectIds(availableRect, GIS_ROAD, roadIds);

        RoadIdIter roadIdIter = roadIds.begin();
        while (roadIdIter != roadIds.end()) {
            if (ignoreRoadIds.find(*roadIdIter) != ignoreRoadIds.end()) {
                roadIdIter = roadIds.erase(roadIdIter);
            } else {
                roadIdIter++;
            }
        }
    }

    typedef vector<BuildingIdType>::const_iterator BuildingIdIter;

    double minDistance = DBL_MAX;

    success = false;

    for(RoadIdIter roadIdIter = roadIds.begin(); roadIdIter != roadIds.end(); roadIdIter++) {
        const RoadIdType roadId = (*roadIdIter);
        const Road& road = roadLayerPtr->GetRoad(roadId);
        const Vertex& nearestPosition = road.GetNearestPosition(entrancePosition);
        const double distance = entrancePosition.XYDistanceTo(nearestPosition);

        if (distance < minDistance) {
            minDistance = distance;
            entranceRoadId = roadId;
            success = true;
        }
    }
}



Vertex GisSubsystem::CalculateMedianPoint(const vector<VertexIdType>& vertexIds) const
{
    typedef vector<VertexIdType>::const_iterator IterType;

    Vertex vertex;

    for(IterType iter = vertexIds.begin(); iter != vertexIds.end(); iter++) {
        vertex += vertices[*iter].vertex;
    }

    return vertex / double(vertexIds.size());
}



GisPositionIdType GisSubsystem::GetConnectedPositionId(
    const VertexIdType& srcVertexId,
    const VertexIdType& destVertexId) const
{
    const GisVertex& srcVertex = vertices[srcVertexId];
    const map<VertexIdType, GisPositionIdType>& connectionsPerVertex = srcVertex.connectionsPerVertex;

    typedef map<VertexIdType, GisPositionIdType>::const_iterator IterType;

    IterType iter = connectionsPerVertex.find(destVertexId);

    assert(iter != connectionsPerVertex.end());

    return (*iter).second;
}



Vertex GisSubsystem::GetPositionVertex(
    const GisPositionIdType& positionId) const
{
    Vertex vertex;

    switch (positionId.type) {
    case GIS_AREA: vertex = areaLayerPtr->GetArea(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_POINT: vertex = poiLayerPtr->GetPoint(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_ROAD: vertex = roadLayerPtr->GetRoad(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_INTERSECTION: vertex = roadLayerPtr->GetIntersection(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_POI: vertex = poiLayerPtr->GetPoi(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_RAILROAD: vertex = railRoadLayerPtr->GetRailRoad(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_RAILROAD_INTERSECTION: vertex = railRoadLayerPtr->GetRailRoadIntersection(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_RAILROAD_STATION: vertex = railRoadLayerPtr->GetStation(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_PARK: vertex = parkLayerPtr->GetPark(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_WALL: vertex = buildingLayerPtr->GetWall(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_BUILDING: vertex = buildingLayerPtr->GetBuilding(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_BUSSTOP: vertex = roadLayerPtr->GetBusStop(positionId.id).GetMinRectangle().GetCenter(); break;
    default:
        assert(false);
        break;
    }

    return vertex;
}



void GisSubsystem::GetNearEntranceVertexIds(
    const GisPositionIdType& positionId,
    const Vertex& position,
    vector<VertexIdType>& vertexIds) const
{
    vertexIds.clear();

    switch (positionId.type) {
    case GIS_PARK: parkLayerPtr->GetPark(positionId.id).GetNearEntranceVertexIds(position, vertexIds); break;
    case GIS_BUILDING: buildingLayerPtr->GetBuilding(positionId.id).GetNearEntranceVertexIds(position, vertexIds); break;
    case GIS_POI: vertexIds.push_back(poiLayerPtr->GetPoi(positionId.id).GetVertexId()); break;
    default:
        // no entrance vertex
        break;
    }
}



bool GisSubsystem::HasConnection(
    const VertexIdType& srcVertexId,
    const VertexIdType& destVertexId) const
{
    const GisVertex& srcVertex = vertices[srcVertexId];
    const map<VertexIdType, GisPositionIdType>& connectionsPerVertex = srcVertex.connectionsPerVertex;

    typedef map<VertexIdType, GisPositionIdType>::const_iterator IterType;

    IterType iter = connectionsPerVertex.find(destVertexId);

    return (iter != connectionsPerVertex.end());
}



double GisSubsystem::GetElevationMetersAt(
    const Vertex& pos,
    const GisPositionIdType& positionId) const
{
    const double groundElevationMeters = groundLayerPtr->GetElevationMetersAt(pos);
    const GisObjectType& objectType = positionId.type;
    const VariantIdType& variantId = positionId.id;

    double elevationFromGroundMeters = 0.0;

    switch (objectType) {
    case GIS_ROAD: elevationFromGroundMeters = roadLayerPtr->GetRoad(variantId).GetElevationFromGroundMeters(); break;
    case GIS_BUILDING: elevationFromGroundMeters = buildingLayerPtr->GetBuilding(variantId).GetElevationFromGroundMeters(); break;
    case GIS_PARK: elevationFromGroundMeters = parkLayerPtr->GetPark(variantId).GetElevationFromGroundMeters(); break;
    case GIS_ENTRANCE: elevationFromGroundMeters = (*this).GetEntrance(variantId).GetElevationFromGroundMeters(); break;
    case GIS_BUSSTOP: roadLayerPtr->GetBusStop(variantId).GetElevationFromGroundMeters(); break;
    case GIS_WALL: elevationFromGroundMeters = buildingLayerPtr->GetWall(variantId).GetElevationFromGroundMeters(); break;
    case GIS_AREA: elevationFromGroundMeters = areaLayerPtr->GetArea(variantId).GetElevationFromGroundMeters(); break;
    case GIS_POINT: elevationFromGroundMeters = poiLayerPtr->GetPoint(variantId).GetElevationFromGroundMeters(); break;
    case GIS_INTERSECTION: elevationFromGroundMeters = roadLayerPtr->GetIntersection(variantId).GetElevationFromGroundMeters(); break;
    case GIS_RAILROAD: elevationFromGroundMeters = railRoadLayerPtr->GetRailRoad(variantId).GetElevationFromGroundMeters(); break;
    case GIS_RAILROAD_INTERSECTION: elevationFromGroundMeters = railRoadLayerPtr->GetRailRoadIntersection(variantId).GetElevationFromGroundMeters(); break;
    case GIS_RAILROAD_STATION: elevationFromGroundMeters = railRoadLayerPtr->GetStation(variantId).GetElevationFromGroundMeters(); break;
    case GIS_TRAFFICLIGHT: elevationFromGroundMeters = roadLayerPtr->GetTrafficLight(variantId).GetElevationFromGroundMeters(); break;
    case GIS_POI: elevationFromGroundMeters = poiLayerPtr->GetPoi(variantId).GetElevationFromGroundMeters(); break;
    }

    return groundElevationMeters + elevationFromGroundMeters;
}



void GisSubsystem::GetARandomPosition(
    const GisObjectType& objectType,
    const set<GisPositionIdType>& ignoredDestinationIds,
    HighQualityRandomNumberGenerator& aRandomNumberGenerator,
    bool& found,
    GisPositionIdType& randomPositionId) const
{
    vector<GisPositionIdType> candidatePositionIds;

    switch(objectType) {
    case GIS_BUILDING: {
        const vector<Building>& buildings = buildingLayerPtr->GetBuildings();

        for(size_t i = 0; i < buildings.size(); i++) {
            const GisPositionIdType positionId(GIS_BUILDING, buildings[i].GetBuildingId());

            if (ignoredDestinationIds.find(positionId) == ignoredDestinationIds.end()) {
                candidatePositionIds.push_back(positionId);
            }
        }
        break;
    }

    case GIS_PARK: {
        const vector<Park>& parks = parkLayerPtr->GetParks();

        for(size_t i = 0; i < parks.size(); i++) {
            const GisPositionIdType positionId(GIS_PARK, parks[i].GetParkId());

            if (ignoredDestinationIds.find(positionId) == ignoredDestinationIds.end()) {
                candidatePositionIds.push_back(positionId);
            }
        }
        break;
    }

    case GIS_POI: {
        const vector<Poi>& pois = poiLayerPtr->GetPois();

        for(size_t i = 0; i < pois.size(); i++) {
            const GisPositionIdType positionId(GIS_POI, pois[i].GetPoiId());

            if (ignoredDestinationIds.find(positionId) == ignoredDestinationIds.end()) {
                candidatePositionIds.push_back(positionId);
            }
        }
        break;
    }

    case GIS_INTERSECTION: {
        const vector<Intersection>& intersections = roadLayerPtr->GetIntersections();

        for(size_t i = 0; i < intersections.size(); i++) {
            const Intersection& intersection = intersections[i];

            if (!intersection.GetObjectName().empty()) {
                const GisPositionIdType positionId(GIS_INTERSECTION, intersection.GetIntersectionId());

                if (ignoredDestinationIds.find(positionId) == ignoredDestinationIds.end()) {
                    candidatePositionIds.push_back(positionId);
                }
            }
        }
        break;
    }

    default:
        break;
    }

    if (candidatePositionIds.empty()) {
        found = false;
        randomPositionId = GisPositionIdType();
    } else {
        found = true;

        if (candidatePositionIds.size() == 1) {
            randomPositionId = candidatePositionIds.front();
        } else {
            randomPositionId = candidatePositionIds[aRandomNumberGenerator.GenerateRandomInt(0, static_cast<int32_t>(candidatePositionIds.size() - 1))];
        }
    }
}



GisObjectIdType GisSubsystem::GetARandomGisObjectId(
    const GisObjectType& objectType,
    RandomNumberGenerator& aRandomNumberGenerator) const
{
    switch(objectType) {
    case GIS_ROAD: {
        const vector<shared_ptr<Road> >& roadPtrs = roadLayerPtr->GetRoadPtrs();
        return roadPtrs[aRandomNumberGenerator.GenerateRandomInt(0, static_cast<int32_t>(roadPtrs.size() - 1))]->GetObjectId();
    }

    case GIS_INTERSECTION: {
        const vector<Intersection>& intersections = roadLayerPtr->GetIntersections();
        return intersections[aRandomNumberGenerator.GenerateRandomInt(0, static_cast<int32_t>(intersections.size() - 1))].GetObjectId();
    }

    default:
        break;
    }

    assert(false);

    return INVALID_GIS_OBJECT_ID;
}



void GisSubsystem::GetBuildingPositionIdsInArea(
    const AreaIdType& areaId,
    vector<GisPositionIdType>& buildingPositionIds) const
{
    set<GisPositionIdType> ignoredDestinationIds;

    (*this).GetBuildingPositionIdsInArea(areaId, ignoredDestinationIds, buildingPositionIds);
}



void GisSubsystem::GetBuildingPositionIdsInArea(
    const AreaIdType& areaId,
    const set<GisPositionIdType>& ignoredDestinationIds,
    vector<GisPositionIdType>& buildingPositionIds) const
{
    buildingPositionIds.clear();

    const vector<Building>& buildings = buildingLayerPtr->GetBuildings();

    const Area area = areaLayerPtr->GetArea(areaId);
    const Rectangle& areaRect = area.GetMinRectangle();
    const vector<Vertex>& areaPolygon = area.GetPolygon();

    vector<BuildingIdType> buildingIds;

    (*this).GetSpatialIntersectedGisObjectIds(areaRect, GIS_BUILDING, buildingIds);

    typedef vector<BuildingIdType>::const_iterator IterType;

    bool foundABuilding = false;

    for(IterType iter = buildingIds.begin(); iter != buildingIds.end(); iter++) {

        const BuildingIdType& buildingId = (*iter);
        const Building& building = buildings[buildingId];
        const GisPositionIdType positionId(GIS_BUILDING, *iter);

        if (ignoredDestinationIds.find(positionId) != ignoredDestinationIds.end()) {
            continue;
        }

        if (CompleteCoveredPolygon(areaPolygon, building.GetBuildingPolygon())) {

            foundABuilding = true;
            buildingPositionIds.push_back(positionId);
        }
    }

    if (!foundABuilding) {
        cerr << "Error: Specified area " << area.GetObjectName() << " doesn't contain any building." << endl;
        exit(1);
    }

}



VertexIdType GisSubsystem::GetNearestVertexId(
    const GisPositionIdType& positionId,
    const Vertex& position) const
{
    if (positionId.type == GIS_BUILDING) {
        const Building& building = buildingLayerPtr->GetBuilding(positionId.id);
        return building.GetNearestEntranceVertexId(position);

    } else if (positionId.type == GIS_ROAD) {
        const Road& road = roadLayerPtr->GetRoad(positionId.id);
        return road.GetNearestVertexId(position);

    } else if (positionId.type == GIS_PARK) {
        const Park& park = parkLayerPtr->GetPark(positionId.id);
        return park.GetNearestEntranceVertexId(position);
    }

    assert(false);
    return INVALID_VERTEX_ID;
}



bool GisSubsystem::IsParkingVertex(const VertexIdType& vertexId) const
{
    const vector<RoadIdType> connectedRoadIds =
        vertices[vertexId].GetConnectedObjectIds(GIS_ROAD);

    for(size_t i = 0; i < connectedRoadIds.size(); i++) {
        if ((*this).GetRoad(connectedRoadIds[i]).IsParking()) {
            return true;
        }
    }

    return false;
}




RoadIdType GisSubsystem::GetParkingRoadId(const VertexIdType& vertexId) const
{
    const vector<RoadIdType> connectedRoadIds =
        vertices[vertexId].GetConnectedObjectIds(GIS_ROAD);

    for(size_t i = 0; i < connectedRoadIds.size(); i++) {
        const RoadIdType& roadId = connectedRoadIds[i];

        if ((*this).GetRoad(roadId).IsParking()) {
            return roadId;
        }
    }

    assert(false);

    return INVALID_VARIANT_ID;
}



RoadIdType GisSubsystem::GetRoadId(const VertexIdType& vertexId) const
{
    return vertices[vertexId].GetConnectedObjectId(GIS_ROAD);
}


PoiIdType GisSubsystem::GetPoiId(const VertexIdType& vertexId) const
{
    return vertices[vertexId].GetConnectedObjectId(GIS_POI);
}


IntersectionIdType GisSubsystem::GetIntersectionId(
    const VertexIdType& vertexId) const
{
    return vertices[vertexId].GetConnectedObjectId(GIS_INTERSECTION);
}


RailRoadStationIdType GisSubsystem::GetStationId(
    const VertexIdType& vertexId) const
{
    return vertices[vertexId].GetConnectedObjectId(GIS_RAILROAD_STATION);
}


BusStopIdType GisSubsystem::GetBusStopId(
    const VertexIdType& vertexId) const
{
    return vertices[vertexId].GetConnectedObjectId(GIS_BUSSTOP);
}


bool GisSubsystem::IsIntersectionVertex(
    const VertexIdType& vertexId) const
{
    return (*this).IsVertexOf(GIS_INTERSECTION, vertexId);
}


bool GisSubsystem::IsVertexOf(
    const GisObjectType& objectType,
    const VertexIdType& vertexId) const
{
    const GisVertex& gisVertex = vertices[vertexId];

    typedef map<GisObjectType, vector<VertexConnection> >::const_iterator IterType;

    IterType iter = gisVertex.connections.find(objectType);

    if (iter == gisVertex.connections.end()) {
        return false;
    }

    return !(*iter).second.empty();
}



bool GisSubsystem::VertexContains(
    const GisPositionIdType& positionId,
    const VertexIdType& vertexId) const
{
    const GisVertex& gisVertex = vertices[vertexId];

    typedef map<GisObjectType, vector<VertexConnection> >::const_iterator IterType;

    IterType iter = gisVertex.connections.find(positionId.type);

    if (iter == gisVertex.connections.end()) {
        return false;
    }

    const vector<VertexConnection>& vertexConnections = (*iter).second;

    for(size_t i = 0; i < vertexConnections.size(); i++) {
        if (vertexConnections[i].variantId == positionId.id) {
            return true;
        }
    }

    return false;
}




double GisSubsystem::CalculateDistance(
    const VertexIdType& vertexId1,
    const VertexIdType& vertexId2) const
{
    return vertices[vertexId1].vertex.DistanceTo(vertices[vertexId2].vertex);
}



shared_ptr<const GenericGisLayer> GisSubsystem::GetGenericLayerPtr(const string& layerName) const
{
    typedef map<string, GenericGisLayerIdType>::const_iterator IterType;

    IterType iter = genericGisLayerIds.find(layerName);

    if (iter != genericGisLayerIds.end()) {
        return (*this).GetGenericLayerPtr((*iter).second);
    }

    return shared_ptr<const GenericGisLayer>();
}



MaterialIdType GisSubsystem::GetMaterialId(const string& materialName) const
{
    return materials.GetMaterialId(materialName);
}



const Material& GisSubsystem::GetMaterial(const MaterialIdType& materialId) const
{
    return materials.GetMaterial(materialId);
}



void GisSubsystem::LoadLocalMaterials(const string& materialFilePath)
{
    ifstream inFile(materialFilePath.c_str());

    if (!inFile.good()) {
        cerr << "Could Not open material file: " << materialFilePath << endl;
        exit(1);
    }

    while(!inFile.eof()) {
        string aLine;
        getline(inFile, aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }

        DeleteTrailingSpaces(aLine);
        istringstream lineStream(aLine);

        string materialName;
        string materialType;
        double transmissionLossDb;

        lineStream >> materialName >> materialType;
        ConvertStringToLowerCase(materialType);

        if (materialType == "cost231indoor") {

            lineStream >> transmissionLossDb;
            materials.AddMaterial(materialName, transmissionLossDb);

        } else if (materials.GetMaterial(materialName).name.empty()) {
            materials.AddMaterial(materialName, 0);
        }

        if (lineStream.fail()) {
            cerr << "Error: Bad material file line: " << aLine << endl;
            exit(1);
        }
    }
}



bool GisSubsystem::IsVertexPoint(const Vertex& point) const
{
    Vertex vertex = point;

    if (positionInLatlongDegree) {
        vertex = vertex.ToXyVertex(
            latitudeOriginDegrees, longitudeOriginDegrees);
    }

    vector<VariantIdType> vertexIds;

    const double integrationLength = 0.01;//1cm

    spatialVertexMap.GetGisObject(
        Rectangle(vertex, integrationLength), vertexIds);

    typedef vector<VertexIdType>::const_iterator IterType;

    for(IterType iter = vertexIds.begin(); iter != vertexIds.end(); iter++) {
        const VertexIdType& vertexId = *iter;

        if (vertex.DistanceTo(vertices[vertexId].vertex) <= integrationLength) {
            return true;
        }
    }

    return false;
}



VertexIdType GisSubsystem::GetVertexId(const Vertex& baseVertex)
{
    Vertex vertex = baseVertex;

    if (positionInLatlongDegree) {
        vertex = vertex.ToXyVertex(
            latitudeOriginDegrees, longitudeOriginDegrees);
    }

    vector<VariantIdType> vertexIds;

    const double integrationLength = 0.01;//1cm

    spatialVertexMap.GetGisObject(
        Rectangle(vertex, integrationLength), vertexIds);

    typedef vector<VertexIdType>::const_iterator IterType;

    for(IterType iter = vertexIds.begin(); iter != vertexIds.end(); iter++) {
        const VertexIdType& vertexId = *iter;

        if (vertex.DistanceTo(vertices[vertexId].vertex) <= integrationLength) {
            return vertexId;
        }
    }

    VertexIdType vertexId = static_cast<VertexIdType>(vertices.size());

    vertices.push_back(GisVertex(vertex));
    spatialVertexMap.InsertVertex(vertex, vertexId);

    return vertexId;
}

bool GisSubsystem::IsVertexPos(const Vertex& baseVertex)
{
    Vertex vertex = baseVertex;

    if (positionInLatlongDegree) {
        vertex = vertex.ToXyVertex(
            latitudeOriginDegrees, longitudeOriginDegrees);
    }

    vector<VariantIdType> vertexIds;

    const double integrationLength = 0.01;//1cm

    spatialVertexMap.GetGisObject(
        Rectangle(vertex, integrationLength), vertexIds);

    typedef vector<VertexIdType>::const_iterator IterType;

    for(IterType iter = vertexIds.begin(); iter != vertexIds.end(); iter++) {
        const VertexIdType& vertexId = *iter;

        if (vertex.DistanceTo(vertices[vertexId].vertex) <= integrationLength) {
            return true;
        }
    }

    return false;
}


void GisSubsystem::SynchronizeTopology(const SimTime& currentTime)
{
    changedGisTopology = false;

    insiteGeometryPtr->SyncMovingObjectTime(currentTime);

    buildingLayerPtr->SyncMovingObjectTime(currentTime);

    roadLayerPtr->SyncTrafficLight(currentTime);

    if (!executeEventsManually) {
        (*this).ExecuteEvents(currentTime);
        //cout << "SynchronizeTopology" << endl;//呼び出しあり
    }
}



void GisSubsystem::AddGisChangeEventHandler(
    const string& instanceId,
    const shared_ptr<GisChangeEventHandler>& initGisChangeEventHandlerPtr)
{
    gisChangeEventHandlerPtrs[instanceId] = initGisChangeEventHandlerPtr;
}



void GisSubsystem::DeleteGisChangeEventHandler(
    const string& instanceId)
{
    gisChangeEventHandlerPtrs.erase(instanceId);
}



void GisSubsystem::ExecuteEvents(const SimTime& currentTime)
{
    bool changed = false;

    while (!gisEventInfos.empty() &&
           gisEventInfos.top().eventTime <= currentTime) {

        shared_ptr<SimulationEvent> eventPtr = gisEventInfos.top().eventPtr;

        gisEventInfos.pop();

        eventPtr->ExecuteEvent();
        //cout << "ExecuteEvents" << endl;//呼び出しなし

        changed = true;
    }

    if (changed) {
        typedef map<string, shared_ptr<GisChangeEventHandler> >::const_iterator IterType;

        for(IterType iter = gisChangeEventHandlerPtrs.begin();
            iter != gisChangeEventHandlerPtrs.end(); iter++) {

            (*iter).second->GisInformationChanged();
        }
    }
}



GisObjectIdType GisSubsystem::CreateNewObjectId()
{
    reservedObjectId++;
    return reservedObjectId;
}



EntranceIdType GisSubsystem::CreateEntrance(const Vertex& vertex)
{
    const EntranceIdType entranceId = static_cast<EntranceIdType>(entrances.size());
    const GisObjectIdType objectId = (*this).CreateNewObjectId();
    const VertexIdType vertexId = (*this).GetVertexId(vertex);

    entrances.push_back(Entrance(this, objectId, entranceId, vertexId));

    //Entrance import/export feature
    //Folowing registration code may be necessary for very complicated GIS topology.

    //future// Entrance& entrance = entrances.back();
    //future// (*this).ConnectGisObject(vertexId, GIS_ENTRANCE, entranceId);
    //future// (*this).RegisterGisObject(entrance, entranceId);

    return entranceId;
}



void GisSubsystem::ImportEntrance(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder referedObjectIdFinder(hDBF, GIS_DBF_OBJECTID_STRING);

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        assert(shpObjPtr->nParts == 0);
        assert(shpObjPtr->nVertices == 1);

        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const double x = shpObjPtr->padfX[0];
        const double y = shpObjPtr->padfY[0];
        double z = shpObjPtr->padfZ[0];

        if (ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId)) {
            z += (*this).GetGroundElevationMetersAt(x, y);
        }

        Vertex point(x, y, z);

        const EntranceIdType entranceId = static_cast<EntranceIdType>(entrances.size());

        GisObjectIdType referedObjectId;
        bool foundLocation = true;

        if (referedObjectIdFinder.IsAvailable()) {
            referedObjectId = referedObjectIdFinder.GetGisObjectId(entryId);

            if (objectIdMap.find(referedObjectId) != objectIdMap.end()) {
                const GisPositionIdType positionId = (*this).GetPositionId(referedObjectId);

                switch (positionId.type) {
                case GIS_PARK: point = parkLayerPtr->GetPark(positionId.id).AddEntrance(entranceId, point); break;
                case GIS_BUILDING: point = buildingLayerPtr->GetBuilding(positionId.id).AddEntrance(entranceId, point); break;
                case GIS_RAILROAD_STATION: point = railRoadLayerPtr->GetStation(positionId.id).AddEntrance(entranceId, point); break;
                case GIS_BUSSTOP: point = roadLayerPtr->GetBusStop(positionId.id).AddEntrance(entranceId, point); break;
                default:
                    foundLocation = false;
                break;
                }
            } else {
                foundLocation = false;
            }
        } else {
            foundLocation = false;
        }

        if (!foundLocation) {
            vector<RailRoadStationIdType> stationIds;
            vector<ParkIdType> parkIds;
            vector<BuildingIdType> buildingIds;

            (*this).GetSpatialIntersectedGisObjectIds(point, GIS_PARK, parkIds);
            (*this).GetSpatialIntersectedGisObjectIds(point, GIS_BUILDING, buildingIds);
            (*this).GetSpatialIntersectedGisObjectIds(point, GIS_RAILROAD_STATION, stationIds);

            typedef vector<VariantIdType>::const_iterator IterType;

            for(IterType iter = parkIds.begin(); iter != parkIds.end(); iter++) {
                Park& park = parkLayerPtr->GetPark(*iter);
                if (PolygonContainsPoint(park.GetPolygon(), point)) {
                    point = park.AddEntrance(entranceId, point);
                }
            }
            for(IterType iter = buildingIds.begin(); iter != buildingIds.end(); iter++) {
                Building& building = buildingLayerPtr->GetBuilding(*iter);
                if (PolygonContainsPoint(building.GetBuildingPolygon(), point)) {
                    point = building.AddEntrance(entranceId, point);
                }
            }
            for(IterType iter = stationIds.begin(); iter != stationIds.end(); iter++) {
                RailRoadStation& station = railRoadLayerPtr->GetStation(*iter);
                if (PolygonContainsPoint(station.GetPolygon(), point)) {
                    point = station.AddEntrance(entranceId, point);
                }
            }
        }

        const VertexIdType& vertexId = (*this).GetVertexId(point);
        entrances.push_back(Entrance(this, objectId, entranceId, vertexId));
        Entrance& entrance = entrances.back();

        if (nameFinder.IsAvailable()) {
            entrance.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }

        (*this).ConnectGisObject(vertexId, GIS_ENTRANCE, entranceId);

        (*this).RegisterGisObject(entrance, entranceId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}


void GisSubsystem::OutputVertexInformation() const
{
    static const char* GIS_OBJECTTYPE_NAMES[] = {
        "Area",
        "Point",
        "Road",
        "Intersection",
        "RailRoad",
        "RailRoadIntersection",
        "RailRoadStation",
        "Park",
        "Wall",
        "Building",
        "PedestrianlPath",
        "BusStop",
        "Poi"
        "Poi"
        "TrafficLight"
        "Entrance"
    };

    for(size_t i = 0; i < vertices.size(); i++) {
        const GisVertex& gisVertex = vertices[i];

        typedef map<GisObjectType, vector<VertexConnection> >::const_iterator IterType;

        const map<GisObjectType, vector<VertexConnection> >& connections = gisVertex.connections;

        cout << "vertex" << i << "(" << gisVertex.vertex.x << "," << gisVertex.vertex.y << "," << gisVertex.vertex.z << ")";
        for(IterType iter = connections.begin(); iter != connections.end(); iter++) {
            const GisObjectType& objectType = (*iter).first;
            const vector<VertexConnection>& vertexConnections = (*iter).second;

            for(size_t j = 0; j < vertexConnections.size(); j++) {
                const VertexConnection& vertexConnection = vertexConnections[j];

                cout << " ";

                if (objectType < GisObjectType(sizeof(GIS_OBJECTTYPE_NAMES)/sizeof(GIS_OBJECTTYPE_NAMES[0]))) {
                    cout << GIS_OBJECTTYPE_NAMES[objectType];
                }

                cout << vertexConnection.variantId << "(" << vertexConnection.vertexId << ")";
            }
        }

        cout << endl;
    }
}


void GisSubsystem::OutputGisObjectVertexInformation() const
{
    const vector<shared_ptr<Road> >& roadPtrs = roadLayerPtr->GetRoadPtrs();

    for(size_t i = 0; i < roadPtrs.size(); i++) {
        const Road& road = *roadPtrs[i];

        cout << "Road" << i;
        for(size_t j = 0; j < road.NumberOfVertices(); j++) {
            const Vertex& v = road.GetVertex(j);

            cout << " (" << v.x << "," << v.y << "," << v.z << ")";
        }//for//

        cout << endl;
    }//for//

    const vector<Intersection>& intersections = roadLayerPtr->GetIntersections();

    for(size_t i = 0; i < intersections.size(); i++) {
        const Intersection& intersection = intersections[i];

        cout << "Intersection" << i;

        const Vertex& v = intersection.GetVertex();

        cout << " (" << v.x << "," << v.y << "," << v.z << ")" << endl;
    }//for//

    const vector<Building>& buildings = buildingLayerPtr->GetBuildings();

    for(size_t i = 0; i < buildings.size(); i++) {
        const Building& building = buildings[i];

        cout << "Building" << i;
        for(size_t j = 0; j < building.NumberOfVertices(); j++) {
            const Vertex& v = building.GetVertex(j);

            cout << " (" << v.x << "," << v.y << "," << v.z << ")";
        }//for//

        cout << endl;
    }//for//

    const vector<Park>& parks = parkLayerPtr->GetParks();

    for(size_t i = 0; i < parks.size(); i++) {
        const Park& park = parks[i];

        cout << "Park" << i;
        for(size_t j = 0; j < park.NumberOfVertices(); j++) {
            const Vertex& v = park.GetVertex(j);

            cout << " (" << v.x << "," << v.y << "," << v.z << ")";
        }//for//

        cout << endl;
    }//for//

    const vector<Area>& areas = areaLayerPtr->GetAreas();

    for(size_t i = 0; i < areas.size(); i++) {
        const Area& area = areas[i];

        cout << "Area" << i;
        for(size_t j = 0; j < area.NumberOfVertices(); j++) {
            const Vertex& v = area.GetVertex(j);

            cout << " (" << v.x << "," << v.y << "," << v.z << ")";
        }//for//

        cout << endl;
    }//for//

    const vector<Poi>& pois = poiLayerPtr->GetPois();

    for(size_t i = 0; i < pois.size(); i++) {
        const Poi& poi = pois[i];

        cout << "Poi" << i;

        const Vertex& v = poi.GetVertex();

        cout << " (" << v.x << "," << v.y << "," << v.z << ")" << endl;
    }//for//

    const vector<RailRoad>& railRoads = railRoadLayerPtr->GetRailRoads();

    for(size_t i = 0; i < railRoads.size(); i++) {
        const RailRoad& railRoad = railRoads[i];

        cout << "RailRoad" << i;
        for(size_t j = 0; j < railRoad.NumberOfVertices(); j++) {
            const Vertex& v = railRoad.GetVertex(j);

            cout << " (" << v.x << "," << v.y << "," << v.z << ")";
        }//for//

        cout << endl;
    }//for//

    const vector<RailRoadStation>& railRoadStations = railRoadLayerPtr->GetStations();

    for(size_t i = 0; i < railRoadStations.size(); i++) {
        const RailRoadStation& railRoadStation = railRoadStations[i];

        cout << "RailRoadStation" << i;

        const Vertex& v = railRoadStation.GetVertex();

        cout << " (" << v.x << "," << v.y << "," << v.z << ")" << endl;
    }//for//

    const vector<RailRoadIntersection>& railRoadIntersections = railRoadLayerPtr->GetIntersections();

    for(size_t i = 0; i < railRoadIntersections.size(); i++) {
        const RailRoadIntersection& railRoadIntersection = railRoadIntersections[i];

        cout << "RailRoadIntersection" << i;
        for(size_t j = 0; j < railRoadIntersection.NumberOfVertices(); j++) {
            const Vertex& v = railRoadIntersection.GetVertex(j);

            cout << " (" << v.x << "," << v.y << "," << v.z << ")";
        }//for//

        cout << endl;
    }//for//

    const vector<TrafficLight>& trafficLights = roadLayerPtr->GetTrafficLights();

    for(size_t i = 0; i < trafficLights.size(); i++) {
        const TrafficLight& trafficLight = trafficLights[i];

        cout << "TrafficLight" << i;;
        for(size_t j = 0; j < trafficLight.NumberOfVertices(); j++) {
            const Vertex& v = trafficLight.GetVertex(j);

            cout << " (" << v.x << "," << v.y << "," << v.z << ")";
        }//for//

        cout << endl;
    }//for//

    const vector<BusStop>& busStops = roadLayerPtr->GetBusStops();

    for(size_t i = 0; i < busStops.size(); i++) {
        const BusStop& busStop = busStops[i];

        cout << "BusStop" << i;

        const Vertex& v = busStop.GetVertex();

        cout << " (" << v.x << "," << v.y << "," << v.z << ")" << endl;
    }//for//

    const vector<Wall>& walls = buildingLayerPtr->GetWalls();

    for(size_t i = 0; i < walls.size(); i++) {
        const Wall& wall = walls[i];

        cout << "Wall" << i;
        for(size_t j = 0; j < wall.NumberOfVertices(); j++) {
            const Vertex& v = wall.GetVertex(j);

            cout << " (" << v.x << "," << v.y << "," << v.z << ")";
        }//for//

        cout << endl;
    }//for//

    for(size_t i = 0; i < entrances.size(); i++) {
        const Entrance& entrance = entrances[i];

        cout << "Entrance" << i;

        const Vertex& v = entrance.GetVertex();

        cout << " (" << v.x << "," << v.y << "," << v.z << ")" << endl;
    }//for//
}


void GisSubsystem::SetEnabled(
    const GisObjectType& objectType,
    const VariantIdType& variantId,
    const bool isEnable)
{
    changedGisTopology = true;

    switch (objectType) {
    case GIS_ROAD: roadLayerPtr->GetRoad(variantId).SetEnabled(isEnable); break;
    case GIS_PARK: parkLayerPtr->GetPark(variantId).SetEnabled(isEnable); break;
    case GIS_AREA: areaLayerPtr->GetArea(variantId).SetEnabled(isEnable); break;
    case GIS_POINT: poiLayerPtr->GetPoint(variantId).SetEnabled(isEnable); break;
    case GIS_INTERSECTION: roadLayerPtr->GetIntersection(variantId).SetEnabled(isEnable); break;
    case GIS_RAILROAD: railRoadLayerPtr->GetRailRoad(variantId).SetEnabled(isEnable); break;
    case GIS_RAILROAD_INTERSECTION: railRoadLayerPtr->GetRailRoadIntersection(variantId).SetEnabled(isEnable); break;
    case GIS_RAILROAD_STATION: railRoadLayerPtr->GetStation(variantId).SetEnabled(isEnable); break;
    case GIS_TRAFFICLIGHT: roadLayerPtr->GetTrafficLight(variantId).SetEnabled(isEnable); break;
    case GIS_POI: poiLayerPtr->GetPoi(variantId).SetEnabled(isEnable); break;
    case GIS_BUILDING: buildingLayerPtr->GetBuilding(variantId).SetEnabled(isEnable); break;
    case GIS_WALL: buildingLayerPtr->GetWall(variantId).SetEnabled(isEnable); break;
    case GIS_BUSSTOP: roadLayerPtr->GetBusStop(variantId).SetEnabled(isEnable); break;
    case GIS_ENTRANCE: (*this).GetEntrance(variantId).SetEnabled(isEnable); break;

    default:
        assert(false);
        break;
    }
}



void GisSubsystem::EnableMovingObjectIfNecessary(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeId& theNodeId,
    const SimTime& currentTime,
    const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
    const string& stationTypeString)
{
    if (theParameterDatabaseReader.ParameterExists("object-shape-type", theNodeId)) {
        string shapeType = theParameterDatabaseReader.ReadString("object-shape-type", theNodeId);
        ConvertStringToLowerCase(shapeType);

        buildingLayerPtr->AddMovingObject(materials, theNodeId, mobilityModelPtr, shapeType);
    }

    insiteGeometryPtr->AddObject(
        theParameterDatabaseReader,
        theNodeId,
        currentTime,
        mobilityModelPtr,
        stationTypeString);
}

void GisSubsystem::OvtputCurrentEntranceShapeFile(const string& shapeDirName)
{
    const int shapeType = SHPT_POINTZ;

    using boost::filesystem::path;

    const path entranceFilePath = path(shapeDirName) / "entrance";
    const string entranfeFileName = entranceFilePath.string();

    SHPHandle hSHP = SHPCreate(entranfeFileName.c_str(), SHPT_POINTZ);
    DBFHandle hDBF = DBFCreate(entranfeFileName.c_str());

    if ((hSHP == nullptr) || (hDBF == nullptr)) {
        cerr << "Error: Failed to open " << entranceFilePath << endl;
        exit(1);
    }//if//

    const int idFieldId = 0;
    const int nameFieldId = 1;
    const int objectIdFieldId = 2;

    const int idFieldLength = 10;
    const int nameFieldLength = 128;

    if (DBFAddField(hDBF, GIS_DBF_ID_STRING.c_str(), FTInteger, idFieldLength, 0) == -1) {
        cerr << "Error: Failed to add atrribute \"" << GIS_DBF_ID_STRING << "\"" << endl;
        exit(1);
    }//if//

    if (DBFAddField(hDBF, GIS_DBF_NAME_STRING.c_str(), FTString, nameFieldLength, 0) == -1) {
        cerr << "Error: Failed to add atrribute \"" << GIS_DBF_NAME_STRING << "\"" << endl;
        exit(1);
    }//if//

    if (DBFAddField(hDBF, GIS_DBF_OBJECTID_STRING.c_str(), FTInteger, idFieldLength, 0) == -1) {
        cerr << "Error: Failed to add atrribute \"" << GIS_DBF_OBJECTID_STRING << "\"" << endl;
        exit(1);
    }//if//

    const GisObjectIdType GISOBJECT_ENTRANCE_START_ID = 107000001;

    map<EntranceIdType, GisObjectIdType> entranceIdToGisObjectIdMap;

    (*this).CalculateEntranceIdToGisObjectIdMap(entranceIdToGisObjectIdMap);

    int entranceNumber = 0;

    for(size_t i = 0; i < entrances.size(); i++) {
        const EntranceIdType entranceId = static_cast<EntranceIdType>(i);
        const Entrance& entrance = entrances[i];

        if (entranceIdToGisObjectIdMap.find(entranceId) == entranceIdToGisObjectIdMap.end()) {
            continue;
        }//if//

        const GisObjectIdType baseGisObjectId = entranceIdToGisObjectIdMap[entranceId];
        Vertex position = entrance.GetVertex();

        SHPObject* shpObj = SHPCreateSimpleObject(
            shapeType, 1,
            &position.x,
            &position.y,
            &position.z);

        assert(shpObj != nullptr);

        //add id
        DBFWriteIntegerAttribute(hDBF, entranceNumber, idFieldId, entranceNumber + GISOBJECT_ENTRANCE_START_ID);

        DBFWriteStringAttribute(hDBF, entranceNumber, nameFieldId, entrance.GetObjectName().c_str());

        DBFWriteIntegerAttribute(hDBF, entranceNumber, objectIdFieldId, baseGisObjectId);

        SHPWriteObject(hSHP, -1, shpObj);
        SHPDestroyObject(shpObj);

        entranceNumber++;
    }//for//

    SHPClose(hSHP);
    DBFClose(hDBF);

}//OvtputCurrentEntranceShapeFile//


void GisSubsystem::CalculateEntranceIdToGisObjectIdMap(
    map<EntranceIdType, GisObjectIdType>& entranceIdToGisObjectIdMap)
{
    entranceIdToGisObjectIdMap.clear();

    const vector<Park>& parks = parkLayerPtr->GetParks();

    typedef set<EntranceIdType>::const_iterator IterType;

    for(size_t i = 0; i < parks.size(); i++) {
        const Park& park = parks[i];
        const set<EntranceIdType>& entranceIds = park.GetEntranceIds();

        for(IterType iter = entranceIds.begin();
            (iter != entranceIds.end()); iter++) {

            entranceIdToGisObjectIdMap[*iter] = park.GetObjectId();
        }//for//
    }//for//

    const vector<Building>& buildings = buildingLayerPtr->GetBuildings();

    for(size_t i = 0; i < buildings.size(); i++) {
        const Building& building = buildings[i];
        const set<EntranceIdType>& entranceIds = building.GetEntranceIds();

        for(IterType iter = entranceIds.begin();
            (iter != entranceIds.end()); iter++) {

            entranceIdToGisObjectIdMap[*iter] = building.GetObjectId();
        }//for//
    }//for//

    const vector<RailRoadStation>& stations = railRoadLayerPtr->GetStations();

    for(size_t i = 0; i < stations.size(); i++) {
        const RailRoadStation& station = stations[i];
        const set<EntranceIdType>& entranceIds = station.GetEntranceIds();

        for(IterType iter = entranceIds.begin();
            (iter != entranceIds.end()); iter++) {

            entranceIdToGisObjectIdMap[*iter] = station.GetObjectId();
        }//for//
    }//for//

    const vector<BusStop>& busStops = roadLayerPtr->GetBusStops();

    for(size_t i = 0; i < busStops.size(); i++) {
        const BusStop& busStop = busStops[i];
        const set<EntranceIdType>& entranceIds = busStop.GetEntranceIds();

        for(IterType iter = entranceIds.begin();
            (iter != entranceIds.end()); iter++) {

            entranceIdToGisObjectIdMap[*iter] = busStop.GetObjectId();
        }//for//
    }//for//

}//CalculateEntranceIdToGisObjectIdMap//

void GisSubsystem::OvtputCurrentRoadShapeFile(const string& shapeDirName)
{
    using boost::filesystem::path;

    const path shapeFilePath = path(shapeDirName) / "road";

    roadLayerPtr->ExportRoad(shapeFilePath.string());
}//OvtputCurrentRoadShapeFile//

void GisSubsystem::OvtputCurrentIntersectionShapeFile(const string& shapeDirName)
{
    using boost::filesystem::path;

    const path shapeFilePath = path(shapeDirName) / "intersection";

    roadLayerPtr->ExportIntersection(shapeFilePath.string());
}//OvtputCurrentIntersectionShapeFile//


void GisSubsystem::GetMainRoadAndVertexConnectedToEntrance(
    const EntranceIdType entranceId,
    const double& minRoadLengthMeters,
    RoadIdType& mainRoadId,
    VertexIdType& mainRoadVertexId,
    Vertex& mainRoadVertex) const
{
    const VertexIdType entranceVertexId = (*this).GetEntrance(entranceId).GetVertexId();
    const GisVertex& entranceGisVertex = (*this).GetGisVertex(entranceVertexId);
    const vector<RoadIdType>& roadIds = entranceGisVertex.GetConnectedObjectIds(GIS_ROAD);

    mainRoadId = InvalidVariantId;
    mainRoadVertexId = InvalidVertexId;

    // This code assumes special "Entrance Road" that can be quite short or zero length.

    for(size_t i = 0; i < roadIds.size(); i++) {
        const RoadIdType& roadId = roadIds[i];
        const Road& aRoad = (*this).GetRoad(roadId);

        if (!aRoad.IsParking()) {
            if (aRoad.GetArcDistanceMeters() >= minRoadLengthMeters) {
                mainRoadId = roadId;
                mainRoadVertexId = entranceVertexId;
            }
            else {
                assert(aRoad.NumberOfVertices() == 2);

                if (aRoad.GetStartVertexId() != entranceVertexId) {
                    assert(aRoad.GetEndVertexId() == entranceVertexId);
                    mainRoadVertexId = aRoad.GetStartVertexId();
                    mainRoadVertex = aRoad.GetStartVertex();
                }
                else {
                    assert(aRoad.GetStartVertexId() == entranceVertexId);
                    mainRoadVertexId = aRoad.GetEndVertexId();
                    mainRoadVertex = aRoad.GetEndVertex();
                }//if//

                const GisVertex& mainRoadGisVertex =
                    (*this).GetGisVertex(mainRoadVertexId);

                const vector<RoadIdType>& nearRoadIds = mainRoadGisVertex.GetConnectedObjectIds(GIS_ROAD);

                double longestRoadLength = 0.0;
                RoadIdType longestRoadId = InvalidVariantId;

                for(size_t j = 0; j < nearRoadIds.size(); j++) {
                    const RoadIdType& nearRoadId = nearRoadIds[j];
                    const Road& nearRoad = (*this).GetRoad(nearRoadId);
                    const double roadLengthMeters = nearRoad.GetArcDistanceMeters();

                    if (roadLengthMeters > longestRoadLength) {
                        longestRoadLength = roadLengthMeters;
                        mainRoadId = nearRoadId;
                    }//if//
                }//for//
            }//if//
        }//if//
    }//for//

    assert(mainRoadId != InvalidVariantId);

}//GetNearestMainRoadIdToEntrance//



}; //namespace ScenSim
