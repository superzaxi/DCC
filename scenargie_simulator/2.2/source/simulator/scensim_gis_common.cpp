// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scensim_gis.h"
#include "scensim_tracedefs.h"

namespace ScenSim {


const size_t SpatialObjectMap::MAX_MESH_SIZE = 1000;
const size_t SpatialObjectMap::MAX_VERTEX_MESH_SIZE = 2000;

static const double DEFAULT_MESH_UNIT_METERS = 1000.0;

static const double MAX_STATION_SEARCH_AREA_METERS = 5000;
static const double MAX_INTERSECTION_SEARCH_AREA_METERS = 1000;


enum ShapeType {
    POINT_TYPE,
    ARC_TYPE,
    POLYGON_TYPE
};


string TrimmedString(const string& aString)
{
    string trimmedString = aString;

    size_t pos = trimmedString.find_last_not_of(" \t");
    if (pos == string::npos) {
        trimmedString.clear();
    } else if (pos < (aString.size() - 1)) {
        trimmedString.erase(pos+1);
    }

    pos = trimmedString.find_first_not_of(" \t");

    if (pos != string::npos) {
        trimmedString = trimmedString.substr(pos);
    }
    else {
        // No trailing spaces => don't modify string.
    }

    return trimmedString;
}



void TokenizeToTrimmedLowerString(
    const string& aString,
    const string& deliminator,
    deque<string>& tokens,
    const bool skipEmptyToken)
{
    tokens.clear();

    size_t posOfReading = 0;
    size_t posOfDelim = aString.find_first_of(deliminator);

    while (posOfDelim != string::npos) {

        const string token =
            TrimmedString(aString.substr(posOfReading, posOfDelim - posOfReading));

        if (!skipEmptyToken || !token.empty()) {
            tokens.push_back(token);

            ConvertStringToLowerCase(tokens.back());
        }

        posOfReading = posOfDelim + 1;
        posOfDelim = aString.find_first_of(deliminator, posOfReading);
    }

    const size_t lastToenPos =
        aString.find_first_not_of(" ", posOfReading);

    if (lastToenPos != string::npos) {
        const string lastOneToken = aString.substr(
            lastToenPos,
            aString.find_last_not_of(" ") + 1);

        if (!skipEmptyToken || !lastOneToken.empty()) {
            tokens.push_back(lastOneToken);

            ConvertStringToLowerCase(tokens.back());
        }
    }
}

//----------------------------------------------------------
// LoS
//----------------------------------------------------------

void LosMovingObject::UpdateMovingObjectMobilityPosition(
    const GroundLayer& groundLayer,
    const SimTime& currentTime,
    bool& positionChanged)
{
    ObjectMobilityPosition currentMobilityPosition;
    mobilityModelPtr->GetPositionForTime(currentTime, currentMobilityPosition);

    if (lastMobilityPosition.X_PositionMeters() == currentMobilityPosition.X_PositionMeters() &&
        lastMobilityPosition.Y_PositionMeters() == currentMobilityPosition.Y_PositionMeters() &&
        lastMobilityPosition.HeightFromGroundMeters() == currentMobilityPosition.HeightFromGroundMeters() &&
        lastMobilityPosition.AttitudeAzimuthFromNorthClockwiseDegrees() == currentMobilityPosition.AttitudeAzimuthFromNorthClockwiseDegrees() &&
        lastMobilityPosition.AttitudeElevationFromHorizonDegrees() == currentMobilityPosition.AttitudeElevationFromHorizonDegrees()) {
        positionChanged = false;
        return;
    }

    positionChanged = true;

    lastMobilityPosition = currentMobilityPosition;

    (*this).ReconstructLosPolygon(groundLayer);
}



void LosMovingObject::ReconstructLosPolygon(const GroundLayer& groundLayer)
{
    Vertex baseVertex = MakeVertexFromMobilityPosition(lastMobilityPosition);

    // Adjust ground height if necessary
    if (!lastMobilityPosition.TheHeightContainsGroundHeightMeters()) {
        baseVertex.z += groundLayer.GetElevationMetersAt(baseVertex);
    }

    const double azimuthDegrees =
        lastMobilityPosition.AttitudeAzimuthFromNorthClockwiseDegrees();
    const double elevationDegrees =
        lastMobilityPosition.AttitudeElevationFromHorizonDegrees();

    const RotationMatrix rotationMatrix(-azimuthDegrees, 0., elevationDegrees);

    *losObbPtr = LosOrientedBoundingBox(
        baseVertex, rotationMatrix, length, width, height, theNodeId, shieldingLossDb);
}



inline
void JudgeLinePlane(
    const LosRay& ray,
    const LosPolygon& a,
    double& t)
{
    t = -1;

    const Vertex normalDir = ray.dir.Normalized();
    const Vertex e1 = a.triangle.GetP2() - a.triangle.GetP1();
    const Vertex e2 = a.triangle.GetP3() - a.triangle.GetP1();
    const Vertex pvec = normalDir.Cross(e2);
    const double det = e1.Dot(pvec);

    if (det > -DBL_EPSILON && det < DBL_EPSILON) {
        return;
    }

    const double invDet = 1.0 / det;
    const Vertex tvec = ray.orig - a.triangle.GetP1();
    const double u = tvec.Dot(pvec)*invDet;

    if (u < 0.0 || u > 1.0) {
        return;
    }

    const Vertex qvec = tvec.Cross(e1);
    const double v = normalDir.Dot(qvec)*invDet;

    if (v < 0.0 || u + v > 1.0) {
        return;
    }

    t = (e2.Dot(qvec) * invDet) / ray.dir.Distance();
}



inline
void JudgeLineOrientedBoundingBox(
    const LosRay& ray,
    const LosOrientedBoundingBox& obb,
    double& outTmin,
    double& outTmax)
{
    outTmin = 0.;
    outTmax = DBL_MAX;

    double minT = -DBL_MAX;
    double maxT = DBL_MAX;

    const Vertex dirToCenter = obb.center - ray.orig;

    for(int i = 0; i < 3; i++) {
        const Vertex& axisDirection = obb.rotationMatrix.cv[i];
        const double e = axisDirection.Dot(dirToCenter);
        const double f = ray.dir.Dot(axisDirection);
        const double extent = obb.extentPerAxis[i];

        // ray is parallel
        if (f > -DBL_EPSILON && f < DBL_EPSILON) {

            // passing outside of OBB
            if (std::fabs(extent) < std::fabs(e))  {
                outTmin = -DBL_MAX;
                outTmax = DBL_MAX;
                return;
            }
            continue;
        }

        double t1 = (e - extent)/f;
        double t2 = (e + extent)/f;
        if (t1 > t2) {
            std::swap(t1, t2);
        }

        assert(t1 <= t2);
        minT = std::max(minT, t1); //close
        maxT = std::min(maxT, t2); //far

        if (maxT <= 0 || minT > maxT) {
            outTmin = -DBL_MAX;
            outTmax = DBL_MAX;
            return;
        }
    }

    outTmin = minT;
    outTmax = maxT;
}



LosQuadTree::~LosQuadTree()
{
    for(int i = 0; i < 4; i++) {
        delete child[i];
    }
}



void LosQuadTree::PushLosPolygon(const LosPolygon& wall)
{
    if (depth < MAXDEPTH) {
        const Triangle& triangle = wall.triangle;
        const QuadrantType p1Quadrant = place.GetQuadrantOf(triangle.GetP1());
        const QuadrantType p2Quadrant = place.GetQuadrantOf(triangle.GetP2());
        const QuadrantType p3Quadrant = place.GetQuadrantOf(triangle.GetP3());

        if ((p1Quadrant == p2Quadrant) &&
            (p2Quadrant == p3Quadrant)) {
            // Put polygon to a more deep child rectangle

            (*this).PushLosPolygonToChild(p1Quadrant, wall);
        }
        else {
            losPolygons.push_back(wall);
        }
    }
    else {
        losPolygons.push_back(wall);
    }
}



void LosQuadTree::PushLosOrientedBoundingBox(
    const shared_ptr<LosOrientedBoundingBox>& obbPtr,
    LosQuadTree*& insertedTreePtr)
{
    if (depth < MAXDEPTH) {
        const Rectangle& rect = obbPtr->GetRect();
        const QuadrantType bottomLeftQuadrant = place.GetQuadrantOf(rect.GetBottomLeft());
        const QuadrantType topRightQuadrant = place.GetQuadrantOf(rect.GetTopRight());

        if (bottomLeftQuadrant == topRightQuadrant) {
            // Put obb to a more deep child rectangle

            (*this).PushLosOrientedBoundingBoxToChild(bottomLeftQuadrant, obbPtr, insertedTreePtr);
        }
        else {
            insertedTreePtr = this;
            losObbPtrs.push_back(obbPtr);
        }
    }
    else {
        insertedTreePtr = this;
        losObbPtrs.push_back(obbPtr);
    }
}



void LosQuadTree::PushLosPolygonToChild(
    const QuadrantType& childQuadrant,
    const LosPolygon& wall)
{
    if (child[childQuadrant] == nullptr) {
        const Rectangle childPlace = place.MakeChildRect(childQuadrant);

        child[childQuadrant] = new LosQuadTree(childPlace, depth + 1);
    }

    child[childQuadrant]->PushLosPolygon(wall);
}



void LosQuadTree::PushLosOrientedBoundingBoxToChild(
    const QuadrantType& childQuadrant,
    const shared_ptr<LosOrientedBoundingBox>& obbPtr,
    LosQuadTree*& insertedTreePtr)
{
    if (child[childQuadrant] == nullptr) {
        const Rectangle childPlace = place.MakeChildRect(childQuadrant);

        child[childQuadrant] = new LosQuadTree(childPlace, depth + 1);
    }

    child[childQuadrant]->PushLosOrientedBoundingBox(obbPtr, insertedTreePtr);
}



void LosQuadTree::CheckChildCollision(
    const QuadrantType& childQuadrant,
    const LosRay& ray,
    const set<NodeId>& ignoredNodeIds,
    const bool checkJustACollsion,
    const bool isJustHorizontalCheck,
    map<double, WallCollisionInfoType>& collisions)
{
    if (child[childQuadrant] == nullptr) {
        // no collision
        return;
    }

    child[childQuadrant]->CheckCollision(ray, ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);
}



inline
bool QuadrantXIsSame(const QuadrantType& q1, const QuadrantType& q2)
{
    if (q1 == q2) {
        return true;
    }

    return ((int(q1) + int(q2)) == 3);
}



inline
bool QuadrantYIsSame(const QuadrantType& q1, const QuadrantType& q2)
{
    if (q1 == q2) {
        return true;
    }

    return (((int(q1) + int(q2)) == 1) ||
            ((int(q1) + int(q2)) == 5));
}



void LosQuadTree::CheckCollision(
    const LosRay& ray,
    const set<NodeId>& ignoredNodeIds,
    const bool checkJustACollsion,
    const bool isJustHorizontalCheck,
    map<double, WallCollisionInfoType>& collisions)
{
    if (checkJustACollsion && !collisions.empty()) {
        // already found a collision.
        return;
    }

    // check child place collision
    if (depth < MAXDEPTH) {
        const QuadrantType origQuadrant = place.GetQuadrantOf(ray.orig);
        const QuadrantType destQuadrant = place.GetQuadrantOf(ray.dest);

        if (origQuadrant == destQuadrant) {

            (*this).CheckChildCollision(origQuadrant, ray, ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);

        } else {
            const Vertex center = place.GetCenter();
            const Vertex edgeX = ray.CrossPointX(center.x);
            const Vertex edgeY = ray.CrossPointY(center.y);

            if (QuadrantXIsSame(origQuadrant, destQuadrant)) {
                // divide points by horizontal Y line
                (*this).CheckChildCollision(origQuadrant, LosRay(ray.orig, edgeY), ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);
                (*this).CheckChildCollision(destQuadrant, LosRay(edgeY, ray.dest), ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);
            }
            else if (QuadrantYIsSame(origQuadrant, destQuadrant)) {
                // divide points by vertical X line
                (*this).CheckChildCollision(origQuadrant, LosRay(ray.orig, edgeX), ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);
                (*this).CheckChildCollision(destQuadrant, LosRay(edgeX, ray.dest), ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);
            }
            else { // pass 3-place (2-place for the center point passing)

                // divide points by X and Y
                Vertex origEdge;
                Vertex destEdge;

                if (ray.orig.DistanceTo(edgeX) <= ray.orig.DistanceTo(edgeY)) {
                    origEdge = edgeX;
                    destEdge = edgeY;
                } else {
                    origEdge = edgeY;
                    destEdge = edgeX;
                }

                (*this).CheckChildCollision(origQuadrant, LosRay(ray.orig, origEdge), ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);
                (*this).CheckChildCollision(destQuadrant, LosRay(destEdge, ray.dest), ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);

                // If edges are not on center point, check third place.
                if (origEdge != destEdge) {
                    const QuadrantType middleQuadrant = place.GetQuadrantOf((origEdge + destEdge) * 0.5);

                    if (middleQuadrant != origQuadrant &&
                        middleQuadrant != destQuadrant) {
                        (*this).CheckChildCollision(middleQuadrant, LosRay(origEdge, destEdge), ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);
                    }
                }
            }
        }
    }

    (*this).CheckLosPolygonCollision(ray, ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);

    (*this).CheckLosOrientedBoundingBoxCollision(ray, ignoredNodeIds, checkJustACollsion, collisions);
}



inline
void InsertCollisionPoint(
    const double t,
    const VariantIdType& variantId,
    const double& shieldingLossDb,
    const LosPolygon::ObstructionType& anObstructionType,
    map<double, WallCollisionInfoType>& collisions)
{
    if (0.0 < t && t < 1.0) {
        const int roundInt = 100000000;

        const double roundedT =  floor(t * roundInt) / double(roundInt);
        const WallCollisionInfoType collisionData(variantId, shieldingLossDb, anObstructionType);

        typedef map<double, WallCollisionInfoType>::iterator IterType;

        IterType iter = collisions.find(roundedT);

        if (iter != collisions.end()) {
            const VariantIdType& dataId = (*iter).second.variantId;

            // update by low id data
            if (collisionData.variantId < dataId) {
                (*iter).second = collisionData;
            }

        } else {

            collisions.insert(make_pair(roundedT, collisionData));
        }
    }
}



void LosQuadTree::CheckLosPolygonCollision(
    const LosRay& ray,
    const set<NodeId>& ignoredNodeIds,
    const bool checkJustACollsion,
    const bool isJustHorizontalCheck,
    map<double, WallCollisionInfoType>& collisions)
{
    if (checkJustACollsion && !collisions.empty()) {
        // already found a collision.
        return;
    }

    typedef vector<LosPolygon>::iterator IterType;

    // check own place collision
    for(IterType iter = losPolygons.begin(); iter != losPolygons.end(); iter++) {
        const LosPolygon& losPolygon = (*iter);

        // ignore this polygon
        if (ignoredNodeIds.find(losPolygon.theNodeId) != ignoredNodeIds.end()) {
            continue;
        }

        if (isJustHorizontalCheck) {
            assert(ray.orig.z == 0.0);
            assert(ray.dest.z == 0.0);

            if ((losPolygon.anObstructionType == LosPolygon::Roof) &&
                losPolygon.triangle.IntersectsWithLine(ray.orig, ray.dest)) {

                const double distance = ray.dir.Distance();

                vector<Vertex> intersectionPoints;

                intersectionPoints.push_back(
                    CalculateIntersectionPositionBetweenLine(
                        ray.orig, ray.dest,
                        losPolygon.triangle.GetP1().XYPoint(),
                        losPolygon.triangle.GetP2().XYPoint()));

                intersectionPoints.push_back(
                    CalculateIntersectionPositionBetweenLine(
                        ray.orig, ray.dest,
                        losPolygon.triangle.GetP2().XYPoint(),
                        losPolygon.triangle.GetP3().XYPoint()));

                intersectionPoints.push_back(
                    CalculateIntersectionPositionBetweenLine(
                        ray.orig, ray.dest,
                        losPolygon.triangle.GetP3().XYPoint(),
                        losPolygon.triangle.GetP1().XYPoint()));

                for(size_t i = 0; i < intersectionPoints.size(); i++) {

                    const double t = (intersectionPoints[i] - ray.orig).Distance() / distance;

                    InsertCollisionPoint(
                        t, losPolygon.variantId, losPolygon.shieldingLossDb, losPolygon.anObstructionType,
                        collisions);
                }
            }

        } else {
            double t;
            JudgeLinePlane(ray, losPolygon, t);

            InsertCollisionPoint(
                t, losPolygon.variantId, losPolygon.shieldingLossDb, losPolygon.anObstructionType,
                collisions);
        }
    }
}



void LosQuadTree::CheckLosOrientedBoundingBoxCollision(
    const LosRay& ray,
    const set<NodeId>& ignoredNodeIds,
    const bool checkJustACollsion,
    map<double, WallCollisionInfoType>& collisions)
{
    if (checkJustACollsion && !collisions.empty()) {
        // already found a collision.
        return;
    }

    typedef vector<shared_ptr<LosOrientedBoundingBox> >::iterator IterType;

    // check own place collision
    for(IterType iter = losObbPtrs.begin(); iter != losObbPtrs.end(); iter++) {
        const LosOrientedBoundingBox& obb = *(*iter);

        // ignore this obb
        if (ignoredNodeIds.find(obb.theNodeId) != ignoredNodeIds.end()) {
            continue;
        }

        double outTmin;
        double outTmax;

        JudgeLineOrientedBoundingBox(ray, obb, outTmin, outTmax);

        InsertCollisionPoint(
            outTmin, INVALID_GIS_OBJECT_ID, obb.shieldingLossDb, LosPolygon::Invalid,
            collisions);
        InsertCollisionPoint(
            outTmax, INVALID_GIS_OBJECT_ID, obb.shieldingLossDb, LosPolygon::Invalid,
            collisions);
    }
}



bool LosQuadTree::HasCollision(
    const LosRay& ray,
    const set<NodeId>& ignoredNodeIds)
{
    map<double, WallCollisionInfoType> collisions;

    (*this).CheckCollision(ray, ignoredNodeIds, true/*checkJustACollsion*/, false/*isJustHorizontalCheck*/, collisions);

    return (!collisions.empty());
}

//------------------------------------------------------------

enum CrossProductDirection {
    COROSS_PRODUCT_POSITIVE,
    COROSS_PRODUCT_NEGATIVE,
};


// ciclic list
struct PointData {
    list<PointData* >::iterator iterToFarthestList;

    Vertex position;

    PointData* prev;
    PointData* next;

    PointData() : prev(nullptr), next(nullptr) {}
};



typedef vector<PointData* >::iterator PointDataPtrIter;

class FarthestPoint {
public:
    FarthestPoint(const double initOffsetX, double initOffsetY)
        : offsetX(initOffsetX), offsetY(initOffsetY)
    {}

    bool operator()(const PointData* p1, const PointData* p2) const {
        return (((p1->position.x - offsetX) * (p1->position.x - offsetX) +
                 (p1->position.y - offsetY) * (p1->position.y - offsetY))
                >
                ((p2->position.x - offsetX) * (p2->position.x - offsetX) +
                 (p2->position.y - offsetY) * (p2->position.y - offsetY)));
    }
private:
    const double offsetX;
    const double offsetY;
};



// reinplementation for optimization

inline
CrossProductDirection Get2dCrossProductDirection(
    const Vertex& p1,
    const Vertex& p2,
    const Vertex& p3)
{
    const double d21 = p1.x - p2.x;
    const double d22 = p1.y - p2.y;

    const double d31 = p1.x - p3.x;
    const double d32 = p1.y - p3.y;

    if((d21*d32 - d22*d31) >= 0) {
        return COROSS_PRODUCT_POSITIVE;
    } else {
        return COROSS_PRODUCT_NEGATIVE;
    }
}



inline
bool IsInsideOfTheTriangle(
    const Vertex& checkPoint,
    const Vertex& p1,
    const Vertex& p2,
    const Vertex& p3)
{
    const CrossProductDirection d12 = Get2dCrossProductDirection(checkPoint, p1, p2);
    const CrossProductDirection d23 = Get2dCrossProductDirection(checkPoint, p2, p3);
    const CrossProductDirection d31 = Get2dCrossProductDirection(checkPoint, p3, p1);

    return ((d12 == d23) && (d12 == d31));
}



inline
bool IsInsideOf3dTheTriangle(
    const Vertex& checkPoint,
    const Vertex& v1,
    const Vertex& v2,
    const Vertex& v3)
{
    const Vertex v12 = v2 - v1;
    const Vertex v2p = checkPoint - v2;

    const Vertex v23 = v3 - v2;
    const Vertex v3p = checkPoint - v3;

    const Vertex v31 = v3 - v1;
    const Vertex v1p = checkPoint - v1;

    const Vertex c1 = v12.Cross(v2p);
    const Vertex c2 = v23.Cross(v3p);
    const Vertex c3 = v31.Cross(v1p);

    const double d12 = c1.Dot(c2);
    const double d13 = c1.Dot(c3);

    if(d12 > 0 && d13 > 0) {
        return true;
    }

    return false;
}



inline
bool IsInsideOfOtherPoint(const PointData& pointData)
{
    const PointData* prev = pointData.prev;

    for(const PointData* followingData = pointData.next->next;
        followingData != prev; followingData = followingData->next) {

        if(IsInsideOfTheTriangle(
               followingData->position,
               pointData.position,
               pointData.next->position,
               pointData.prev->position)) {
            return true;
        }
    }
    return false;
}



inline
Triangle MakeRightHandRuleTriangle(
    const Vertex& p1,
    const Vertex& p2,
    const Vertex& p3)
{
    const CrossProductDirection direction =
        Get2dCrossProductDirection(p2, p1, p3);

    if(direction == COROSS_PRODUCT_POSITIVE) {
        return Triangle(p3, p2, p1);
    } else {
        return Triangle(p1, p2, p3);
    }
}



inline
bool IsCompletePolygon(const vector<Vertex>& polygon)
{
    return (polygon.size() >= 4 && polygon.front() == polygon.back());
}



void PolygonToTriangles(const vector<Vertex>& polygon, vector<Triangle>& triangles)
{
    assert(IsCompletePolygon(polygon));

    vector<Vertex> minimalPolygon;
    minimalPolygon.push_back(polygon.front());
    for(size_t i = 0; i < polygon.size() - 1; i++) {
        if(minimalPolygon.back() != polygon[i]) {
            minimalPolygon.push_back(polygon[i]);
        }
    }

    const size_t numberVertices = minimalPolygon.size();
    if(numberVertices < 3) {
        return;
    }

    // prepare data sets
    PointData* pointDataChunk = new PointData[numberVertices];
    PointData* pointData;
    list<PointData*> farthests;

    const size_t lastVertexIndex = numberVertices - 1;
    for(size_t i = 0; i < lastVertexIndex; i++) {
        farthests.push_front(&pointDataChunk[i]);
        pointData = &pointDataChunk[i];
        pointData->position = minimalPolygon[i];
        pointData->iterToFarthestList = farthests.begin();
        pointData->next = &pointDataChunk[i + 1];
        pointDataChunk[i + 1].prev = pointData;
    }

    farthests.push_front(&pointDataChunk[lastVertexIndex]);
    pointData = &pointDataChunk[lastVertexIndex];
    pointData->position = minimalPolygon[lastVertexIndex];
    pointData->iterToFarthestList = farthests.begin();
    pointData->next = &pointDataChunk[0];
    pointDataChunk[0].prev = pointData;

    const PointData* firstPointData = &pointDataChunk[0];
    const PointData* lastPointData = &pointDataChunk[lastVertexIndex];

    for(const PointData* p1 = firstPointData; p1 != lastPointData; p1 = p1->next) {

        for(const PointData* p2 = p1->next; p2 != firstPointData; p2 = p2->next) {

            if(p1->position.x == p2->position.x &&
               p1->position.y == p2->position.y) {
                // illegal polygon
                delete [] pointDataChunk;
                return;
            }
        }
    }

    farthests.sort(
        FarthestPoint(
            minimalPolygon[0].x,
            minimalPolygon[0].y));


    while(farthests.size() > 3) {
        const PointData* startPointData = farthests.front();

        const CrossProductDirection crossProductDirection =
            Get2dCrossProductDirection(
                startPointData->position,
                startPointData->prev->position,
                startPointData->next->position);

        const PointData* currentPointData = startPointData;

        CrossProductDirection currentCrossProductDirection =
            crossProductDirection;

        while(true) {
            if(currentCrossProductDirection == crossProductDirection &&
               !IsInsideOfOtherPoint(*currentPointData)) {
                triangles.push_back(
                    MakeRightHandRuleTriangle(
                        currentPointData->position,
                        currentPointData->prev->position,
                        currentPointData->next->position));

                currentPointData->next->prev = currentPointData->prev;
                currentPointData->prev->next = currentPointData->next;
                farthests.erase(currentPointData->iterToFarthestList);
                break;
            }

            currentPointData = currentPointData->next;
            assert(currentPointData != nullptr);
            if(currentPointData == startPointData) {
                // no more available triangles.
                delete [] pointDataChunk;
                return;
            }

            currentCrossProductDirection =
                Get2dCrossProductDirection(
                    currentPointData->position,
                    currentPointData->prev->position,
                    currentPointData->next->position);
        }
    }
    assert(farthests.size() == 3);

    const PointData* basePoint = farthests.front();
    triangles.push_back(
        MakeRightHandRuleTriangle(
            basePoint->position,
            basePoint->next->position,
            basePoint->prev->position));

    delete [] pointDataChunk;
}


//------------------------------------------------------------



Vertex Vertex::Normalized() const
{
    const double distance = (*this).Distance();

    if (distance == 0) {
        return Vertex();
    }

    return Vertex(x/distance, y/distance, z/distance);
}



double Vertex::DistanceTo(const Vertex& vertex) const
{
    return XYZDistanceBetweenVertices(*this, vertex);
}



double Vertex::XYDistanceTo(const Vertex& vertex) const
{
    return XYDistanceBetweenVertices(*this, vertex);
}



Vertex Vertex::ToXyVertex(
    const double latitudeOriginDegrees,
    const double longitudeOriginDegrees) const
{
    return Vertex(
        ConvertLongitudeDegreesToXMeters(latitudeOriginDegrees, longitudeOriginDegrees, x),
        ConvertLatitudeDegreesToYMeters(latitudeOriginDegrees, y),
        z);
}



Vertex Vertex::Inverted() const
{
    double invX = 0.;
    if (x != 0.) {
        invX = 1. /x;
    }

    double invY = 0.;
    if (y != 0.) {
        invY = 1. /y;
    }

    double invZ = 0.;
    if (z != 0.) {
        invZ = 1. /z;
    }

    return Vertex(invX, invY, invZ);
}



void Vertex::operator+=(const Vertex& right)
{
    x += right.x;
    y += right.y;
    z += right.z;
}



Vertex Vertex::operator+(const Vertex& right) const
{
    return Vertex(x + right.x, y + right.y, z + right.z);
}



Vertex Vertex::operator-(const Vertex& right) const
{
    return Vertex(x - right.x, y - right.y, z - right.z);
}



Vertex Vertex::operator/(const double scale) const
{
    return Vertex(x/scale, y/scale, z/scale);
}



Vertex Vertex::operator*(const double scale) const
{
    return Vertex(x*scale, y*scale, z*scale);
}



VariantIdType GisVertex::GetConnectedObjectId(const GisObjectType& objectType) const
{
    typedef map<GisObjectType, vector<VertexConnection> >::const_iterator IterType;

    IterType iter = connections.find(objectType);

    assert(iter != connections.end());

    return (*iter).second.front().variantId;
}



vector<VariantIdType> GisVertex::GetConnectedObjectIds(const GisObjectType& objectType) const
{
    typedef map<GisObjectType, vector<VertexConnection> >::const_iterator IterType;

    vector<VariantIdType> variantIds;

    IterType iter = connections.find(objectType);

    if (iter != connections.end()) {

        const vector<VertexConnection>& vertexConnections = (*iter).second;

        for(size_t i = 0; i < vertexConnections.size(); i++) {
            variantIds.push_back(vertexConnections[i].variantId);
        }
    }

    return variantIds;
}



bool GisVertex::HasConnection(const GisObjectType& objectType) const
{
    typedef map<GisObjectType, vector<VertexConnection> >::const_iterator IterType;

    IterType iter = connections.find(objectType);

    if (iter != connections.end()) {
        return !(*iter).second.empty();
    }

    return false;
}



void Rectangle::operator+=(const Rectangle& right)
{
    minX = std::min(minX, right.minX);
    minY = std::min(minY, right.minY);
    maxX = std::max(maxX, right.maxX);
    maxY = std::max(maxY, right.maxY);
}



QuadrantType Rectangle::GetQuadrantOf(const Vertex& v) const
{
    const Vertex center = (*this).GetCenter();

    if (v.x >= center.x) {
        if (v.y >= center.y) {
            return QUADRANT_1_PX_PY;
        } else {
            return QUADRANT_4_PX_NY;
        }
    } else {
        if (v.y >= center.y) {
            return QUADRANT_2_NX_PY;
        } else {
            return QUADRANT_3_NX_NY;
        }
    }
}



Rectangle Rectangle::MakeChildRect(const QuadrantType& quadrantType) const
{
    const Vertex center = (*this).GetCenter();
    const double halfWidth = (*this).GetWidth()*0.5;
    const double halfHeight = (*this).GetHeight()*0.5;

    switch (quadrantType) {
    case QUADRANT_1_PX_PY: return Rectangle(center, center + Vertex(halfWidth, halfHeight));
    case QUADRANT_2_NX_PY: return Rectangle(center, center + Vertex(-halfWidth, halfHeight));
    case QUADRANT_3_NX_NY: return Rectangle(center, center + Vertex(-halfWidth, -halfHeight));
    case QUADRANT_4_PX_NY: return Rectangle(center, center + Vertex(halfWidth, -halfHeight));
        break;
    }

    return Rectangle();
}



bool Rectangle::Contains(const Vertex& point) const
{
    if (minX >= maxX || minY >= maxY) {
        return false;
    }

    return ((minX <= point.x) && (point.x <= maxX) &&
            (minY <= point.y) && (point.y <= maxY));
}



bool Rectangle::Contains(const Rectangle& rectangle) const
{
    return ((minX <= rectangle.minX && rectangle.maxX <= maxX) &&
            (minY <= rectangle.minY && rectangle.maxY <= maxY));
}



Rectangle Rectangle::Expanded(const double margin) const
{
    return Rectangle(minX - margin, minY - margin, maxX + margin, maxY + margin);
}



bool Rectangle::OverlappedWith(const Rectangle& rectangle) const
{
    return (!((rectangle.maxX < minX) || (maxX < rectangle.minX) ||
              (rectangle.maxY < minY) || (maxY < rectangle.minY)));
}



Rectangle Rectangle::GetOverlappedRectangle(const Rectangle& rectangle) const
{
    assert((*this).OverlappedWith(rectangle));

    return Rectangle(
        std::max(minX, rectangle.minX),
        std::max(minY, rectangle.minY),
        std::min(maxX, rectangle.maxX),
        std::min(maxY, rectangle.maxY));
}



bool Rectangle::IntersectsWithLine(
    const Vertex& lineEdge1,
    const Vertex& lineEdge2) const
{
    if ((*this).Contains(lineEdge1) ||
        (*this).Contains(lineEdge2)) {
        return true;
    }

    const Vertex topLeft(minX, maxY);
    const Vertex bottomLeft(minX, minY);
    const Vertex topRight(maxX,maxY);
    const Vertex bottomRight(maxX, minY);

    if (HorizontalLinesAreIntersection(lineEdge1, lineEdge2, topLeft, bottomLeft) ||
        HorizontalLinesAreIntersection(lineEdge1, lineEdge2, bottomLeft, bottomRight) ||
        HorizontalLinesAreIntersection(lineEdge1, lineEdge2, bottomRight, topRight) ||
        HorizontalLinesAreIntersection(lineEdge1, lineEdge2, topRight, topLeft)) {
        return true;
    }

    return false;
}



NlosPathData::NlosPathData(
    const RoadIdType& initRoadId1,
    const RoadIdType& initRoadId2,
    const IntersectionIdType& initIntersectionId1,
    const IntersectionIdType& initIntersectionId2,
    const IntersectionIdType& initNlosIntersectionId)
    :
    numberEndToEndRoadIds(0),
    numberEndToEndIntersectionIds(0)
{
    (*this).PushBackRoadId(initRoadId1);
    (*this).PushBackRoadId(initRoadId2);

    (*this).PushBackIntersectionId(initIntersectionId1);
    (*this).PushBackIntersectionId(initNlosIntersectionId);
    (*this).PushBackIntersectionId(initIntersectionId2);

    assert((*this).GetNumberOfIntersections() == (*this).GetNumberOfRoads() + 1);
}



NlosPathData::NlosPathData(
    const RoadIdType& initRoadId1,
    const RoadIdType& initRoadId2,
    const IntersectionIdType& initIntersectionId1,
    const IntersectionIdType& initIntersectionId2,
    const IntersectionIdType& initNlosIntersectionId1,
    const IntersectionIdType& initNlosIntersectionId2,
    const RoadIdType& initNlosRoadId)
    :
    numberEndToEndRoadIds(0),
    numberEndToEndIntersectionIds(0)
{
    (*this).PushBackRoadId(initRoadId1);
    (*this).PushBackRoadId(initNlosRoadId);
    (*this).PushBackRoadId(initRoadId2);

    (*this).PushBackIntersectionId(initIntersectionId1);
    (*this).PushBackIntersectionId(initNlosIntersectionId1);
    (*this).PushBackIntersectionId(initNlosIntersectionId2);
    (*this).PushBackIntersectionId(initIntersectionId2);

    assert((*this).GetNumberOfIntersections() == (*this).GetNumberOfRoads() + 1);
}



NlosPathData NlosPathData::Clone() const
{
    NlosPathData cloneData;

    cloneData.numberEndToEndRoadIds = numberEndToEndRoadIds;
    cloneData.numberEndToEndIntersectionIds = numberEndToEndIntersectionIds;

    cloneData.endToEndRoadIds = shared_array<RoadIdType>(new RoadIdType[numberEndToEndIntersectionIds]);
    for(int i = 0; i < numberEndToEndIntersectionIds; i++) {
        cloneData.endToEndRoadIds[i] = endToEndRoadIds[i];
    }

    cloneData.endToEndIntersectionIds = shared_array<IntersectionIdType>(new IntersectionIdType[numberEndToEndIntersectionIds]);
    for(int i = 0; i < numberEndToEndIntersectionIds; i++) {
        cloneData.endToEndIntersectionIds[i] = endToEndIntersectionIds[i];
    }

    return cloneData;
}



void NlosPathData::PushBackRoadId(const RoadIdType& roadId)
{
    shared_array<RoadIdType> newRoadIds = shared_array<RoadIdType>(new RoadIdType[numberEndToEndRoadIds + 1]);

    for(int i = 0; i < numberEndToEndRoadIds; i++) {
        newRoadIds[i] = endToEndRoadIds[i];
    }
    newRoadIds[numberEndToEndRoadIds] = roadId;

    numberEndToEndRoadIds++;

    endToEndRoadIds = newRoadIds;
}



void NlosPathData::PushFrontRoadId(const RoadIdType& roadId)
{
    shared_array<RoadIdType> newRoadIds = shared_array<RoadIdType>(new RoadIdType[numberEndToEndRoadIds + 1]);

    newRoadIds[0] = roadId;

    for(int i = 0; i < numberEndToEndRoadIds; i++) {
        newRoadIds[i+1] = endToEndRoadIds[i];
    }

    numberEndToEndRoadIds++;

    endToEndRoadIds = newRoadIds;
}


RoadIdType NlosPathData::GetFrontRoadId() const
{
    assert(numberEndToEndRoadIds > 0);
    return endToEndRoadIds[0];
}


RoadIdType NlosPathData::GetBackRoadId() const
{
    assert(numberEndToEndRoadIds > 0);
    return endToEndRoadIds[numberEndToEndRoadIds-1];
}


RoadIdType NlosPathData::GetNlosRoadId() const
{
    assert(numberEndToEndRoadIds > 1);
    return endToEndRoadIds[1];
}


RoadIdType NlosPathData::GetLastNlosRoadId() const
{
    assert(numberEndToEndRoadIds > 1);
    return endToEndRoadIds[numberEndToEndRoadIds-2];
}


bool NlosPathData::RoadIdIsEmpty() const
{
    return (numberEndToEndRoadIds == 0);
}


size_t NlosPathData::GetNumberOfRoads() const
{
    return numberEndToEndRoadIds;
}


bool NlosPathData::ContainsRoad(const RoadIdType& roadId) const
{
    for(int i = 0; i < numberEndToEndRoadIds; i++) {
        if (endToEndRoadIds[i] == roadId) {
            return true;
        }
    }

    return false;
}


void NlosPathData::PushBackIntersectionId(const IntersectionIdType& intersectionId)
{
    shared_array<IntersectionIdType> newIntersectionIds = shared_array<IntersectionIdType>(new IntersectionIdType[numberEndToEndIntersectionIds + 1]);

    for(int i = 0; i < numberEndToEndIntersectionIds; i++) {
        newIntersectionIds[i] = endToEndIntersectionIds[i];
    }
    newIntersectionIds[numberEndToEndIntersectionIds] = intersectionId;

    numberEndToEndIntersectionIds++;

    endToEndIntersectionIds = newIntersectionIds;
}


void NlosPathData::PushFrontIntersectionId(const IntersectionIdType& intersectionId)
{
    shared_array<IntersectionIdType> newIntersectionIds = shared_array<IntersectionIdType>(new IntersectionIdType[numberEndToEndIntersectionIds + 1]);

    newIntersectionIds[0] = intersectionId;

    for(int i = 0; i < numberEndToEndIntersectionIds; i++) {
        newIntersectionIds[i+1] = endToEndIntersectionIds[i];
    }

    numberEndToEndIntersectionIds++;

    endToEndIntersectionIds = newIntersectionIds;
}


IntersectionIdType NlosPathData::GetFrontIntersectionId() const
{
    assert(numberEndToEndIntersectionIds > 0);
    return endToEndIntersectionIds[0];
}


IntersectionIdType NlosPathData::GetBackIntersectionId() const
{
    assert(numberEndToEndIntersectionIds > 0);
    return endToEndIntersectionIds[numberEndToEndIntersectionIds-1];
}



size_t NlosPathData::GetNumberOfIntersections() const
{
    return numberEndToEndIntersectionIds;
}


std::pair<RoadIdType, RoadIdType> NlosPathData::GetRoadRelation() const
{
    assert(!(*this).RoadIdIsEmpty());
    return make_pair((*this).GetFrontRoadId(), (*this).GetBackRoadId());
}


void NlosPathData::Normalize()
{
    assert((*this).GetNumberOfRoads() > 1);
    assert((*this).GetFrontRoadId() != (*this).GetBackRoadId());
    assert((*this).GetNumberOfIntersections() == (*this).GetNumberOfRoads() + 1);

    if ((*this).GetFrontRoadId() > (*this).GetBackRoadId()) {
        (*this).Inverse();
    }
}


void NlosPathData::Inverse()
{
    for(int i = 0; i < numberEndToEndRoadIds / 2; i++) {
        std::swap(endToEndRoadIds[i], endToEndRoadIds[numberEndToEndRoadIds - i - 1]);
    }
    for(int i = 0; i < numberEndToEndIntersectionIds / 2; i++) {
        std::swap(endToEndIntersectionIds[i], endToEndIntersectionIds[numberEndToEndIntersectionIds - i - 1]);
    }
}


bool NlosPathData::IsMultipleNlosPath() const
{
    return ((*this).GetNlosCount() > 0);
}


size_t NlosPathData::GetNlosCount() const
{
    assert(!(*this).RoadIdIsEmpty());
    assert((*this).GetNumberOfIntersections() == (*this).GetNumberOfRoads() + 1);

    return (*this).GetNumberOfRoads() - 1;
}


IntersectionIdType NlosPathData::GetIntersectionId(
    const RoadIdType& startRoadId,
    const size_t intersectionNumber) const
{
    if ((*this).GetFrontRoadId() == startRoadId) {

        return endToEndIntersectionIds[intersectionNumber];

    } else {
        assert((*this).GetBackRoadId() == startRoadId);

        return endToEndIntersectionIds[(*this).GetNumberOfIntersections() - intersectionNumber - 1];
    }
}


IntersectionIdType NlosPathData::GetStartIntersectionId(
    const RoadIdType& startRoadId) const
{
    if ((*this).GetFrontRoadId() == startRoadId) {
        return (*this).GetFrontIntersectionId();
    } else {
        assert((*this).GetBackRoadId() == startRoadId);

        return (*this).GetBackIntersectionId();
    }
}


IntersectionIdType NlosPathData::GetEndIntersectionId(
    const RoadIdType& startRoadId) const
{
    if ((*this).GetFrontRoadId() == startRoadId) {
        return (*this).GetBackIntersectionId();
    } else {
        assert((*this).GetBackRoadId() == startRoadId);

        return (*this).GetFrontIntersectionId();
    }
}

RoadIdType NlosPathData::GetRoadId(
    const RoadIdType& startRoadId,
    const size_t roadNumber) const
{
    if ((*this).GetFrontRoadId() == startRoadId) {

        return endToEndRoadIds[roadNumber];

    } else {
        assert((*this).GetBackRoadId() == startRoadId);

        return endToEndRoadIds[(*this).GetNumberOfRoads() - roadNumber - 1];
    }
}


RoadIdType NlosPathData::GetEndRoadId(const RoadIdType& startRoadId) const
{
    if ((*this).GetFrontRoadId() == startRoadId) {
        return (*this).GetBackRoadId();
    } else {
        assert((*this).GetBackRoadId() == startRoadId);

        return (*this).GetFrontRoadId();
    }
}


RoadLosChecker::RoadLosChecker(
    const shared_ptr<const RoadLayer>& initRoadLayerPtr,
    const shared_ptr<NlosPathValueCalculator>& initNlosValueCalculatorPtr,
    const size_t initMaxDiffractionCount,
    const double initLosThresholdRadians,
    const double initMaxNlosDistance)
    :
    roadLayerPtr(initRoadLayerPtr),
    nlosValueCalculatorPtr(initNlosValueCalculatorPtr),
    maxDiffractionCount(initMaxDiffractionCount),
    losThresholdRadians(initLosThresholdRadians),
    maxNlosDistance(initMaxNlosDistance)
{
}



void RoadLosChecker::MakeLosRelation()
{
    map<IntersectionIdType, set<RoadIdType> > losRoadIdsPerIntersection;

    (*this).MakeLosRelation(losRoadIdsPerIntersection);

    (*this).MakeNlos1Relation(losRoadIdsPerIntersection);

    if (maxDiffractionCount > 1) {
        (*this).MakeNlos2ToNRelation(losRoadIdsPerIntersection);
    }

    //(*this).OutputLosRelation();
}



double RoadLosChecker::CalculateNlosPointRadians(
    const NlosPathData& nlosPath,
    const GisObjectIdType& startRoadId) const
{
    const vector<Intersection>& intersections = roadLayerPtr->intersections;

    const IntersectionIdType startIntersectionId =
        nlosPath.GetIntersectionId(startRoadId, 0);

    const Road& startRoad = roadLayerPtr->GetRoad(startRoadId);
    const Road& nlosRoad = roadLayerPtr->GetRoad(nlosPath.GetRoadId(startRoadId, 1));

    const IntersectionIdType startRoadIntersectionId =
        startRoad.GetOtherSideIntersectionId(startIntersectionId);

    const IntersectionIdType nlosIntersectionId =
        nlosPath.GetIntersectionId(startRoadId, 1);

    const IntersectionIdType endIntersectionId =
        nlosRoad.GetOtherSideIntersectionId(nlosIntersectionId);

    const Vertex& startPosition = intersections.at(startRoadIntersectionId).GetVertex();
    const Vertex& nlos1Position = intersections.at(nlosIntersectionId).GetVertex();
    const Vertex& endPosition = intersections.at(endIntersectionId).GetVertex();

    const Vertex startVector = Vertex(
        startPosition.positionX - nlos1Position.positionX,
        startPosition.positionY - nlos1Position.positionY,
        startPosition.positionY - nlos1Position.positionY).Normalized();

    const Vertex endVector = Vertex(
        endPosition.positionX - nlos1Position.positionX,
        endPosition.positionY - nlos1Position.positionY,
        endPosition.positionY - nlos1Position.positionY).Normalized();

    const double cosAlpha =
        (startVector.positionX*endVector.positionX +
         startVector.positionY*endVector.positionY) /
        (startVector.XYDistance()*endVector.XYDistance());

    const double rad = (std::acos(cosAlpha));

    return rad;
}



vector<Vertex> RoadLosChecker::CalculateNlosPoints(
    const NlosPathData& nlosPath,
    const RoadIdType& startRoadId) const
{
    const vector<Intersection>& intersections = roadLayerPtr->intersections;

    vector<Vertex> nlosPoints;

    for(int i = 0; i < nlosPath.numberEndToEndIntersectionIds; i++) {
        const IntersectionIdType nlos1IntersectionId = nlosPath.GetIntersectionId(startRoadId, i);

        nlosPoints.push_back(intersections.at(nlos1IntersectionId).GetVertex());
    }

    return nlosPoints;
}



double NlosPathData::GetNlosPathDistance(const vector<Intersection>& intersections) const
{
    double distance = 0;

    for(int i = 0; i < numberEndToEndIntersectionIds - 1; i++) {
        distance += XYDistanceBetweenVertices(
            intersections.at(endToEndIntersectionIds[i]).GetVertex(),
            intersections.at(endToEndIntersectionIds[i+1]).GetVertex());
    }

    return distance;
}



void NlosPathData::ExpandRoad(
    const RoadIdType& startRoadId,
    const IntersectionIdType& baseIntersectionId,
    const RoadIdType& expandRoadId,
    const IntersectionIdType& endIntersectionId)
{
    assert(numberEndToEndIntersectionIds > 0);

    if (startRoadId == (*this).GetFrontRoadId()) {
        endToEndIntersectionIds[numberEndToEndIntersectionIds - 1] = baseIntersectionId;

        (*this).PushBackRoadId(expandRoadId);
        (*this).PushBackIntersectionId(endIntersectionId);
    } else {
        endToEndIntersectionIds[0] = baseIntersectionId;

        (*this).PushFrontRoadId(expandRoadId);
        (*this).PushFrontIntersectionId(endIntersectionId);
    }
}



double RoadLosChecker::CalculateNlosPointToStartPointCenterDistance(
    const NlosPathData& nlosPath,
    const RoadIdType& startRoadId,
    const Vertex& startPosition) const
{
    const vector<Intersection>& intersections = roadLayerPtr->intersections;

    const IntersectionIdType startRoadIntersectionId = nlosPath.GetStartIntersectionId(startRoadId);
    const IntersectionIdType nlosIntersectionId = nlosPath.GetIntersectionId(startRoadId, 1);
    const Road& road = roadLayerPtr->GetRoad(startRoadId);
    const IntersectionIdType othersideRoadIntersectionId =
        road.GetOtherSideIntersectionId(startRoadIntersectionId);

    const Vertex& v0 = intersections.at(othersideRoadIntersectionId).GetVertex();
    const Vertex& v1 = intersections.at(startRoadIntersectionId).GetVertex();
    const Vertex& v2 = intersections.at(nlosIntersectionId).GetVertex();

    const Vertex startPositionOnRoadCenter = CalculateIntersectionPosition(startPosition, v0, v1);

    return startPositionOnRoadCenter.XYDistanceTo(v1) + v1.XYDistanceTo(v2);
}



double RoadLosChecker::CalculateDistanceToLastNlosPoint(
    const NlosPathData& nlosPath,
    const RoadIdType& startRoadId,
    const Vertex& startPosition) const
{
    const vector<Intersection>& intersections = roadLayerPtr->intersections;

    double totalDistance =
        (*this).CalculateNlosPointToStartPointCenterDistance(nlosPath, startRoadId, startPosition);

    for(int i = 1; i < nlosPath.numberEndToEndIntersectionIds - 2; i++) {
        const IntersectionIdType intersectionId1 = nlosPath.GetIntersectionId(startRoadId, i);
        const IntersectionIdType intersectionId2 = nlosPath.GetIntersectionId(startRoadId, i+1);

        totalDistance += XYDistanceBetweenVertices(
            intersections.at(intersectionId1).GetVertex(),
            intersections.at(intersectionId2).GetVertex());
    }

    return totalDistance;
}



double RoadLosChecker::CalculateNlosPointToEndPointCenterDistance(
    const NlosPathData& nlosPath,
    const RoadIdType& startRoadId,
    const Vertex& endPosition) const
{
    const RoadIdType endRoadId = nlosPath.GetEndRoadId(startRoadId);

    return (*this).CalculateDistanceToLastNlosPoint(nlosPath, endRoadId, endPosition);
}



double RoadLosChecker::CalculateStartPointToEndPointCenterDistance(
    const NlosPathData& nlosPath,
    const RoadIdType& startRoadId,
    const Vertex& startPosition,
    const Vertex& endPosition) const
{
    return ((*this).CalculateNlosPointToStartPointCenterDistance(nlosPath, startRoadId, startPosition) +
            (*this).CalculateNlosPointToEndPointCenterDistance(nlosPath, startRoadId, endPosition));
}



bool RoadLosChecker::IsCompleteNlosPath(
    const NlosPathData& nlosPath,
    const RoadIdType& startRoadId,
    const Vertex& startPosition,
    const Vertex& endPosition) const
{
    const vector<shared_ptr<Road> >& roadPtrs = roadLayerPtr->roadPtrs;
    const RoadIdType endRoadId = nlosPath.GetEndRoadId(startRoadId);

    const double w12 =
        roadPtrs.at(nlosPath.GetRoadId(startRoadId, 1))->GetRoadWidthMeters();

    const double x1 = (*this).CalculateNlosPointToStartPointCenterDistance(
        nlosPath, startRoadId, startPosition);

    if (x1 < w12/2.) {
        return false;
    }

    const double w22 =
        roadPtrs.at(nlosPath.GetRoadId(endRoadId, 1))->GetRoadWidthMeters();

    const double x2 = (*this).CalculateNlosPointToStartPointCenterDistance(
        nlosPath, endRoadId, endPosition);

    if (x2 < w22/2.) {
        return false;
    }

    return true;
}



inline
double NormalizedAbsRadians(const double radians)
{
    const double absRadians = (std::abs(radians));

    return std::min(std::abs(absRadians), std::abs(absRadians - PI));
}



void RoadLosChecker::MakeLosRelation(
    map<IntersectionIdType, set<RoadIdType> >& losRoadIdsPerIntersection)
{
    const vector<shared_ptr<Road> >& roadPtrs = roadLayerPtr->roadPtrs;
    const vector<Intersection>& intersections = roadLayerPtr->intersections;

    losRoadIdsPerIntersection.clear();

    for(RoadIdType roadId = 0; roadId < RoadIdType(roadPtrs.size()); roadId++) {
        const Road& aRoad = *roadPtrs[roadId];

        if (aRoad.NumberOfVertices() != 2) {
            continue;
            //cerr << "Number of vertices of road must be 2 to load LOS relation." << endl;
            //exit(1);
        }

        set<RoadIdType>& losRoadIdsForStart = losRoadIdsPerIntersection[aRoad.GetStartIntersectionId()];
        set<RoadIdType>& losRoadIdsForEnd = losRoadIdsPerIntersection[aRoad.GetEndIntersectionId()];

        losRoadIdsForStart.insert(roadId);
        losRoadIdsForEnd.insert(roadId);

        const double baseRadians = aRoad.GetDirectionRadians();

        set<std::pair<RoadIdType, RoadIdType> > checkedRoadPair;
        std::queue<IntersectionIdType> losIntersectionIds;

        losIntersectionIds.push(aRoad.GetStartIntersectionId());
        losIntersectionIds.push(aRoad.GetEndIntersectionId());

        while (!losIntersectionIds.empty()) {

            const IntersectionIdType& losIntersectionId = losIntersectionIds.front();

            const Intersection& losIntersection = intersections[losIntersectionId];
            const vector<RoadIdType> connectedRoadIds = losIntersection.GetConnectedRoadIds();

            for(size_t i = 0; i < connectedRoadIds.size(); i++) {
                const RoadIdType& connectedRoadId = connectedRoadIds[i];
                const Road& connectedRoad = *roadPtrs[connectedRoadId];

                const std::pair<RoadIdType, RoadIdType> roadPair =
                    make_pair(roadId, connectedRoadId);

                if (checkedRoadPair.find(roadPair) == checkedRoadPair.end()) {
                    checkedRoadPair.insert(roadPair);

                    const double directionDifferenceRadians =
                        NormalizedAbsRadians(baseRadians - connectedRoad.GetDirectionRadians());

                    if (directionDifferenceRadians < losThresholdRadians) {
                        losRelationBetweenRoads[roadPair].relationType = ROAD_LOSRELATION_LOS;

                        IntersectionIdType otherIntersectionId;
                        if (connectedRoad.GetStartIntersectionId() == losIntersectionId) {
                            otherIntersectionId = connectedRoad.GetEndIntersectionId();
                        } else {
                            assert(connectedRoad.GetStartIntersectionId() != losIntersectionId);
                            otherIntersectionId = connectedRoad.GetStartIntersectionId();
                        }

                        losRoadIdsForStart.insert(connectedRoadId);
                        losRoadIdsForEnd.insert(connectedRoadId);
                        losRoadIdsPerIntersection[otherIntersectionId].insert(connectedRoadId);
                        losIntersectionIds.push(otherIntersectionId);
                    }
                }
            }

            losIntersectionIds.pop();
        }
    }
}



void RoadLosChecker::MakeNlos1Relation(
    const map<IntersectionIdType, set<RoadIdType> >& losRoadIdsPerIntersection)
{
    const vector<shared_ptr<Road> >& roadPtrs = roadLayerPtr->roadPtrs;
    const vector<Intersection>& intersections = roadLayerPtr->intersections;

    typedef map<IntersectionIdType, set<RoadIdType> >::const_iterator IntersectionIter;

    // Make first depth
    for(IntersectionIter intersectionIter = losRoadIdsPerIntersection.begin();
        intersectionIter != losRoadIdsPerIntersection.end(); intersectionIter++) {

        const IntersectionIdType& intersectionId = (*intersectionIter).first;
        const set<RoadIdType>& losRoadIds = (*intersectionIter).second;
        const Vertex& intersectionPos = intersections[intersectionId].GetVertex();

        typedef set<RoadIdType>::const_iterator LosRoadIter;

        for(LosRoadIter roadIter1 = losRoadIds.begin(); roadIter1 != losRoadIds.end(); roadIter1++) {

            for(LosRoadIter roadIter2 = losRoadIds.begin(); roadIter2 != losRoadIds.end(); roadIter2++) {
                const RoadIdType& roadId1 = *roadIter1;
                const RoadIdType& roadId2 = *roadIter2;

                const std::pair<RoadIdType, RoadIdType> roadPair =
                    make_pair(roadId1, roadId2);

                typedef map<std::pair<IntersectionIdType, RoadIdType>, RoadLosRelationData>::iterator LosRelationIter;

                LosRelationIter losRelationIter = losRelationBetweenRoads.find(roadPair);

                if (losRelationIter != losRelationBetweenRoads.end() &&
                    (*losRelationIter).second.relationType == ROAD_LOSRELATION_LOS) {
                    continue;
                }

                const Road& road1 = *roadPtrs[roadId1];
                const Road& road2 = *roadPtrs[roadId2];

                if (road1.IsParking() || road2.IsParking()) {
                    continue;
                }

                const IntersectionIdType startIntersection1Id = road1.GetStartIntersectionId();
                const IntersectionIdType endIntersection1Id = road1.GetEndIntersectionId();
                const IntersectionIdType startIntersection2Id = road2.GetStartIntersectionId();
                const IntersectionIdType endIntersection2Id = road2.GetEndIntersectionId();

                if (startIntersection1Id == startIntersection2Id &&
                    endIntersection1Id == endIntersection2Id) {
                    RoadLosRelationData& losRelation = losRelationBetweenRoads[roadPair];
                    losRelation.relationType = ROAD_LOSRELATION_LOS;

                    continue;
                }

                const double distanceToRoad1Start =
                    SquaredXYDistanceBetweenVertices(
                        intersectionPos,
                        intersections[startIntersection1Id].GetVertex());

                const double distanceToRoad1End =
                    SquaredXYDistanceBetweenVertices(
                        intersectionPos,
                        intersections[endIntersection1Id].GetVertex());

                const double distanceToRoad2Start =
                    SquaredXYDistanceBetweenVertices(
                        intersectionPos,
                        intersections[startIntersection2Id].GetVertex());

                const double distanceToRoad2End =
                    SquaredXYDistanceBetweenVertices(
                        intersectionPos,
                        intersections[endIntersection2Id].GetVertex());

                IntersectionIdType startIntersectionId;
                if (distanceToRoad1Start < distanceToRoad1End) {
                    startIntersectionId = road1.GetStartIntersectionId();
                } else {
                    startIntersectionId = road1.GetEndIntersectionId();
                }

                assert(!(startIntersection1Id == startIntersection2Id &&
                         endIntersection1Id == endIntersection2Id));

                IntersectionIdType endIntersectionId;
                if (distanceToRoad2Start < distanceToRoad2End) {
                    endIntersectionId = road2.GetStartIntersectionId();
                } else {
                    endIntersectionId = road2.GetEndIntersectionId();
                }

                NlosPathData nlosPath(
                    roadId1,
                    roadId2,
                    startIntersectionId,
                    endIntersectionId,
                    intersectionId);

                if (nlosPath.GetNlosPathDistance(roadLayerPtr->intersections) <=
                    maxNlosDistance) {

                    RoadLosRelationData& losRelation = losRelationBetweenRoads[roadPair];
                    losRelation.relationType = ROAD_LOSRELATION_NLOS;

                    nlosPath.pathValue = nlosValueCalculatorPtr->GetNlosPathValue(nlosPath);

                    const std::pair<IntersectionIdType, IntersectionIdType> intersectionPair = nlosPath.GetIntersectionPair();

                    if (losRelation.nlosPaths.find(intersectionPair) == losRelation.nlosPaths.end() ||
                        losRelation.nlosPaths[intersectionPair].pathValue > nlosPath.pathValue) {

                        losRelation.nlosPaths[intersectionPair] = nlosPath;

                        assert(losRelation.nlosPaths.size() <= 4);
                    }
                }
            }
        }
    }
}



void RoadLosChecker::MakeNlos2ToNRelation(
    const map<IntersectionIdType, set<RoadIdType> >& losRoadIdsPerIntersection)
{
    const vector<shared_ptr<Road> >& roadPtrs = roadLayerPtr->roadPtrs;

    typedef map<std::pair<IntersectionIdType, RoadIdType>, RoadLosRelationData>::iterator LosRelationIter;
    typedef map<IntersectionIdType, set<RoadIdType> >::const_iterator IntersectionIter;

    map<IntersectionIdType, set<IntersectionIdType> > losIntersectionIdsPerIntersection;

    for(IntersectionIter intersectionIter = losRoadIdsPerIntersection.begin();
        intersectionIter != losRoadIdsPerIntersection.end(); intersectionIter++) {

        const IntersectionIdType& intersectionId = (*intersectionIter).first;
        const set<RoadIdType>& roadIds = (*intersectionIter).second;

        set<IntersectionIdType>& intersectionIds = losIntersectionIdsPerIntersection[intersectionId];

        typedef set<RoadIdType>::const_iterator RoadIter;

        for(RoadIter roadIter = roadIds.begin();
            roadIter != roadIds.end(); roadIter++) {

            const Road& road = *roadPtrs[*roadIter];

            intersectionIds.insert(road.GetStartIntersectionId());
            intersectionIds.insert(road.GetEndIntersectionId());
        }
    }

    for(size_t i = 1; i < maxDiffractionCount; i++) {

        map<std::pair<RoadIdType, RoadIdType>, RoadLosRelationData> newLosRelationBetweenRoads = losRelationBetweenRoads;

        for(LosRelationIter losRelationIter = losRelationBetweenRoads.begin();
            losRelationIter != losRelationBetweenRoads.end(); losRelationIter++) {

            const RoadLosRelationData& relation = (*losRelationIter).second;
            const map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>& nlosPaths = relation.nlosPaths;

            typedef map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>::const_iterator NlosIter;

            vector<NlosPathData> extendedNlosPaths;

            for(NlosIter nlosIter = nlosPaths.begin(); nlosIter != nlosPaths.end(); nlosIter++) {
                const NlosPathData& nlosPath = (*nlosIter).second;

                if (nlosPath.GetNlosCount() == i) {

                    (*this).PushExtendedNlosPath(
                        losRoadIdsPerIntersection,
                        losIntersectionIdsPerIntersection,
                        nlosPath,
                        nlosPath.GetFrontRoadId(),
                        extendedNlosPaths);
                }
            }

            for(size_t j = 0; j < extendedNlosPaths.size(); j++) {
                const NlosPathData& nlosPath = extendedNlosPaths[j];
                const std::pair<RoadIdType, RoadIdType> roadIdPair = nlosPath.GetRoadRelation();

                RoadLosRelationData& losRelation = newLosRelationBetweenRoads[roadIdPair];

                if (losRelation.relationType != ROAD_LOSRELATION_LOS) {
                    losRelation.relationType = ROAD_LOSRELATION_NLOS;

                    const std::pair<IntersectionIdType, IntersectionIdType> intersectionPair =
                        nlosPath.GetIntersectionPair();

                    if (losRelation.nlosPaths.find(intersectionPair) == losRelation.nlosPaths.end() ||
                        losRelation.nlosPaths[intersectionPair].pathValue > nlosPath.pathValue) {

                        losRelation.nlosPaths[intersectionPair] = nlosPath;

                        assert(losRelation.nlosPaths.size() <= 4);
                    }
                }
            }
        }

        losRelationBetweenRoads = newLosRelationBetweenRoads;
    }
}



void RoadLosChecker::PushExtendedNlosPath(
    const map<IntersectionIdType, set<RoadIdType> >& losRoadIdsPerIntersection,
    const map<IntersectionIdType, set<RoadIdType> >& losIntersectionIdsPerIntersection,
    const NlosPathData& nlosPath,
    const RoadIdType& startRoadId,
    vector<NlosPathData>& nlosPaths) const
{
    const vector<Intersection>& intersections = roadLayerPtr->intersections;
    const RoadIdType endRoadId = nlosPath.GetEndRoadId(startRoadId);
    const vector<shared_ptr<Road> >& roadPtrs = roadLayerPtr->roadPtrs;

    assert(nlosPath.GetNumberOfIntersections() > 2);

    typedef map<std::pair<RoadIdType, RoadIdType>, RoadLosRelationData>::const_iterator LosRelationIter;
    typedef map<IntersectionIdType, set<RoadIdType> >::const_iterator IntersectionIter;
    typedef set<RoadIdType>::const_iterator NlosRoadIter;

    const Road& endRoad = roadLayerPtr->GetRoad(endRoadId);

    const IntersectionIdType lastEndIntersectionId =
        nlosPath.GetEndIntersectionId(startRoadId);

    const IntersectionIdType endIntersectionId =
        endRoad.GetOtherSideIntersectionId(lastEndIntersectionId);

    IntersectionIter nlosIntersectionIter = losRoadIdsPerIntersection.find(lastEndIntersectionId);
    IntersectionIter endIntersectionIter = losRoadIdsPerIntersection.find(endIntersectionId);
    IntersectionIter endLosIntersectionIter = losIntersectionIdsPerIntersection.find(endIntersectionId);

    const Vertex& endIntersectionPos = intersections[endIntersectionId].GetVertex();

    assert(nlosIntersectionIter != losRoadIdsPerIntersection.end());
    assert(endIntersectionIter != losRoadIdsPerIntersection.end());
    assert(endLosIntersectionIter != losIntersectionIdsPerIntersection.end());

    const set<RoadIdType>& endRoadIds = (*endIntersectionIter).second;
    const set<RoadIdType>& ignoreRoadIds = (*nlosIntersectionIter).second;

    for(NlosRoadIter nlosRoadIter = endRoadIds.begin();
        nlosRoadIter != endRoadIds.end(); nlosRoadIter++) {

        const RoadIdType& currentEndRoadId = (*nlosRoadIter);

        LosRelationIter losRelationIter =
            losRelationBetweenRoads.find(make_pair(startRoadId, currentEndRoadId));

        if (losRelationIter != losRelationBetweenRoads.end()) {
            const RoadLosRelationData& losRelation = (*losRelationIter).second;

            if (losRelation.relationType == ROAD_LOSRELATION_LOS) {
                continue;
            }
        }

        if (ignoreRoadIds.find(currentEndRoadId) != ignoreRoadIds.end() ||
            nlosPath.ContainsRoad(currentEndRoadId)) {
            continue;
        }

        const Road& road = *roadPtrs[currentEndRoadId];

        NlosPathData newNlosPath = nlosPath.Clone();

        const IntersectionIdType intersectionId1 = road.GetStartIntersectionId();
        const IntersectionIdType intersectionId2 = road.GetEndIntersectionId();

        const double distance1 =
            SquaredXYDistanceBetweenVertices(
                endIntersectionPos,
                intersections[intersectionId1].GetVertex());

        const double distance2 =
            SquaredXYDistanceBetweenVertices(
                endIntersectionPos,
                intersections[intersectionId2].GetVertex());

        IntersectionIdType expandIntersectionId;

        if (distance1 < distance2) {
            expandIntersectionId = intersectionId1;
        } else {
            expandIntersectionId = intersectionId2;
        }

        newNlosPath.ExpandRoad(startRoadId, endIntersectionId, currentEndRoadId, expandIntersectionId);

        if (newNlosPath.GetNlosPathDistance(roadLayerPtr->intersections) <= maxNlosDistance) {
            newNlosPath.pathValue = nlosValueCalculatorPtr->GetNlosPathValue(newNlosPath);
            nlosPaths.push_back(newNlosPath);
        }
    }
}



void RoadLosChecker::OutputLosRelation() const
{
    const vector<Intersection>& intersections = roadLayerPtr->intersections;
    const vector<shared_ptr<Road> >& roadPtrs = roadLayerPtr->roadPtrs;

    typedef map<std::pair<RoadIdType, RoadIdType>, RoadLosRelationData>::const_iterator LosRelationIter;

    for(LosRelationIter iter = losRelationBetweenRoads.begin();
        iter != losRelationBetweenRoads.end(); iter++) {

        const std::pair<RoadIdType, RoadIdType>& roadIdPair = (*iter).first;
        const RoadLosRelationData& losRelation = (*iter).second;
        const Road& road1 = *roadPtrs[roadIdPair.first];
        const Road& road2 = *roadPtrs[roadIdPair.second];

        const Vertex& road1StartPos = road1.GetStartVertex();
        const Vertex& road1EndPos = road1.GetEndVertex();

        const Vertex& road2StartPos = road2.GetStartVertex();
        const Vertex& road2EndPos = road2.GetEndVertex();

        cout << roadIdPair.first
             << "(" << road1StartPos.x << "," << road1StartPos.y << ")"
             << "(" << road1EndPos.x << "," << road1EndPos.y << ")" << ", "
             << roadIdPair.second
             << "(" << road2StartPos.x << "," << road2StartPos.y << ")"
             << "(" << road2EndPos.x << "," << road2EndPos.y << "):";

        if (losRelation.relationType == ROAD_LOSRELATION_LOS) {
            cout << "LoS" << endl;
        } else if (losRelation.relationType == ROAD_LOSRELATION_NLOS) {
            cout << "NLoS";

            const map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>& nlosPaths = losRelation.nlosPaths;

            typedef map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>::const_iterator NlosIter;

            int i = 0;
            for(NlosIter nlosIter = nlosPaths.begin(); nlosIter != nlosPaths.end(); nlosIter++, i++) {
                const NlosPathData& nlosPath = (*nlosIter).second;
                const shared_array<RoadIdType> endToEndRoadIds = nlosPath.endToEndRoadIds;
                const shared_array<IntersectionIdType> endToEndIntersectionIds = nlosPath.endToEndIntersectionIds;

                if (losRelation.nlosPaths.size() > 1) {
                    cout << endl;
                }

                cout << "  - Path" << i << "(Road";
                for(unsigned int j = 0; j < nlosPath.GetNumberOfRoads(); j++) {
                    cout <<  "," << endToEndRoadIds[j];
                }
                cout << ",Intersection";
                for(unsigned int j = 0; j < nlosPath.GetNumberOfIntersections(); j++) {

                    const Vertex& position = intersections[endToEndIntersectionIds[j]].GetVertex();

                    cout << "," << endToEndIntersectionIds[j]
                         << "(" << position.x << "," << position.y << ")";

                }
                cout << ")";

            }
            cout << endl;

        } else {
            cout << "out of NLoS" << endl;
        }
    }
}



const RoadLosRelationData& RoadLosChecker::GetLosRelation(
    const RoadIdType& roadId1,
    const RoadIdType& roadId2) const
{
    const std::pair<RoadIdType, RoadIdType> roadPair =
        make_pair(roadId1, roadId2);

    typedef map<std::pair<RoadIdType, RoadIdType>, RoadLosRelationData>::const_iterator IterType;

    IterType iter = losRelationBetweenRoads.find(roadPair);

    if (iter != losRelationBetweenRoads.end()) {
        return (*iter).second;
    }

    return outofnlosRelation;
}



bool RoadLosChecker::PositionsAreLineOfSight(
    const Vertex& position1,
    const Vertex& position2) const
{
    vector<RoadIdType> roadIds1;
    roadLayerPtr->GetRoadIdsAt(position1, roadIds1);

    vector<RoadIdType> roadIds2;
    roadLayerPtr->GetRoadIdsAt(position2, roadIds2);

    for(size_t i = 0; i < roadIds1.size(); i++) {
        const RoadIdType& roadId1 = roadIds1[i];

        for(size_t j = 0; j < roadIds2.size(); j++) {
            const RoadIdType& roadId2 = roadIds2[j];

            const RoadLosRelationData& losRelationData =
                (*this).GetLosRelation(roadId1, roadId2);

            if (losRelationData.relationType == ROAD_LOSRELATION_LOS) {
                return true;
            }
        }
    }

    return false;
}



//------------------------------------------------

double SquaredXYZDistanceBetweenVertices(
    const Vertex& vertex1,
    const Vertex& vertex2)
{
    const double deltaX = vertex1.x - vertex2.x;
    const double deltaY = vertex1.y - vertex2.y;
    const double deltaZ = vertex1.z - vertex2.z;

    return ((deltaX * deltaX) + (deltaY * deltaY) + (deltaZ * deltaZ));

}//SquaredXYZDistanceBetweenVertices//



double XYZDistanceBetweenVertices(
    const Vertex& vertex1,
    const Vertex& vertex2)
{
    return (sqrt(SquaredXYZDistanceBetweenVertices(vertex1, vertex2)));

}//XYZDistanceBetweenVertices//



double SquaredXYDistanceBetweenVertices(
    const Vertex& vertex1,
    const Vertex& vertex2)
{
    const double deltaX = vertex1.x - vertex2.x;
    const double deltaY = vertex1.y - vertex2.y;

    return ((deltaX * deltaX) + (deltaY * deltaY));

}//SquaredXYDistanceBetweenVertices//



double XYDistanceBetweenVertices(
    const Vertex& vertex1,
    const Vertex& vertex2)
{
    return (sqrt(SquaredXYDistanceBetweenVertices(vertex1, vertex2)));

}//XYZDistanceBetweenVertices//



void CalculateAzimuthAndElevationDegrees(
    const Vertex& firstPoint,
    const Vertex& secondPoint,
    double& azimuthDegrees,
    double& elevationDegrees)
{
    const double deltaX =
        secondPoint.x - firstPoint.x;
    const double deltaY =
        secondPoint.y - firstPoint.y;
    const double deltaZ =
        secondPoint.z - firstPoint.z;
    const double xyDistanceSquared =
        deltaX * deltaX + deltaY * deltaY;

    azimuthDegrees = 0;
    azimuthDegrees =
        90.0 - (180.0 / PI) * (std::atan2(deltaY, deltaX));

    elevationDegrees = (180.0 / PI) * (std::atan2(deltaZ, (sqrt(xyDistanceSquared))));

    NormalizeAzimuthAndElevation(azimuthDegrees, elevationDegrees);

}



double CalculateFullRadiansBetweenVector(
    const Vertex& vertex1,
    const Vertex& middleVertex,
    const Vertex& vertex2)
{
    const double rad1 = (std::atan2(vertex1.y - middleVertex.y, vertex1.x - middleVertex.x));
    const double rad2 = (std::atan2(vertex2.y - middleVertex.y, vertex2.x - middleVertex.x));

    const double totalRad = rad2 - rad1;

    if (totalRad < 0) {
        return totalRad + 2*PI;
    } else if (totalRad > 2*PI) {
        return totalRad - 2*PI;
    }

    return totalRad;
}



double CalculateRadiansBetweenVector(
    const Vertex& vertex1,
    const Vertex& middleVertex,
    const Vertex& vertex2)
{
    const Vertex vector1 = vertex1 - middleVertex;
    const Vertex vector2 = vertex2 - middleVertex;

    const double distance1 = vector1.XYDistance();
    const double distance2 = vector2.XYDistance();

    if (distance1 == 0 || distance2 == 0) {
        return 0;
    }

    const double cosAlpha =
        (vector1.x*vector2.x +
         vector1.y*vector2.y) /
        (distance1*distance2);

    return ((std::acos(std::min(1., std::max(-1., cosAlpha)))));
}



inline
double CalculatePointToLineCrossPoint(
    const Vertex& point,
    const Vertex& lineEdge1,
    const Vertex& lineEdge2)
{
    const double dx = lineEdge2.x - lineEdge1.x;
    const double dy = lineEdge2.y - lineEdge1.y;
    const double dz = lineEdge2.z - lineEdge1.z;

    const double innerProduct =
        dx * (lineEdge1.x - point.x) +
        dy * (lineEdge1.y - point.y) +
        dz * (lineEdge1.z - point.z);

    double t;
    if (innerProduct >= 0.0) {
        t = 0.0;
    } else {
        t = - innerProduct / (dx*dx + dy*dy + dz*dz);

        if (t > 1.0) {
            t = 1.0;
        }
    }

    return t;
}



Vertex CalculatePointToLineNearestPosition(
    const Vertex& point,
    const Vertex& lineEdge1,
    const Vertex& lineEdge2)
{
    const double t = CalculatePointToLineCrossPoint(point, lineEdge1, lineEdge2);

    return (lineEdge2 - lineEdge1)*t + lineEdge1;
}



Vertex CalculatePointToHorizontalLineNearestPosition(
    const Vertex& point,
    const Vertex& lineEdge1,
    const Vertex& lineEdge2)
{
    const double t = CalculatePointToLineCrossPoint(
        point.XYPoint(),
        lineEdge1.XYPoint(),
        lineEdge2.XYPoint());

    return (lineEdge2 - lineEdge1)*t + lineEdge1;
}



double CalculatePointToLineDistance(
    const Vertex& point,
    const Vertex& lineEdge1,
    const Vertex& lineEdge2)
{
    return CalculatePointToLineNearestPosition(
        point, lineEdge1, lineEdge2).DistanceTo(point);
}



Vertex CalculateIntersectionPosition(
    const Vertex& point,
    const Vertex& lineEdge1,
    const Vertex& lineEdge2)
{
    const double dx = lineEdge2.x - lineEdge1.x;
    const double dy = lineEdge2.y - lineEdge1.y;
    const double dz = lineEdge2.z - lineEdge1.z;

    const double innerProduct =
        dx * (lineEdge1.x - point.x) +
        dy * (lineEdge1.y - point.y) +
        dz * (lineEdge1.z - point.z);

    const double t = - innerProduct / (dx*dx + dy*dy + dz*dz);

    return Vertex(
        t * dx + lineEdge1.x,
        t * dy + lineEdge1.y,
        t * dz + lineEdge1.z);
}



inline
Vertex CalculatePointToPlaneNearestPoint(
    const Vertex& point,
    const Vertex& planeV1,
    const Vertex& planeV2,
    const Vertex& planeV3)
{
    const Vertex planeVector = (planeV2 - planeV1).Cross(planeV3 - planeV1).Normalized();
    const double d = -planeVector.Dot(planeV1);
    const double distance = planeVector.Dot(point) + d;

    return point - planeVector*distance;
}



Vertex CalculatePointToRectNearestVertex(
    const Vertex& point,
    const Vertex& planeBottomLeft,
    const Vertex& planeBottomRight,
    const Vertex& planeTopRight,
    const Vertex& planeTopLeft)
{
    const Vertex nearestPlaneVertex =
        CalculatePointToPlaneNearestPoint(point, planeBottomLeft, planeBottomRight, planeTopRight);

    if (IsInsideOf3dTheTriangle(nearestPlaneVertex, planeBottomLeft, planeBottomRight, planeTopRight) ||
        IsInsideOf3dTheTriangle(nearestPlaneVertex, planeBottomRight, planeTopRight, planeTopLeft)) {

        return nearestPlaneVertex;
    }

    vector<Vertex> polygonVertices;

    polygonVertices.push_back(planeBottomLeft);
    polygonVertices.push_back(planeBottomRight);
    polygonVertices.push_back(planeTopRight);
    polygonVertices.push_back(planeTopLeft);
    polygonVertices.push_back(planeBottomLeft);

    Vertex nearestPosition;
    size_t vertexNumber;

    CalculatePointToArcNearestPosition(polygonVertices, nearestPlaneVertex, nearestPosition, vertexNumber);

    return nearestPosition;
}



Vertex CalculatePointToPolygonNearestVertex(
    const Vertex& point,
    const vector<Vertex>& planePolygon)
{
    if (planePolygon.size() >= 4 &&
        planePolygon.front() == planePolygon.back()) {

        vector<Triangle> triangles;

        PolygonToTriangles(planePolygon, triangles);

        assert(!triangles.empty());

        const Triangle& topTriangle = triangles[0];
        const Vertex nearestPlaneVertex =
            CalculatePointToPlaneNearestPoint(point, topTriangle.GetP1(), topTriangle.GetP2(), topTriangle.GetP3());

        for(size_t i = 0; i < triangles.size(); i++) {
            const Triangle& triangle = triangles[i];

            if (IsInsideOf3dTheTriangle(nearestPlaneVertex, triangle.GetP1(), triangle.GetP2(), triangle.GetP3())) {
                return nearestPlaneVertex;
            }
        }
    }

    Vertex nearestPosition;
    size_t vertexNumber;

    CalculatePointToArcNearestPosition(planePolygon, point, nearestPosition, vertexNumber);

    return nearestPosition;
}



Vertex CalculatePointTo3dPolygonNearestVertex(
    const Vertex& point,
    const vector<Vertex>& vertices,
    const double polygonHeight)
{
    assert(!vertices.empty());

    if (vertices.size() == 1) {
        return vertices.front();
    }

    if (polygonHeight == 0.) {
        return CalculatePointToPolygonNearestVertex(point, vertices);
    }

    Vertex nearestPosition = vertices.front();
    size_t vertexNumber;

    CalculatePointToArcNearestVertex(vertices, point, nearestPosition, vertexNumber);

    double minDistance = DBL_MAX;

    const Vertex hv(0., 0., polygonHeight);

    if (vertexNumber > 0) {
        const Vertex& v1 = vertices[vertexNumber - 1];
        const Vertex& v2 = vertices[vertexNumber];

        const Vertex rectPoosition =
            CalculatePointToRectNearestVertex(point, v1, v2, v2 + hv, v1 + hv);
        const double distance = point.DistanceTo(rectPoosition);

        if (distance < minDistance) {
            minDistance = distance;
            nearestPosition = rectPoosition;
        }
    }
    if (vertexNumber < vertices.size() - 1) {
        const Vertex& v1 = vertices[vertexNumber];
        const Vertex& v2 = vertices[vertexNumber + 1];

        const Vertex rectPoosition =
            CalculatePointToRectNearestVertex(point, v1, v2, v2 + hv, v1 + hv);
        const double distance = point.DistanceTo(rectPoosition);

        if (distance < minDistance) {
            minDistance = distance;
            nearestPosition = rectPoosition;
        }
    }

    // if closed polugon, check roof and floor plane.
    if (vertices.front() == vertices.back()) {
        const double polygonZ = vertices.front().z;

        if (point.z > polygonZ + polygonHeight) {
            vector<Vertex> roofVertices = vertices;

            for(size_t i = 0; i < vertices.size(); i++) {
                roofVertices[i] += hv;
            }

            const Vertex polygonPosition =
                CalculatePointToPolygonNearestVertex(point, roofVertices);
            const double distance = point.DistanceTo(polygonPosition);

            if (distance < minDistance) {
                minDistance = distance;
                nearestPosition = polygonPosition;
            }

        } else if (point.z < polygonZ) {
            const Vertex polygonPosition =
                CalculatePointToPolygonNearestVertex(point, vertices);
            const double distance = point.DistanceTo(polygonPosition);

            if (distance < minDistance) {
                minDistance = distance;
                nearestPosition = polygonPosition;
            }
        }
    }

    return nearestPosition;
}



inline
Vertex CalculateMiddlePoint(const vector<Vertex>& points, const double ratio)
{
    double distanceToPoint = CalculateArcDistance(points) * ratio;

    assert(!points.empty());
    Vertex middlePoint = points.back();

    for(size_t i = 0; i < points.size() - 1; i++) {
        const Vertex& p1 = points[i];
        const Vertex& p2 = points[i+1];

        const double aLineLength = p1.DistanceTo(p2);

        if (distanceToPoint < aLineLength) {
            middlePoint =  (p2 - p1)*(distanceToPoint/aLineLength) + p1;
            break;
        }

        distanceToPoint -= aLineLength;
    }

    return middlePoint;
}


inline
bool LinesAreParallel(
    const Vertex& lineEdge11,
    const Vertex& lineEdge12,
    const Vertex& lineEdge21,
    const Vertex& lineEdge22)
{
    const Vertex vector1 = lineEdge12 - lineEdge11;
    const Vertex vector2 = lineEdge22 - lineEdge21;
    const Vertex vector3 = lineEdge21 - lineEdge11;

    return ((vector1.x*vector2.y) - (vector1.y*vector2.x) == 0);
}



Vertex CalculateIntersectionPositionBetweenLine(
    const Vertex& baseLineEdge11,
    const Vertex& baseLineEdge12,
    const Vertex& horizontalCossedLineEdge21,
    const Vertex& horizontalCossedLineEdge22)
{
    const Vertex vector1 = baseLineEdge12 - baseLineEdge11;
    const Vertex vector2 = horizontalCossedLineEdge22 - horizontalCossedLineEdge21;
    const Vertex vector3 = horizontalCossedLineEdge21 - baseLineEdge11;

    const double denom = (vector1.x*vector2.y) - (vector1.y*vector2.x);

    if (denom == 0) {
        // return middle point
        return (baseLineEdge11+baseLineEdge12+horizontalCossedLineEdge21+horizontalCossedLineEdge22) /4.;
    }

    assert(denom != 0);

    const double t =
        ((vector2.y*vector3.x) - (vector2.x*vector3.y))/denom;

    return (baseLineEdge12 - baseLineEdge11)*t + baseLineEdge11;
}



void GetIntersectionPositionBetweenLineAndPolygon(
    const Vertex& lineEdge1,
    const Vertex& lineEdge2,
    const vector<Vertex>& polygon,
    vector<Vertex>& intersectionPoints)
{
    intersectionPoints.clear();

    if (PolygonContainsPoint(polygon, lineEdge1)) {
        intersectionPoints.push_back(lineEdge1);
    }

    for(size_t i = 0; i < polygon.size() - 1; i++) {
        const Vertex& p1 = polygon[i];
        const Vertex& p2 = polygon[i+1];

        if (HorizontalLinesAreIntersection(lineEdge1, lineEdge2, p1, p2)) {
            const Vertex& intersectionPos =
                CalculateIntersectionPositionBetweenLine(
                    lineEdge1, lineEdge2, p1, p2);

            if (intersectionPos != lineEdge1 &&
                intersectionPos != lineEdge2) {
                intersectionPoints.push_back(intersectionPos);
            }
        }
    }

    if (PolygonContainsPoint(polygon, lineEdge2)) {
        intersectionPoints.push_back(lineEdge2);
    }
}



//----------------------------------------------------------------
// VertexMap Mesh
//----------------------------------------------------------------

SpatialObjectMap::SpatialObjectMap()
    :
    meshUnit(0),
    numberHorizontalMeshes(0),
    numberVerticalMeshes(0)
{
}



void SpatialObjectMap::SetMesh(
    const Rectangle& initMinRect,
    const double& initMeshUnit,
    const size_t maxMeshSize)
{
    minRect = initMinRect;
    meshUnit = initMeshUnit;

    numberHorizontalMeshes =
        std::max<size_t>(1, size_t(ceil((minRect.maxX - minRect.minX) / meshUnit)));

    numberVerticalMeshes =
        std::max<size_t>(1, size_t(ceil((minRect.maxY - minRect.minY) / meshUnit)));

    if ((numberHorizontalMeshes * numberVerticalMeshes) > (maxMeshSize * maxMeshSize)) {
        if (numberHorizontalMeshes > numberVerticalMeshes) {
            numberHorizontalMeshes =
                size_t(ceil((double)(maxMeshSize * maxMeshSize) / numberVerticalMeshes));
            meshUnit = ceil((minRect.maxX - minRect.minX) / numberHorizontalMeshes);
        } else {
            numberVerticalMeshes =
                size_t(ceil((double)(maxMeshSize * maxMeshSize) / numberHorizontalMeshes));
            meshUnit = ceil((minRect.maxY - minRect.minY) / numberVerticalMeshes);
        }
    }//if//

    assert(numberHorizontalMeshes * numberVerticalMeshes != 0);

    ids.clear();
    ids.resize(numberHorizontalMeshes * numberVerticalMeshes);
}



void SpatialObjectMap::RemoveGisObject(
    const GisObject& gisObject,
    const VariantIdType& variantId)
{
    Rectangle rect = gisObject.GetMinRectangle();

    if (rect.OverlappedWith(minRect)) {
        rect = rect.GetOverlappedRectangle(minRect);
    }

    const size_t minHorizontalId = (*this).GetMinHorizontalId(rect);
    const size_t maxHorizontalId = (*this).GetMaxHorizontalId(rect);
    const size_t minVerticalId = (*this).GetMinVerticalId(rect);
    const size_t maxVerticalId = (*this).GetMaxVerticalId(rect);

    for(size_t verticalId = minVerticalId; verticalId <= maxVerticalId; verticalId++) {
        for(size_t horizontalId = minHorizontalId; horizontalId <= maxHorizontalId; horizontalId++) {

            const Rectangle meshRect(
                minRect.minX + meshUnit*horizontalId,
                minRect.minY + meshUnit*verticalId,
                minRect.minX + meshUnit*(horizontalId+1),
                minRect.minY + meshUnit*(verticalId+1));

            if (gisObject.IntersectsWith(meshRect)) {
                const size_t meshId = numberHorizontalMeshes * verticalId + horizontalId;
                ids[meshId].erase(variantId);
            }
        }
    }
}



void SpatialObjectMap::InsertGisObject(
    const GisObject& gisObject,
    const VariantIdType& variantId)
{
    Rectangle rect = gisObject.GetMinRectangle();

    if (rect.OverlappedWith(minRect)) {
        rect = rect.GetOverlappedRectangle(minRect);
    }

    const size_t minHorizontalId = (*this).GetMinHorizontalId(rect);
    const size_t maxHorizontalId = (*this).GetMaxHorizontalId(rect);
    const size_t minVerticalId = (*this).GetMinVerticalId(rect);
    const size_t maxVerticalId = (*this).GetMaxVerticalId(rect);

    for(size_t verticalId = minVerticalId; verticalId <= maxVerticalId; verticalId++) {
        for(size_t horizontalId = minHorizontalId; horizontalId <= maxHorizontalId; horizontalId++) {

            const Rectangle meshRect(
                minRect.minX + meshUnit*horizontalId,
                minRect.minY + meshUnit*verticalId,
                minRect.minX + meshUnit*(horizontalId+1),
                minRect.minY + meshUnit*(verticalId+1));

            if (gisObject.IntersectsWith(meshRect)) {
                const size_t meshId = numberHorizontalMeshes * verticalId + horizontalId;
                ids[meshId].insert(variantId);
            }
        }
    }
}



void SpatialObjectMap::InsertVertex(
    const Vertex& vertex,
    const VariantIdType& variantId)
{
    const size_t horizontalId = (*this).GetHorizontalId(vertex);
    const size_t verticalId = (*this).GetVerticalId(vertex);

    const size_t meshId = numberHorizontalMeshes * verticalId + horizontalId;

    ids[meshId].insert(variantId);
}



void SpatialObjectMap::GetGisObject(
    const Rectangle& targetRect,
    vector<VariantIdType>& variantIds) const
{
    variantIds.clear();

    if (!(*this).IsAvailable()) {
        return;
    }

    if (!targetRect.OverlappedWith(minRect)) {
        return;
    }

    const Rectangle rect = targetRect.GetOverlappedRectangle(minRect);
    const size_t minHorizontalId = (*this).GetMinHorizontalId(rect);
    const size_t maxHorizontalId = (*this).GetMaxHorizontalId(rect);
    const size_t minVerticalId = (*this).GetMinVerticalId(rect);
    const size_t maxVerticalId = (*this).GetMaxVerticalId(rect);

    set<VariantIdType> uniqueIds;

    for(size_t verticalId = minVerticalId; verticalId <= maxVerticalId; verticalId++) {
        for(size_t horizontalId = minHorizontalId; horizontalId <= maxHorizontalId; horizontalId++) {

            const size_t meshId = numberHorizontalMeshes * verticalId + horizontalId;

            if (meshId < ids.size()) {
                const set<VariantIdType>& targetIds = ids[meshId];

                uniqueIds.insert(targetIds.begin(), targetIds.end());
            }//if//
        }//for//
    }//for//

    variantIds.assign(uniqueIds.begin(), uniqueIds.end());
}



void SpatialObjectMap::GetGisObject(
    const Vertex& pos,
    vector<VariantIdType>& variantIds) const
{
    const double integrationLength = 0.01;//1cm
    const Rectangle searchRect(pos, integrationLength);

    (*this).GetGisObject(searchRect, variantIds);
}



size_t SpatialObjectMap::GetMinHorizontalId(const Rectangle& rect) const
{
    const size_t horizontalId = size_t(floor((rect.minX - minRect.minX) / meshUnit));

    return std::max<size_t>(0, std::min(horizontalId, numberHorizontalMeshes - 1));
}


size_t SpatialObjectMap::GetMaxHorizontalId(const Rectangle& rect) const
{
    const size_t horizontalId = size_t(ceil((rect.maxX - minRect.minX) / meshUnit));

    return std::max<size_t>(0, std::min(horizontalId, numberHorizontalMeshes - 1));
}


size_t SpatialObjectMap::GetMinVerticalId(const Rectangle& rect) const
{
    const size_t verticalId = size_t(floor((rect.minY - minRect.minY) / meshUnit));

    return std::max<size_t>(0, std::min(verticalId, numberVerticalMeshes - 1));
}


size_t SpatialObjectMap::GetMaxVerticalId(const Rectangle& rect) const
{
    const size_t verticalId = size_t(ceil((rect.maxY - minRect.minY) / meshUnit));

    return std::max<size_t>(0, std::min(verticalId, numberVerticalMeshes - 1));
}


size_t SpatialObjectMap::GetHorizontalId(const Vertex& vertex) const
{
    const size_t horizontalId = size_t(floor((vertex.x - minRect.minX) / meshUnit));

    return std::max<size_t>(0, std::min(horizontalId, numberHorizontalMeshes - 1));
}


size_t SpatialObjectMap::GetVerticalId(const Vertex& vertex) const
{
    const size_t verticalId = size_t(floor((vertex.y - minRect.minY) / meshUnit));

    return std::max<size_t>(0, std::min(verticalId, numberVerticalMeshes - 1));
}

//--------------------------------------------------------


MaterialSet::MaterialSet()
    :
    defaultMaterialPtr(new Material("", 0)),
    cacheIter(materialIds.end())
{}



void MaterialSet::AddMaterial(
    const string& name,
    const double transmissionLossDb)
{
    assert(materialIds.find(name) == materialIds.end());

    const MaterialIdType materialId =
        static_cast<MaterialIdType>(materialIds.size());

    materialIds[name] = materialId;

    materialPtrs.push_back(
        shared_ptr<Material>(new Material(name, transmissionLossDb)));
}



MaterialIdType MaterialSet::GetMaterialId(const string& name) const
{
    if (cacheIter == materialIds.end() ||
        (*cacheIter).first != name) {

        cacheIter = materialIds.find(name);

        if (cacheIter == materialIds.end()) {
            return INVALID_MATERIAL_ID;
        }
    }

    return (*cacheIter).second;
}



const Material& MaterialSet::GetMaterial(const string& name) const
{
    return (*this).GetMaterial((*this).GetMaterialId(name));
}



const Material& MaterialSet::GetMaterial(const MaterialIdType& materialId) const
{
    if (materialId == INVALID_MATERIAL_ID) {
        return *defaultMaterialPtr;
    }

    return *materialPtrs[materialId];
}


class GisObject::GisObjectEnableDisableEvent : public SimulationEvent {
public:
    GisObjectEnableDisableEvent(
        const shared_ptr<Implementation>& initCommonImplPtr,
        const bool initIsEnable)
        :
        commonImplPtr(initCommonImplPtr),
        isEnable(initIsEnable)
    {}

    virtual void ExecuteEvent() {
        commonImplPtr->subsystemPtr->SetEnabled(
            commonImplPtr->objectType,
            commonImplPtr->variantId,
            isEnable);
    }

private:
    shared_ptr<Implementation> commonImplPtr;
    bool isEnable;
};



GisObject::GisObject(
    GisSubsystem* initSubsystemPtr,
    const GisObjectType& initObjectType,
    const GisObjectIdType& initObjectId,
    const VariantIdType& initVariantId)
    :
    commonImplPtr(
        new Implementation(initSubsystemPtr, initObjectType, initObjectId, initVariantId))
{
}



GisObject::GisObject(
    GisSubsystem* initSubsystemPtr,
    const GisObjectType& initObjectType,
    const GisObjectIdType& initObjectId,
    const VariantIdType& initVariantId,
    const VertexIdType& initVertexId)
    :
    commonImplPtr(
        new Implementation(initSubsystemPtr, initObjectType, initObjectId, initVariantId))
{
    commonImplPtr->vertexIds.push_back(initVertexId);
}



void GisObject::LoadParameters(const ParameterDatabaseReader& theParameterDatabaseReader)
{
    const GisObjectIdType objectId = commonImplPtr->objectId;

    if (theParameterDatabaseReader.ParameterExists("gisobject-disable-time", objectId)) {

        const SimTime disableTime = theParameterDatabaseReader.ReadTime("gisobject-disable-time", objectId);

        if (disableTime < INFINITE_TIME) {
            commonImplPtr->subsystemPtr->ScheduleGisEvent(
                shared_ptr<SimulationEvent>(
                    new GisObjectEnableDisableEvent(commonImplPtr, false)),
                disableTime);
        }
    }

    if (theParameterDatabaseReader.ParameterExists("gisobject-enable-time", objectId)) {

        const SimTime enableTime = theParameterDatabaseReader.ReadTime("gisobject-enable-time", objectId);

        if (enableTime < INFINITE_TIME) {
            commonImplPtr->subsystemPtr->ScheduleGisEvent(
                shared_ptr<SimulationEvent>(
                    new GisObjectEnableDisableEvent(commonImplPtr, true)),
                enableTime);
        }
    }

    if (theParameterDatabaseReader.ParameterExists("trace-enabled-tags", objectId)) {
        string enabledTagsString = theParameterDatabaseReader.ReadString("trace-enabled-tags", objectId);
        ConvertStringToLowerCase(enabledTagsString);

        if (enabledTagsString.find("gis") != string::npos) {
            commonImplPtr->outputGisTrace = true;
        }
        if (enabledTagsString.find("mas") != string::npos) {
            commonImplPtr->outputMasTrace = true;
        }
    }
}



GisObjectIdType GisObject::GetObjectId() const
{
    return commonImplPtr->objectId;
}



const string& GisObject::GetObjectName() const
{
    return commonImplPtr->objectName;
}



GisObjectType GisObject::GetObjectType() const
{
    return commonImplPtr->objectType;
}



const Vertex& GisObject::GetVertex(const size_t index) const
{
    return commonImplPtr->subsystemPtr->GetVertex(commonImplPtr->vertexIds.at(index));
}



const VertexIdType& GisObject::GetVertexId(const size_t index) const
{
    assert(index < commonImplPtr->vertexIds.size());
    return commonImplPtr->vertexIds.at(index);
}



const VertexIdType& GisObject::GetStartVertexId() const
{
    return commonImplPtr->vertexIds.front();
}



const VertexIdType& GisObject::GetEndVertexId() const
{
    return commonImplPtr->vertexIds.back();
}



const VertexIdType& GisObject::GetNearestVertexId(const Vertex& vertex) const
{
    double minDistance = DBL_MAX;
    size_t nearesVeretxNumber = 0;

    assert(!commonImplPtr->vertexIds.empty());

    for(size_t i = 0; i < commonImplPtr->vertexIds.size(); i++) {
        const double distance = vertex.DistanceTo((*this).GetVertex(i));

        if (distance < minDistance) {
            minDistance = distance;
            nearesVeretxNumber = i;
        }
    }

    return commonImplPtr->vertexIds.at(nearesVeretxNumber);
}



bool GisObject::IsStartOrEndVertex(const VertexIdType& vertexId) const
{
    return ((*this).IsStartVertex(vertexId) ||
            (*this).IsEndVertex(vertexId));
}



bool GisObject::IsStartVertex(const VertexIdType& vertexId) const
{
    return (commonImplPtr->vertexIds.front() == vertexId);
}



bool GisObject::IsEndVertex(const VertexIdType& vertexId) const
{
    return (commonImplPtr->vertexIds.back() == vertexId);
}



bool GisObject::ContainsVertexId(const VertexIdType& vertexId) const
{
    for(size_t i = 0; i < commonImplPtr->vertexIds.size(); i++) {
        if (commonImplPtr->vertexIds[i] == vertexId) {
            return true;
        }
    }

    return false;
}



const VertexIdType& GisObject::GetOthersideVertexId(const VertexIdType& vertexId) const
{
    assert((*this).IsStartOrEndVertex(vertexId));

    if (commonImplPtr->vertexIds.front() == vertexId) {
        return commonImplPtr->vertexIds.back();
    }

    return commonImplPtr->vertexIds.front();
}


const GisVertex& GisObject::GetGisVertex(const size_t index) const
{
    return commonImplPtr->subsystemPtr->GetGisVertex(commonImplPtr->vertexIds.at(index));
}


const Vertex& GisObject::GetStartVertex() const
{
    return commonImplPtr->subsystemPtr->GetVertex(commonImplPtr->vertexIds.front());
}


const Vertex& GisObject::GetEndVertex() const
{
    return commonImplPtr->subsystemPtr->GetVertex(commonImplPtr->vertexIds.back());
}


Vertex GisObject::GetCenterPoint() const
{
    if (commonImplPtr->vertexIds.empty()) {
        return Vertex();
    }

    Vertex vertexSum;

    for(size_t i = 0; i < commonImplPtr->vertexIds.size(); i++) {
        vertexSum += (*this).GetVertex(i);
    }

    return vertexSum / double(commonImplPtr->vertexIds.size());
}


vector<Vertex> GisObject::GetVertices() const
{
    vector<Vertex> vertices;

    for(size_t i = 0; i < commonImplPtr->vertexIds.size(); i++) {
        vertices.push_back((*this).GetVertex(i));
    }

    return vertices;
}


size_t GisObject::NumberOfVertices() const
{
    return commonImplPtr->vertexIds.size();
}


const Rectangle& GisObject::GetMinRectangle() const
{
    return commonImplPtr->minRectangle;
}


void GisObject::UpdateMinRectangle() const
{
    assert((*this).NumberOfVertices() > 0);

    const double margin = 0.00001;

    Rectangle& minRectangle = commonImplPtr->minRectangle;

    minRectangle.minX = DBL_MAX;
    minRectangle.minY = DBL_MAX;
    minRectangle.maxX = -DBL_MAX;
    minRectangle.maxY = -DBL_MAX;

    for(size_t i = 0; i < (*this).NumberOfVertices(); i++) {
        const Vertex& vertex = (*this).GetVertex(i);

        minRectangle.minX = std::min(minRectangle.minX, vertex.x - margin);
        minRectangle.minY = std::min(minRectangle.minY, vertex.y - margin);
        minRectangle.maxX = std::max(minRectangle.maxX, vertex.x + margin);
        minRectangle.maxY = std::max(minRectangle.maxY, vertex.y + margin);
    }
}



size_t GisObject::CalculateNumberIntersections(const Vertex& p1, const Vertex& p2) const
{
    size_t numberIntersections = 0;

    for(size_t i = 0; i < (*this).NumberOfVertices() - 1; i++) {

        const Vertex& v1 = (*this).GetVertex(i);
        const Vertex& v2 = (*this).GetVertex(i+1);

        if (HorizontalLinesAreIntersection(p1, p2, v1, v2)) {
            numberIntersections++;
        }
    }

    return numberIntersections;
}


void GisObject::InsertVertex(const Vertex& newVertex)
{
    GisSubsystem* subsystemPtr = commonImplPtr->subsystemPtr;
    vector<VertexIdType>& vertexIds = commonImplPtr->vertexIds;

    const double nearDistance = 0.01; //1cm
    const VertexIdType newVertexId = subsystemPtr->GetVertexId(newVertex);

    if (std::find(vertexIds.begin(), vertexIds.end(), newVertexId) !=
        vertexIds.end()) {
        return;
    }

    size_t vertexNumber;

    for(vertexNumber = 0; vertexNumber < vertexIds.size() - 1; vertexNumber++) {
        const Vertex& edge1 = (*this).GetVertex(vertexNumber);
        const Vertex& edge2 = (*this).GetVertex(vertexNumber+1);

        const double distanceToMiddlePoint =
            CalculatePointToLineDistance(newVertex, edge1, edge2);

        if (distanceToMiddlePoint < nearDistance) {
            break;
        }
    }

    assert(vertexNumber < vertexIds.size() - 1);

    const VertexIdType prevVertexId = vertexIds[vertexNumber];
    const VertexIdType nextVertexId = vertexIds[vertexNumber+1];

    (*this).InsertVertex(prevVertexId, newVertexId, nextVertexId);
}



void GisObject::InsertVertex(
    const VertexIdType& prevVertexId,
    const VertexIdType& newVertexId,
    const VertexIdType& nextVertexId)
{
    GisSubsystem* subsystemPtr = commonImplPtr->subsystemPtr;
    vector<VertexIdType>& vertexIds = commonImplPtr->vertexIds;

    if (std::find(vertexIds.begin(), vertexIds.end(), newVertexId) !=
        vertexIds.end()) {
        return;
    }

    const GisObjectType objectType = (*this).GetObjectType();
    const VariantIdType& variantId = commonImplPtr->variantId;

    subsystemPtr->DisconnectBidirectionalGisObject(
        prevVertexId, objectType, nextVertexId, variantId);

    subsystemPtr->ConnectBidirectionalGisObject(
        prevVertexId, objectType, newVertexId, variantId);

    subsystemPtr->ConnectBidirectionalGisObject(
        nextVertexId, objectType, newVertexId, variantId);

    vector<VertexIdType>::iterator iter = vertexIds.begin();

    for(size_t i = 0; i < vertexIds.size(); i++) {
        if (vertexIds[i] == nextVertexId) {
            assert(vertexIds[i-1] == prevVertexId);
            break;
        }
        assert(vertexIds[i] != newVertexId);

        iter++;
    }
    assert(vertexIds.back() != newVertexId);
    assert(iter != vertexIds.end());

    vertexIds.insert(iter, newVertexId);
}



const vector<VertexIdType>& GisObject::GetVertexIds() const
{
    return commonImplPtr->vertexIds;
}



void GisObject::SetEnabled(const bool enable)
{
    commonImplPtr->isEnabled = enable;

    if (commonImplPtr->outputGisTrace) {

        assert(commonImplPtr->subsystemPtr->theSimulationEnginePtr != nullptr);

        const SimTime currentTime =
            commonImplPtr->subsystemPtr->theSimulationEnginePtr->CurrentTime();

        TraceSubsystem& traceSubsystem =
            commonImplPtr->subsystemPtr->theSimulationEnginePtr->GetTraceSubsystem();

        StateTraceRecord stateTrace;
        ostringstream stateStream;

        const double value = 1.0;
        const string modelName = "Gis";
        const string instanceName = "";
        const string eventName = "State";
        const size_t threadPartitionIndex = 0;

        string stateName;

        if (enable) {
            stateName = "Enabled";
        } else {
            stateName = "Disabled";
        }

        const size_t stateSize =
            std::min(stateName.size(), sizeof(stateTrace.stateId) - 1);

        for(size_t i = 0; i < stateSize; i++) {
            stateTrace.stateId[i] = stateName[i];
        }
        stateTrace.stateId[stateSize] = '\0';

        if (traceSubsystem.BinaryOutputIsOn()) {
            traceSubsystem.OutputTraceInBinary(
                currentTime,
                commonImplPtr->objectId,
                modelName,
                instanceName,
                eventName,
                reinterpret_cast<const unsigned char* >(&stateTrace),
                sizeof(stateTrace),
                threadPartitionIndex);

        } else {

            ostringstream outStream;
            outStream << "V= " << value;

            traceSubsystem.OutputTrace(
                currentTime,
                commonImplPtr->objectId,
                modelName,
                instanceName,
                eventName,
                outStream.str(),
                threadPartitionIndex);
        }
    }
}



double GisObject::GetElevationFromGroundMeters() const
{
    return commonImplPtr->elevationFromGroundMeters;
}


bool GisObject::MasTraceIsOn() const
{
    return commonImplPtr->outputMasTrace;
}


bool GisObject::IsEnabled() const
{
    return commonImplPtr->isEnabled;
}

Rectangle Triangle::GetRect() const
{
    return Rectangle(
        std::min(std::min(p1.x, p2.x), p3.x),
        std::min(std::min(p1.y, p2.y), p3.y),
        std::max(std::max(p1.x, p2.x), p3.x),
        std::max(std::max(p1.y, p2.y), p3.y));
}



double Triangle::GetSurfaceHeightAt(const Vertex& pos) const
{
    const Vertex vec1 = p2 - p1;
    const Vertex vec2 = p3 - p1;
    const Vertex p(vec1.y*vec2.z - vec1.z*vec2.y, vec1.z*vec2.x - vec1.x*vec2.z, vec1.x*vec2.y - vec1.y*vec2.x);
    const double d = -(p.x*p1.x + p.y*p1.y + p.z*p1.z);

    if (p.z == 0.) {
        return CalculatePointToLineNearestPosition(pos, p2, p3).z;
    }

    return - (d + (p.x*pos.x + p.y*pos.y))/p.z;
}



bool Triangle::Contains(const Vertex& pos) const
{
    return IsInsideOfTheTriangle(pos, p1, p2, p3);
}



bool Triangle::IntersectsWith(const Rectangle& rect) const
{
    vector<Vertex> polygon;

    polygon.push_back(p1);
    polygon.push_back(p2);
    polygon.push_back(p3);
    polygon.push_back(p1);

    return RectIsIntersectsWithPolygon(rect, polygon);
}



bool Triangle::IntersectsWithLine(
    const Vertex& lineEdge1,
    const Vertex& lineEdge2) const
{
    if ((*this).Contains(lineEdge1) ||
        (*this).Contains(lineEdge2)) {
        return true;
    }

    if (HorizontalLinesAreIntersection(lineEdge1, lineEdge2, p1, p2) ||
        HorizontalLinesAreIntersection(lineEdge1, lineEdge2, p2, p3) ||
        HorizontalLinesAreIntersection(lineEdge1, lineEdge2, p3, p1)) {
        return true;
    }

    return false;
}


void WavefrontObjReader::ReadPolygon(
const vector<string>& objFileNames,
const Vertex& basePosition,
const bool offsettedFileOutput,
vector<Vertex>& vertices,
vector<vector<int> >& faces)
{

    assert(vertices.empty());
    assert(faces.empty());

    size_t vertexIndexOffset = 0;

    for (size_t i = 0; i < objFileNames.size(); i++) {

        ifstream inFile(objFileNames[i].c_str());

        ofstream offsettedFile;
        if (offsettedFileOutput) {
            const size_t dotPos = objFileNames[i].find_last_of(".");
            const string outputFileName =
                objFileNames[i].substr(0, dotPos) + "_offsetted" + objFileNames[i].substr(dotPos);

            offsettedFile.open(outputFileName);
        };


        if (!inFile.good()) {
            cerr << "Could Not open obj file: " << objFileNames[i] << endl;
            exit(1);
        }//if//

        while (!inFile.eof()) {
            string aLine;
            getline(inFile, aLine);

            if (IsAConfigFileCommentLine(aLine)) {
                if (offsettedFileOutput) {
                    offsettedFile << aLine << endl;
                }
                continue;
            }//if//

            DeleteTrailingSpaces(aLine);
            std::istringstream lineStream(aLine);

            string firstLetter;

            lineStream >> firstLetter;

            Vertex offsettedVertex;
            if (firstLetter == "v") {

                //vertex
                double x;
                double y;
                double z;
                lineStream >> x >> y >> z;

                offsettedVertex = Vertex(x, y, z) - basePosition;

                vertices.push_back(offsettedVertex);

            }
            else if (firstLetter == "f") {

                vector<int> face;
                string faceIndices;
                while ((!lineStream.eof()) && (!lineStream.fail())) {

                    lineStream >> faceIndices;
                    deque<string> tokens;
                    TokenizeToTrimmedLowerString(faceIndices, "/", tokens);

                    int vertexIndex = 0;

                    if (tokens.empty() || (tokens.size() > 3)) {
                        cerr << "Error: Bad obj file line: " << aLine << endl;
                        exit(1);
                    }//if//

                    if (tokens.size() >= 1) {
                        bool success;
                        ConvertStringToInt(tokens[0], vertexIndex, success);

                        if (!success) {
                            cerr << "Error: Bad obj file line: " << aLine << endl;
                            exit(1);
                        }//if//

                    }//if//

                    if (tokens.size() >= 2) {
                        //textureIndex
                    }//if//

                    if (tokens.size() == 3) {
                        //normalIndex
                    }//if//

                    //TBD check vertexIndex is positive or not

                    if ((vertexIndexOffset + vertexIndex) > INT_MAX) {
                        cerr << "Error: Too many objects: " << objFileNames[i] << endl;
                        exit(1);

                    }//if//

                    face.push_back(vertexIndex + static_cast<int>(vertexIndexOffset));

                }//while//

                faces.push_back(face);

            }
            else if (firstLetter == "vt") {
                //texture coordinates
                //ignore
            }
            else if (firstLetter == "vn") {
                //vertex normals
                //ignore
            }
            else if (firstLetter == "vp") {
                //parameter space vertices
                //ignore
            }//if//
            else if (firstLetter == "mtllib") {
                //external .mtl file name
                //ignore
            }
            else if (firstLetter == "usemtl") {
                //material name
                //ignore
            }
            else if (firstLetter == "o") {
                //object name
                //ignore
            }
            else if (firstLetter == "g") {
                //group name
                //ignore
            }
            else if (firstLetter == "s") {
                //smoothing
                //ignore
            }
            else {
                //unknown charactor
                //ignore
            }//if//

            if (lineStream.fail()) {
                cerr << "Error: Bad obj file line: " << aLine << endl;
                exit(1);
            }//if//

            if (offsettedFileOutput) {

                if (firstLetter == "v") {
                    offsettedFile << "v " << offsettedVertex.x << " " << offsettedVertex.y << " " << offsettedVertex.z << endl;
                }
                else {
                    offsettedFile << aLine << endl;
                }//if//
            }//if//

        }//while//

        //update vertex index offset
        vertexIndexOffset = vertices.size();

        if (offsettedFileOutput) {
            offsettedFile.close();
        }//if//

    }//for//

    //debug output
    //cout << "#Vertices: " << vertices.size() << endl;
    //for (size_t i = 0; i < vertices.size(); i++) {
    //    cout << "v " << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << endl;
    //}

    //cout << "#Faces: " << faces.size() << endl;
    //for (size_t i = 0; i < faces.size(); i++) {
    //    const vector<int>& face = faces[i];
    //    cout << "f";
    //    for (size_t j = 0; j < face.size(); j++) {
    //        cout << " " << face[j];
    //    }
    //    cout << endl;
    //}

}//ReadPolygon//



}; //namespace ScenSim
