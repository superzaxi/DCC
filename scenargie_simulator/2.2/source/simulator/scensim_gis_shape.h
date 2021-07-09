// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_GIS_SHAPE_H
#define SCENSIM_GIS_SHAPE_H

#include "shapefil.h"

#include "scensim_gis.h"


namespace ScenSim {

//well known field
//Note: case sensitive string with "10" charactors limitation
static const string GIS_DBF_ID_STRING = "id";
static const string GIS_DBF_NAME_STRING = "name";
static const string GIS_DBF_WIDTH_STRING = "width";
static const string GIS_DBF_HEIGHT_STRING = "height";
static const string GIS_DBF_ROOF_MATERIAL_STRING = "roofmateri";
static const string GIS_DBF_FLOOR_MATERIAL_STRING = "floormater";
static const string GIS_DBF_MATERIAL_STRING = "material";
static const string GIS_DBF_WIDTHTYPE_STRING = "widthType";
static const string GIS_DBF_TYPE_STRING = "type";
static const string GIS_DBF_RADIUS_STRING = "radius";
static const string GIS_DBF_GENERATION_VOLUME_STRING = "generation";
static const string GIS_DBF_START_POINT_ID_STRING = "startPtId";
static const string GIS_DBF_END_POINT_ID_STRING = "endPtId";
static const string GIS_DBF_STATION_ID_STRING = "staId";
static const string GIS_DBF_STATION_NAME_STRING = "staName";
static const string GIS_DBF_BUILDING_ID_STRING = "buildingId";
static const string GIS_DBF_LANE12_STRING = "lane12";
static const string GIS_DBF_LANE21_STRING = "lane21";
static const string GIS_DBF_MESH_KIND_STRING = "meshKind";
static const string GIS_DBF_NUMBER_OF_ROOF_FACES_STRING = "nofRoof";
static const string GIS_DBF_NUMBER_OF_WALL_FACES_STRING = "nofWall";
static const string GIS_DBF_NUMBER_OF_FLOOR_FACES_STRING = "nofFloor";
static const string GIS_DBF_CAPACITY_STRING = "capacity";
static const string GIS_DBF_VEHICLE_CAPACITY_STRING = "vehiclecap";
static const string GIS_DBF_SPEED_LIMIT_STRING = "speedlimit";
static const string GIS_DBF_INFO_STRING = "info";

static const string GIS_DBF_OFFSET_STRING = "signaloff";
static const string GIS_DBF_GREEN_STRING = "green";
static const string GIS_DBF_YELLOW_STRING = "yellow";
static const string GIS_DBF_RED_STRING = "red";
static const string GIS_DBF_PATTERN_STRING = "pattern";
static const string GIS_DBF_INTERSECTIONID_STRING = "intersecid";
static const string GIS_DBF_OBJECTID_STRING = "objectid";
static const string GIS_DBF_ROADID_STRING = "roadid";

//----------------------------------------------------------
// Shape Handling
//----------------------------------------------------------

class AttributeFinder {
public:
    AttributeFinder()
        :
        dbfHandle(nullptr),
        fieldNumber(-1),
        fieldType(FTInvalid)
    {}

    AttributeFinder(
        const DBFHandle initDbfHandle,
        const string& initAttributeName)
        :
        dbfHandle(initDbfHandle),
        attributeName(initAttributeName),
        fieldNumber(-1)
    {
        string lowerAttributeName = attributeName;
        ConvertStringToLowerCase(lowerAttributeName);

        const int numberAttributes = DBFGetFieldCount(dbfHandle);

        for (int i = 0; i < numberAttributes; i++) {

            char rawAttributeName[12];
            int width;
            int digits;

            DBFFieldType aFeldType =
                DBFGetFieldInfo(dbfHandle, i, rawAttributeName, &width, &digits);

            string fieldNameString = rawAttributeName;
            ConvertStringToLowerCase(fieldNameString);

            if (fieldNameString == lowerAttributeName) {
                fieldType = aFeldType;
                fieldNumber = i;
                break;
            }
        }
    }

    bool IsAvailable() const { return (fieldNumber != -1); }

    double GetDouble(const int shapeId) const {
        (*this).ErrorIfNotAvailable();

        if (fieldType == FTDouble) {
            return (ReadDoubleValue(shapeId));
        }
        else if (fieldType == FTInteger) {
            return (ReadIntValue(shapeId));
        }
        else {
            const string stringValue = ReadStringValue(shapeId);
            bool success;
            double aValue;
            ConvertStringToDouble(stringValue, aValue, success);

            if (!success) {
                cerr << "Wrong format string for double value" << stringValue << endl;
                exit(1);
            }
            return aValue;
        }
    }

    int GetInt(const int shapeId) const {
        (*this).ErrorIfNotAvailable();

        if (fieldType == FTInteger) {
            return (ReadIntValue(shapeId));
        }
        else if (fieldType == FTDouble) {
            return (static_cast<int>(ReadDoubleValue(shapeId)));
        }
        else {
            const string stringValue = ReadStringValue(shapeId);
            bool success;
            int aValue;
            ConvertStringToInt(stringValue, aValue, success);

            if (!success) {
                cerr << "Wrong format string for int value" << stringValue << endl;
                exit(1);
            }
            return aValue;
        }
    }

    const string GetString(const int shapeId) const {
        (*this).ErrorIfNotAvailable();

        if (fieldType == FTDouble) {
            return (ConvertToString(ReadDoubleValue(shapeId)));
        }
        else if (fieldType == FTInteger) {
            return(ConvertToString(ReadIntValue(shapeId)));
        }
        return (ReadStringValue(shapeId));
    }

    const string GetLowerString(const int shapeId) const {
        return (MakeLowerCaseString(GetString(shapeId)));
    }

    GisObjectIdType GetGisObjectId(const int shapeId) const {
        return (GisObjectIdType(GetInt(shapeId)));
    }

private:
    void ErrorIfNotAvailable() const {
        if (!(*this).IsAvailable()) {
            cerr << "Error: Attribute " << attributeName << " doesn't exist in DBF" << endl;
            exit(1);
        }
    }

    const DBFHandle dbfHandle;
    const string attributeName;

    int fieldNumber;
    DBFFieldType fieldType;

    double ReadDoubleValue(const int shapeId) const
    {
        assert(fieldType == FTDouble);
        return (DBFReadDoubleAttribute(dbfHandle, shapeId, fieldNumber));
    }


    int ReadIntValue(const int shapeId) const
    {
        assert(fieldType == FTInteger);
        return (DBFReadIntegerAttribute(dbfHandle, shapeId, fieldNumber));
    }


    string ReadStringValue(const int shapeId) const
    {
        assert(fieldType == FTString);
        return (DBFReadStringAttribute(dbfHandle, shapeId, fieldNumber));
    }

};//AttributeFinder//

inline
void GetStandardShapeInfo(
    const string& filePath,
    const bool isDebugMode,
    SHPHandle& hSHP,
    DBFHandle& hDBF,
    int& entities,
    Rectangle& rect)
{
    hSHP = SHPOpen(filePath.c_str(), "rb");
    if (hSHP == nullptr) {
        cerr << "Cannot open .shp file: " << filePath << endl;
        exit(1);
    }

    const size_t dotPos = filePath.find_last_of(".");
    const string fileNamePrefix = filePath.substr(0, dotPos);
    const string dbfFilePath = fileNamePrefix + ".dbf";

    hDBF = DBFOpen(dbfFilePath.c_str(), "rb");
    if (hDBF == NULL) {
        cerr << "Cannot open .dbf file: " << dbfFilePath << endl;
        SHPClose(hSHP);
        exit(1);
    }

    double adfMinBound[4];
    double adfMaxBound[4];
    int shapeFileType;

    SHPGetInfo(hSHP, &entities, &shapeFileType, adfMinBound, adfMaxBound);

    rect.minX = adfMinBound[0];
    rect.minY = adfMinBound[1];
    rect.maxX = adfMaxBound[0];
    rect.maxY = adfMaxBound[1];

    const int fields = DBFGetFieldCount(hDBF);
    const int records = DBFGetRecordCount(hDBF);

    if (isDebugMode) {
        cout << "Loading... " << filePath << endl
            << "Entries: " << entities << ", Fields: " << fields
            << ", Records: " << records << ", GisObjectType: " << GIS_ROAD
            << ", ShapeType: " << shapeFileType << endl;
    }

    assert(entities == records);
}

inline
Rectangle PeekLayerRectangle(const string& filePath)
{
    const bool isDebugMode = false;
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, isDebugMode, hSHP, hDBF, entities, layerRect);

    SHPClose(hSHP);
    DBFClose(hDBF);

    const double extensionMargin = 100;

    return layerRect.Expanded(extensionMargin);
}


}; //namespace ScenSim

#endif
