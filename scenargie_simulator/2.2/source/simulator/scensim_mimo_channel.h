// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_MIMO_CHANNEL_H
#define SCENSIM_MIMO_CHANNEL_H

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <complex>
#include <array>
#include "boost/numeric/ublas/matrix.hpp"
#include "boost/thread.hpp"

#include "scensim_time.h"
#include "scensim_nodeid.h"
#include "scensim_support.h"
#include "scensim_parmio.h"
#include "tgn_mimo_ch_model.h"
#include "scensim_mobility.h"
#include "scensim_prop.h"

namespace ScenSim {

using std::cerr;
using std::endl;
using std::string;
using std::unique_ptr;
using std::shared_ptr;
using std::enable_shared_from_this;
using std::map;
using std::vector;
using std::complex;
using std::array;
using std::istream;
using std::istringstream;
using ScenSim::SimTime;
using ScenSim::INFINITE_TIME;
using ScenSim::NodeId;
using ScenSim::ConvertToString;
using ScenSim::IsAConfigFileCommentLine;
using ScenSim::DeleteTrailingSpaces;
using ScenSim::ConvertStringToTime;
using ScenSim::ConvertStringToLowerCase;
using ScenSim::FastIntToDataMap;
using namespace boost::numeric;
using TgnMimoChModel::sqrt2;
using ScenSim::PropagationLinkPrecalculationInterface;


//--------------------------------------------------------------------------------------------------
//
// Localized (to node) interface for for future caching and thread issues (sharing) to global
// models such as a pathloss matrix.
//

typedef ublas::matrix<complex<double> > MimoChannelMatrix;

const unsigned int MaxMimoMatrixRowsOrColumns = 8;

class MimoChannelModelInterface {
public:
    virtual ~MimoChannelModelInterface() { }

    virtual unsigned int GetNumberOfAntennas() const = 0;
    virtual unsigned int GetNumberOfAntennasFor(const NodeId& otherNodeId) const = 0;

    // No sectors version:

    virtual void GetChannelMatrix(
        const SimTime& time,
        const unsigned int channelNumber,
        const NodeId& sourceNodeId,
        const unsigned int subcarrierIndex,
        MimoChannelMatrix& mimoChannelMatrix) = 0;

    virtual void GetChannelMatrixInTimeRange(
        const SimTime& earliestTime,
        const SimTime& time,
        const unsigned int channelNumber,
        const NodeId& sourceNodeId,
        const unsigned int subcarrierIndex,
        MimoChannelMatrix& mimoChannelMatrix) = 0;

    // Has sectors version:

    virtual void GetChannelMatrix(
        const SimTime& time,
        const unsigned int channelNumber,
        const NodeId& sourceNodeId,
        const unsigned int cellSectorIndex,
        const unsigned int subcarrierIndex,
        MimoChannelMatrix& mimoChannelMatrix) = 0;

    // Support for no copy method version (reference to internal structure).
    // (Likely use: does not have to support reverse links (transposed matrices)).
    // Logically "const" but not physically const as the channel matrix may
    // be generated "ondemand" (as opposed to during SetTime()).

    virtual const MimoChannelMatrix& GetChannelMatrix(
        const SimTime& time,
        const unsigned int channelNumber,
        const NodeId& sourceNodeId,
        const unsigned int subcarrierIndex) = 0;

    virtual const MimoChannelMatrix& GetChannelMatrix(
        const SimTime& time,
        const unsigned int channelNumber,
        const NodeId& sourceNodeId,
        const unsigned int cellSectorIndex,
        const unsigned int subcarrierIndex) = 0;


    virtual void InitializeLinkIfNotYetInited(
        const SimTime& time,
        const unsigned int channelNumber,
        const NodeId& sourceNodeId) = 0;

};//MimoChannelModelInterface//



class MimoChannelModel {
public:
    virtual ~MimoChannelModel() {}

    virtual void CreateNewMimoChannelModelInterface(
        const ParameterDatabaseReader& theParameterDatabase,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
        shared_ptr<MimoChannelModelInterface>& interfacePtr) = 0;

    // For multi-sector basestations and APs.

    virtual void CreateNewMimoChannelModelInterfaceForBasestation(
        const ParameterDatabaseReader& theParameterDatabase,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int cellSectorId,
        const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
        shared_ptr<MimoChannelModelInterface>& interfacePtr) = 0;

};//MimoChannelModel//




class CannedFileMimoChannelModel : public MimoChannelModel,
    public enable_shared_from_this<CannedFileMimoChannelModel> {
public:

    CannedFileMimoChannelModel(
        const ParameterDatabaseReader& theParameterDatabase,
        const InterfaceOrInstanceId& propagationModelInstanceId,
        const unsigned int initBaseChannelNumber,
        const unsigned int numberChannels);

    virtual void CreateNewMimoChannelModelInterface(
            const ParameterDatabaseReader& theParameterDatabase,
            const NodeId& theNodeId,
            const InterfaceId& theInterfaceId,
            const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
            shared_ptr<MimoChannelModelInterface>& interfacePtr) override;

    virtual void CreateNewMimoChannelModelInterfaceForBasestation(
            const ParameterDatabaseReader& theParameterDatabase,
            const NodeId& theNodeId,
            const InterfaceId& theInterfaceId,
            const unsigned int cellSectorId,
            const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
            shared_ptr<MimoChannelModelInterface>& interfacePtr) override;

private:
    void SetTime(
        const unsigned int channelNumber,
        const SimTime& timeForData);

    void SetTime(const SimTime& timeForData);

    unsigned int GetNumberOfAntennasFor(const NodeId& theNodeId) const;

    void GetChannelMatrix(
        const unsigned int channelIndex,
        const NodeId& nodeId1,
        const unsigned int nodeId1CellSectorId,
        const NodeId& nodeId2,
        const unsigned int nodeId2CellSectorId,
        const unsigned int subcarrierIndex,
        MimoChannelMatrix& mimoChannelMatrix) const;

    // Note: Const reference to internal structure for efficiency.

    const MimoChannelMatrix& GetChannelMatrix(
        const unsigned int channelIndex,
        const NodeId& nodeId1,
        const unsigned int nodeId1CellSectorId,
        const NodeId& nodeId2,
        const unsigned int nodeId2CellSectorId,
        const unsigned int subcarrierIndex) const;

    InterfaceOrInstanceId propagationModelInstanceId;

    struct FileDataDescriptorType {
        bool hasBeenSet;
        SimTime timeForData;
        NodeId nodeId1;
        unsigned int node1CellSectorIndex;
        unsigned int node1NumberAntennas;
        NodeId nodeId2;
        unsigned int node2CellSectorIndex;
        unsigned int node2NumberAntennas;
        unsigned int numberSubcarriers;

        FileDataDescriptorType() : hasBeenSet(false) { }

    };//FileDataDescriptorType//

    //----------------------------------

    struct LinkKeyType {

        NodeId nodeId1;
        unsigned int node1CellSector;
        NodeId nodeId2;
        unsigned int node2CellSector;

        LinkKeyType(
            const NodeId& initNodeId1,
            const unsigned int initNode1CellSector,
            const NodeId& initNodeId2,
            const unsigned int initNode2CellSector)
        :
            nodeId1(initNodeId1),
            node1CellSector(initNode1CellSector),
            nodeId2(initNodeId2),
            node2CellSector(initNode2CellSector)
        {
            if (nodeId1 > nodeId2) {
                std::swap(nodeId1, nodeId2);
                std::swap(node1CellSector, node2CellSector);
            }//if//
        }

        bool operator<(const LinkKeyType& right) const
        {
            assert(nodeId1 <= nodeId2);

            if (nodeId1 < right.nodeId1) {
                return true;
            }

            if (nodeId1 > right.nodeId1) {
                return false;
            }

            if (node1CellSector < right.node1CellSector) {
                return true;
            }

            if (node1CellSector > right.node1CellSector) {
                return false;
            }

            if (nodeId2 < right.nodeId2) {
                return true;
            }

            if (nodeId2 > right.nodeId2) {
                return false;
            }

            return (node2CellSector < right.node2CellSector);

        }//"<"

    };//LinkKeyType//

    //----------------------------------


    struct MimoLinkInfoType {
        // Link direction is defined as nodeId1 --> nodeId2 where nodeId1 < nodeId2
        // Data is "reversed" if data is from a higher node ID to a lower node ID.
        bool linkDataDirectionIsReversed;
        vector<MimoChannelMatrix> mimoChannelMatrices;

    };//MimoLinkInfoType//


    struct ChannelDataType {

        string mimoDataFileName;
        std::ifstream mimoDataFile;

        // For error messages:

        unsigned int currentFileLineNumber;

        FileDataDescriptorType currentFileDataDescriptor;

        map<LinkKeyType, MimoLinkInfoType> linkInfos;

        SimTime loopStartTime;

        // For assert checks only.
        SimTime lastSetTimeCallTime;

        ChannelDataType() :
            currentFileLineNumber(0),
            lastSetTimeCallTime(ZERO_TIME),
            loopStartTime(ZERO_TIME) {}

    };//ChannelDataType//

    FastIntToDataMap<NodeId, unsigned int> numberOfAntennasMap;

    vector<shared_ptr<ChannelDataType> > channels;
    unsigned int baseChannelNumber;

    bool dataLoopingIsEnabled;

    //----------------------------------------------------------------------------------------------

    class ModelInterface: public MimoChannelModelInterface {
    public:

        virtual unsigned int GetNumberOfAntennas() const override
        {
            return(channelModelPtr->GetNumberOfAntennasFor(theNodeId));
        }

        virtual unsigned int GetNumberOfAntennasFor(const NodeId& otherNodeId) const override
        {
            return(channelModelPtr->GetNumberOfAntennasFor(otherNodeId));
        }

        virtual void GetChannelMatrix(
            const SimTime& time,
            const unsigned int channelNumber,
            const NodeId& sourceNodeId,
            const unsigned int subcarrierIndex,
            MimoChannelMatrix& mimoChannelMatrix) override
        {
            channelModelPtr->SetTime(time);

            channelModelPtr->GetChannelMatrix(
                channelNumber,
                sourceNodeId,
                0,
                theNodeId,
                0,
                subcarrierIndex,
                mimoChannelMatrix);
        }


        virtual void GetChannelMatrixInTimeRange(
            const SimTime& earliestTime,
            const SimTime& time,
            const unsigned int channelNumber,
            const NodeId& sourceNodeId,
            const unsigned int subcarrierIndex,
            MimoChannelMatrix& mimoChannelMatrix) override
        {
            channelModelPtr->SetTime(time);

            channelModelPtr->GetChannelMatrix(
                channelNumber,
                sourceNodeId,
                0,
                theNodeId,
                0,
                subcarrierIndex,
                mimoChannelMatrix);
        }


        virtual void GetChannelMatrix(
            const SimTime& time,
            const unsigned int channelNumber,
            const NodeId& otherNodeId,
            const unsigned int otherNodeCellSectorId,
            const unsigned int subcarrierIndex,
            MimoChannelMatrix& mimoChannelMatrix) override
        {
            channelModelPtr->SetTime(time);

            channelModelPtr->GetChannelMatrix(
                channelNumber,
                otherNodeId,
                otherNodeCellSectorId,
                theNodeId,
                cellSectorId,
                subcarrierIndex,
                mimoChannelMatrix);
        }

        virtual void InitializeLinkIfNotYetInited(
            const SimTime& time,
            const unsigned int channelNumber,
            const NodeId& sourceNodeId) override
        {
        }

        virtual const MimoChannelMatrix& GetChannelMatrix(
            const SimTime& time,
            const unsigned int channelNumber,
            const NodeId& sourceNodeId,
            const unsigned int subcarrierIndex) override
        {
            channelModelPtr->SetTime(time);

            return (
                channelModelPtr->GetChannelMatrix(
                    channelNumber, sourceNodeId, 0, theNodeId, 0, subcarrierIndex));
        }

        virtual const MimoChannelMatrix& GetChannelMatrix(
            const SimTime& time,
            const unsigned int channelNumber,
            const NodeId& sourceNodeId,
            const unsigned int sourceCellSectorId,
            const unsigned int subcarrierIndex) override
        {
            channelModelPtr->SetTime(time);

            return (
                channelModelPtr->GetChannelMatrix(
                    channelNumber, sourceNodeId, sourceCellSectorId,
                    theNodeId, cellSectorId, subcarrierIndex));
        }


    private:

        ModelInterface(
            const shared_ptr<CannedFileMimoChannelModel>& channelModelPtr,
            const NodeId& theNodeId,
            const unsigned int cellSectorId);

        shared_ptr<CannedFileMimoChannelModel> channelModelPtr;

        NodeId theNodeId;
        unsigned int cellSectorId;

        friend class CannedFileMimoChannelModel;

    };//ModelInterface//


    //----------------------------------------------------------------------------------------------

    void SaveNumberOfAntennaValues(const FileDataDescriptorType& dataDescriptor);

    static
    void ReadALine(
        std::istream& aStream,
        unsigned int& currentFileLineNum,
        bool& success,
        string& aLine);

    static
    void ReadDataDescriptorLine(
        const string& mimoDataFileName,
        istream& mimoDataFile,
        unsigned int& currentFileLineNumber,
        FileDataDescriptorType& dataDescriptor,
        bool& success);

    static
    void ReadMimoChannelMatricesForAllSubcarriers(
        const string& mimoDataFileName,
        istream& mimoDataFile,
        unsigned int& currentFileLineNumber,
        const FileDataDescriptorType& dataDescriptor,
        MimoLinkInfoType& linkInfo);

};//CannedFileMimoChannelModel//


inline
CannedFileMimoChannelModel::CannedFileMimoChannelModel(
    const ParameterDatabaseReader& theParameterDatabase,
    const InterfaceOrInstanceId& initPropagationModelInstanceId,
    const unsigned int initBaseChannelNumber,
    const unsigned int numberChannels)
    :
    propagationModelInstanceId(initPropagationModelInstanceId),
    channels(numberChannels),
    baseChannelNumber(initBaseChannelNumber),
    dataLoopingIsEnabled(false)
{
    for(unsigned int i = 0; (i < numberChannels); i++) {
        channels[i].reset(new ChannelDataType());
        ChannelDataType& channel = *channels[i];

        if ((numberChannels == 1) &&
            theParameterDatabase.ParameterExists(
                "mimo-channel-file-name",
                propagationModelInstanceId)) {

            channel.mimoDataFileName = theParameterDatabase.ReadString(
                "mimo-channel-file-name",
                propagationModelInstanceId);
        }
        else {
            const string parameterName =
                ("mimo-channel-" + ConvertToString(baseChannelNumber + i) + "-file-name");

            channel.mimoDataFileName =
                theParameterDatabase.ReadString(parameterName, propagationModelInstanceId);
        }//if//

        channel.mimoDataFile.open(channel.mimoDataFileName.c_str());

        if (channel.mimoDataFile.fail()) {
            cerr << "Error in Mimo Channel Model: Could not open "
                 << ("mimo-channel-model-file-prefix") << " file:" << endl;
            cerr << "     " << channel.mimoDataFileName << endl;
            exit(1);
        }//if//

        (*this).SetTime((baseChannelNumber + i), ZERO_TIME);
    }//for//

    if (theParameterDatabase.ParameterExists(
        "mimo-channel-model-enable-file-looping", propagationModelInstanceId)) {

        dataLoopingIsEnabled =
            theParameterDatabase.ReadBool(
                "mimo-channel-model-enable-file-looping", propagationModelInstanceId);

    }//if//



}//CannedFileMimoChannelModel()//


inline
unsigned int CannedFileMimoChannelModel::GetNumberOfAntennasFor(const NodeId& theNodeId) const
{
    if (!numberOfAntennasMap.IsMapped(theNodeId)) {
        cerr << "Error in CannedFileMimoChannelModel: No data for Node ID = " << theNodeId << "." << endl;
        exit(1);
    }//if//

    return (numberOfAntennasMap.GetValue(theNodeId));
}



inline
void CannedFileMimoChannelModel::ReadALine(
    std::istream& aStream,
    unsigned int& currentFileLineNum,
    bool& success,
    string& aLine)
{
    success = false;
    aLine.clear();

    do {
        if (aStream.eof()) {
            return;
        }//if//

        getline(aStream, aLine);
        currentFileLineNum++;

        if (aStream.fail()) {
            return;
        }//if//
    } while (IsAConfigFileCommentLine(aLine));

    DeleteTrailingSpaces(aLine);
    success = true;

}//ReadALine//



inline
void CannedFileMimoChannelModel::ReadDataDescriptorLine(
    const string& mimoDataFileName,
    istream& mimoDataFile,
    unsigned int& currentFileLineNumber,
    FileDataDescriptorType& dataDescriptor,
    bool& success)
{
    dataDescriptor.hasBeenSet = false;
    success = false;

    string aLine;
    ReadALine(mimoDataFile, currentFileLineNumber, success, aLine);
    if (!success) {
        return;
    }//if//

    ConvertStringToLowerCase(aLine);

    istringstream lineStream(aLine);

    string timeForDataString;
    lineStream >> timeForDataString;

    ConvertStringToTime(timeForDataString, dataDescriptor.timeForData, success);

    lineStream >> dataDescriptor.nodeId1;
    lineStream >> dataDescriptor.node1CellSectorIndex;
    lineStream >> dataDescriptor.node1NumberAntennas;
    lineStream >> dataDescriptor.nodeId2;
    lineStream >> dataDescriptor.node2CellSectorIndex;
    lineStream >> dataDescriptor.node2NumberAntennas;
    lineStream >> dataDescriptor.numberSubcarriers;

    if ((!success) || (lineStream.fail())) {
        cerr << "Error in Mimo Data file: "
             << "\"" << mimoDataFileName << "\"" << " at line # "
             << currentFileLineNumber << " " << endl;
        cerr << "     " << aLine << endl;
        exit(1);
    }//if//

    dataDescriptor.hasBeenSet = true;

    assert(success);

}//ReadDataDescriptorLine//



inline
void ReadInNextNonSpaceChar(
    istream& inputStream,
    bool& success,
    char& aChar)
{
    success = false;

    aChar = ' ';
    while ((!inputStream.eof()) && (aChar == ' ')) {
        inputStream >> aChar;
        assert((!inputStream.fail()) || (inputStream.eof()));
    }//while//

    success = (aChar != ' ');

}//ReadInNextNonSpaceChar//



inline
void ReadComplexMatrix(
    istream& inputStream,
    bool& success,
    MimoChannelMatrix& matrix)
{
    assert((matrix.size1() != 0) && (matrix.size2() != 0) && "Matrix must be preallocated");

    success = false;

    bool readWasSuccessful;
    char aChar;
    ReadInNextNonSpaceChar(inputStream, readWasSuccessful, aChar);

    if ((!readWasSuccessful) || (aChar != '{')) {
        return;
    }//if//

    unsigned int i = 0;
    while(true) {

        ReadInNextNonSpaceChar(inputStream, readWasSuccessful, aChar);
        if ((!readWasSuccessful) || (aChar != '{')) {
            return;
        }//if//

        array<complex<double>, MaxMimoMatrixRowsOrColumns> rowVector;

        unsigned int j = 0;
        while (true) {

            if ((i >= matrix.size1()) || (j >= matrix.size2())) {
                return;
            }

            inputStream >> matrix(i, j);

            if (inputStream.fail()) {
                return;
            }//if//

            ReadInNextNonSpaceChar(inputStream, readWasSuccessful, aChar);
            if (!readWasSuccessful) {
                return;
            }//if//

            if (aChar == '}') {
                break;
            }//if//

            if (aChar != ',') {
                return;
            }//if//

            j++;

        }//while//

        if ((j+1) != matrix.size2()) {
           // Too few values in the row.
           return;
        }//if//

        ReadInNextNonSpaceChar(inputStream, readWasSuccessful, aChar);
        if (!readWasSuccessful) {
            return;
        }//if//

        if (aChar == '}') {
            break;
        }//if//

        if (aChar != ',') {
            return;
        }//if//

        i++;

    }//while//

    if ((i+1) != matrix.size1()) {
       // Too few rows.
       return;
    }//if//

    success = true;

}//ReadComplexMatrix//


inline
void CannedFileMimoChannelModel::ReadMimoChannelMatricesForAllSubcarriers(
    const string& mimoDataFileName,
    istream& mimoDataFile,
    unsigned int& currentFileLineNumber,
    const FileDataDescriptorType& dataDescriptor,
    MimoLinkInfoType& linkInfo)
{
    linkInfo.linkDataDirectionIsReversed = (dataDescriptor.nodeId1 > dataDescriptor.nodeId2);
    linkInfo.mimoChannelMatrices.resize(dataDescriptor.numberSubcarriers);

    for(unsigned int i = 0; (i < dataDescriptor.numberSubcarriers); i++) {

        MimoChannelMatrix& mimoMatrix = linkInfo.mimoChannelMatrices[i];

        mimoMatrix.resize(
            dataDescriptor.node2NumberAntennas,
            dataDescriptor.node1NumberAntennas,
            false);

        string aLine;
        bool success;

        ReadALine(
            mimoDataFile,
            currentFileLineNumber,
            success,
            aLine);

        if (!success) {
            cerr << "Error in MIMO File: " << mimoDataFileName << " Not enough data for all subcarriers." << endl;
            cerr << "    Line# " << currentFileLineNumber << endl;
            exit(1);
        }//if//

        istringstream lineStream(aLine);

        ReadComplexMatrix(
            lineStream,
            success,
            mimoMatrix);

        if (!success) {
            cerr << "Error in MIMO File: " << mimoDataFileName << " Line # " << currentFileLineNumber << endl;
            cerr << "     " << aLine << endl;
            cerr << "   Bad format or Matrix dimensions do not agree with data header." << endl;
            exit(1);
        }//if//

    }//for//

}//ReadMimoChannelMatricesForAllSubcarriers//



inline
void CannedFileMimoChannelModel::SaveNumberOfAntennaValues(
    const FileDataDescriptorType& dataDescriptor)
{
    if (!numberOfAntennasMap.IsMapped(dataDescriptor.nodeId1)) {
        numberOfAntennasMap.InsertMapping(
            dataDescriptor.nodeId1, dataDescriptor.node1NumberAntennas);

    }
    else {
        assert(numberOfAntennasMap.GetValue(dataDescriptor.nodeId1) ==
               dataDescriptor.node1NumberAntennas);
    }//if//

    if (!numberOfAntennasMap.IsMapped(dataDescriptor.nodeId2)) {
        numberOfAntennasMap.InsertMapping(
            dataDescriptor.nodeId2, dataDescriptor.node2NumberAntennas);

    }
    else {
        assert(numberOfAntennasMap.GetValue(dataDescriptor.nodeId2) ==
               dataDescriptor.node2NumberAntennas);
    }//if//

}//SaveNumberOfAntennaValues//




inline
void CannedFileMimoChannelModel::SetTime(
    const unsigned int channelNumber,
    const SimTime& timeForData)
{
    ChannelDataType& channel = *channels.at(channelNumber - baseChannelNumber);

    if (channel.mimoDataFile.eof()) {
        // No more data to read.
        return;
    }//if//

    if (!channel.currentFileDataDescriptor.hasBeenSet) {
        bool success;
        (*this).ReadDataDescriptorLine(
            channel.mimoDataFileName,
            channel.mimoDataFile,
            channel.currentFileLineNumber,
            channel.currentFileDataDescriptor,
            success);

        if (!success) {
            cerr << "Error reading Mimo Data File: " << channel.mimoDataFileName << endl;
            exit(1);
        }//if//

        (*this).SaveNumberOfAntennaValues(channel.currentFileDataDescriptor);

    }//if//

    assert(channel.lastSetTimeCallTime <= timeForData);
    channel.lastSetTimeCallTime = timeForData;

    // Read in data until next data set will be in the future.

    while ((channel.loopStartTime + channel.currentFileDataDescriptor.timeForData) <= timeForData) {

        const LinkKeyType linkKey(
            channel.currentFileDataDescriptor.nodeId1,
            channel.currentFileDataDescriptor.node1CellSectorIndex,
            channel.currentFileDataDescriptor.nodeId2,
            channel.currentFileDataDescriptor.node2CellSectorIndex);

        MimoLinkInfoType& linkInfo = channel.linkInfos[linkKey];

        (*this).ReadMimoChannelMatricesForAllSubcarriers(
            channel.mimoDataFileName,
            channel.mimoDataFile,
            channel.currentFileLineNumber,
            channel.currentFileDataDescriptor,
            linkInfo);

        bool success;

        (*this).ReadDataDescriptorLine(
            channel.mimoDataFileName,
            channel.mimoDataFile,
            channel.currentFileLineNumber,
            channel.currentFileDataDescriptor,
            success);

        if (!success) {
            if (!channel.mimoDataFile.eof()) {
                cerr << "Error reading Mimo Data File: " << channel.mimoDataFileName << endl;
                exit(1);
            }//if//

            if (dataLoopingIsEnabled) {
                channel.mimoDataFile.close();
                channel.mimoDataFile.open(channel.mimoDataFileName.c_str());

                channel.loopStartTime += channel.currentFileDataDescriptor.timeForData;

                (*this).ReadDataDescriptorLine(
                    channel.mimoDataFileName,
                    channel.mimoDataFile,
                    channel.currentFileLineNumber,
                    channel.currentFileDataDescriptor,
                    success);

                if (!success) {
                    cerr << "Error reading Mimo Data File: " << channel.mimoDataFileName << endl;
                    exit(1);
                }//if//
            }
            else {
                break;

            }//if//
        }//if//

        (*this).SaveNumberOfAntennaValues(channel.currentFileDataDescriptor);

    }//while//

}//SetTime//


inline
void CannedFileMimoChannelModel::SetTime(const SimTime& timeForData)
{
    for(unsigned int i = 0; (i < channels.size()); i++) {
        (*this).SetTime((baseChannelNumber + i), timeForData);
    }//for//

}//SetTime//


inline
const MimoChannelMatrix& CannedFileMimoChannelModel::GetChannelMatrix(
    const unsigned int channelNumber,
    const NodeId& nodeId1,
    const unsigned int node1CellSectorId,
    const NodeId& nodeId2,
    const unsigned int node2CellSectorId,
    const unsigned int subcarrierIndex) const
{
    assert(nodeId1 != nodeId2);

    const ChannelDataType& channelData = *channels.at(channelNumber - baseChannelNumber);

    const bool linkIsReversed = (nodeId1 > nodeId2);

    LinkKeyType linkKey(
        nodeId1,
        node1CellSectorId,
        nodeId2,
        node2CellSectorId);

    typedef map<LinkKeyType, MimoLinkInfoType>::const_iterator IterType;

    IterType iter = channelData.linkInfos.find(linkKey);

    if (iter == channelData.linkInfos.end()) {
        cerr << "Error in file MIMO Channel Model: No data for link Node1 = "
             << nodeId1 << " Sector = " << node1CellSectorId << " to Node2 = "
             << nodeId2 << " Sector = " << node2CellSectorId << endl;
        exit(1);
    }//if//

    const MimoLinkInfoType& linkInfo = iter->second;

    if (linkIsReversed != linkInfo.linkDataDirectionIsReversed) {
        cerr << "Error in file MIMO Channel Model: Link data is the wrong direction for Node1 = "
             << nodeId1 << " Sector = " << node1CellSectorId << " to Node2 = "
             << nodeId2 << " Sector = " << node2CellSectorId << endl;
        exit(1);
    }//if//

    if (subcarrierIndex >= linkInfo.mimoChannelMatrices.size()) {
        cerr << "Error in file MIMO Channel Model: Incorrect subcarrier #: "
             << subcarrierIndex << " for link Node1 = "
             << nodeId1 << " Sector = " << node1CellSectorId << " to Node2 = "
             << nodeId2 << " Sector = " << node2CellSectorId << endl;
        exit(1);
    }//if//

    return (linkInfo.mimoChannelMatrices.at(subcarrierIndex));

}//GetChannelMatrix//



inline
void CannedFileMimoChannelModel::GetChannelMatrix(
    const unsigned int channelNumber,
    const NodeId& nodeId1,
    const unsigned int node1CellSectorId,
    const NodeId& nodeId2,
    const unsigned int node2CellSectorId,
    const unsigned int subcarrierIndex,
    MimoChannelMatrix& mimoChannelMatrix) const
{
    assert(nodeId1 != nodeId2);

    const ChannelDataType& channelData = *channels.at(channelNumber - baseChannelNumber);

    const bool linkIsReversed = (nodeId1 > nodeId2);

    LinkKeyType linkKey(
        nodeId1,
        node1CellSectorId,
        nodeId2,
        node2CellSectorId);

    typedef map<LinkKeyType, MimoLinkInfoType>::const_iterator IterType;

    IterType iter = channelData.linkInfos.find(linkKey);

    if (iter == channelData.linkInfos.end()) {
        cerr << "Error in file MIMO Channel Model: No data for link Node1 = "
             << nodeId1 << "Sector = " << node1CellSectorId << " to Node2 = "
             << nodeId2 << "Sector = " << node2CellSectorId << endl;
        exit(1);
    }//if//

    const MimoLinkInfoType& linkInfo = iter->second;
    const MimoChannelMatrix& theMatrix = linkInfo.mimoChannelMatrices.at(subcarrierIndex);


    if (linkIsReversed == linkInfo.linkDataDirectionIsReversed) {
        mimoChannelMatrix.resize(theMatrix.size1(), theMatrix.size2(), false);
        mimoChannelMatrix = theMatrix;
    }
    else {
        // Transpose stored matrix

        mimoChannelMatrix.resize(theMatrix.size2(), theMatrix.size1(), false);

        for(unsigned int i = 0; (i < mimoChannelMatrix.size1()); i++) {
            for(unsigned int j = 0; (j < mimoChannelMatrix.size2()); j++) {
                mimoChannelMatrix(i, j) = theMatrix(j, i);
            }//for//
        }//for//
    }//if//

}//GetChannelMatrix//


//--------------------------------------------------------------------------------------------------

inline
CannedFileMimoChannelModel::ModelInterface::ModelInterface(
    const shared_ptr<CannedFileMimoChannelModel>& initChannelModelPtr,
    const NodeId& initNodeId,
    const unsigned int initCellSectorId)
    :
    channelModelPtr(initChannelModelPtr),
    theNodeId(initNodeId),
    cellSectorId(initCellSectorId)
{
}


inline
void CannedFileMimoChannelModel::CreateNewMimoChannelModelInterface(
        const ParameterDatabaseReader& theParameterDatabase,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
        shared_ptr<MimoChannelModelInterface>& interfacePtr)
{
    interfacePtr.reset(new ModelInterface(shared_from_this(), theNodeId, 0));
}


inline
void CannedFileMimoChannelModel::CreateNewMimoChannelModelInterfaceForBasestation(
    const ParameterDatabaseReader& theParameterDatabase,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId,
    const unsigned int cellSectorId,
    const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
    shared_ptr<MimoChannelModelInterface>& interfacePtr)
{
    interfacePtr.reset(new ModelInterface(shared_from_this(), theNodeId, cellSectorId));
}


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


class OnDemandTgnMimoChannelModel : public MimoChannelModel,
    public enable_shared_from_this<OnDemandTgnMimoChannelModel> {
public:
    static
    shared_ptr<OnDemandTgnMimoChannelModel> Create(
        const ParameterDatabaseReader& theParameterDatabase,
        const InterfaceOrInstanceId& propagationModelInstanceId,
        const unsigned int baseChannelNumber,
        const unsigned int numberChannels,
        const vector<double>& channelCarrierFrequencyListMhz,
        const vector<double>& channelBandwidthListMhz,
        const unsigned int numberSubcarriersPerChannel,
        const RandomNumberGeneratorSeed& runSeed)
    {
        const shared_ptr<OnDemandTgnMimoChannelModel> modelPtr =
            shared_ptr<OnDemandTgnMimoChannelModel>(
                new OnDemandTgnMimoChannelModel(
                    theParameterDatabase,
                    propagationModelInstanceId,
                    baseChannelNumber,
                    numberChannels,
                    channelCarrierFrequencyListMhz,
                    channelBandwidthListMhz,
                    numberSubcarriersPerChannel,
                    runSeed));

        modelPtr->CompleteInitialization();
        return (modelPtr);
    }

    virtual ~OnDemandTgnMimoChannelModel() override;

    virtual void CreateNewMimoChannelModelInterface(
        const ParameterDatabaseReader& theParameterDatabase,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
        shared_ptr<MimoChannelModelInterface>& interfacePtr) override;

    virtual void CreateNewMimoChannelModelInterfaceForBasestation(
        const ParameterDatabaseReader& theParameterDatabase,
        const NodeId& theNodeId,
        const InterfaceId& theInterfaceId,
        const unsigned int cellSectorId,
        const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
        shared_ptr<MimoChannelModelInterface>& interfacePtr) override;

    unique_ptr<PropagationLinkPrecalculationInterface> GetParallelLinkPrecalculationInterfacePtr();

    void NotifyOfImminentLinkUse(
        const unsigned int channelNumber,
        const NodeId& nodeId1,
        const NodeId& nodeId2,
        const SimTime& timeOfUse);

    void DoTheParallelPrecalculation();

private:

    OnDemandTgnMimoChannelModel(
        const ParameterDatabaseReader& theParameterDatabase,
        const InterfaceOrInstanceId& propagationModelInstanceId,
        const unsigned int initBaseChannelNumber,
        const unsigned int numberChannels,
        const vector<double>& channelCarrierFrequencyListMhz,
        const vector<double>& channelBandwidthListMhz,
        const unsigned int numberSubcarriersPerChannel,
        const RandomNumberGeneratorSeed& runSeed);

    void CompleteInitialization();

    unsigned int GetNumberOfAntennasFor(const NodeId& theNodeId) const;

    void GetChannelMatrix(
        const SimTime& time,
        const unsigned int channelIndex,
        const NodeId& nodeId1,
        const unsigned int nodeId1CellSectorId,
        const NodeId& nodeId2,
        const unsigned int nodeId2CellSectorId,
        const unsigned int subcarrierIndex,
        MimoChannelMatrix& mimoChannelMatrix);

    // For Efficiency: Returns matrix for "time" if nothing already calculed in range.

    void GetChannelMatrixInTimeRange(
        const SimTime& earliestTime,
        const SimTime& time,
        const unsigned int channelIndex,
        const NodeId& nodeId1,
        const unsigned int nodeId1CellSectorId,
        const NodeId& nodeId2,
        const unsigned int nodeId2CellSectorId,
        const unsigned int subcarrierIndex,
        MimoChannelMatrix& mimoChannelMatrix);

    // Note: Const reference to internal structure for efficiency.

    const MimoChannelMatrix& GetChannelMatrix(
        const SimTime& time,
        const unsigned int channelIndex,
        const NodeId& nodeId1,
        const unsigned int nodeId1CellSectorId,
        const NodeId& nodeId2,
        const unsigned int nodeId2CellSectorId,
        const unsigned int subcarrierIndex);

    InterfaceOrInstanceId propagationModelInstanceId;

    // Time delay between taps, e.g. 0ns 10ns 20ns 30ns ... (is 10ns).

    double tapDelayIncrementSecs;

    double cutOffFrequencyHz;

    // Possibly make variables link specific (link specific doppler effect).
    SimTime samplingIntervalTime;
    SimTime fadingSamplingIntervalTime;

    //----------------------------------

    struct LinkKeyType {

        NodeId nodeId1;
        unsigned int node1CellSector;
        NodeId nodeId2;
        unsigned int node2CellSector;

        LinkKeyType(
            const NodeId& initNodeId1,
            const unsigned int initNode1CellSector,
            const NodeId& initNodeId2,
            const unsigned int initNode2CellSector)
        :
            nodeId1(initNodeId1),
            node1CellSector(initNode1CellSector),
            nodeId2(initNodeId2),
            node2CellSector(initNode2CellSector)
        {
            if (nodeId1 > nodeId2) {
                std::swap(nodeId1, nodeId2);
                std::swap(node1CellSector, node2CellSector);
            }//if//
        }

        bool operator<(const LinkKeyType& right) const
        {
            assert(nodeId1 <= nodeId2);

            if (nodeId1 < right.nodeId1) {
                return true;
            }

            if (nodeId1 > right.nodeId1) {
                return false;
            }

            if (node1CellSector < right.node1CellSector) {
                return true;
            }

            if (node1CellSector > right.node1CellSector) {
                return false;
            }

            if (nodeId2 < right.nodeId2) {
                return true;
            }

            if (nodeId2 > right.nodeId2) {
                return false;
            }

            return (node2CellSector < right.node2CellSector);

        }//"<"

    };//LinkKeyType//


    struct LinkInfoType {

        TgnMimoChModel::LinkParametersType linkParameters;

        RandomNumberGenerator aRandomNumberGenerator;

        ublas::vector<complex<double> > riceVectorLos;

        // Time domain gaussian noise filtering state variables:

        SimTime startTime;
        SimTime lastUpdateTime;
        SimTime fadingSamplingIntervalTime;
        unsigned long long int currentFadingSampleNumber;

        bool mustAddLineOfSightComponent;

        // Sample time just provides a time "granularity" to channel matrix changes to lower
        // computatation costs.

        SimTime samplingIntervalTime;
        SimTime nextSampleTime;

        vector<TgnMimoChModel::OutputInfoForTapType> outputInfoForTaps;

        vector<MimoChannelMatrix> mimoChannelMatrices;

        LinkInfoType()
            : aRandomNumberGenerator(1), nextSampleTime(ZERO_TIME), lastUpdateTime(ZERO_TIME),
              currentFadingSampleNumber(0), mustAddLineOfSightComponent(true) {}

    };//LinkInfoType//

    struct ChannelDataType {
        map<LinkKeyType, LinkInfoType> linkInfos;
        double carrierFrequencyHz;
        double carrierBandwidthHz;
        unsigned int numberSubcarriers;
        RandomNumberGeneratorSeed channelLevelSeed;

    };//ChannelDataType//

    vector<shared_ptr<ChannelDataType> > channels;
    unsigned int baseChannelNumber;

    struct NodeInfoType {
        shared_ptr<ObjectMobilityModel> mobilityModelPtr;
        unsigned int numberAntennas;
        double normalizedAntennaSpacing;

    };//NodeInfoType//

    FastIntToDataMap<NodeId, NodeInfoType> nodeInfoMap;

    TgnMimoChModel::ChannelModelChoicesType modelChoice;
    vector<TgnMimoChModel::InfoForTapType> infoForTaps;
    unsigned int tgacChannelSamplingRateExpansionFactor;

    double kFactor;
    double kFactorBreakpointMeters;

    RandomNumberGeneratorSeed instanceLevelSeed;
    static const int channelSeedHashingInput = 375620163;


    // Parallel Execution

    void DoOneThreadParallelComputation();

    void ParallelCalculationThreadRoutine(const unsigned int threadNumber);

    class ThreadFunctor {
    public:
        ThreadFunctor(
            const shared_ptr<OnDemandTgnMimoChannelModel>& initChannelModelPtr,
            const unsigned int initThreadNumber)
        :
            channelModelPtr(initChannelModelPtr),
            threadNumber(initThreadNumber)
        {}

        void operator()() { channelModelPtr->ParallelCalculationThreadRoutine(threadNumber); }

    private:
        NoOverhead_weak_ptr<OnDemandTgnMimoChannelModel> channelModelPtr;
        unsigned int threadNumber;
    };//ThreadFunctor//

    unsigned int numberParallelCalculationThreads;

    vector<shared_ptr<boost::thread> > threadList;

    unique_ptr<boost::barrier> threadBarrierPtr;
    volatile bool shuttingDownThreads;
    volatile long int numberCalculationsLeft;

    struct ParallelJobInfoType {
        unsigned int channelNumber;
        NodeId linkNodeId1;
        NodeId linkNodeId2;
        SimTime timeOfUse;
        bool linkIsNew;

        ParallelJobInfoType() { }
        ParallelJobInfoType(
            const unsigned int initChannelNumber,
            const NodeId& initLinkNodeId1,
            const NodeId& initLinkNodeId2,
            const SimTime& initTimeOfUse,
            const bool initLinkIsNew)
        :
            channelNumber(initChannelNumber),
            linkNodeId1(initLinkNodeId1),
            linkNodeId2(initLinkNodeId2),
            timeOfUse(initTimeOfUse),
            linkIsNew(initLinkIsNew)
        {}
    };//ParallelJobInfoType//

    vector<ParallelJobInfoType> parallelJobs;

    class PrecalculationInterface: public PropagationLinkPrecalculationInterface {
    public:
        PrecalculationInterface(const shared_ptr<OnDemandTgnMimoChannelModel>& initModelPtr)
            : modelPtr(initModelPtr), isEnabled(false) { }

        virtual void EnablePrecalculation() override { isEnabled = true; }
        virtual void DisablePrecalculation() override { isEnabled = false; }

        virtual void NotifyOfImminentLinkUse(
            const unsigned int channelNumber,
            const NodeId& nodeId1,
            const NodeId& nodeId2,
            const SimTime& timeOfUse) override
        {
            if (!isEnabled) {
                return;
            }
            modelPtr->NotifyOfImminentLinkUse(
                    channelNumber,
                    nodeId1,
                    nodeId2,
                    timeOfUse);
        }

        virtual void DoThePrecalculations() override
        {
            if (!isEnabled) {
                return;
            }
            modelPtr->DoTheParallelPrecalculation();
        }

    private:
        bool isEnabled;
        NoOverhead_weak_ptr<OnDemandTgnMimoChannelModel> modelPtr;
    };

    //----------------------------------------------------------------------------------------------

    class ModelInterface: public MimoChannelModelInterface {
    public:
        virtual unsigned int GetNumberOfAntennas() const override
        {
            return(channelModelPtr->GetNumberOfAntennasFor(theNodeId));
        }

        virtual unsigned int GetNumberOfAntennasFor(const NodeId& otherNodeId) const override
        {
            return(channelModelPtr->GetNumberOfAntennasFor(otherNodeId));
        }

        virtual void GetChannelMatrix(
            const SimTime& time,
            const unsigned int channelNumber,
            const NodeId& sourceNodeId,
            const unsigned int subcarrierIndex,
            MimoChannelMatrix& mimoChannelMatrix) override
        {
            channelModelPtr->GetChannelMatrix(
                time,
                channelNumber,
                sourceNodeId,
                0,
                theNodeId,
                0,
                subcarrierIndex,
                mimoChannelMatrix);
        }


        virtual void GetChannelMatrixInTimeRange(
            const SimTime& earliestTime,
            const SimTime& time,
            const unsigned int channelNumber,
            const NodeId& sourceNodeId,
            const unsigned int subcarrierIndex,
            MimoChannelMatrix& mimoChannelMatrix) override
        {
            channelModelPtr->GetChannelMatrixInTimeRange(
                earliestTime,
                time,
                channelNumber,
                sourceNodeId,
                0,
                theNodeId,
                0,
                subcarrierIndex,
                mimoChannelMatrix);
        }



        virtual void GetChannelMatrix(
            const SimTime& time,
            const unsigned int channelNumber,
            const NodeId& otherNodeId,
            const unsigned int otherNodeCellSectorId,
            const unsigned int subcarrierIndex,
            MimoChannelMatrix& mimoChannelMatrix) override
        {
            channelModelPtr->GetChannelMatrix(
                time,
                channelNumber,
                otherNodeId,
                otherNodeCellSectorId,
                theNodeId,
                cellSectorId,
                subcarrierIndex,
                mimoChannelMatrix);
        }

        virtual void InitializeLinkIfNotYetInited(
            const SimTime& time,
            const unsigned int channelNumber,
            const NodeId& sourceNodeId) override
        {
            channelModelPtr->InitializeLinkIfNotYetInited(
                time,
                channelNumber,
                sourceNodeId,
                theNodeId);
        }


        virtual const MimoChannelMatrix& GetChannelMatrix(
            const SimTime& time,
            const unsigned int channelNumber,
            const NodeId& sourceNodeId,
            const unsigned int subcarrierIndex) override
        {
            assert(false && "Not Implemented"); abort(); return *(new MimoChannelMatrix());
        }

        virtual const MimoChannelMatrix& GetChannelMatrix(
            const SimTime& time,
            const unsigned int channelNumber,
            const NodeId& sourceNodeId,
            const unsigned int sourceCellSectorId,
            const unsigned int subcarrierIndex) override
        {
            assert(false && "Not Implemented"); abort(); return *(new MimoChannelMatrix());
        }


    private:

        ModelInterface(
            const shared_ptr<OnDemandTgnMimoChannelModel>& channelModelPtr,
            const NodeId& theNodeId,
            const unsigned int cellSectorId);

        shared_ptr<OnDemandTgnMimoChannelModel> channelModelPtr;

        NodeId theNodeId;
        unsigned int cellSectorId;

        friend class OnDemandTgnMimoChannelModel;

    };//ModelInterface//

    //----------------------------------------------------------------------------------------------

    void InitializeLink(
        const RandomNumberGeneratorSeed& channelLevelSeed,
        const SimTime& startTime,
        const LinkKeyType& linkKey,
        LinkInfoType& linkInfo) const;

    void UpdateChannelMatricesForLink(
        const SimTime& time,
        const double& channelCenterFrequencyHz,
        const double& channelBandwidthHz,
        const unsigned int numberSubcarriers,
        LinkInfoType& linkInfo);

    double CalcDistanceMeters(
        const SimTime& time,
        const NodeId& nodeId1,
        const NodeId& nodeId2) const;

    // Only for Parallel result matching while running sequential.
    // Delete if parallelism in this model is removed.

    void InitializeLinkIfNotYetInited(
        const SimTime& time,
        const unsigned int channelNumber,
        const NodeId& sourceNodeId,
        const NodeId& destinationNodeId);

};//OnDemandTgnMimoChannelModel//




inline
OnDemandTgnMimoChannelModel::OnDemandTgnMimoChannelModel(
    const ParameterDatabaseReader& theParameterDatabase,
    const InterfaceOrInstanceId& initPropagationModelInstanceId,
    const unsigned int initBaseChannelNumber,
    const unsigned int numberChannels,
    const vector<double>& channelCarrierFrequencyListMhz,
    const vector<double>& channelBandwidthListMhz,
    const unsigned int numberSubcarriersPerChannel,
    const RandomNumberGeneratorSeed& runSeed)
    :
    propagationModelInstanceId(initPropagationModelInstanceId),
    channels(numberChannels),
    baseChannelNumber(initBaseChannelNumber),
    tapDelayIncrementSecs(pow(10.0, -8.0)),
    instanceLevelSeed(HashInputsToMakeSeed(runSeed, HashStringToInt(initPropagationModelInstanceId))),
    numberParallelCalculationThreads(0),
    shuttingDownThreads(false),
    numberCalculationsLeft(0)
{
    // Could make movement for doppler link specific.

    double scattererMovementSpeedMSec = TgnMimoChModel::DefaultScatterMovementSpeedMSec;

    if (theParameterDatabase.ParameterExists(
            "tgn-mimo-channel-scatterer-movement-meters-sec",
            propagationModelInstanceId)) {

        scattererMovementSpeedMSec =
            theParameterDatabase.ReadDouble(
                "tgn-mimo-channel-scatterer-movement-meters-sec",
                propagationModelInstanceId);
    }//if//

    // Assumption: first channel frequency is representative of all channels for doppler.

    const double carrierFrequencyHz = 1000000.0 * channelCarrierFrequencyListMhz.front();
    const double wavelengthMeters = TgnMimoChModel::SpeedOfLightMSecs / carrierFrequencyHz;
    (*this).cutOffFrequencyHz = (scattererMovementSpeedMSec / wavelengthMeters);

    // Tables for time domain filtering are based on this value for "normalizedDopplerSpread".
    // Do not change!

    const double normalizedDopplerSpread = (1.0/300);

    (*this).fadingSamplingIntervalTime =
        ConvertDoubleSecsToTime(normalizedDopplerSpread / cutOffFrequencyHz);

    samplingIntervalTime = fadingSamplingIntervalTime;
    if (theParameterDatabase.ParameterExists(
        "tgn-mimo-channel-sampling-interval-time", propagationModelInstanceId)) {

        samplingIntervalTime =
            theParameterDatabase.ReadTime(
                "tgn-mimo-channel-sampling-interval-time",
                propagationModelInstanceId);

        if (samplingIntervalTime == ZERO_TIME) {
            cerr << "Error in TGN MIMO: parameter \"tgn-mimo-channel-sampling-interval-time\" "
                 << "cannot be 0." << endl;
            exit(1);
        }//if//
    }//if//

    tgacChannelSamplingRateExpansionFactor = 1;

    if (theParameterDatabase.ParameterExists(
        "tgn-mimo-tgac-channel-sampling-rate-expansion-factor",
        propagationModelInstanceId))  {

        tgacChannelSamplingRateExpansionFactor =
            theParameterDatabase.ReadInt(
                "tgn-mimo-tgac-channel-sampling-rate-expansion-factor",
                propagationModelInstanceId);

        tapDelayIncrementSecs /= tgacChannelSamplingRateExpansionFactor;

    }//if//

    for(unsigned int i = 0; (i < numberChannels); i++) {
        channels[i].reset(new ChannelDataType());
        ChannelDataType& channel = *channels[i];
        channel.carrierFrequencyHz = 1000000.0 * channelCarrierFrequencyListMhz.at(i);
        if (channelBandwidthListMhz.at(i) != channelBandwidthListMhz.at(0)) {
            cerr << "Error in Tgn Mimo Model: Channel Bandwidths must be the same" << endl;
            exit(1);
        }//if//
        channel.carrierBandwidthHz = 1000000.0 * channelBandwidthListMhz.at(i);
        channel.numberSubcarriers = numberSubcarriersPerChannel;
        channel.channelLevelSeed = HashInputsToMakeSeed(instanceLevelSeed, i, channelSeedHashingInput);
    }//for//

    string modelChoiceString =
        theParameterDatabase.ReadString("tgn-mimo-channel-model-letter", propagationModelInstanceId);

    ConvertStringToUpperCase(modelChoiceString);

    if (modelChoiceString.size() != 1) {
        cerr << "Invalid TGN Mimo Model Letter (.e.g. \"B\": " << modelChoiceString << endl;
        exit(1);
    }//if//

    if (modelChoiceString[0] == 'B') {
        modelChoice = TgnMimoChModel::ModelB;
    }
    else if (modelChoiceString[0] == 'C') {
        modelChoice = TgnMimoChModel::ModelC;
    }
    else if (modelChoiceString[0] == 'D') {
        modelChoice = TgnMimoChModel::ModelD;
    }
    else if (modelChoiceString[0] == 'E') {
        modelChoice = TgnMimoChModel::ModelE;
    }
    else {
        cerr << "Invalid TGN Mimo Model Letter (.e.g. \"B\": " << modelChoiceString << endl;
        exit(1);
    }//if//

    TgnMimoChModel::GetModelParameters(
        modelChoice, tgacChannelSamplingRateExpansionFactor,
        kFactor, kFactorBreakpointMeters, infoForTaps);

    TgnMimoChModel::CalculatePowerAngleFactorQs(infoForTaps);

    numberParallelCalculationThreads = 0;

    if (theParameterDatabase.ParameterExists(
        "tgn-mimo-number-parallel-calculation-threads", propagationModelInstanceId)) {

        numberParallelCalculationThreads =
            theParameterDatabase.ReadNonNegativeInt(
                "tgn-mimo-number-parallel-calculation-threads", propagationModelInstanceId);
    }//if//

}//OnDemandTgnMimoChannelModel()//


inline
void OnDemandTgnMimoChannelModel::CompleteInitialization() {
    if (numberParallelCalculationThreads > 0) {
        threadBarrierPtr.reset(new boost::barrier(numberParallelCalculationThreads));
        for(unsigned int i = 0; (i < (numberParallelCalculationThreads - 1)); i++) {
            threadList.push_back(
                shared_ptr<boost::thread>(new boost::thread(ThreadFunctor(shared_from_this(), i))));
        }//for//
    }//if//
}//CompleteInitialization//



inline
OnDemandTgnMimoChannelModel::~OnDemandTgnMimoChannelModel()
{
    if (!threadList.empty()) {

        (*this).shuttingDownThreads = true;
        threadBarrierPtr->wait();

        for(size_t i = 0; (i < threadList.size()); i++) {
            threadList[i]->join();
        }//for//
    }//if//

}//~OnDemandTgnMimoChannelModel//



inline
unsigned int OnDemandTgnMimoChannelModel::GetNumberOfAntennasFor(const NodeId& theNodeId) const
{
    return (nodeInfoMap.GetValue(theNodeId).numberAntennas);
}


inline
void OnDemandTgnMimoChannelModel::InitializeLink(
    const RandomNumberGeneratorSeed& channelLevelSeed,
    const SimTime& startTime,
    const LinkKeyType& linkKey,
    LinkInfoType& linkInfo) const
{
    const RandomNumberGeneratorSeed linkSeed =
        HashInputsToMakeSeed(
            HashInputsToMakeSeed(channelLevelSeed, linkKey.nodeId1, linkKey.node1CellSector),
            linkKey.nodeId2,
            linkKey.node2CellSector);

    linkInfo.aRandomNumberGenerator.SetSeed(linkSeed);

    const NodeInfoType& node1Info = nodeInfoMap[linkKey.nodeId1];
    const NodeInfoType& node2Info = nodeInfoMap[linkKey.nodeId2];
    const unsigned int numTxAntennas = node1Info.numberAntennas;
    const unsigned int numRxAntennas = node2Info.numberAntennas;

    linkInfo.linkParameters.departureNodeNumberAntennas = node1Info.numberAntennas;
    linkInfo.linkParameters.departureNodeAntennaSpacing = node1Info.normalizedAntennaSpacing;
    linkInfo.linkParameters.arrivalNodeNumberAntennas = node2Info.numberAntennas;
    linkInfo.linkParameters.arrivalNodeAntennaSpacing = node2Info.normalizedAntennaSpacing;

    linkInfo.startTime = startTime;
    linkInfo.lastUpdateTime = ZERO_TIME;

    linkInfo.fadingSamplingIntervalTime = fadingSamplingIntervalTime;
    linkInfo.samplingIntervalTime = samplingIntervalTime;

    linkInfo.outputInfoForTaps.resize(infoForTaps.size());
    for(unsigned int i = 0; (i < linkInfo.outputInfoForTaps.size()); i++) {
        TgnMimoChModel::OutputInfoForTapType& tapInfo = linkInfo.outputInfoForTaps[i];

        TgnMimoChModel::CalculateSpatialCorrelationMatrix(
            linkInfo.linkParameters,
            infoForTaps[i],
            tapInfo.correlationMatrix);

        // Initialize filter states.

        tapInfo.currentFilterStates.resize(node2Info.numberAntennas * node1Info.numberAntennas);
        tapInfo.currentFadingVector.resize(node2Info.numberAntennas * node1Info.numberAntennas);
        tapInfo.nextFadingVector.resize(node2Info.numberAntennas * node1Info.numberAntennas);

        for(unsigned int j = 0; (j <tapInfo.currentFilterStates.size()); j++) {
            for(unsigned int k = 0; (k < TgnMimoChModel::numberFadingSamplesForFilterInitialization); k++) {
                complex<double> notUsed;

                TgnMimoChModel::MakeBellFilterFadingValue(
                    linkInfo.aRandomNumberGenerator, tapInfo.currentFilterStates[j], notUsed);
            }//for//

            TgnMimoChModel::MakeBellFilterFadingValue(
                linkInfo.aRandomNumberGenerator,
                tapInfo.currentFilterStates[j],
                tapInfo.currentFadingVector[j]);

            TgnMimoChModel::MakeBellFilterFadingValue(
                linkInfo.aRandomNumberGenerator,
                tapInfo.currentFilterStates[j],
                tapInfo.nextFadingVector[j]);

        }//for//
    }//for//

    linkInfo.riceVectorLos =
        TgnMimoChModel::CreateLosRiceVector(
            kFactor,
            infoForTaps.at(0),
            linkInfo.linkParameters);

}//InitializeLink//



// This routine is only to make sequential execution have the same results as parallel execution.

inline
void OnDemandTgnMimoChannelModel::InitializeLinkIfNotYetInited(
    const SimTime& time,
    const unsigned int channelNumber,
    const NodeId& sourceNodeId,
    const NodeId& destinationNodeId)
{
    MimoChannelMatrix notUsed;

    void GetChannelMatrix(
        const SimTime& time,
        const unsigned int channelIndex,
        const NodeId& nodeId1,
        const unsigned int nodeId1CellSectorId,
        const NodeId& nodeId2,
        const unsigned int nodeId2CellSectorId,
        const unsigned int subcarrierIndex,
        MimoChannelMatrix& mimoChannelMatrix);


    (*this).GetChannelMatrix(
        time,
        channelNumber,
        sourceNodeId,
        0,
        destinationNodeId,
        0,
        0,
        notUsed);

}//InitializeLinkIfNotYetInited//




inline
void OnDemandTgnMimoChannelModel::UpdateChannelMatricesForLink(
    const SimTime& time,
    const double& channelCenterFrequencyHz,
    const double& channelBandwidthHz,
    const unsigned int numberSubcarriers,
    LinkInfoType& linkInfo)
{
    using std::max;

    if(linkInfo.nextSampleTime > time) {
        // Up to date.
        return;
    }//if//

    assert(time >= linkInfo.lastUpdateTime);

    const unsigned long long int targetFadingSampleNumber =
        (time - linkInfo.startTime) / linkInfo.fadingSamplingIntervalTime;

    assert(targetFadingSampleNumber >= linkInfo.currentFadingSampleNumber);

    const unsigned long long int numberFadingSamples =
        (targetFadingSampleNumber - linkInfo.currentFadingSampleNumber);

    const SimTime currentFadingSampleTime =
        linkInfo.startTime + (targetFadingSampleNumber * linkInfo.fadingSamplingIntervalTime);

    const double currentFadingTimeSecs = ConvertTimeToDoubleSecs(currentFadingSampleTime);
    const double nextFadingTimeSecs =
        (currentFadingTimeSecs + ConvertTimeToDoubleSecs(linkInfo.fadingSamplingIntervalTime));

    for(unsigned int i = 0; (i < linkInfo.outputInfoForTaps.size()); i++) {
        TgnMimoChModel::InfoForTapType& tapInfo = infoForTaps[i];
        TgnMimoChModel::OutputInfoForTapType& outputTapInfo = linkInfo.outputInfoForTaps[i];

        if (numberFadingSamples > 0) {
            for(unsigned int j = 0; (j < outputTapInfo.currentFadingVector.size()); j++) {

                if (numberFadingSamples == 1) {
                    outputTapInfo.currentFadingVector = outputTapInfo.nextFadingVector;
                }
                else {
                    for(unsigned long long int k = 0; (k < (numberFadingSamples-1)); k++) {
                        TgnMimoChModel::MakeBellFilterFadingValue(
                            linkInfo.aRandomNumberGenerator,
                            outputTapInfo.currentFilterStates[j],
                            outputTapInfo.currentFadingVector[j]);
                    }//for//
                }//if//

                TgnMimoChModel::MakeBellFilterFadingValue(
                    linkInfo.aRandomNumberGenerator,
                    outputTapInfo.currentFilterStates[j],
                    outputTapInfo.nextFadingVector[j]);

            }//for//
        }//if//

        // Make local copies:

        ublas::vector<complex<double> > currentFadingVector = outputTapInfo.currentFadingVector;
        ublas::vector<complex<double> > nextFadingVector = outputTapInfo.nextFadingVector;

        // Transform fading vectors with the spatial correlation and power normalize.

        const double sqrtOfPower =
            sqrt(tapInfo.GetNormalizedPowerDelayProfilePower(linkInfo.mustAddLineOfSightComponent));

        currentFadingVector = sqrtOfPower * prod(outputTapInfo.correlationMatrix, currentFadingVector);

        nextFadingVector = sqrtOfPower * prod(outputTapInfo.correlationMatrix, nextFadingVector);

        // Addition of the Rice component, only for first tap.

        if ((i == 0) && (linkInfo.mustAddLineOfSightComponent)) {
            // Calculation of the Rice phasor
            // AoA/AoD hard-coded to 45 degrees

            // Note: This calculation (and riceVectorLos) should closely match TGn MATLAB model.

            const double aFactor = sqrt(1.0 / (kFactor + 1.0));

            const complex<double> currentRicePhasor = exp(
                complex<double>(
                    0.0,
                    ((2.0 * PI * cutOffFrequencyHz) * cos(PI/4) * currentFadingTimeSecs)));

            currentFadingVector  =
                ((aFactor * currentFadingVector) + (currentRicePhasor * linkInfo.riceVectorLos));

            const complex<double> nextRicePhasor = exp(
                complex<double>(
                    0.0,
                    ((2.0 * PI * cutOffFrequencyHz) * cos(PI/4) * nextFadingTimeSecs)));

            nextFadingVector  =
                ((aFactor * nextFadingVector) + (nextRicePhasor * linkInfo.riceVectorLos));

        }//if//

        // Sample time just provides a time "granularity" to channel matrix changes for
        // the purpose of computation minimization.

        const unsigned long long int sampleNumber =
            (time - linkInfo.startTime) / linkInfo.samplingIntervalTime;

        linkInfo.nextSampleTime =
            linkInfo.startTime + ((sampleNumber + 1) * linkInfo.samplingIntervalTime);

        const unsigned int numTxAntennas = linkInfo.linkParameters.departureNodeNumberAntennas;
        const unsigned int numRxAntennas = linkInfo.linkParameters.arrivalNodeNumberAntennas;

        outputTapInfo.hMatrix.resize(numRxAntennas, numTxAntennas, false);

        const double timeSecs = ConvertTimeToDoubleSecs(time);

        for(unsigned int k = 0; (k < (numRxAntennas * numTxAntennas)); k++) {
            outputTapInfo.hMatrix((k % numRxAntennas), (k / numRxAntennas)) =
                ScenSim::CalcInterpolatedValue(
                    currentFadingTimeSecs,
                    currentFadingVector[k],
                    nextFadingTimeSecs,
                    nextFadingVector[k],
                    timeSecs);
        }//for//
    }//for//

    linkInfo.lastUpdateTime = time;
    linkInfo.currentFadingSampleNumber = targetFadingSampleNumber;

    // Combine Tap H matrices into subcarrier channel matrices.

    TgnMimoChModel::CombineTapHMatricesIntoChannelMatrices(
        linkInfo.linkParameters,
        channelCenterFrequencyHz,
        channelBandwidthHz,
        numberSubcarriers,
        tapDelayIncrementSecs,
        linkInfo.outputInfoForTaps,
        linkInfo.mimoChannelMatrices);

}//UpdateChannelMatricesForLink//


inline
double OnDemandTgnMimoChannelModel::CalcDistanceMeters(
    const SimTime& time,
    const NodeId& nodeId1,
    const NodeId& nodeId2) const
{
    ObjectMobilityPosition position1;
    ObjectMobilityPosition position2;
    nodeInfoMap.GetValue(nodeId1).mobilityModelPtr->GetPositionForTime(time, position1);
    nodeInfoMap.GetValue(nodeId2).mobilityModelPtr->GetPositionForTime(time, position2);

    return (ScenSim::CalcDistanceMeters(position1, position2));

}//CalcDistanceMeters//



inline
void OnDemandTgnMimoChannelModel::GetChannelMatrixInTimeRange(
    const SimTime& earliestTime,
    const SimTime& time,
    const unsigned int channelNumber,
    const NodeId& nodeId1,
    const unsigned int node1CellSectorId,
    const NodeId& nodeId2,
    const unsigned int node2CellSectorId,
    const unsigned int subcarrierIndex,
    MimoChannelMatrix& mimoChannelMatrix)
{
    assert(nodeId1 != nodeId2);

    ChannelDataType& channelData = *channels.at(channelNumber - baseChannelNumber);

    const bool linkIsReversed = (nodeId1 > nodeId2);

    LinkKeyType linkKey(
        nodeId1,
        node1CellSectorId,
        nodeId2,
        node2CellSectorId);

    typedef map<LinkKeyType, LinkInfoType>::iterator IterType;

    IterType iter = channelData.linkInfos.find(linkKey);

    if (iter == channelData.linkInfos.end()) {

        LinkInfoType linkInfo;
        (*this).InitializeLink(
            channelData.channelLevelSeed,
            time,
            linkKey,
            linkInfo);

        channelData.linkInfos.insert(
            std::pair<LinkKeyType, LinkInfoType>(linkKey, linkInfo));

        iter = channelData.linkInfos.find(linkKey);
    }//if//

    LinkInfoType& linkInfo = iter->second;

    assert((linkInfo.lastUpdateTime <= time) && "Parallel simulation time failure");

    if(linkInfo.nextSampleTime <= earliestTime) {
        linkInfo.mustAddLineOfSightComponent =
            (CalcDistanceMeters(time, nodeId1, nodeId2) < kFactorBreakpointMeters);

        (*this).UpdateChannelMatricesForLink(
            earliestTime,
            channelData.carrierFrequencyHz,
            channelData.carrierBandwidthHz,
            channelData.numberSubcarriers,
            linkInfo);
    }//if//

    const MimoChannelMatrix& theMatrix = linkInfo.mimoChannelMatrices.at(subcarrierIndex);

    if (!linkIsReversed) {
        mimoChannelMatrix.resize(theMatrix.size1(), theMatrix.size2(), false);
        mimoChannelMatrix = theMatrix;
    }
    else {
        // Transpose stored matrix

        mimoChannelMatrix.resize(theMatrix.size2(), theMatrix.size1(), false);

        for(unsigned int i = 0; (i < mimoChannelMatrix.size1()); i++) {
            for(unsigned int j = 0; (j < mimoChannelMatrix.size2()); j++) {
                mimoChannelMatrix(i, j) = theMatrix(j, i);
            }//for//
        }//for//
    }//if//

}//GetChannelMatrix//


inline
void OnDemandTgnMimoChannelModel::GetChannelMatrix(
    const SimTime& time,
    const unsigned int channelNumber,
    const NodeId& nodeId1,
    const unsigned int node1CellSectorId,
    const NodeId& nodeId2,
    const unsigned int node2CellSectorId,
    const unsigned int subcarrierIndex,
    MimoChannelMatrix& mimoChannelMatrix)
{
    (*this).GetChannelMatrixInTimeRange(
        time,
        time,
        channelNumber,
        nodeId1,
        node1CellSectorId,
        nodeId2,
        node2CellSectorId,
        subcarrierIndex,
        mimoChannelMatrix);
}


//--------------------------------------------------------------------------------------------------

inline
OnDemandTgnMimoChannelModel::ModelInterface::ModelInterface(
    const shared_ptr<OnDemandTgnMimoChannelModel>& initChannelModelPtr,
    const NodeId& initNodeId,
    const unsigned int initCellSectorId)
    :
    channelModelPtr(initChannelModelPtr),
    theNodeId(initNodeId),
    cellSectorId(initCellSectorId)
{
}


//--------------------------------------------------------------------------------------------------


inline
void OnDemandTgnMimoChannelModel::CreateNewMimoChannelModelInterfaceForBasestation(
    const ParameterDatabaseReader& theParameterDatabase,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId,
    const unsigned int cellSectorId,
    const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
    shared_ptr<MimoChannelModelInterface>& interfacePtr)
{
    interfacePtr.reset(new ModelInterface(shared_from_this(), theNodeId, cellSectorId));

    NodeInfoType nodeInfo;

    nodeInfo.mobilityModelPtr = mobilityModelPtr;
    nodeInfo.numberAntennas =
        theParameterDatabase.ReadNonNegativeInt("tgn-mimo-channel-number-antennas", theNodeId, theInterfaceId);
    nodeInfo.normalizedAntennaSpacing =
        theParameterDatabase.ReadDouble("tgn-mimo-channel-normalized-antenna-spacing", theNodeId, theInterfaceId);

    (*this).nodeInfoMap.InsertMapping(theNodeId, nodeInfo);

}//CreateNewMimoChannelModelInterfaceForBasestation//


inline
void OnDemandTgnMimoChannelModel::CreateNewMimoChannelModelInterface(
    const ParameterDatabaseReader& theParameterDatabase,
    const NodeId& theNodeId,
    const InterfaceId& theInterfaceId,
    const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
    shared_ptr<MimoChannelModelInterface>& interfacePtr)
{
    (*this).CreateNewMimoChannelModelInterfaceForBasestation(
        theParameterDatabase, theNodeId, theInterfaceId, 0, mobilityModelPtr, interfacePtr);
}



inline
void OnDemandTgnMimoChannelModel::DoOneThreadParallelComputation()
{
   while (true) {

       // Claim a parallel computation (index of array) to compute.

       const long int calcIndex = AtomicDecrement(&(*this).numberCalculationsLeft);

       if (calcIndex < 0) {
           // No more parallel calculations
           break;
       }//if//

       const ParallelJobInfoType& jobInfo = (*this).parallelJobs[calcIndex];

       ChannelDataType& channelData = *channels.at(jobInfo.channelNumber - baseChannelNumber);

       typedef map<LinkKeyType, LinkInfoType>::iterator IterType;

       LinkKeyType linkKey(jobInfo.linkNodeId1, 0, jobInfo.linkNodeId2, 0);

       const IterType iter = channelData.linkInfos.find(linkKey);

       assert((iter != channelData.linkInfos.end()) && "Should have been preallocated");

       LinkInfoType& linkInfo = iter->second;

       if (jobInfo.linkIsNew) {
           InitializeLink(channelData.channelLevelSeed, jobInfo.timeOfUse, linkKey, linkInfo);
       }//if//

       if(linkInfo.nextSampleTime <= jobInfo.timeOfUse) {

           linkInfo.mustAddLineOfSightComponent =
               (CalcDistanceMeters(jobInfo.timeOfUse, jobInfo.linkNodeId1, jobInfo.linkNodeId2) <
                kFactorBreakpointMeters);

           (*this).UpdateChannelMatricesForLink(
               jobInfo.timeOfUse,
               channelData.carrierFrequencyHz,
               channelData.carrierBandwidthHz,
               channelData.numberSubcarriers,
               linkInfo);

       }//if//

   }//while//

}//DoOneThreadParallelComputation//




inline
void OnDemandTgnMimoChannelModel::ParallelCalculationThreadRoutine(const unsigned int threadNumber)
{
    while (true) {

        threadBarrierPtr->wait();

        if (shuttingDownThreads) {
            break;
        }//if//

        // Everyone starts computing

        (*this).DoOneThreadParallelComputation();

        // Wait for everyone to complete calculations.

        threadBarrierPtr->wait();

    }//while//

}//ParallelCalculationThreadRoutine//



inline
unique_ptr<PropagationLinkPrecalculationInterface>
    OnDemandTgnMimoChannelModel::GetParallelLinkPrecalculationInterfacePtr()
{
    if (numberParallelCalculationThreads == 0) {
        return (unique_ptr<PropagationLinkPrecalculationInterface>());
    }//if//

    unique_ptr<PropagationLinkPrecalculationInterface> interfacePtr(
        new PrecalculationInterface(shared_from_this()));
    return (interfacePtr);
}


inline
void OnDemandTgnMimoChannelModel::NotifyOfImminentLinkUse(
    const unsigned int channelNumber,
    const NodeId& nodeId1,
    const NodeId& nodeId2,
    const SimTime& timeOfUse)
{
    // Very wasteful, but probably minor compared to mimo;

    ChannelDataType& channelData = *channels.at(channelNumber - baseChannelNumber);

    typedef map<LinkKeyType, LinkInfoType>::iterator IterType;

    LinkKeyType linkKey(nodeId1, 0, nodeId2, 0);

    const bool linkIsNew = (channelData.linkInfos.find(linkKey) == channelData.linkInfos.end());

    assert(((parallelJobs.empty()) ||
            (nodeId1 == parallelJobs.back().linkNodeId1) ||
            (nodeId2 == parallelJobs.back().linkNodeId2)) &&
           "Routine restriction: collect jobs for one transmission/channel switch only");

    (*this).parallelJobs.push_back(
        ParallelJobInfoType(
            channelNumber,
            nodeId1,
            nodeId2,
            timeOfUse,
            linkIsNew));

    if (linkIsNew) {
        // Preallocate link to prepare for parallel execution.

        channelData.linkInfos.insert(
            std::pair<LinkKeyType, LinkInfoType>(linkKey, LinkInfoType()));
    }//if//

}//NotifyOfImminentLinkUse//


inline
void OnDemandTgnMimoChannelModel::DoTheParallelPrecalculation()
{
    if (parallelJobs.empty()) {
        // Nothing to do.
        return;
    }//if//

    numberCalculationsLeft = static_cast<long int>(parallelJobs.size());

    // Computational threads go!

    threadBarrierPtr->wait();

    // Reuse main thread.

    (*this).DoOneThreadParallelComputation();

    // Wait for them all to complete.

    threadBarrierPtr->wait();

    assert(numberCalculationsLeft < 0);

    parallelJobs.clear();

}//DoTheParallelPrecalculation//


}//namespace//


#endif
