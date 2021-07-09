// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_PARMIO_H
#define SCENSIM_PARMIO_H

#include <string>
#include <cctype>
#include <map>
#include <set>
#include <memory>

#include "scensim_support.h"
#include "scensim_time.h"
#include "scensim_nodeid.h"
#include <memory>

namespace ScenSim {

using std::string;
using std::multimap;
using std::set;
using std::unique_ptr;
using std::shared_ptr;


class GuiInterfacingSubsystem;

//=============================================================================

class ParameterDatabaseReader {
public:

    explicit
    ParameterDatabaseReader(const string& parameterFileName);

    ~ParameterDatabaseReader();

    void DisableUnusedParameterWarning();

    bool ParameterExists(const string& parameterName) const;
    bool ParameterExists(const string& parameterName, const InterfaceOrInstanceId& instanceId) const;
    bool ParameterExists(const string& parameterName, const NodeId& theNodeId) const;
    bool ParameterExists(
        const string& parameterName,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId) const;

    bool ReadBool(const string& parameterName) const;
    bool ReadBool(const string& parameterName, const InterfaceOrInstanceId& instanceId) const;
    bool ReadBool(const string& parameterName, const NodeId& theNodeId) const;
    bool ReadBool(
        const string& parameterName,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId) const;

    int ReadInt(const string& parameterName) const;
    int ReadInt(const string& parameterName, const InterfaceOrInstanceId& instanceId) const;
    int ReadInt(const string& parameterName, const NodeId& theNodeId) const;
    int ReadInt(
        const string& parameterName,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId) const;

    unsigned int ReadNonNegativeInt(const string& parameterName) const;
    unsigned int ReadNonNegativeInt(
            const string& parameterName,
            const InterfaceOrInstanceId& instanceId) const;
    unsigned int ReadNonNegativeInt(const string& parameterName, const NodeId& theNodeId) const;
    unsigned int ReadNonNegativeInt(
        const string& parameterName,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId) const;

    unsigned int ReadPositiveInt(const string& parameterName) const;
    unsigned int ReadPositiveInt(
            const string& parameterName,
            const InterfaceOrInstanceId& instanceId) const;
    unsigned int ReadPositiveInt(const string& parameterName, const NodeId& theNodeId) const;
    unsigned int ReadPositiveInt(
        const string& parameterName,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId) const;

    long long int ReadBigInt(const string& parameterName) const;
    long long int ReadBigInt(const string& parameterName, const InterfaceOrInstanceId& instanceId) const;
    long long int ReadBigInt(const string& parameterName, const NodeId& theNodeId) const;
    long long int ReadBigInt(
        const string& parameterName,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId) const;

    unsigned long long int ReadNonNegativeBigInt(const string& parameterName) const;
    unsigned long long int ReadNonNegativeBigInt(
        const string& parameterName,
        const InterfaceOrInstanceId& instanceId) const;
    unsigned long long int ReadNonNegativeBigInt(const string& parameterName, const NodeId& theNodeId) const;
    unsigned long long int ReadNonNegativeBigInt(
        const string& parameterName,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId) const;

    unsigned long long int ReadPositiveBigInt(const string& parameterName) const;
    unsigned long long int ReadPositiveBigInt(
        const string& parameterName,
        const InterfaceOrInstanceId& instanceId) const;
    unsigned long long int ReadPositiveBigInt(const string& parameterName, const NodeId& theNodeId) const;
    unsigned long long int ReadPositiveBigInt(
        const string& parameterName,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId) const;

    double ReadDouble(const string& parameterName) const;
    double ReadDouble(const string& parameterName, const InterfaceOrInstanceId& instanceId) const;
    double ReadDouble(const string& parameterName, const NodeId& theNodeId) const;
    double ReadDouble(
        const string& parameterName,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId) const;

    SimTime ReadTime(const string& parameterName) const;
    SimTime ReadTime(const string& parameterName, const InterfaceOrInstanceId& instanceId) const;
    SimTime ReadTime(const string& parameterName, const NodeId& theNodeId) const;
    SimTime ReadTime(
        const string& parameterName,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId) const;

    string ReadString(const string& parameterName) const;
    string ReadString(const string& parameterName, const InterfaceOrInstanceId& instanceId) const;
    string ReadString(const string& parameterName, const NodeId& theNodeId) const;
    string ReadString(
        const string& parameterName,
        const NodeId& theNodeId,
        const InterfaceOrInstanceId& theInterfaceId) const;

    string GetContainingNodeIdSetNameFor(const NodeId& theNodeId) const;

    //Communication nodes and GIS nodes

    void MakeSetOfAllNodeIds(set<NodeId>& setOfNodeIds) const;

    //only communication nodes

    void MakeSetOfAllCommNodeIds(set<NodeId>& setOfNodeIds) const;
    bool CommNodeIdExists(const NodeId& theNodeId) const;

    void MakeSetOfAllNodeIdsWithParameter(const string& parameterName, set<NodeId>& setOfNodeIds) const;
    void MakeSetOfAllNodeIdsWithParameter(
        const string& parameterName,
        const string& parameterValue,
        set<NodeId>& setOfNodeIds) const;


    void MakeSetOfAllInterfaceIdsForANode(
        const NodeId& theNodeId,
        set<InterfaceOrInstanceId>& setOfInterfaces) const
    {
        MakeSetOfAllInterfaceIdsForANode(theNodeId, "network-address", setOfInterfaces);
    }

    void MakeSetOfAllInterfaceIdsForANode(
        const NodeId& theNodeId,
        const string& parameterName,
        set<InterfaceOrInstanceId>& setOfInterfaces) const;

    void MakeSetOfAllInterfaceIds(
        const string& parameterName,
        set<InterfaceOrInstanceId>& setOfInterfaces) const;

    void MakeSetOfAllInterfaceIds(
        const string& parameterName,
        const string& parameterMustBeEqual,
        set<InterfaceOrInstanceId>& setOfInterfaces) const;


    // Just an aliases for "MakeSetOfAllInterfaceIds*" methods.

    void MakeSetOfAllInstanceIdsForANode(
        const NodeId& theNodeId,
        const string& parameterName,
        set<InterfaceOrInstanceId>& setOfInstances) const
    {
        MakeSetOfAllInterfaceIdsForANode(theNodeId, parameterName, setOfInstances);
    }


    void MakeSetOfAllInstanceIds(
        const string& parameterName,
        set<InterfaceOrInstanceId>& setOfInstances) const
    {
        MakeSetOfAllInterfaceIds(parameterName, setOfInstances);
    }


    // Supports "Split Node" functionality.

    class NodeIdRemapper {
    public:
        virtual NodeId MapNodeIdTo(const NodeId& theNodeId) const = 0;
    };

    void SetNodeIdRemapper(const shared_ptr<NodeIdRemapper>& nodeIdRemapperPtr);

    NodeId GetPossibleNodeIdRemap(const NodeId& theNodeId) const;


    void AddNewDefinitionToDatabase(
        const string& definitionLineInFileFormat,
        bool& foundAnError);

private:

    // For now restricting in-simulation writing to just certain subsystems.
    friend class GuiInterfacingSubsystem;

    class Implementation;

    unique_ptr<Implementation> implementationPtr;

    static void CheckThatIntegerValueIsNonNegative(
        const long long int anInt,
        const string& parameterName);

    static void CheckThatIntegerValueIsPositive(
        const long long int anInt,
        const string& parameterName);

    // Disable:
    ParameterDatabaseReader(ParameterDatabaseReader&);
    void operator=(ParameterDatabaseReader&);

};//ParameterDatabaseReader//


inline
void ParameterDatabaseReader::CheckThatIntegerValueIsNonNegative(
    const long long int anInt,
    const string& parameterName)
{
    if (anInt < 0) {
        cerr << "Error in Configuration File: Parameter \"" << parameterName
             << "\" is a negative value." << endl;
        exit(1);
    }//if//
}

inline
void ParameterDatabaseReader::CheckThatIntegerValueIsPositive(
    const long long int anInt,
    const string& parameterName)
{
    if (anInt <= 0) {
        cerr << "Error in Configuration File: Parameter \"" << parameterName
             << "\" is negative or zero value." << endl;
        exit(1);
    }//if//
}

//---------------------------------------

inline
unsigned int ParameterDatabaseReader::ReadNonNegativeInt(const string& parameterName) const
{
    const int aValue = ReadInt(parameterName);
    CheckThatIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned int>(aValue));
}

inline
unsigned int ParameterDatabaseReader::ReadNonNegativeInt(
    const string& parameterName,
    const InterfaceOrInstanceId& instanceId) const
{
    const int aValue = ReadInt(parameterName, instanceId);
    CheckThatIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned int>(aValue));
}


inline
unsigned int ParameterDatabaseReader::ReadNonNegativeInt(
    const string& parameterName,
    const NodeId& theNodeId) const
{
    const int aValue = ReadInt(parameterName, theNodeId);
    CheckThatIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned int>(aValue));
}



inline
unsigned int ParameterDatabaseReader::ReadNonNegativeInt(
    const string& parameterName,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId) const
{
    const int aValue = ReadInt(parameterName, theNodeId, theInterfaceId);
    CheckThatIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned int>(aValue));
}


inline
unsigned long long int ParameterDatabaseReader::ReadNonNegativeBigInt(const string& parameterName) const
{
    const long long int aValue = ReadBigInt(parameterName);
    CheckThatIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned long long int>(aValue));
}


inline
unsigned long long int ParameterDatabaseReader::ReadNonNegativeBigInt(
    const string& parameterName,
    const InterfaceOrInstanceId& instanceId) const
{
    const long long int aValue = ReadBigInt(parameterName, instanceId);
    CheckThatIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned long long int>(aValue));
}


inline
unsigned long long int ParameterDatabaseReader::ReadNonNegativeBigInt(
    const string& parameterName, const NodeId& theNodeId) const
{
    const long long int aValue = ReadBigInt(parameterName, theNodeId);
    CheckThatIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned long long int>(aValue));
}


inline
unsigned long long int ParameterDatabaseReader::ReadNonNegativeBigInt(
    const string& parameterName,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId) const
{
    const long long int aValue = ReadBigInt(parameterName, theNodeId, theInterfaceId);
    CheckThatIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned long long int>(aValue));
}

//---------------------------------------

inline
unsigned int ParameterDatabaseReader::ReadPositiveInt(const string& parameterName) const
{
    const int aValue = ReadInt(parameterName);
    CheckThatIntegerValueIsPositive(aValue, parameterName);
    return (static_cast<unsigned int>(aValue));
}

inline
unsigned int ParameterDatabaseReader::ReadPositiveInt(
    const string& parameterName,
    const InterfaceOrInstanceId& instanceId) const
{
    const int aValue = ReadInt(parameterName, instanceId);
    CheckThatIntegerValueIsPositive(aValue, parameterName);
    return (static_cast<unsigned int>(aValue));
}


inline
unsigned int ParameterDatabaseReader::ReadPositiveInt(
    const string& parameterName,
    const NodeId& theNodeId) const
{
    const int aValue = ReadInt(parameterName, theNodeId);
    CheckThatIntegerValueIsPositive(aValue, parameterName);
    return (static_cast<unsigned int>(aValue));
}



inline
unsigned int ParameterDatabaseReader::ReadPositiveInt(
    const string& parameterName,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId) const
{
    const int aValue = ReadInt(parameterName, theNodeId, theInterfaceId);
    CheckThatIntegerValueIsPositive(aValue, parameterName);
    return (static_cast<unsigned int>(aValue));
}


inline
unsigned long long int ParameterDatabaseReader::ReadPositiveBigInt(const string& parameterName) const
{
    const long long int aValue = ReadBigInt(parameterName);
    CheckThatIntegerValueIsPositive(aValue, parameterName);
    return (static_cast<unsigned long long int>(aValue));
}


inline
unsigned long long int ParameterDatabaseReader::ReadPositiveBigInt(
    const string& parameterName,
    const InterfaceOrInstanceId& instanceId) const
{
    const long long int aValue = ReadBigInt(parameterName, instanceId);
    CheckThatIntegerValueIsPositive(aValue, parameterName);
    return (static_cast<unsigned long long int>(aValue));
}


inline
unsigned long long int ParameterDatabaseReader::ReadPositiveBigInt(
    const string& parameterName, const NodeId& theNodeId) const
{
    const long long int aValue = ReadBigInt(parameterName, theNodeId);
    CheckThatIntegerValueIsPositive(aValue, parameterName);
    return (static_cast<unsigned long long int>(aValue));
}


inline
unsigned long long int ParameterDatabaseReader::ReadPositiveBigInt(
    const string& parameterName,
    const NodeId& theNodeId,
    const InterfaceOrInstanceId& theInterfaceId) const
{
    const long long int aValue = ReadBigInt(parameterName, theNodeId, theInterfaceId);
    CheckThatIntegerValueIsPositive(aValue, parameterName);
    return (static_cast<unsigned long long int>(aValue));
}


}//namespace//

#endif
