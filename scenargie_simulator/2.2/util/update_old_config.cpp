#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include <map>
#include <sstream>
#include <vector>

using std::string;
using std::cout;
using std::cerr;
using std::endl;
using std::set;
using std::ifstream;
using std::istringstream;
using std::map;
using std::multimap;
using std::make_pair;
using std::pair;
using std::vector;

typedef int NodeId;
typedef int ApplicationId;
typedef string ValueType;
typedef string ParameterNameType;

struct NodeApplicationId {
    NodeId theNodeId;
    ApplicationId theApplicationId;

    bool operator<(const NodeApplicationId& right) const {
        return ((theNodeId < right.theNodeId) ||
                ((theNodeId == right.theNodeId) &&
                 (theApplicationId < right.theApplicationId)));
    }

    NodeApplicationId() {}

    NodeApplicationId(
        const NodeId& initNodeId,
        const ApplicationId& initApplicationId)
        :
        theNodeId(initNodeId),
        theApplicationId(initApplicationId)
    {}

};//NodeApplicationId//

struct ApplicationSet {
    map<ValueType, set<NodeApplicationId> > apps;
};//ApplicationSet//

typedef map<ParameterNameType, ApplicationSet> ParameterMap;

inline
void ConvertStringToLowerCase(string& aString)
{
    for(size_t i = 0; i < aString.length(); i++) {
        aString[i] = tolower(aString[i]);
    }//for//
}//ConvertStringToLowerCase//

inline
string MakeLowerCaseString(const string& aString)
{
    string returnString = aString;
    ConvertStringToLowerCase(returnString);
    return (returnString);
}//MakeLowerCaseString//

inline
bool IsAConfigFileCommentLine(const string& aLine)
{
    if ((aLine.length() == 0) || (aLine.at(0) == '#')) {
        return true;
    }//if//

    size_t position = aLine.find_first_not_of(' ');

    if (position == string::npos) {
        // No non-space characters found. Blank line (is comment).

        return true;
    }//if//

    if (aLine[position] == '#') {
        return true;
    }
    else if (aLine[position] == '\t') {
        cerr << "Error: No Tabs allowed in config files." << endl;
        cerr << "Line:" << aLine << endl;
        exit(1);
    }//if//

    return false;

}//IsAConfigFileCommentLine//

inline
void DeleteTrailingSpaceAndQuotes(string& aString)
{
    size_t pos = aString.find_first_not_of(" \t\"");

    if (pos < (aString.size() - 1)) {
        aString = aString.substr(pos);
    }//if//

    pos = aString.find_last_not_of(" \t\"");

    if (pos < (aString.size() - 1)) {
        aString.erase(pos+1);
    }//if//

}//DeleteTrailingSpaceAndQuotes//


inline
void OutputApplicationBasedConifg(const map<string, ParameterMap>& paramMapPerApp)
{
    typedef map<string, ParameterMap>::const_iterator ModelIter;
    typedef map<ParameterNameType, ApplicationSet>::const_iterator ParamIter;
    typedef map<ValueType, set<NodeApplicationId> >::const_iterator ValueIter;
    typedef set<NodeApplicationId>::const_iterator NodeApplicationIter;
    typedef multimap<int, pair<ValueType, set<NodeApplicationId> > >::const_reverse_iterator AppsIter;
    typedef map<NodeApplicationId, ValueType>::const_iterator AppIter;

    for(ModelIter modelIter = paramMapPerApp.begin(); modelIter != paramMapPerApp.end(); modelIter++) {
        const string& modelName = (*modelIter).first;
        const ParameterMap& paramMap = (*modelIter).second;

        for(ParamIter paramIter = paramMap.begin(); paramIter != paramMap.end(); paramIter++) {
            const ParameterNameType& paramName = (*paramIter).first;
            const map<ValueType, set<NodeApplicationId> >& values = (*paramIter).second.apps;

            multimap<int, pair<ValueType, set<NodeApplicationId> > > valuesSortedByByNumberOfApps;

            for(ValueIter valueIter = values.begin(); valueIter != values.end(); valueIter++) {
                const int numberApps = (*valueIter).second.size();

                valuesSortedByByNumberOfApps.insert(make_pair(numberApps, *valueIter));
            }//for//

            map<NodeApplicationId, ValueType> valuePerNodeApplication;

            for(AppsIter appsIter = valuesSortedByByNumberOfApps.rbegin();
                appsIter != valuesSortedByByNumberOfApps.rend(); appsIter++) {
                const int numberApps = (*appsIter).first;
                const pair<ValueType, set<NodeApplicationId> >& valueAndApps = (*appsIter).second;
                const ValueType& value = valueAndApps.first;
                const set<NodeApplicationId>& nodeApplicationIds = valueAndApps.second;

                if ((appsIter == valuesSortedByByNumberOfApps.rbegin()) &&
                    (numberApps > 1) &&
                    (paramName.find("-destination") == string::npos)) {
                    cout << paramName << " = " << value << endl;
                }
                else {
                    for(NodeApplicationIter nodeAppIter = nodeApplicationIds.begin();
                        nodeAppIter != nodeApplicationIds.end(); nodeAppIter++) {
                        const NodeApplicationId& nodeApplicationId = (*nodeAppIter);

                        valuePerNodeApplication[nodeApplicationId] = value;
                    }
                }//if//
            }//for//

            for(AppIter appIter = valuePerNodeApplication.begin();
                appIter != valuePerNodeApplication.end(); appIter++) {
                const NodeApplicationId& nodeApplicationId = (*appIter).first;
                const ValueType& value = (*appIter).second;

                cout << "[" << nodeApplicationId.theNodeId << ";" << modelName << nodeApplicationId.theApplicationId << "] " << paramName << " = " << value << endl;
            }//for//
        }//for//

        cout << endl;
    }//for//
}//OutputApplicationBasedConifg//

inline
void OutputApplicationBasedConifg(const string& appFileName)
{
    ifstream inFile(appFileName.c_str());

    if (!inFile.good()) {
        cerr << "Could Not open file: " << appFileName << endl;
        return;
    }

    int globalApplicationNumber = 0;
    map<string, int> applicationNumbers;
    map<string, ParameterMap> paramMapPerApp;

    while(!inFile.eof()) {

        string aLine;
        getline(inFile, aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }//if//

        istringstream lineStream(aLine);

        string appLabel;
        NodeId theNodeId;
        vector<string> values;

        lineStream >> appLabel;
        lineStream >> theNodeId;

        while (lineStream.good()) {
            string value;
            lineStream >> value;

            values.push_back(value);
        }//while//

        ConvertStringToLowerCase(appLabel);

        if (applicationNumbers.find(appLabel) == applicationNumbers.end()) {
            applicationNumbers[appLabel] = 0;
        }//if//

        applicationNumbers[appLabel]++;
        globalApplicationNumber++;

        int applicationNumber = applicationNumbers[appLabel];
        ParameterMap& paramMap = paramMapPerApp[appLabel];

        const NodeApplicationId id(theNodeId, applicationNumber);

        if ((appLabel == "cbr") && (values.size() >= 6)) {
            paramMap["cbr-destination"].apps[values[0]].insert(id);
            paramMap["cbr-payload-size-bytes"].apps[values[1]].insert(id);
            paramMap["cbr-interval"].apps[values[2]].insert(id);
            paramMap["cbr-start-time"].apps[values[3]].insert(id);
            paramMap["cbr-end-time"].apps[values[4]].insert(id);
            paramMap["cbr-priority"].apps[values[5]].insert(id);

        }
        else if ((appLabel == "cbrwithqos") && (values.size() >= 8)) {
            paramMap["cbr-with-qos-destination"].apps[values[0]].insert(id);
            paramMap["cbr-with-qos-payload-size-bytes"].apps[values[1]].insert(id);
            paramMap["cbr-with-qos-interval"].apps[values[2]].insert(id);
            paramMap["cbr-with-qos-start-time"].apps[values[3]].insert(id);
            paramMap["cbr-with-qos-end-time"].apps[values[4]].insert(id);
            paramMap["cbr-with-qos-priority"].apps[values[5]].insert(id);
            paramMap["cbr-with-qos-baseline-bandwidth-bytes"].apps[values[6]].insert(id);
            paramMap["cbr-with-qos-max-bandwidth-bytes"].apps[values[7]].insert(id);
            if (values.size() >= 9) {
                paramMap["cbr-with-qos-schedule-scheme"].apps[values[8]].insert(id);
            }//if//

        }
        else if ((appLabel == "vbr") && (values.size() >= 8)) {
            paramMap["vbr-destination"].apps[values[0]].insert(id);
            paramMap["vbr-payload-size-bytes"].apps[values[1]].insert(id);
            paramMap["vbr-mean-packet-interval"].apps[values[2]].insert(id);
            paramMap["vbr-minimum-packet-interval"].apps[values[3]].insert(id);
            paramMap["vbr-maximum-packet-interval"].apps[values[4]].insert(id);
            paramMap["vbr-start-time"].apps[values[5]].insert(id);
            paramMap["vbr-end-time"].apps[values[6]].insert(id);
            paramMap["vbr-priority"].apps[values[7]].insert(id);

        }
        else if ((appLabel == "vbrwithqos") && (values.size() >= 10)) {
            paramMap["vbr-with-qos-destination"].apps[values[0]].insert(id);
            paramMap["vbr-with-qos-payload-size-bytes"].apps[values[1]].insert(id);
            paramMap["vbr-with-qos-mean-packet-interval"].apps[values[2]].insert(id);
            paramMap["vbr-with-qos-minimum-packet-interval"].apps[values[3]].insert(id);
            paramMap["vbr-with-qos-maximum-packet-interval"].apps[values[4]].insert(id);
            paramMap["vbr-with-qos-start-time"].apps[values[5]].insert(id);
            paramMap["vbr-with-qos-end-time"].apps[values[6]].insert(id);
            paramMap["vbr-with-qos-priority"].apps[values[7]].insert(id);
            paramMap["vbr-with-qos-baseline-bandwidth-bytes"].apps[values[8]].insert(id);
            paramMap["vbr-with-qos-max-bandwidth-bytes"].apps[values[9]].insert(id);
            if (values.size() >= 11) {
                paramMap["vbr-with-qos-schedule-scheme"].apps[values[10]].insert(id);
            }//if//

        }
        else if ((appLabel == "ftp") && (values.size() >= 5)) {
            paramMap["ftp-destination"].apps[values[0]].insert(id);
            paramMap["ftp-flow-size-bytes"].apps[values[1]].insert(id);
            paramMap["ftp-flow-start-time"].apps[values[2]].insert(id);
            paramMap["ftp-flow-end-time"].apps[values[3]].insert(id);
            paramMap["ftp-priority"].apps[values[4]].insert(id);

        }
        else if ((appLabel == "ftpwithqos") && (values.size() >= 9)) {
            paramMap["ftp-with-qos-destination"].apps[values[0]].insert(id);
            paramMap["ftp-with-qos-flow-size-bytes"].apps[values[1]].insert(id);
            paramMap["ftp-with-qos-flow-start-time"].apps[values[2]].insert(id);
            paramMap["ftp-with-qos-flow-end-time"].apps[values[3]].insert(id);
            paramMap["ftp-with-qos-priority"].apps[values[4]].insert(id);
            paramMap["ftp-with-qos-baseline-bandwidth-bytes"].apps[values[5]].insert(id);
            paramMap["ftp-with-qos-max-bandwidth-bytes"].apps[values[6]].insert(id);
            paramMap["ftp-with-qos-baseline-reverse-bandwidth-bytes"].apps[values[7]].insert(id);
            paramMap["ftp-with-qos-max-reverse-bandwidth-bytes"].apps[values[8]].insert(id);
            if (values.size() >= 10) {
                paramMap["ftp-with-qos-schedule-scheme"].apps[values[9]].insert(id);
            }//if//

        }
        else if (((appLabel == "multiftp") || (appLabel == "multipleftp")) && (values.size() >= 8)) {
            paramMap["multiftp-destination"].apps[values[0]].insert(id);
            paramMap["multiftp-max-flow-data-bytes"].apps[values[1]].insert(id);
            paramMap["multiftp-mean-flow-data-bytes"].apps[values[2]].insert(id);
            paramMap["multiftp-standard-deviation-flow-data-bytes"].apps[values[3]].insert(id);
            paramMap["multiftp-mean-reading-time"].apps[values[4]].insert(id);
            paramMap["multiftp-start-time"].apps[values[5]].insert(id);
            paramMap["multiftp-end-time"].apps[values[6]].insert(id);
            paramMap["multiftp-priority"].apps[values[7]].insert(id);

        }
        else if (((appLabel == "multiftpwithqos") || (appLabel == "multipleftpwithqos")) && (values.size() >= 12)) {
            paramMap["multiftp-with-qos-destination"].apps[values[0]].insert(id);
            paramMap["multiftp-with-qos-max-flow-data-bytes"].apps[values[1]].insert(id);
            paramMap["multiftp-with-qos-mean-flow-data-bytes"].apps[values[2]].insert(id);
            paramMap["multiftp-with-qos-standard-deviation-flow-data-bytes"].apps[values[3]].insert(id);
            paramMap["multiftp-with-qos-mean-reading-time"].apps[values[4]].insert(id);
            paramMap["multiftp-with-qos-start-time"].apps[values[5]].insert(id);
            paramMap["multiftp-with-qos-end-time"].apps[values[6]].insert(id);
            paramMap["multiftp-with-qos-priority"].apps[values[7]].insert(id);
            paramMap["multiftp-with-qos-baseline-bandwidth-bytes"].apps[values[8]].insert(id);
            paramMap["multiftp-with-qos-max-bandwidth-bytes"].apps[values[9]].insert(id);
            paramMap["multiftp-with-qos-baseline-reverse-bandwidth-bytes"].apps[values[10]].insert(id);
            paramMap["multiftp-with-qos-max-reverse-bandwidth-bytes"].apps[values[11]].insert(id);
            if (values.size() >= 13) {
                paramMap["multiftp-with-qos-schedule-scheme"].apps[values[12]].insert(id);
            }//if//

        }
        else if (((appLabel == "video") || (appLabel == "videostreaming")) && (values.size() >= 13)) {
            paramMap["video-destination"].apps[values[0]].insert(id);
            paramMap["video-frame-rate"].apps[values[1]].insert(id);
            paramMap["video-number-packets-in-a-frame"].apps[values[2]].insert(id);
            paramMap["video-min-packet-payload-size-bytes"].apps[values[3]].insert(id);
            paramMap["video-max-packet-payload-size-bytes"].apps[values[4]].insert(id);
            paramMap["video-mean-packet-size-bytes"].apps[values[5]].insert(id);
            paramMap["video-min-inter-arrival-time"].apps[values[6]].insert(id);
            paramMap["video-max-inter-arrival-time"].apps[values[7]].insert(id);
            paramMap["video-mean-inter-arrival-time"].apps[values[8]].insert(id);
            paramMap["video-jitter-buffer-window"].apps[values[9]].insert(id);
            paramMap["video-start-time"].apps[values[10]].insert(id);
            paramMap["video-end-time"].apps[values[11]].insert(id);
            paramMap["video-priority"].apps[values[12]].insert(id);

        }
        else if (((appLabel == "videowithqos") || (appLabel == "videostreamingwithqos")) && (values.size() >= 15)) {
            paramMap["video-with-qos-destination"].apps[values[0]].insert(id);
            paramMap["video-with-qos-frame-rate"].apps[values[1]].insert(id);
            paramMap["video-with-qos-number-packets-in-a-frame"].apps[values[2]].insert(id);
            paramMap["video-with-qos-min-packet-payload-size-bytes"].apps[values[3]].insert(id);
            paramMap["video-with-qos-max-packet-payload-size-bytes"].apps[values[4]].insert(id);
            paramMap["video-with-qos-mean-packet-size-bytes"].apps[values[5]].insert(id);
            paramMap["video-with-qos-min-inter-arrival-time"].apps[values[6]].insert(id);
            paramMap["video-with-qos-max-inter-arrival-time"].apps[values[7]].insert(id);
            paramMap["video-with-qos-mean-inter-arrival-time"].apps[values[8]].insert(id);
            paramMap["video-with-qos-jitter-buffer-window"].apps[values[9]].insert(id);
            paramMap["video-with-qos-start-time"].apps[values[10]].insert(id);
            paramMap["video-with-qos-end-time"].apps[values[11]].insert(id);
            paramMap["video-with-qos-priority"].apps[values[12]].insert(id);
            paramMap["video-with-qos-baseline-bandwidth-bytes"].apps[values[13]].insert(id);
            paramMap["video-with-qos-max-bandwidth-bytes"].apps[values[14]].insert(id);
            if (values.size() >= 16) {
                paramMap["video-with-qos-schedule-scheme"].apps[values[15]].insert(id);
            }//if//

        }
        else if ((appLabel == "voip") && (values.size() >= 8)) {
            paramMap["voip-destination"].apps[values[0]].insert(id);
            paramMap["voip-mean-state-duration"].apps[values[1]].insert(id);
            paramMap["voip-state-transition-probability"].apps[values[2]].insert(id);
            paramMap["voip-beta-for-packet-arrival-delay-jitter"].apps[values[3]].insert(id);
            paramMap["voip-jitter-buffer-window"].apps[values[4]].insert(id);
            paramMap["voip-start-time"].apps[values[5]].insert(id);
            paramMap["voip-end-time"].apps[values[6]].insert(id);
            paramMap["voip-priority"].apps[values[7]].insert(id);

        }
        else if ((appLabel == "voipwithqos") && (values.size() >= 10)) {
            paramMap["voip-with-qos-destination"].apps[values[0]].insert(id);
            paramMap["voip-with-qos-mean-state-duration"].apps[values[1]].insert(id);
            paramMap["voip-with-qos-state-transition-probability"].apps[values[2]].insert(id);
            paramMap["voip-with-qos-beta-for-packet-arrival-delay-jitter"].apps[values[3]].insert(id);
            paramMap["voip-with-qos-jitter-buffer-window"].apps[values[4]].insert(id);
            paramMap["voip-with-qos-start-time"].apps[values[5]].insert(id);
            paramMap["voip-with-qos-end-time"].apps[values[6]].insert(id);
            paramMap["voip-with-qos-priority"].apps[values[7]].insert(id);
            paramMap["voip-with-qos-baseline-bandwidth-bytes"].apps[values[8]].insert(id);
            paramMap["voip-with-qos-max-bandwidth-bytes"].apps[values[9]].insert(id);
            if (values.size() >= 11) {
                paramMap["voip-with-qos-schedule-scheme"].apps[values[10]].insert(id);
            }//if//

        }
        else if ((appLabel == "http") && (values.size() >= 17)) {
            paramMap["http-destination"].apps[values[0]].insert(id);
            paramMap["http-min-main-object-bytes"].apps[values[1]].insert(id);
            paramMap["http-max-main-object-bytes"].apps[values[2]].insert(id);
            paramMap["http-mean-main-object-bytes"].apps[values[3]].insert(id);
            paramMap["http-standard-deviation-main-object-bytes"].apps[values[4]].insert(id);
            paramMap["http-min-number-embedded-objects"].apps[values[5]].insert(id);
            paramMap["http-max-number-embedded-objects"].apps[values[6]].insert(id);
            paramMap["http-mean-number-embedded-objects"].apps[values[7]].insert(id);
            paramMap["http-min-embedded-object-bytes"].apps[values[8]].insert(id);
            paramMap["http-max-embedded-object-bytes"].apps[values[9]].insert(id);
            paramMap["http-mean-embedded-object-bytes"].apps[values[10]].insert(id);
            paramMap["http-standard-deviation-embedded-object-bytes"].apps[values[11]].insert(id);
            paramMap["http-mean-page-reading-time"].apps[values[12]].insert(id);
            paramMap["http-mean-embedded-reading-time"].apps[values[13]].insert(id);
            paramMap["http-start-time"].apps[values[14]].insert(id);
            paramMap["http-end-time"].apps[values[15]].insert(id);
            paramMap["http-priority"].apps[values[16]].insert(id);

        }
        else if ((appLabel == "httpwithqos") && (values.size() >= 21)) {
            paramMap["http-with-qos-destination"].apps[values[0]].insert(id);
            paramMap["http-with-qos-min-main-object-bytes"].apps[values[1]].insert(id);
            paramMap["http-with-qos-max-main-object-bytes"].apps[values[2]].insert(id);
            paramMap["http-with-qos-mean-main-object-bytes"].apps[values[3]].insert(id);
            paramMap["http-with-qos-standard-deviation-main-object-bytes"].apps[values[4]].insert(id);
            paramMap["http-with-qos-min-number-embedded-objects"].apps[values[5]].insert(id);
            paramMap["http-with-qos-max-number-embedded-objects"].apps[values[6]].insert(id);
            paramMap["http-with-qos-mean-number-embedded-objects"].apps[values[7]].insert(id);
            paramMap["http-with-qos-min-embedded-object-bytes"].apps[values[8]].insert(id);
            paramMap["http-with-qos-max-embedded-object-bytes"].apps[values[9]].insert(id);
            paramMap["http-with-qos-mean-embedded-object-bytes"].apps[values[10]].insert(id);
            paramMap["http-with-qos-standard-deviation-embedded-object-bytes"].apps[values[11]].insert(id);
            paramMap["http-with-qos-mean-page-reading-time"].apps[values[12]].insert(id);
            paramMap["http-with-qos-mean-embedded-reading-time"].apps[values[13]].insert(id);
            paramMap["http-with-qos-start-time"].apps[values[14]].insert(id);
            paramMap["http-with-qos-end-time"].apps[values[15]].insert(id);
            paramMap["http-with-qos-priority"].apps[values[16]].insert(id);
            paramMap["http-with-qos-baseline-bandwidth-bytes"].apps[values[17]].insert(id);
            paramMap["http-with-qos-max-bandwidth-byte"].apps[values[18]].insert(id);
            paramMap["http-with-qos-baseline-reverse-bandwidth-bytes"].apps[values[19]].insert(id);
            paramMap["http-with-qos-max-reverse-bandwidth-bytes"].apps[values[20]].insert(id);
            if (values.size() >= 22) {
                paramMap["http-with-qos-schedule-scheme"].apps[values[21]].insert(id);
            }//if//

        }
        else if ((appLabel == "flooding") && (values.size() >= 11)) {
            paramMap["flooding-destination"].apps[values[0]].insert(id);
            paramMap["flooding-payload-size-bytes"].apps[values[1]].insert(id);
            paramMap["flooding-interval"].apps[values[2]].insert(id);
            paramMap["flooding-start-time"].apps[values[3]].insert(id);
            paramMap["flooding-end-time"].apps[values[4]].insert(id);
            paramMap["flooding-priority"].apps[values[5]].insert(id);
            paramMap["flooding-max-hop-count"].apps[values[6]].insert(id);
            paramMap["flooding-min-waiting-period"].apps[values[7]].insert(id);
            paramMap["flooding-max-waiting-period"].apps[values[8]].insert(id);
            paramMap["flooding-counter-threshold"].apps[values[9]].insert(id);
            paramMap["flooding-distance-threshold-in-meters"].apps[values[10]].insert(id);

        }
        else if ((appLabel == "iperfudp") && (values.size() >= 3)) {
            paramMap["iperf-udp-destination"].apps[values[0]].insert(id);
            paramMap["iperf-udp-priority"].apps[values[1]].insert(id);
            paramMap["iperf-udp-start-time"].apps[values[2]].insert(id);

        }
        else if ((appLabel == "iperftcp") && (values.size() >= 3)) {
            paramMap["iperf-tcp-destination"].apps[values[0]].insert(id);
            paramMap["iperf-tcp-priority"].apps[values[1]].insert(id);
            paramMap["iperf-tcp-start-time"].apps[values[2]].insert(id);
        }
        else {
            cerr << "Error: Application file line format incorrect: " << endl;
            cerr << '"' << aLine << '"' << values.size() << endl;

            for(size_t i = 0 ; i < values.size(); i++) {
                cerr << "Error" << i << " " << values[i] << endl;
            }//for//

            exit(1);
        }//if//
    }//while//

    OutputApplicationBasedConifg(paramMapPerApp);

}//OutputApplicationBasedConifg//

int main(int argc, char* argv[])
{

    if (argc != 2) {
        cerr << "update_old_config [source config file(.config)]" << endl;
        exit(1);
    }//if//

    const string fileName(argv[1]);
    ifstream inFile(fileName.c_str());

    if (!inFile.good()) {
        cerr << "Could Not open file: " << fileName << endl;
        exit(1);
    }//if//

    vector<string> lines;

    while(!inFile.eof()) {
        string aLine;
        getline(inFile, aLine);

        lines.push_back(aLine);
    }//while//

    for(size_t i = 0; i < lines.size(); i++) {
        string& aLine = lines[i];

        const string lowerLine = MakeLowerCaseString(aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            cout << aLine;
        }
        else {
            const size_t equalPos = aLine.find("=");

            if ((lowerLine.find("basic-applications-file") == 0) &&
                (equalPos != string::npos)) {

                string oldAppFileName = aLine.substr(equalPos + 1);
                DeleteTrailingSpaceAndQuotes(oldAppFileName);

                OutputApplicationBasedConifg(oldAppFileName);

            }
            else if (lowerLine.find("config-based-application-file") == 0) {
                //nothing to do.
            }
            else {
                cout << aLine;
            }//if//
        }//if//

        if (i != lines.size() - 1) {
            cout << endl;
        }//if//
    }//for//

    return 0;

}//main//

