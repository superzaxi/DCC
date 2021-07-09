// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_MAIN_FUN_SUPPORT_H
#define SCENSIM_MAIN_FUN_SUPPORT_H

// Purpose of this header file is provide simple functions to reused in custom main() functions.

#include <iostream>
#include <fstream>
#include "randomnumbergen.h"
#include "scensim_stats.h"
#include "sysstuff.h"


namespace ScenSim {

using std::cerr;
using std::endl;
using std::cout;

inline
void MainFunctionArgvProcessingBasicSequentialOnly(
    int argc,
    const char * const argv[],
    string& configFileName,
    bool& isControlledByGui,
    bool& seedIsSet,
    RandomNumberGeneratorSeed& runSeed)
{
    SetCpuFloatingPointBehavior();

    isControlledByGui = false;
    seedIsSet = false;

    if ((argc < 2) || (argc > 5)) {
        cout << "Simulator Command Format: sim <configfile> -seed <seed number>" << endl;
        exit(1);
    }//if//

    int i = 2;
    while (i < argc) {
        string option(argv[i]);

        if (option == "-g") {
            isControlledByGui = true;
        }
        else if (option == "-b") {

            // batch simulation mode controlled by GUI
        }
        else if (option == "-seed") {

            ++i;
            if (i == argc) {
                cerr << "No seed value parameter: Example: -seed 123" << endl;
                exit(1);
            }

            string seedString(argv[i]);
            std::istringstream seedStream(seedString);
            seedStream >> runSeed;
            if (seedStream.fail()) {
                cerr << "Bad seed value: " << seedString << endl;
                exit(1);
            }

            assert(seedStream.eof());

            seedIsSet = true;
        }
        else {
            cerr << "Bad command line parameter: " << option << endl;
            exit(1);

        }//if//

        ++i;

    }//while//

    configFileName.assign(argv[1]);

}//MainFunctionArgvProcessingBasicSequentialOnly//



inline
void MainFunctionArgvProcessingEmulatorVersion(
    int argc,
    const char * const argv[],
    string& configFileName,
    bool& isControlledByGui,
    bool& partitionIndexIsSet,
    unsigned int& partitionIndex,
    bool& seedIsSet,
    RandomNumberGeneratorSeed& runSeed)
{
    SetCpuFloatingPointBehavior();

    isControlledByGui = false;
    partitionIndexIsSet = false;
    seedIsSet = false;

    if ((argc < 2) || (argc > 8)) {
        cout << "Simulator Command Format: sim <configfile> -seed <seed number>" << endl;
        exit(1);
    }//if//

    int i = 2;
    while (i < argc) {
        string option(argv[i]);

        if (option == "-g") {
            isControlledByGui = true;
        }
        else if (option == "-b") {

            // batch simulation mode controlled by GUI
        }
        //Future// else if (option == "-p") {
        //Future//     ++i;
        //Future//     if (i == argc) {
        //Future//         cerr << "No partition parameter value: Example: -p 0" << endl;
        //Future//         exit(1);
        //Future//     }

        //Future//     string partitionIndexString(argv[i]);
        //Future//     std::istringstream partitionIndexStream(partitionIndexString);
        //Future//     partitionIndexStream >> partitionIndex;
        //Future//     if (partitionIndexStream.fail()) {
        //Future//         cerr << "Bad partition index parameter: " << partitionIndexString << endl;
        //Future//         exit(1);
        //Future//     }//if//

        //Future//     assert(partitionIndexStream.eof());
        //Future//     partitionIndexIsSet = true;
        //Future// }
        else if (option == "-seed") {

            ++i;
            if (i == argc) {
                cerr << "No seed value parameter: Example: -seed 123" << endl;
                exit(1);
            }

            string seedString(argv[i]);
            std::istringstream seedStream(seedString);
            seedStream >> runSeed;
            if (seedStream.fail()) {
                cerr << "Bad seed value: " << seedString << endl;
                exit(1);
            }

            assert(seedStream.eof());

            seedIsSet = true;
        }
        else {
            cerr << "Bad command line parameter: " << option << endl;
            exit(1);

        }//if//

        ++i;

    }//while//

    configFileName.assign(argv[1]);

}//MainFunctionArgvProcessingEmulatorVersion//



inline
void MainFunctionArgvProcessingBasicParallelVersion1(
    int argc,
    const char * const argv[],
    string& configFileName,
    bool& isControlledByGui,
    unsigned int& numberParallelThreads,
    bool& runSequentially,
    bool& seedIsSet,
    RandomNumberGeneratorSeed& runSeed)
{
    SetCpuFloatingPointBehavior();

    isControlledByGui = false;
    numberParallelThreads = 1;
    runSequentially = true;
    seedIsSet = false;

    if ((argc < 2) || (argc > 7)) {
        cout << "Simulator Command Format: sim <configfile> -seed <seed number>" << endl;
        exit(1);
    }//if//

    int i = 2;
    while (i < argc) {
        string option(argv[i]);

        if (option == "-g") {
            isControlledByGui = true;
        }
        else if (option == "-b") {

            // batch simulation mode controlled by GUI
        }
        //Future// else if (option == "-n") {
        //Future//     ++i;
        //Future//     if (i == argc) {
        //Future//         cerr << "No number thread parameter: Example: -n 4" << endl;
        //Future//         exit(1);
        //Future//     }

        //Future//     string numberThreadsString(argv[i]);
        //Future//     std::istringstream numberThreadsStream(numberThreadsString);
        //Future//     numberThreadsStream >> numberParallelThreads;
        //Future//     if (numberThreadsStream.fail()) {
        //Future//         cerr << "Bad number thread parameter: " << numberThreadsString << endl;
        //Future//         exit(1);
        //Future//     }//if//

        //Future//     assert(numberThreadsStream.eof());

        //Future//     if (numberParallelThreads == 0) {
        //Future//         cerr << "Number threads  can't be 0 (-n 0)" << endl;
        //Future//         exit(1);

        //Future//     }//if//

        //Future//     runSequentially = false;
        //Future// }
        else if (option == "-seed") {

            ++i;
            if (i == argc) {
                cerr << "No seed value parameter: Example: -seed 123" << endl;
                exit(1);
            }

            string seedString(argv[i]);
            std::istringstream seedStream(seedString);
            seedStream >> runSeed;
            if (seedStream.fail()) {
                cerr << "Bad seed value: " << seedString << endl;
                exit(1);
            }

            assert(seedStream.eof());

            seedIsSet = true;
        }
        else {
            cerr << "Bad command line parameter: " << option << endl;
            exit(1);

        }//if//

        ++i;

    }//while//

    configFileName.assign(argv[1]);

}//MainArgvProcessingBasicParallelVersion1//


inline
void MainFunctionArgvProcessingMultiSystemsParallelVersion1(
    int argc,
    const char * const argv[],
    string& configFileName,
    bool& isControlledByGui,
    unsigned int& numberParallelThreads,
    bool& runSequentially,
    bool& seedIsSet,
    RandomNumberGeneratorSeed& runSeed,
    bool& isScenarioSettingOutputMode,
    string& outputConfigFileName)
{
    SetCpuFloatingPointBehavior();

    isControlledByGui = false;
    numberParallelThreads = 1;
    runSequentially = true;
    seedIsSet = false;
    isScenarioSettingOutputMode = false;

    if ((argc < 2) || (argc > 7)) {
        cout << "Simulator Command Format: sim <configfile> -seed <seed number> -scenario <output configfile>" << endl;
        exit(1);
    }//if//

    int i = 2;
    while (i < argc) {
        string option(argv[i]);

        if (option == "-g") {
            isControlledByGui = true;
        }
        else if (option == "-b") {

            // batch simulation mode controlled by GUI
        }
        //Future// else if (option == "-n") {
        //Future//     ++i;
        //Future//     if (i == argc) {
        //Future//         cerr << "No number thread parameter: Example: -n 4" << endl;
        //Future//         exit(1);
        //Future//     }

        //Future//     string numberThreadsString(argv[i]);
        //Future//     std::istringstream numberThreadsStream(numberThreadsString);
        //Future//     numberThreadsStream >> numberParallelThreads;
        //Future//     if (numberThreadsStream.fail()) {
        //Future//         cerr << "Bad number thread parameter: " << numberThreadsString << endl;
        //Future//         exit(1);
        //Future//     }//if//

        //Future//     assert(numberThreadsStream.eof());

        //Future//     if (numberParallelThreads == 0) {
        //Future//         cerr << "Number threads  can't be 0 (-n 0)" << endl;
        //Future//         exit(1);

        //Future//     }//if//

        //Future//     runSequentially = false;
        //Future// }
        else if (option == "-seed") {

            ++i;
            if (i == argc) {
                cerr << "No seed value parameter: Example: -seed 123" << endl;
                exit(1);
            }

            string seedString(argv[i]);
            std::istringstream seedStream(seedString);
            seedStream >> runSeed;
            if (seedStream.fail()) {
                cerr << "Bad seed value: " << seedString << endl;
                exit(1);
            }

            assert(seedStream.eof());

            seedIsSet = true;
        }
        else if (option == "-scenario") {

            isScenarioSettingOutputMode = true;

            ++i;
            if (i == argc) {
                cerr << "No scenario value parameter: Example: -scenario out.config" << endl;
                exit(1);
            }

            outputConfigFileName = argv[i];
        }
        else {
            cerr << "Bad command line parameter: " << option << endl;
            exit(1);

        }//if//

        ++i;

    }//while//

    configFileName.assign(argv[1]);

}//MainArgvProcessingBasicParallelVersion1//




inline
void SetupFileControlledStats(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    RuntimeStatisticsSystem& theRuntimeStatisticsSystem,
    StatViewCollection& theStatViewCollection,
    string& statsOutputFilename,
    bool& noDataOutputIsEnabled)
{
    if (theParameterDatabaseReader.ParameterExists("statistics-configuration-file")) {
        string statConfigFilename =
            theParameterDatabaseReader.ReadString("statistics-configuration-file");

        ReadStatConfigFile(
            theRuntimeStatisticsSystem,
            statConfigFilename,
            theStatViewCollection);

        if (!theParameterDatabaseReader.ParameterExists("statistics-output-file")) {
            cerr << "Warning: No statistics-output-file defined." << endl;
            statsOutputFilename = "";
        }
        else {
            statsOutputFilename = theParameterDatabaseReader.ReadString("statistics-output-file");

            // Open/Create file to check for any problems and delete any old data  and then close it.

            std::ofstream tempFile(statsOutputFilename.c_str());

            if (!tempFile.good()) {
                cerr << "Error: could not open statistics output file: " << statsOutputFilename << endl;
                exit(1);
            }//if//

            tempFile.close();

        }//if//

        noDataOutputIsEnabled = true;//default//
        if (theParameterDatabaseReader.ParameterExists("statistics-output-for-no-data")) {
            noDataOutputIsEnabled = theParameterDatabaseReader.ReadBool("statistics-output-for-no-data");
        }//if//

    }//if//
}//SetupFileControlledStats//




}//namespace//

#endif
