/**
 *  Pipeline :
 *  
 *  1. Read in the merged-filter file,
 *     which should contain the information of all particles that we care about.
 *     Using these information we can maintain a map that uses the information of the particles as
 *     keys, and other statictical information (e.g. # of frames with above mean error) as values.
 *  
 *  2. Given a GT(GroundTruth) pcd seqnence, and synthesized pcd sequence.
 *     For each corresponding frame in the both sequences :
 *         Compute the p2p distance through a command to CloudCompare, obtain the mean error distance
 *             and the result file with the error distance of each point.
 *         For each entry/point in the result file, determine whether the error distance of a particle
 *             is above the mean error distance.
 * 
 *  3. Export a new filter that contains only the particles that have more frames of error distance
 *         lower than the mean error distance of the same frame.
 */

// Usage ./main > ../filter.txt

#include <memory>
#include <string>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <experimental/filesystem>

#include "Command.hpp"
#include "CustomPointType.hpp"
#include "FloatSerialization.hpp"

#include <ctime>

namespace fs = std::experimental::filesystem;
namespace
{
    const std::string groundTruthDirectory = "your value";
    const std::string simulationResultDirectory = "your value";

    // the 15th file in simulationResultDirectory matches the 0th pcd in groundTruthDirectory
    const int offset = 15;

    // key : initial state, representing a single particle
    // value : # of frames that the particle has p2p error distance greater than mean error distance
    std::unordered_map<PointSpraySimInfo, int> errorMap;
}

int main(void)
{
    std::cerr << std::setprecision(16);

    // Note : Converting floating point values from float literals 
    // and hex strings results in different values. However, if we
    // turn the value recovered from hex strings to string again and 
    // obtain floating point value from that string, the final value
    // will be identical to that recovered from float literals in the
    // first place.
    std::vector<fs::path> pathsGT(
        fs::directory_iterator{groundTruthDirectory}, fs::directory_iterator{});
    std::sort(pathsGT.begin(), pathsGT.end());

    std::vector<fs::path> pathsSim(
        fs::directory_iterator{simulationResultDirectory}, fs::directory_iterator{});
    std::sort(pathsSim.begin(), pathsSim.end());

    for (size_t i = 0; i < pathsGT.size(); ++i)
    {
        // Step0 : Filter out all points that are not particles in the simulation result.
        // Notice that this is destructive, so make sure to back up the files.
        ssfm::ReplacePCDFileWithFilteredVersion(pathsSim[i + offset]);

        // Step1 : Compute the p2p distance, using simulation result as compare group, and
        // groundtruth as reference group. We need to obtain the per-point distance statistics 
        // 
        // With AUTO_SAVE ON, the distance pcd will be stored to some place like
        // "<your path>/result0015_C2C_DIST_2021-03-12_14h49_55_133.pcd"
        // which will be displayed in the output of CloudCompare in the terminal as ...
        // [I/O] File '<your path>/result0015_C2C_DIST_2021-03-12_14h49_55_133.pcd' saved successfully
        //
        // We might need to extract this file name and read in as a point cloud, then compare each points' p2p error distance
        // to see whether a single particle have over-average p2p error distance.

        std::string command = std::string("cloudcompare.CloudCompare -silent -C_EXPORT_FMT PCD ");
        // Ref : https://www.cloudcompare.org/doc/wiki/index.php?title=Command_line_mode
        // The first point cloud is compared, and the second is reference
        command += std::string(" -o ") + std::string(pathsSim[i + offset]) + std::string(" ");
        command += std::string(" -o ") + std::string(pathsGT[i]) + std::string(" ");
        command += std::string(" -c2c_dist ");
        command += std::string(" 2> /dev/null ");
        
        std::string commandResult = ssfm::ExecuteCommand(command.c_str());
        float meanError = ssfm::GetMeanP2PDistanceFromCloudCompareOutput(commandResult);
        std::string p2pDistanceFileName = ssfm::GetOutputFileNameFromCloudCompareOutput(commandResult);

        /* std::cerr << "p2p Distance btw " << std::string(pathsGT[i]) << " and "
            << std::string(pathsSim[i + offset]) << " is : " << meanError 
            << ", p2p file name is : " << p2pDistanceFileName
            << " \n"; */

        pcl::PointCloud<PointWC2CD>::Ptr cloud (new pcl::PointCloud<PointWC2CD>);

        if (pcl::io::loadPCDFile<PointWC2CD> (p2pDistanceFileName, *cloud) == -1) //* load the file
        {
            std::cerr << "Error opening pcd file : " << p2pDistanceFileName << " \n";
            return 1;
        }

        for(auto& e : *cloud)
        {
            PointSpraySimInfo key = ssfm::PointWC2CD_to_PointSpraySimInfo(e);

            if(errorMap.count(key) == 0)
            {
                errorMap.emplace(key, 0);
            }

            if(e.C2C_absolute_distances <= meanError)
                ++errorMap.at(key);
            else
                --errorMap.at(key);
        }
    }

    std::cerr << "errorMap.size() = " << errorMap.size() << "\n";

    std::cout << "# groundTruthDirectory = " << groundTruthDirectory << "\n";
    std::cout << "# simulationResultDirectory = " << simulationResultDirectory << "\n";
    std::cout << "# offset = " << offset << "\n";

    for(auto& e : errorMap)
    {
        if(e.second >= 0)
        {
            auto p = e.first;
            std::cout << std::dec << p.genTime << " " << p.genCode << " " << 0 << " "
                << std::hex << std::showbase
                << ssfm::FloatSerialization::GetHexValueFromFloat(p.initVelX) << " "
                << ssfm::FloatSerialization::GetHexValueFromFloat(p.initVelY) << " "
                << ssfm::FloatSerialization::GetHexValueFromFloat(p.initVelZ) << " "
                << ssfm::FloatSerialization::GetHexValueFromFloat(p.initPosX) << " "
                << ssfm::FloatSerialization::GetHexValueFromFloat(p.initPosY) << " "
                << ssfm::FloatSerialization::GetHexValueFromFloat(p.initPosZ) << "\n"
                ;
        }
    }

    return 0;
}
