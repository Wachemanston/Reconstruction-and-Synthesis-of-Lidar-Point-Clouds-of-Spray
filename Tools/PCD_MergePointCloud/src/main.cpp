#include <memory>
#include <string>
#include <sstream>
#include <utility>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <unordered_map>
#include <limits>
#include <experimental/filesystem>

// #include "rapidjson/document.h"
// #include "rapidjson/stringbuffer.h"

// #include "linalg.h"

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "CustomPointType.hpp"

namespace fs = std::experimental::filesystem;
namespace
{    
    // the 15th file in augmentDataDirectory matches the 0th pcd in realDataDirectory
    const int offset = 15;

    constexpr float pi() { return std::atan(1.0f) * 4.0f; }
    constexpr float pi_d2() { return std::atan(1.0f) * 2.0f; }
    constexpr float pi_m2() { return std::atan(1.0f) * 8.0f; }

    // compile-time Waymo lidar position
    constexpr std::array<float, 3> lidarPositionInWaymo
    {
        1.43f,
        0.0f,
        2.184f
    };
    // compile-time inclination data, sorted using < operator (less than)
    constexpr std::array<float, 64> inclinationTable
    {
        -0.30572402f,
        -0.29520813f,
        -0.28440472f,
        -0.273567f,
        -0.26308975f,
        -0.2534273f,
        -0.2439661f,
        -0.23481996f,
        -0.22545554f,
        -0.21602648f,
        -0.20670462f,
        -0.1978009f,
        -0.18938406f,
        -0.18126072f,
        -0.1726162f,
        -0.16460761f,
        -0.15673536f,
        -0.14926358f,
        -0.14163844f,
        -0.13426943f,
        -0.12712012f,
        -0.12003257f,
        -0.113345f,
        -0.10682256f,
        -0.10021271f,
        -0.09397242f,
        -0.08843948f,
        -0.08211779f,
        -0.07702542f,
        -0.07161406f,
        -0.06614764f,
        -0.06126762f,
        -0.05665334f,
        -0.05186487f,
        -0.0474777f,
        -0.04311179f,
        -0.0391097f,
        -0.03539012f,
        -0.03155818f,
        -0.02752789f,
        -0.02453311f,
        -0.02142827f,
        -0.01853052f,
        -0.01531176f,
        -0.01262421f,
        -0.00942546f,
        -0.00681699f,
        -0.00402034f,
        -0.00111514f,
        0.00189587f,
        0.00496216f,
        0.00799484f,
        0.01079605f,
        0.01351344f,
        0.01663751f,
        0.01930892f,
        0.02243253f,
        0.02530314f,
        0.02825533f,
        0.03097034f,
        0.03393894f,
        0.03666058f,
        0.03982922f,
        0.04311189f
    };
}

struct AzimuthPoint
{
    float azimuth;
    pcl::PointXYZI point;

    AzimuthPoint(float azimuth, const pcl::PointXYZI& point)
        : azimuth(azimuth), point(point)
        {}

    bool operator < (const AzimuthPoint& other) const
    {
        if(azimuth == other.azimuth)
        {
            // const float distSqrToLidar = 
            //     (point.x - lidarPositionInWaymo[0]) * (point.x - lidarPositionInWaymo[0]) + 
            //     (point.y - lidarPositionInWaymo[1]) * (point.y - lidarPositionInWaymo[1]) + 
            //     (point.z - lidarPositionInWaymo[2]) * (point.z - lidarPositionInWaymo[2]);
            // const float distSqrToLidarOther = 
            //     (other.point.x - lidarPositionInWaymo[0]) * (other.point.x - lidarPositionInWaymo[0]) + 
            //     (other.point.y - lidarPositionInWaymo[1]) * (other.point.y - lidarPositionInWaymo[1]) + 
            //     (other.point.z - lidarPositionInWaymo[2]) * (other.point.z - lidarPositionInWaymo[2]);

            if(point.x == other.point.x)
            {
                if(point.y == other.point.y)
                    return point.z < other.point.z;
                return point.y < other.point.y;
            }
            return point.x < other.point.x;
        }
        return azimuth < other.azimuth;
    }

    float operator -(const AzimuthPoint& other) const
    {
        return azimuth - other.azimuth;
    }
};

template<class EleType, class ColType>
auto FindClosestInSortedCollection(const EleType& value, const ColType& collection)
{
    const auto low = 
        std::lower_bound(collection.begin(), collection.end(), value)
        - collection.begin();
    const auto upp = 
        std::upper_bound(collection.begin(), collection.end(), value)
        - collection.begin();

    auto idx = low;
    if(idx >= static_cast<int>(collection.size()))
        --idx;
    // if low and upp are not at the same location, determine the closer one
    if(low != upp && upp < static_cast<int>(collection.size()))
    {
        const auto lowDiff = std::abs(value - collection[low]);
        const auto uppDiff = std::abs(value - collection[upp]);
        idx = ( lowDiff < uppDiff ) ? low : upp;
    }

    return idx;
}

std::string exec_command(const char* cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

int main(int, char* argv[])
{
    // usage : ./main </path/to/realdata> </path/to/augmentdata>
    std::string realDataDirectory { argv[1] };
    std::string augmentDataDirectory { argv[2] };

    std::string basenameCommand = std::string("basename ") + augmentDataDirectory + std::string(" .bag");
    std::string basename = exec_command(basenameCommand.c_str());
    // remove newline
    basename = basename.substr(0, basename.size() - 1);
    std::string mkdirCommand = std::string("mkdir ") + std::string(" ../output/") + basename;
    exec_command(mkdirCommand.c_str());
    mkdirCommand = std::string("mkdir ") + std::string(" ../output/bin/") + basename;
    exec_command(mkdirCommand.c_str());

    std::cerr << std::setprecision(16);

    // Note : Converting floating point values from float literals 
    // and hex strings results in different values. However, if we
    // turn the value recovered from hex strings to string again and 
    // obtain floating point value from that string, the final value
    // will be identical to that recovered from float literals in the
    // first place.
    std::vector<fs::path> pathsRealData(
        fs::directory_iterator{realDataDirectory}, fs::directory_iterator{});
    std::sort(pathsRealData.begin(), pathsRealData.end());

    std::vector<fs::path> pathsAugmentData(
        fs::directory_iterator{augmentDataDirectory}, fs::directory_iterator{});
    std::sort(pathsAugmentData.begin(), pathsAugmentData.end());

    for (size_t i = 0; i < pathsRealData.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr realDataCloud (new pcl::PointCloud<pcl::PointXYZI>);
        const std::string& realDataPath { pathsRealData[i] };
        {
            if (pcl::io::loadPCDFile<pcl::PointXYZI> (realDataPath, *realDataCloud) == -1) //* load the file
            {
                std::cerr << "Error opening pcd file : " << realDataPath << " \n";
                return 1;
            }
        }

        pcl::PointCloud<PointSpraySimInfo>::Ptr augmentDataCloud (new pcl::PointCloud<PointSpraySimInfo>);
        {
            const std::string& augmentDataPath { pathsAugmentData[i + offset] };
            if (pcl::io::loadPCDFile<PointSpraySimInfo> (augmentDataPath, *augmentDataCloud) == -1) //* load the file
            {
                std::cerr << "Error opening pcd file : " << augmentDataPath << " \n";
                return 1;
            }
        }

        // std::cerr << "i = " << i << ", pathsRealData[i] = " << pathsRealData[i] << "\n";
        // std::cerr << ", pathsAugmentData[i + offset] = " << pathsAugmentData[i + offset] << "\n";

        // For each point "Pa" in augmentDataCloud, we need to find a point "Pb" in realDataCloud that
        // "Pa" might be occluded by "Pb".
        //
        // Maybe sort all the points in realDataCloud by azimuth and inclination.
        // And for each point in augmentDataCloud, do binary search to find the closest match.

        // The resolution of waymo lidar scan is 2650 * 64
        // Since inclination is not affected by ego-motion compensation, use that as concrete index.
        // And for azimuth, just place the points in and sort them according to azimuth.
        // When determine occlusion, find the point with closest azimuth, and the difference must be under
        // an epsilon value, which in this case should probably be around 1 / 2650 as that corresponds to
        // the horizontal resolution of the lidar.
        std::array<std::vector<AzimuthPoint>, 64> organizedPointTable;
        for(auto&& pointVec : organizedPointTable)
        {
            pointVec.clear();
            pointVec.reserve(2650);
        }

        for(const auto& point : *realDataCloud)
        {
            const float distToLidarXY = std::sqrt(
                (point.x - lidarPositionInWaymo[0]) * (point.x - lidarPositionInWaymo[0]) + 
                (point.y - lidarPositionInWaymo[1]) * (point.y - lidarPositionInWaymo[1]));
            const float inclination = std::atan2(
                point.z - lidarPositionInWaymo[2], distToLidarXY);

            auto idx = FindClosestInSortedCollection(inclination, inclinationTable);

            const float azimuth = std::atan2(
                point.y - lidarPositionInWaymo[1],
                point.x - lidarPositionInWaymo[0]
            );
            
            organizedPointTable[idx].emplace_back(azimuth, point);
        }

        // for(auto&& pointVec : organizedPointTable)
        //     std::sort(pointVec.begin(), pointVec.end());

        // For each point in augmentDataCloud, find the closest match and determine whether it's
        // an occlusion or not.
        std::vector<pcl::PointXYZ> addingPointFromAugmentDataCloud;
        addingPointFromAugmentDataCloud.reserve((*augmentDataCloud).size());

        for(const auto& point : *augmentDataCloud)
        {
            if(point.genTime < 0) continue;

            const float distToLidarXY = std::sqrt(
                (point.x - lidarPositionInWaymo[0]) * (point.x - lidarPositionInWaymo[0]) + 
                (point.y - lidarPositionInWaymo[1]) * (point.y - lidarPositionInWaymo[1]));
            const float inclination = std::atan2(
                point.z - lidarPositionInWaymo[2], distToLidarXY);

            auto idx = FindClosestInSortedCollection(inclination, inclinationTable);

            // Locate azimuth index
            const float azimuth = std::atan2(
                point.y - lidarPositionInWaymo[1],
                point.x - lidarPositionInWaymo[0]
            );

            auto& tableEntry = organizedPointTable[idx];
            // Sort before appling searching algorithms (lower_bound and upper_bound)

            std::sort(tableEntry.begin(), tableEntry.end());
            
            AzimuthPoint dummy { azimuth, pcl::PointXYZI() };
            auto azimuthIdx = FindClosestInSortedCollection(dummy, tableEntry);
            
            // if the difference in azimuth is greater than a threshole / epsilon, just add the augment point
            if(std::abs(tableEntry[azimuthIdx].azimuth - azimuth) >= (50.0f / 2650.0f))
            {
                // tableEntry.emplace_back(azimuth, pcl::PointXYZ(point.x, point.y, point.z));
                addingPointFromAugmentDataCloud.emplace_back(point.x, point.y, point.z);
                continue;
            }

            // Determine which point is closer to lidar
            const float distSqrToLidarAugment = 
                (point.x - lidarPositionInWaymo[0]) * (point.x - lidarPositionInWaymo[0]) + 
                (point.y - lidarPositionInWaymo[1]) * (point.y - lidarPositionInWaymo[1]) + 
                (point.z - lidarPositionInWaymo[2]) * (point.z - lidarPositionInWaymo[2]);
            
            const float distSqrToLidarReal = 
                (tableEntry[azimuthIdx].point.x - lidarPositionInWaymo[0]) * (tableEntry[azimuthIdx].point.x - lidarPositionInWaymo[0]) + 
                (tableEntry[azimuthIdx].point.y - lidarPositionInWaymo[1]) * (tableEntry[azimuthIdx].point.y - lidarPositionInWaymo[1]) + 
                (tableEntry[azimuthIdx].point.z - lidarPositionInWaymo[2]) * (tableEntry[azimuthIdx].point.z - lidarPositionInWaymo[2]);

            if(distSqrToLidarAugment < distSqrToLidarReal)
            {
                // tableEntry[azimuthIdx].azimuth = azimuth;
                // tableEntry[azimuthIdx].point = pcl::PointXYZ(point.x, point.y, point.z);
                //
                // Update : Remove the point from read data
                tableEntry.erase(tableEntry.begin() + azimuthIdx);
                addingPointFromAugmentDataCloud.emplace_back(point.x, point.y, point.z);
            }
        }

        // Note : The current approach might remove points from augmentDataCloud if
        // some of the points were occluding each other, we might need to fix this.

        // TODO : Adjust the program to label vehicle points instead of spray points.
        // By using the parsed detection GT, obtain the OBB and perform point-OBB-intersection test.
        // Inverse the quaternion and perform qmul, then determine whether the point is now inside the extents

        // Put the occluded points into the combinedPointCloud
        pcl::PointCloud<pcl::PointXYZI> combinedPointCloud;
        for(const auto& pointVec : organizedPointTable)
            for(const auto& p : pointVec)
            {
                // pcl::PointXYZI sp;
                // sp.x = p.point.x;
                // sp.y = p.point.y;
                // sp.z = p.point.z;
                // sp.intensity = 0.0f;
                combinedPointCloud.push_back(p.point);
                // combinedPointCloud.push_back(sp);
            }
        //
        for(const auto& p : addingPointFromAugmentDataCloud)
        {
            pcl::PointXYZI sp;
            sp.x = p.x;
            sp.y = p.y;
            sp.z = p.z;
            sp.intensity = 0.0f;
            combinedPointCloud.push_back(sp);
        }
        //

        // Export
        const int pos = realDataPath.find_last_of("/");
        const std::string file_name = realDataPath.substr(pos + 1);
        std::cerr << "Saving file " << file_name << " as ASCII." << std::endl;

        pcl::PCDWriter pcdWriter;
        pcdWriter.writeASCII(std::string("../output/") + basename + std::string("/") + file_name, combinedPointCloud);
    }

    return 0;
}
