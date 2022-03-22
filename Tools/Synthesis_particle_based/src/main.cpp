// usage : ./main > ../filter_nodup.txt

#include <memory>
#include <random>
#include <string>
#include <sstream>
#include <utility>
#include <algorithm>
#include <unordered_map>
#include <experimental/filesystem>

// #include "stb_image.h"
// #include "stb_image_write.h"
#include "HandmadeMath.h"

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "Command.hpp"
#include "CustomPointType.hpp"
#include "FloatSerialization.hpp"

#include <ctime>

namespace fs = std::experimental::filesystem;
namespace
{
    const std::string filterFileName = "your path to filter_nodup.txt";

    // key : [genCode, genIndex] pair, representing a single particle
    // value : the detailed initial condition of that particle
    std::unordered_map<std::pair<int32_t, int32_t>, PointSpraySimInfo> genCodeIdxToParticleMap;
    
    std::vector<PointSpraySimInfo> allParticles;

    // A manually seeded random engine, for reproducible results
    std::default_random_engine g_randomEngine(56687);


    constexpr const float searchRadius = 0.003f;
    constexpr const float perturbationMag = 0.008f;
    // unit in timestep, so 0.001 second
    constexpr const int timeWindowRadius = 50;
}

hmm_vec3 GetInitPosFromGenCodeAndUV(const int genCode, const float U, const float V)
{
    hmm_vec3 minPoint {0.0f, 0.0f, 0.0f};
    hmm_vec3 maxPoint {0.0f, 0.0f, 0.0f};
    switch (genCode)
    {
    case 1:
        minPoint = {-0.985052f, -0.991966f, 1.395269f};
        maxPoint = {-0.790052f, -0.911966f, 1.256705f};
        break;
    case 11:
        minPoint = {0.790052f, -0.991966f, 1.395269f};
        maxPoint = {0.985052f, -0.911966f, 1.256705f};
        break;
    case 21:
        minPoint = {-0.985052f, -0.991966f, -1.742972f};
        maxPoint = {-0.790052f, -0.911966f, -1.881536f};
        break;
    case 31:
        minPoint = {0.790052f, -0.991966f, -1.742972f};
        maxPoint = {0.985052f, -0.911966f, -1.881536f};
        break;
    default:
        std::cerr << "Error : unhandled GetInitPosFromGenCodeAndUV genCode = " << genCode << "!\n";
        break;
    }
    // u vector : [1, 0, 0]
    // v vector : [0, 1/2, sqrt(3)/2] or [maxPos - minPos].Normalize
    const hmm_vec3 delta = maxPoint - minPoint;
    // sqrt(3) / 2 = 0.86602540378
    return hmm_vec3 
    {
        minPoint.X + U * delta.X,
        minPoint.Y + V * delta.Y,
        minPoint.Z + V * delta.Z
    };
}


int main(void)
{
    std::cerr << std::setprecision(16);
    std::cout << std::setprecision(16);
    // stbi_flip_vertically_on_write(true);
    allParticles.reserve(200000);
    // for seg29, multiple fit filter, size = 128818
    std::ifstream filterFile(filterFileName);
    if(filterFile.is_open() == false)
    {
        std::cerr << "Error opening filterFile of name " << filterFileName << ", it might not exist! \n";
    }

    {
        std::string line;
        while (std::getline(filterFile, line))
        {
            if(line.empty() == true || line[0] == '#') continue;
            
            std::istringstream iss(line);
            
            PointSpraySimInfo info;
            if (!(iss >> info))
            {
                std::cerr << "Error parsing PointSpraySimInfo for string : " << line << " \n";
                break;
            }

            // genCode is i*10 + [1, 4], so 1,2,3,4,11,12,13,14, 21 ... etc
            // auto genKey = std::make_pair(info.genCode, info.genIndex);
            genCodeIdxToParticleMap.emplace(std::piecewise_construct,
                std::forward_as_tuple(info.genCode, info.genIndex),
                std::forward_as_tuple(info));
            allParticles.emplace_back(info);
        }
    }

    // Build kd-tree
    pcl::PointCloud<pcl::PointXYZ>::Ptr particleCloud(new pcl::PointCloud<pcl::PointXYZ>);;

    // Generate pointcloud data, Setup kd-tree
    particleCloud->width = allParticles.size();
    particleCloud->height = 1;
    particleCloud->points.resize(particleCloud->width * particleCloud->height);
    for (int i = 0; i < allParticles.size(); ++i)
    {
        (*particleCloud)[i].x = allParticles[i].initPosX;
        (*particleCloud)[i].y = allParticles[i].initPosY;
        (*particleCloud)[i].z = allParticles[i].initPosZ;
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> particleKDtree;
    particleKDtree.setInputCloud (particleCloud);

    // Start sampling, for each particle, add a small perturbation
    std::vector<PointSpraySimInfo> sampledParticleInitialConditions;
    sampledParticleInitialConditions.reserve(200000);
    std::uniform_real_distribution uniformMag1Sampler(-1.0f, 1.0f);
    for (int i = 0; i < allParticles.size(); ++i)
    {
        auto& currentParticle = allParticles[i];

        hmm_vec3 perturbedInitPos
        {
            currentParticle.initPosX,
            currentParticle.initPosY,
            currentParticle.initPosZ
        };

        hmm_vec3 minPoint {0.0f, 0.0f, 0.0f};
        hmm_vec3 maxPoint {0.0f, 0.0f, 0.0f};
        switch (currentParticle.genCode)
        {
        case 1:
            minPoint = {-0.985052f, -0.991966f, 1.395269f};
            maxPoint = {-0.790052f, -0.911966f, 1.256705f};
            break;
        case 11:
            minPoint = {0.790052f, -0.991966f, 1.395269f};
            maxPoint = {0.985052f, -0.911966f, 1.256705f};
            break;
        case 21:
            minPoint = {-0.985052f, -0.991966f, -1.742972f};
            maxPoint = {-0.790052f, -0.911966f, -1.881536f};
            break;
        case 31:
            minPoint = {0.790052f, -0.991966f, -1.742972f};
            maxPoint = {0.985052f, -0.911966f, -1.881536f};
            break;
        default:
            std::cerr << "Error : unhandled particle.genCode = " << currentParticle.genCode << "!\n";
            break;
        }

        const float uMag = (maxPoint - minPoint).X;
        const float vMag = (maxPoint - minPoint).Y;

        hmm_vec3 particleInitPos {currentParticle.initPosX, currentParticle.initPosY, currentParticle.initPosZ};
        hmm_vec3 deltaPosition = particleInitPos - minPoint;
        float textureCoordU {deltaPosition.X / uMag};
        float textureCoordV {deltaPosition.Y / vMag};

        float perturbationU = uniformMag1Sampler(g_randomEngine) * perturbationMag;
        float perturbationV = uniformMag1Sampler(g_randomEngine) * perturbationMag;

        // Control perturbation magnitude to 0.03f (size of a texel)
        perturbedInitPos = GetInitPosFromGenCodeAndUV(
            currentParticle.genCode,
            textureCoordU + perturbationU,
            textureCoordV + perturbationV
        );

        // Radius Search using KD-tree
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        pcl::PointXYZ searchPoint { perturbedInitPos.X, perturbedInitPos.Y, perturbedInitPos.Z };
        //
        if(particleKDtree.radiusSearch(searchPoint, searchRadius, 
            pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            int validSampleCount = 0;
            float distanceSum = 0.0f;
            hmm_vec3 sumInitVel {0.0f, 0.0f, 0.0f};
            float sumGenTime = 0.0f;

            for (int j = 0; j < pointIdxRadiusSearch.size(); ++j)
            {
                // the index of allParticles and particleCloud should be the same
                // so it should be fine to directly query the particle array
                const int neighborParticleIdx = pointIdxRadiusSearch[j];
                PointSpraySimInfo neighborParticle = allParticles[neighborParticleIdx];

                // First : check if genTime difference is less than 0.05 second
                if(std::abs(neighborParticle.genTime - currentParticle.genTime) > timeWindowRadius)
                    continue;

                // Second, start accumulating initial states
                const hmm_vec3 sampledInitPos 
                {
                    neighborParticle.initPosX, neighborParticle.initPosY, neighborParticle.initPosZ
                };

                const float dist = HMM_LengthVec3(perturbedInitPos - sampledInitPos);
                distanceSum += dist;
                
                sumInitVel += dist * hmm_vec3 { 
                    neighborParticle.initVelX, neighborParticle.initVelY, neighborParticle.initVelZ };
                sumGenTime += dist * neighborParticle.genTime;

                ++validSampleCount;
            }

            if(validSampleCount == 0 || distanceSum == 0.0f)
                continue;

            sumInitVel /= distanceSum;
            sumGenTime /= distanceSum;
            sampledParticleInitialConditions.emplace_back(
                sumGenTime,
                sumInitVel.X,
                sumInitVel.Y,
                sumInitVel.Z,
                perturbedInitPos.X,
                perturbedInitPos.Y,
                perturbedInitPos.Z,
                currentParticle.genCode,
                0 // genIdx, ignored
            );
        }
    }


    // also output some useful information as comments in the output file
    std::cout << "# filterFileName = " << filterFileName << "\n";
    std::cout << "# searchRadius = " << searchRadius << "\n";
    std::cout << "# perturbationMag = " << perturbationMag << "\n";
    std::cout << "# timeWindowRadius = " << timeWindowRadius << "\n";

    const auto particleComparer = [](const PointSpraySimInfo& lhs, const PointSpraySimInfo& rhs)
    {
        return lhs.genTime < rhs.genTime;
    };
    std::sort(std::begin(sampledParticleInitialConditions), std::end(sampledParticleInitialConditions), 
        particleComparer);

    return 0;
}
