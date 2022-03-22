// usage : ./main > ../filter_nodup.txt

#include <memory>
#include <random>
#include <string>
#include <sstream>
#include <utility>
#include <algorithm>
#include <unordered_map>
#include <experimental/filesystem>

#include "stb_image.h"
#include "stb_image_write.h"
#include "HandmadeMath.h"

#include "Command.hpp"
#include "CustomPointType.hpp"
#include "FloatSerialization.hpp"

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
    stbi_flip_vertically_on_write(true);
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

    // Transfer into a 3D texture, where [u, v, w] = [initialPos_in_random_domain, timeStamp]
    // Avoid exporting raw data, too much space and lose precision
    // Instead, export as sequence of .png or .jpg images.
    // Those can be compressed efficiently and with good quality, and can be blurred or processed intuitively.

    // sort allParticles by genTime
    const auto particleComparer = [](const PointSpraySimInfo& lhs, const PointSpraySimInfo& rhs)
        {
            return lhs.genTime < rhs.genTime;
        };
    std::sort(std::begin(allParticles), std::end(allParticles), particleComparer);
    
    constexpr const auto textureWidth = 64;
    constexpr const auto textureHeight = 64;
    constexpr const auto textureChannel = 3;
    constexpr const auto textureSize = textureWidth * textureHeight * textureChannel * sizeof(u_char);
    std::array<std::array<u_char, textureSize>, 4> initialConditionTextures = { 0 };
    int32_t startingTimeStamp = allParticles.empty() ? 0 : allParticles[0].genTime;

    constexpr const int timeStampSeparation = 100;
    int windowIndex = 0;
    // num of texel overwrite : 4913 / 128818 = 0.03813907994 (below 4%, so we should be fine)
    // maybe encode the particle genIndex, and use it to sample the overall particle pool?
    for(const auto& particle : allParticles)
    {
        if(particle.genTime - startingTimeStamp >= timeStampSeparation)
        {
            // Output current texture content and reset it to be black texture
            // std::cerr << "Outputing image ... \n";
            for(size_t i = 0; i < initialConditionTextures.size(); ++i)
            {
                auto& texture = initialConditionTextures[i];
                std::string fileName {"../output/newly_output/texture_genCode"};
                
                fileName += std::to_string(i * 10 + 1);
                fileName += std::string{"_window"};

                std::stringstream ss;
                ss << std::setw(4) << std::setfill('0') << windowIndex;
                fileName += ss.str();
                
                fileName += std::string{".png"};

                if (stbi_write_png(fileName.c_str(), textureWidth, textureHeight, textureChannel, texture.data(), 0) == 0)
                    std::cerr << "Error when saving image\n";
                std::memset(texture.data(), 0, textureSize);
            }
            startingTimeStamp = particle.genTime;
            ++windowIndex;
        }

        // u vector : [1, 0, 0]
        // v vector : [0, 1/2, sqrt(3)/2] or [maxPos - minPos].Normalize
        /*
        a0l_tread_pickup : genCode = 1
        { (-0.790052, -0.911966, 1.256705) , 
        (-0.985052, -0.911966, 1.256705) , 
        (-0.985052, -0.991966, 1.395269) , 
        (-0.790052, -0.991966, 1.395269) , 
        } 
        a0r_tread_pickup : genCode = 11
        { (0.985052, -0.911966, 1.256705) , 
        (0.790052, -0.911966, 1.256705) , 
        (0.790052, -0.991966, 1.395269) , 
        (0.985052, -0.991966, 1.395269) , 
        } 
        a1l_tread_pickup : genCode = 21
        { (-0.790052, -0.911966, -1.881536) , 
        (-0.985052, -0.911966, -1.881536) , 
        (-0.985052, -0.991966, -1.742972) , 
        (-0.790052, -0.991966, -1.742972) , 
        } 
        a1r_tread_pickup : genCode = 31
        { (0.985052, -0.911966, -1.881536) , 
        (0.790052, -0.911966, -1.881536) , 
        (0.790052, -0.991966, -1.742972) , 
        (0.985052, -0.991966, -1.742972) , 
        } 

        genCode < 10 : front left
        10 < genCode < 20 : front right
        20 < genCode < 30 : back left
        30 < genCode < 40 : back right
        */

        hmm_vec3 minPoint {0.0f, 0.0f, 0.0f};
        hmm_vec3 maxPoint {0.0f, 0.0f, 0.0f};
        int32_t textureIndex = -1;
        switch (particle.genCode)
        {
        case 1:
            minPoint = {-0.985052f, -0.991966f, 1.395269f};
            maxPoint = {-0.790052f, -0.911966f, 1.256705f};
            textureIndex = 0;
            break;
        case 11:
            minPoint = {0.790052f, -0.991966f, 1.395269f};
            maxPoint = {0.985052f, -0.911966f, 1.256705f};
            textureIndex = 1;
            break;
        case 21:
            minPoint = {-0.985052f, -0.991966f, -1.742972f};
            maxPoint = {-0.790052f, -0.911966f, -1.881536f};
            textureIndex = 2;
            break;
        case 31:
            minPoint = {0.790052f, -0.991966f, -1.742972f};
            maxPoint = {0.985052f, -0.911966f, -1.881536f};
            textureIndex = 3;
            break;
        default:
            std::cerr << "Error : unhandled particle.genCode = " << particle.genCode << "!\n";
            break;
        }

        const float uMag = (maxPoint - minPoint).X;
        const float vMag = (maxPoint - minPoint).Y;

        hmm_vec3 particleInitPos {particle.initPosX, particle.initPosY, particle.initPosZ};
        hmm_vec3 deltaPosition = particleInitPos - minPoint;
        int32_t textureCoordU {static_cast<int32_t>(textureWidth * deltaPosition.X / uMag)};
        int32_t textureCoordV {static_cast<int32_t>(textureHeight * deltaPosition.Y / vMag)};
        textureCoordU = std::clamp(textureCoordU, 0, textureWidth - 1);
        textureCoordV = std::clamp(textureCoordV, 0, textureHeight - 1);

        // Ref : https://stackoverflow.com/questions/14276920/c-separate-int-in-to-char-array
        int32_t particleGenIdx = particle.genIndex;

        // particleGenIdx *= 150;

        u_char b = particleGenIdx % 256;
        // particleGenIdx /= 256;
        particleGenIdx >>= 8;
        u_char g = particleGenIdx % 256;
        particleGenIdx >>= 8;
        u_char r = particleGenIdx % 256;

        // random swap
        // if(particleGenIdx % 2 == 1)
        //     std::swap(r, g);

        const int32_t startTextureCoord = textureCoordV * textureChannel * textureWidth + textureCoordU * textureChannel;
        initialConditionTextures[textureIndex][startTextureCoord + 0] = r; // R
        initialConditionTextures[textureIndex][startTextureCoord + 1] = g; // G
        initialConditionTextures[textureIndex][startTextureCoord + 2] = b; // B
    }

    // Weird entry in filter : ++index_count_map.at(index); index = 249926
    // With different timeStamp of 19854 and 19855, might be a small bug in Unity's particle system
    // However, should be negligible, and only happens once in 128818 particles

    // Next, after we have all the textures, do the sampling and colect the results.
    // Lastly, output the results to a new filter file, which can then be used by the simulator 
    // to create a new point cloud result.
    // 
    // For each texture / image, obtain its genCode and timeStamp interval,
    // ( = [timeStampSeparation * windowIdx, (timeStampSeparation+1) * windowIdx) ).
    // From there, sample the texture a fixed amount of times per timeStamp, perhaps 5 times?
    //
    // So for timeStampSeparation = 100, sample 100 * 5 times, each 100 different timeStamps.
    // Texture coordinates (UV) is choosed by random uniform distribution [0, textureSize - 1].
    // And sample the texture using maybe "Bilinear filtering".
    // So obtain the texel value of 4 texels : 
    //     
    //     static_cast<int>(UV) + (0, 0) = a
    //     static_cast<int>(UV) + (1, 0) = b
    //     static_cast<int>(UV) + (0, 1) = c
    //     static_cast<int>(UV) + (1, 1) = d
    // 
    // and lerp based on the distance of UV to static_cast<float>(a, b, c, d)
    //
    // Update : maybe use 9 texels with 8 neighbors, so we don't need to take care of whether +1 or -1
    // is the closest neighbor. This might need to be done by determining whether the UV exceeds 0.5.
    //
    // Update : After some rethinking and search, bilinear filtering does not need to consider
    // the 0.5 case mentioned above. The corresponding texel is guaranteed to be the lower left texel.
    // Ref : https://github.com/dmikushin/bilinear

    std::vector<PointSpraySimInfo> sampledParticleInitialConditions;
    sampledParticleInitialConditions.reserve(200000);
    std::vector<fs::path> paths(
        fs::directory_iterator{"../output/newly_output"}
        , fs::directory_iterator{}
    );

    constexpr const int32_t numSamplePerTexture = 15;
    std::uniform_int_distribution<int> uniformUSampler(0, textureWidth - 1);
    std::uniform_int_distribution<int> uniformVSampler(0, textureHeight - 1);
    // Ref : https://stackoverflow.com/questions/14178264/c11-correct-stdarray-initialization
    //
    // Unfortunately, we need this ugly syntax to initialize a std::array of std::array
    std::array<std::array<int, 2>, 4> sampleOffsetUV
    {{
        {0, 0},
        {0, 1},
        {1, 1},
        {1, 0}
    }};

    for(auto& file : paths)
    {
        const std::string fileName {file};

        int texWidth, texHeight, texChannel;
        u_char* texData = stbi_load(fileName.c_str(), &texWidth, &texHeight, &texChannel, 0);
        if(texData == nullptr)
        {
            std::cerr << "stbi_load error of file : " << fileName << "\n";
            continue;
        }
        
        const int32_t genCode = ssfm::GetGenCodeFromFileName(fileName);
        const int32_t windowIdx = ssfm::GetWindowIdxFromFileName(fileName);

#if 0
        // for each texel in the texData, synthesis from neighbor texels.
        std::uniform_real_distribution uniform01Sampler(0.0f, 1.0f);
        for(int32_t samplingOriginV = 0; samplingOriginV < textureHeight; ++samplingOriginV)
        {
            for(int32_t samplingOriginU = 0; samplingOriginU < texWidth; ++samplingOriginU)
            {
                const u_char r = texData[3 * samplingOriginV * texWidth + 3 * samplingOriginU + 0];
                const u_char g = texData[3 * samplingOriginV * texWidth + 3 * samplingOriginU + 1];
                const u_char b = texData[3 * samplingOriginV * texWidth + 3 * samplingOriginU + 2];

                const int sampledGenIdx = (r << 16) + (g << 8) + b;
                // if the sampled texel corresponds to nothing, skip to the next sample
                // THIS IS FOR REDUCING SAMPLE COUNT
                if(sampledGenIdx == 0) continue;


                // Obtain the file name of next texture in time axis, if exist
                std::string fileName {"../output/newly_output/texture_genCode"};
                fileName += std::to_string(genCode);
                fileName += std::string{"_window"};
                std::stringstream ss;
                ss << std::setw(4) << std::setfill('0') << windowIdx + 1;
                fileName += ss.str();
                fileName += std::string{".png"};
                u_char* nextTexData = stbi_load(fileName.c_str(), &texWidth, &texHeight, &texChannel, 0);
                if(nextTexData == nullptr)
                {
                    // std::cerr << "stbi_load error of file : " << fileName << "\n";
                    continue;
                }
                ///////

                int validSampleCount = 0;
                float distanceSum = 0.0f;
                hmm_vec3 sumInitVel {0.0f, 0.0f, 0.0f};
                float sumGenTime = 0.0f;

                hmm_vec3 dummyInitPos = GetInitPosFromGenCodeAndUV(
                    genCode, 
                    (static_cast<float>(samplingOriginU) + uniform01Sampler(g_randomEngine))
                        / static_cast<float>(textureWidth), 
                    (static_cast<float>(samplingOriginV) + uniform01Sampler(g_randomEngine))
                        / static_cast<float>(textureHeight));

                for(size_t offIdx = 0; offIdx < sampleOffsetUV.size(); ++offIdx)
                {
                    const int samplingU = samplingOriginU + sampleOffsetUV[offIdx][0];
                    const int samplingV = samplingOriginV + sampleOffsetUV[offIdx][1];

                    if(samplingU < 0 || samplingU >= textureWidth) continue;
                    if(samplingV < 0 || samplingV >= textureWidth) continue;

                    // remember to consider edge cases where u+1 or v+1 might exceed the size of the texture.
                    const u_char r = texData[3 * samplingV * texWidth + 3 * samplingU + 0];
                    const u_char g = texData[3 * samplingV * texWidth + 3 * samplingU + 1];
                    const u_char b = texData[3 * samplingV * texWidth + 3 * samplingU + 2];

                    const int sampledGenIdx = (r << 16) + (g << 8) + b;
                    // if the sampled texel corresponds to nothing, skip to the next sample
                    if(sampledGenIdx == 0) continue;

                    ++validSampleCount;

                    if(genCodeIdxToParticleMap.count(std::make_pair(genCode, sampledGenIdx)) == 0)
                    {
                        std::cerr << "genCodeIdxToParticleMap not found : genCode = " << genCode << " sampledGenIdx = " << sampledGenIdx << "\n";
                        continue;
                    }

                    const PointSpraySimInfo sampledInfo = 
                        genCodeIdxToParticleMap.at(std::make_pair(genCode, sampledGenIdx));
                    const hmm_vec3 sampledInitPos {sampledInfo.initPosX, sampledInfo.initPosY, sampledInfo.initPosZ};

                    const float dist = HMM_LengthVec3(dummyInitPos - sampledInitPos);
                    distanceSum += dist;
                    sumInitVel += dist * hmm_vec3 { sampledInfo.initVelX, sampledInfo.initVelY, sampledInfo.initVelZ };
                    sumGenTime += dist * sampledInfo.genTime;
                    // for now, skip interpolation
                    // sampledParticleInitialConditions.emplace_back(sampledInfo);
                }

                // ALSO LOOP THROUGH NEXTTEXDATA
                /*
                for(size_t offIdx = 0; offIdx < sampleOffsetUV.size(); ++offIdx)
                {
                    const int samplingU = samplingOriginU + sampleOffsetUV[offIdx][0];
                    const int samplingV = samplingOriginV + sampleOffsetUV[offIdx][1];

                    if(samplingU < 0 || samplingU >= textureWidth) continue;
                    if(samplingV < 0 || samplingV >= textureWidth) continue;

                    // remember to consider edge cases where u+1 or v+1 might exceed the size of the texture.
                    const u_char r = nextTexData[3 * samplingV * texWidth + 3 * samplingU + 0];
                    const u_char g = nextTexData[3 * samplingV * texWidth + 3 * samplingU + 1];
                    const u_char b = nextTexData[3 * samplingV * texWidth + 3 * samplingU + 2];

                    const int sampledGenIdx = (r << 16) + (g << 8) + b;
                    // if the sampled texel corresponds to nothing, skip to the next sample
                    if(sampledGenIdx == 0) continue;

                    ++validSampleCount;

                    if(genCodeIdxToParticleMap.count(std::make_pair(genCode, sampledGenIdx)) == 0)
                    {
                        std::cerr << "genCodeIdxToParticleMap not found : genCode = " << genCode << " sampledGenIdx = " << sampledGenIdx << "\n";
                        continue;
                    }

                    const PointSpraySimInfo sampledInfo = 
                        genCodeIdxToParticleMap.at(std::make_pair(genCode, sampledGenIdx));
                    const hmm_vec3 sampledInitPos {sampledInfo.initPosX, sampledInfo.initPosY, sampledInfo.initPosZ};

                    const float dist = HMM_LengthVec3(dummyInitPos - sampledInitPos);
                    distanceSum += dist;
                    sumInitVel += dist * hmm_vec3 { sampledInfo.initVelX, sampledInfo.initVelY, sampledInfo.initVelZ };
                    sumGenTime += dist * sampledInfo.genTime;
                    // for now, skip interpolation
                    // sampledParticleInitialConditions.emplace_back(sampledInfo);
                }
                */
                // record the sampled particle initial conditions
                if(validSampleCount == 0 || distanceSum == 0.0f) continue;

                // if(validSampleCount >= 5)
                //     std::cerr << "validSampleCount = " << validSampleCount << "\n";

                // obtain average initial velocity
                sumInitVel /= distanceSum;
                sumGenTime /= distanceSum;
                sampledParticleInitialConditions.emplace_back(
                    sumGenTime,
                    sumInitVel.X,
                    sumInitVel.Y,
                    sumInitVel.Z,
                    dummyInitPos.X,
                    dummyInitPos.Y,
                    dummyInitPos.Z,
                    genCode,
                    0 // genIdx, ignored
                );

                stbi_image_free(nextTexData);
            }
        }

        stbi_image_free(texData);
    }
#endif

#if 1
        const int32_t timeStampStart = windowIdx * timeStampSeparation + allParticles[0].genTime;
        const int32_t timeStampEnd = timeStampStart + timeStampSeparation;
        // range of sampleTimeStamp = [timeStampStart, timeStampEnd - 1] or [timeStampStart, timeStampEnd)
        for(int32_t sampleTimeStamp = timeStampStart; sampleTimeStamp < timeStampEnd; ++sampleTimeStamp)
        {
            for(int32_t i = 0; i < numSamplePerTexture; ++i)
            {
                const int samplingOriginU = uniformUSampler(g_randomEngine);
                const int samplingOriginV = uniformVSampler(g_randomEngine);
                // Update note : each field of the newly sampled particle info is determind by follow
                //
                // initPos : derive from min and max point of each genCode and samplingUV
                // initVel : interpolated from texels
                // timeStamp : from "sampleTimeStamp"
                // genCode : from "genCode"
                // genIdx : not important
                //
                // Here we should collect neightbor texels with valid data (genIdx != 0)
                // and obtain each fields as aforementioned.

                // Update : here the argument U and V is in range [0, textureSize], so remember to normalize them
                // Update : cast to float "BEFORE" division, 
                // if cast after division, we just get an integer division result casted to float
                hmm_vec3 initPos = GetInitPosFromGenCodeAndUV(
                    genCode, 
                    static_cast<float>(samplingOriginU) / static_cast<float>(textureWidth), 
                    static_cast<float>(samplingOriginV) / static_cast<float>(textureHeight));

                int validSampleCount = 0;
                float distanceSum = 0.0f;
                hmm_vec3 sumInitVel {0.0f, 0.0f, 0.0f};
                for(size_t offIdx = 0; offIdx < sampleOffsetUV.size(); ++offIdx)
                {
                    const int samplingU = samplingOriginU + sampleOffsetUV[offIdx][0];
                    const int samplingV = samplingOriginV + sampleOffsetUV[offIdx][1];

                    if(samplingU < 0 || samplingU >= textureWidth) continue;
                    if(samplingV < 0 || samplingV >= textureWidth) continue;

                    // remember to consider edge cases where u+1 or v+1 might exceed the size of the texture.
                    const u_char r = texData[3 * samplingV * texWidth + 3 * samplingU + 0];
                    const u_char g = texData[3 * samplingV * texWidth + 3 * samplingU + 1];
                    const u_char b = texData[3 * samplingV * texWidth + 3 * samplingU + 2];

                    const int sampledGenIdx = (r << 16) + (g << 8) + b;
                    // if the sampled texel corresponds to nothing, skip to the next sample
                    if(sampledGenIdx == 0) continue;

                    ++validSampleCount;

                    if(genCodeIdxToParticleMap.count(std::make_pair(genCode, sampledGenIdx)) == 0)
                    {
                        std::cerr << "genCodeIdxToParticleMap not found : genCode = " << genCode << " sampledGenIdx = " << sampledGenIdx << "\n";
                        continue;
                    }

                    const PointSpraySimInfo sampledInfo = 
                        genCodeIdxToParticleMap.at(std::make_pair(genCode, sampledGenIdx));
                    const hmm_vec3 sampledInitPos {sampledInfo.initPosX, sampledInfo.initPosY, sampledInfo.initPosZ};

                    const float dist = HMM_LengthVec3(initPos - sampledInitPos);

                    distanceSum += dist;
                    sumInitVel += dist * hmm_vec3 { sampledInfo.initVelX, sampledInfo.initVelY, sampledInfo.initVelZ };

                    // for now, skip interpolation
                    // sampledParticleInitialConditions.emplace_back(sampledInfo);
                }
                // record the sampled particle initial conditions
                if(validSampleCount == 0) continue;

                // obtain average initial velocity
                sumInitVel /= distanceSum;
                sampledParticleInitialConditions.emplace_back(
                    sampleTimeStamp,
                    sumInitVel.X,
                    sumInitVel.Y,
                    sumInitVel.Z,
                    initPos.X,
                    initPos.Y,
                    initPos.Z,
                    genCode,
                    0 // genIdx, ignored
                );
            }
        }
    }
#endif


    // also output some useful information as comments in the output file
    std::cout << "# filterFileName = " << filterFileName << "\n";
    std::cout << "# numSamplePerTexture = " << numSamplePerTexture << "\n";
    std::cout << "# textureWidth = " << textureWidth << "\n";
    std::cout << "# textureHeight = " << textureHeight << "\n";
    std::cout << "# timeStampSeparation = " << timeStampSeparation << "\n";

    std::sort(std::begin(sampledParticleInitialConditions), std::end(sampledParticleInitialConditions), 
        particleComparer);

    for(const auto& sample : sampledParticleInitialConditions)
    {
        std::cout << ssfm::GetFilterString(sample) << "\n";
    }
    return 0;
}
