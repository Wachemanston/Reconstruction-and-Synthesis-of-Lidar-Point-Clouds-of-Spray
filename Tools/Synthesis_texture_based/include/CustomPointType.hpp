#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <unordered_map>

struct PointSpraySimInfo
{
    int32_t genTime = -1;
    float initVelX = 0.0f;
    float initVelY = 0.0f;
    float initVelZ = 0.0f;
    float initPosX = 0.0f;
    float initPosY = 0.0f;
    float initPosZ = 0.0f;
    int32_t genCode = -1;
    int32_t genIndex = -1;

    PointSpraySimInfo() = default;
    PointSpraySimInfo(
        int32_t genTime,
        float initVelX,
        float initVelY,
        float initVelZ,
        float initPosX,
        float initPosY,
        float initPosZ,
        int32_t genCode,
        int32_t genIndex);

    friend std::ostream& operator<<(std::ostream& os, const PointSpraySimInfo& obj);
    friend std::istream& operator>>(std::istream& is, PointSpraySimInfo& obj);

    bool operator==(const PointSpraySimInfo& other) const;

    // float x y z genTime initVelX initVelY initVelZ initPosX initPosY initPosZ genCode;
};

std::ostream& operator<<(std::ostream& os, const PointSpraySimInfo& obj);
std::istream& operator>>(std::istream& is, PointSpraySimInfo& obj);

// Custom type hash function, in order to put PointSpraySimInfo in unordered containers like
// unordered_map and unordered_set.
// Ref : https://stackoverflow.com/questions/17016175/c-unordered-map-using-a-custom-class-type-as-the-key
//
// Update :
// It seems that instantiating unordered_map of incomplete types is Undefined-Behavior.
// So it seems difficult to forward declare the "template <> struct std::hash<PointSpraySimInfo>"
// template specialization, and then define it in a .cpp file. Eventually I just define this in the
// header file.
// 
// Ref : 
// https://stackoverflow.com/questions/61356085/why-does-the-following-code-compile-using-clang-but-not-gcc
// https://stackoverflow.com/questions/61416646/c17-forward-declarations-of-custom-structures-in-not-working-with-custom-hash
// Template Specialization :
// https://en.cppreference.com/w/cpp/language/template_specialization
// template <>
// struct std::hash<PointSpraySimInfo>
// {
//     std::size_t operator()(const PointSpraySimInfo& k) const
//     {
//         return ((((std::hash<int32_t>()(k.genTime)
//                 ^ (std::hash<float>()(k.initVelX) << 1)) >> 1)
//                 ^ (std::hash<float>()(k.initVelY) << 1)) >> 1);

//         // return ((((((((((((((std::hash<int32_t>()(k.genTime)
//         //         ^ (std::hash<float>()(k.initVelX) << 1)) >> 1)
//         //         ^ (std::hash<float>()(k.initVelY) << 1)) >> 1)
//         //         ^ (std::hash<float>()(k.initVelZ) << 1)) >> 1)
//         //         ^ (std::hash<float>()(k.initPosX) << 1)) >> 1)
//         //         ^ (std::hash<float>()(k.initPosY) << 1)) >> 1)
//         //         ^ (std::hash<float>()(k.initPosZ) << 1)) >> 1)
//         //         ^ (std::hash<float>()(k.genCode)  << 1)) >> 1);
//     }
// };

template <class T1, class T2>
struct std::hash<std::pair<T1, T2>>
{
    std::size_t operator() (const std::pair<T1, T2> &pair) const
    {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

POINT_CLOUD_REGISTER_POINT_STRUCT   (PointSpraySimInfo,
                                    (int32_t, genTime,  genTime)
                                    (float, initVelX, initVelX)
                                    (float, initVelY, initVelY)
                                    (float, initVelZ, initVelZ)
                                    (float, initPosX, initPosX)
                                    (float, initPosY, initPosY)
                                    (float, initPosZ, initPosZ)
                                    (int32_t, genCode,  genCode)
                                    (int32_t, genIndex,  genIndex)
)

namespace ssfm
{
    pcl::PointCloud<PointSpraySimInfo>::Ptr LoadSpraySimPointCloud(std::string file);
}


// ====================================================================================

// Using the same idiom as HandmadeMath.h
// Ref : https://github.com/HandmadeMath/Handmade-Math/blob/master/HandmadeMath.h
struct PointWC2CD
{
    // Since the C2C_DIST casts all fields into floating point types
    // we cannot use the "union of struct" hack to do a super quick type
    // convertion. Fine, we can always stick with good old copy construction...

    float genTime;
    float initVelX;
    float initVelY;
    float initVelZ;
    float initPosX;
    float initPosY;
    float initPosZ;
    float genCode;
    float genIndex;
    float C2C_absolute_distances;

    friend std::ostream& operator<<(std::ostream& os, const PointWC2CD& obj);
};

std::ostream& operator<<(std::ostream& os, const PointWC2CD& obj);

POINT_CLOUD_REGISTER_POINT_STRUCT   (PointWC2CD,
                                    (float, genTime,  genTime)
                                    (float, initVelX, initVelX)
                                    (float, initVelY, initVelY)
                                    (float, initVelZ, initVelZ)
                                    (float, initPosX, initPosX)
                                    (float, initPosY, initPosY)
                                    (float, initPosZ, initPosZ)
                                    (float, genCode,  genCode)
                                    (float, genIndex,  genIndex)
                                    (float, C2C_absolute_distances,  C2C_absolute_distances)
)

namespace ssfm
{
    PointSpraySimInfo PointWC2CD_to_PointSpraySimInfo(const PointWC2CD& pointWC2CD);

    std::string GetFilterString(const PointSpraySimInfo& point);
}
