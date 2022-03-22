#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <unordered_map>

struct PointSpraySimInfo
{
    PCL_ADD_POINT4D;
    int32_t genTime = -1;
    float initVelX = 0.0f;
    float initVelY = 0.0f;
    float initVelZ = 0.0f;
    float initPosX = 0.0f;
    float initPosY = 0.0f;
    float initPosZ = 0.0f;
    int32_t genCode = -1;
    int32_t genIndex = -1;
    
    // float x y z genTime initVelX initVelY initVelZ initPosX initPosY initPosZ genCode;
};

POINT_CLOUD_REGISTER_POINT_STRUCT   (PointSpraySimInfo,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
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
