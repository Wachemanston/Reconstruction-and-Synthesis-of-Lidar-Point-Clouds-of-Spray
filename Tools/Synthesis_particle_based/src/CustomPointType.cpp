#include "CustomPointType.hpp"

#include "FloatSerialization.hpp"

PointSpraySimInfo::PointSpraySimInfo(
        int32_t genTime,
        float   initVelX,
        float   initVelY,
        float   initVelZ,
        float   initPosX,
        float   initPosY,
        float   initPosZ,
        int32_t genCode,
        int32_t genIndex)
    : genTime(genTime),
    initVelX(initVelX),
    initVelY(initVelY),
    initVelZ(initVelZ),
    initPosX(initPosX),
    initPosY(initPosY),
    initPosZ(initPosZ),
    genCode(genCode),
    genIndex(genIndex)
    {}

std::ostream& operator<<(std::ostream& os, const PointSpraySimInfo& obj)
{
    os  << " genTime = " << obj.genTime
        << " initVelX = " << obj.initVelX
        << " initVelY = " << obj.initVelY
        << " initVelZ = " << obj.initVelZ
        << " initPosX = " << obj.initPosX
        << " initPosY = " << obj.initPosY
        << " initPosZ = " << obj.initPosZ
        << " genCode = " << obj.genCode
        << " genIndex = " << obj.genIndex
        ;
    return os;
}

std::istream& operator>>(std::istream& is, PointSpraySimInfo& obj)
{
    std::string hexString = "";

    is  >> obj.genTime
        >> obj.genCode
        >> obj.genIndex
        ;

    is >> hexString;
    obj.initVelX = ssfm::FloatSerialization::GetFloatFromHexString(hexString);
    is >> hexString;
    obj.initVelY = ssfm::FloatSerialization::GetFloatFromHexString(hexString);
    is >> hexString;
    obj.initVelZ = ssfm::FloatSerialization::GetFloatFromHexString(hexString);
    is >> hexString;
    obj.initPosX = ssfm::FloatSerialization::GetFloatFromHexString(hexString);
    is >> hexString;
    obj.initPosY = ssfm::FloatSerialization::GetFloatFromHexString(hexString);
    is >> hexString;
    obj.initPosZ = ssfm::FloatSerialization::GetFloatFromHexString(hexString);

    return is;
}

bool PointSpraySimInfo::operator==(const PointSpraySimInfo& other) const
{
    return (genTime == other.genTime)
           && (initVelX == other.initVelX)
           && (initVelY == other.initVelY)
           && (initVelZ == other.initVelZ)
           && (initPosX == other.initPosX)
           && (initPosY == other.initPosY)
           && (initPosZ == other.initPosZ)
           && (genCode == other.genCode)
           && (genIndex == other.genIndex)
           ;
}

namespace ssfm
{
    pcl::PointCloud<PointSpraySimInfo>::Ptr LoadSpraySimPointCloud(std::string file)
    {
        pcl::PointCloud<PointSpraySimInfo>::Ptr cloud (new pcl::PointCloud<PointSpraySimInfo>);

        if (pcl::io::loadPCDFile<PointSpraySimInfo> (file, *cloud) == -1) //* load the file
        {
            PCL_ERROR ((std::string("Couldn't read file ") + file + std::string("\n")).c_str());
        }

        return cloud;
    }
}


// ================= PointWC2CD =================

std::ostream& operator<<(std::ostream& os, const PointWC2CD& obj)
{
    // os  << obj.pssi
    os  << " genTime = " << obj.genTime
        << " initVelX = " << obj.initVelX
        << " initVelY = " << obj.initVelY
        << " initVelZ = " << obj.initVelZ
        << " initPosX = " << obj.initPosX
        << " initPosY = " << obj.initPosY
        << " initPosZ = " << obj.initPosZ
        << " genCode = " << obj.genCode
        << " genIndex = " << obj.genIndex
        << " C2C_absolute_distances = " << obj.C2C_absolute_distances
        ;
    return os;
}

namespace ssfm
{
    PointSpraySimInfo PointWC2CD_to_PointSpraySimInfo(const PointWC2CD& pointWC2CD)
    {
        return PointSpraySimInfo(
            static_cast<int32_t>(pointWC2CD.genTime),
            pointWC2CD.initVelX,
            pointWC2CD.initVelY,
            pointWC2CD.initVelZ,
            pointWC2CD.initPosX,
            pointWC2CD.initPosY,
            pointWC2CD.initPosZ,
            static_cast<int32_t>(pointWC2CD.genCode),
            static_cast<int32_t>(pointWC2CD.genIndex)
        );
    }

    std::string GetFilterString(const PointSpraySimInfo& point)
    {
        std::string result = "";
        result += std::to_string(point.genTime);
        result += ' ';
        result += std::to_string(point.genCode);
        result += ' ';
        result += std::to_string(point.genIndex);
        result += ' ';
        result += FloatSerialization::GetHexStringFromFloat(point.initVelX);
        result += ' ';
        result += FloatSerialization::GetHexStringFromFloat(point.initVelY);
        result += ' ';
        result += FloatSerialization::GetHexStringFromFloat(point.initVelZ);
        result += ' ';
        result += FloatSerialization::GetHexStringFromFloat(point.initPosX);
        result += ' ';
        result += FloatSerialization::GetHexStringFromFloat(point.initPosY);
        result += ' ';
        result += FloatSerialization::GetHexStringFromFloat(point.initPosZ);

        return result;
    }
}
