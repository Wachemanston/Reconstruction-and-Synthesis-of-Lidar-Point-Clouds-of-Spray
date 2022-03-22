#pragma once

#include <string>

namespace ssfm
{
    std::string ExecuteCommand(const char* cmd);

    // Work with CloudCompare
    float GetMeanP2PDistanceFromCloudCompareOutput(const std::string& commandResult);
    std::string GetOutputFileNameFromCloudCompareOutput(const std::string& commandResult);
    void ReplacePCDFileWithFilteredVersion(const std::string& pcdFile);

    // Work with generated textures
    int GetGenCodeFromFileName(const std::string& fileName);
    int GetWindowIdxFromFileName(const std::string& fileName);
}