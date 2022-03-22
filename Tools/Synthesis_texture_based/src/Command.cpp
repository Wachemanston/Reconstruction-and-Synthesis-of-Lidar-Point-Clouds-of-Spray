#include "Command.hpp"

#include <array>
#include <memory>
#include <string_view>

namespace ssfm
{
    std::string ExecuteCommand(const char* cmd)
    {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
        if (!pipe)
        {
            throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
        {
            result += buffer.data();
        }

        return result;
    }

    float GetMeanP2PDistanceFromCloudCompareOutput(const std::string& commandResult)
    {
        constexpr const std::string_view mdstartKey = "[ComputeDistances] Mean distance = ";
        constexpr const std::string_view mdendKey = "/ std deviation";

        std::size_t meanDistanceStart = commandResult.find(mdstartKey) + mdstartKey.size();
        std::size_t meanDistanceEnd = commandResult.find(mdendKey, meanDistanceStart);
        std::string meanDistanceString = commandResult.substr(
            meanDistanceStart, 
            meanDistanceEnd - meanDistanceStart);
        
        return std::stof(meanDistanceString);
    }

    std::string GetOutputFileNameFromCloudCompareOutput(const std::string& commandResult)
    {
        constexpr const std::string_view mdendKey = "' saved successfully";

        std::size_t fileNameEnd = commandResult.find(mdendKey) - 1;
        std::size_t fileNameStart = commandResult.find_last_of('\'', fileNameEnd);

        std::string fileNameString = commandResult.substr(
            fileNameStart + 1, fileNameEnd - fileNameStart);
        
        return fileNameString;
    }

    void ReplacePCDFileWithFilteredVersion(const std::string& pcdFile)
    {
        // cloudcompare.CloudCompare -silent -AUTO_SAVE OFF -o result0015.pcd 
        //  -FILTER_SF 0.0 MAX -C_EXPORT_FMT PCD -SAVE_CLOUDS FILE "result0015.pcd"

        ssfm::ExecuteCommand((
            std::string("cloudcompare.CloudCompare -silent -AUTO_SAVE OFF  -o ")
             + std::string(pcdFile)
             + std::string(" -FILTER_SF 0.0 MAX ")
             + std::string(" -C_EXPORT_FMT PCD -SAVE_CLOUDS FILE ")
             + std::string(" \"") + pcdFile + std::string("\"  2> /dev/null ")
             ).c_str()
        );
    }

    int GetGenCodeFromFileName(const std::string& fileName)
    {
        constexpr const std::string_view startKey = "_genCode";
        constexpr const std::string_view endKey = "_window";

        std::size_t genCodeStart = fileName.find(startKey) + startKey.size();
        std::size_t genCodeEnd = fileName.find(endKey, genCodeStart);
        std::string genCodeString = fileName.substr(
            genCodeStart, 
            genCodeEnd - genCodeStart);

        return std::stoi(genCodeString);
    }

    int GetWindowIdxFromFileName(const std::string& fileName)
    {
        // texture_genCode1_window0003.png
        constexpr const std::string_view startKey = "_window";
        constexpr const std::string_view endKey = ".png";

        std::size_t windowIdxStart = fileName.find(startKey) + startKey.size();
        std::size_t windowIdxEnd = fileName.find(endKey, windowIdxStart);
        std::string windowIdxString = fileName.substr(
            windowIdxStart, 
            windowIdxEnd - windowIdxStart);

        return std::stoi(windowIdxString);
    }
}