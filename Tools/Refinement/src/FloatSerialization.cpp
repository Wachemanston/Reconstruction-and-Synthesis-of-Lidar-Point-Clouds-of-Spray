#include "FloatSerialization.hpp"

#include <sstream>
#include <cstring>

namespace ssfm
{
    // Reference : 
    // https://www.codeproject.com/Questions/782966/How-Do-I-Convert-Hex-String-To-Float-Value
    // https://stackoverflow.com/questions/21323099/convert-a-hexadecimal-to-a-float-and-viceversa-in-c    
    // https://www.youtube.com/watch?v=L06nbZXD2D0&t=290s
    float FloatSerialization::GetFloatFromHexString(const std::string& hexString)
    {
        uint32_t hexValue;
        std::stringstream ss(hexString);
        ss >> std::hex >> hexValue;

        // float floatValue = *((float*)&hexValue);
        //
        // Avoid using reinterpret cast or something like above, use memcpy instead
        float floatValue;
        memcpy(&floatValue, &hexValue, sizeof(floatValue));

        return floatValue;
    }

    static std::stringstream s_SStream;

    float FloatSerialization::GetFloatFromFloatLiteral(const std::string& floatLiterals)
    {
        // Use stringstream instead of atof
        // Ref : https://stackoverflow.com/questions/32495453/atof-and-stringstream-produce-different-results

        s_SStream.str(floatLiterals);
        float fval;
        s_SStream >> fval;

        return fval;
    }

    float FloatSerialization::GetFloatFromFloatLiteralFromHexString(const std::string& hexString)
    {
        float hexFloatVal = GetFloatFromHexString(hexString);
        std::string hexFloatVal2Str = std::to_string(hexFloatVal);
        float ffval = std::atof(hexFloatVal2Str.c_str());

        return ffval;
    }

    uint32_t FloatSerialization::GetHexValueFromFloat(float fval)
    {
        uint32_t hexValue;
        memcpy(&hexValue, &fval, sizeof(hexValue));

        return hexValue;
    }

    std::string FloatSerialization::GetHexStringFromFloat(float fval)
    {
        uint32_t hexValue = GetHexValueFromFloat(fval);
        std::stringstream ss;
        ss << "0x" << std::hex << hexValue;

        return std::string { ss.str() };
    }
}