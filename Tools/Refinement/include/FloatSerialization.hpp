#pragma once

#include <string>

namespace ssfm
{
    class FloatSerialization
    {
    public:
        static float GetFloatFromHexString(const std::string& hexString);
        static float GetFloatFromFloatLiteral(const std::string& floatLiterals);
        static float GetFloatFromFloatLiteralFromHexString(const std::string& hexString);
        static uint32_t GetHexValueFromFloat(float fval);

        static std::string GetHexStringFromFloat(float fval);
    };
}
