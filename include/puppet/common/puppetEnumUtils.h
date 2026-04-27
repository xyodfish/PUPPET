#pragma once

#include <stdexcept>
#include <string>

#include "common/magic_enum.hpp"

class EnumUtils {
   public:
    template <typename T, class U>
    static std::string intToString(const T& value) {
        return std::string(magic_enum::enum_name<U>(static_cast<U>(value)));
    }

    template <typename U>
    static std::string toString(const U& value) {
        return std::string(magic_enum::enum_name<U>(value));
    }

    template <class U>
    static std::string toEnum(const U& enumVar) {
        return std::string(magic_enum::enum_name<U>(enumVar));
    }

    template <class U>
    static U toEnum(const std::string& name) {
        const auto res = magic_enum::enum_cast<U>(name);
        if (res.has_value()) {
            return res.value();
        }
        throw std::runtime_error("the input string is not a valid enum string");
    }
};
