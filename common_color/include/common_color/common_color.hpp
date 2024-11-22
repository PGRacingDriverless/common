#ifndef COMMON_COLOR_HPP
#define COMMON_COLOR_HPP

namespace common {
    typedef struct Color {
        float r;
        float g;
        float b;
        float a;
    } Color;
    extern const common::Color BLUE;
    extern const common::Color LIGHT_BLUE;
    extern const common::Color BLUEISH;
    extern const common::Color YELLOW;
    extern const common::Color LIGHT_YELLOW;
    extern const common::Color YELLOWISH;
    extern const common::Color ORANGE;
    extern const common::Color RED;
    extern const common::Color GRAY;
};

#endif