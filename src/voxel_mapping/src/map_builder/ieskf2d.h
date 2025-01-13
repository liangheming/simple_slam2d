#pragma once

#include "commons.h"
#include <iostream>
using Vec8f = Eigen::Matrix<float, 8, 1>;
using Mat8f = Eigen::Matrix<float, 8, 8>;
using Mat6f = Eigen::Matrix<float, 6, 6>;
using Mat8x6f = Eigen::Matrix<float, 8, 6>;
using Mat2x3f = Eigen::Matrix<float, 2, 3>;

struct XState
{
    Vec2f pos{Vec2f::Zero()};
    Vec2f vel{Vec2f::Zero()};
    float theta{0.0};
    Vec2f ba{Vec2f::Zero()};
    float bg{0.0};
    void operator+=(const Vec8f &delta);
    Vec8f operator-(const XState &other);
    friend std::ostream &operator<<(std::ostream &os, const XState &state)
    {
        std::cout << "========XState========" << std::endl;
        os << "pos: " << state.pos.transpose() << std::endl;
        os << "vel: " << state.vel.transpose() << std::endl;
        os << "theta: " << state.theta << std::endl;
        os << "ba: " << state.ba.transpose() << std::endl;
        os << "bg: " << state.bg << std::endl;
        std::cout << "========------========" << std::endl;
        return os;
    }
};
struct IESKF2D
{
    XState X;
    Mat8f P{Mat8f::Identity()};
    Mat6f Q{Mat6f::Identity()};
    void predict(const IMUData &imu, const float &dt);
};