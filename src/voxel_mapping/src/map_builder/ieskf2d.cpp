#include "ieskf2d.h"

void XState::operator+=(const Vec8f &delta)
{
    pos += delta.segment<2>(0);
    vel += delta.segment<2>(2);
    theta += delta[4];
    ba += delta.segment<2>(5);
    bg += delta[7];
}
Vec8f XState::operator-(const XState &other)
{
    Vec8f delta = Vec8f::Zero();
    delta.segment<2>(0) = pos - other.pos;
    delta.segment<2>(2) = vel - other.vel;
    delta[4] = normalize_angle(theta - other.theta);
    delta.segment<2>(5) = ba - other.ba;
    delta[7] = bg - other.bg;
    return delta;
}

void IESKF2D::predict(const IMUData &imu, const float &dt)
{
    Vec2f acc{imu.acc.head<2>() - X.ba};
    float omega{imu.gyro(2) - X.bg};
    float theta = X.theta;
    Mat2f r_theta = Eigen::Rotation2Df(theta).matrix();

    X.pos = X.pos + X.vel * dt;
    X.vel = X.vel + r_theta * acc * dt;
    X.theta = X.theta + omega * dt;

    Mat8f dp = Mat8f::Identity();
    dp.block<2, 2>(0, 2) = Mat2f::Identity() * dt;
    dp.block<2, 1>(2, 4) = Eigen::Rotation2Df(theta + M_PI_2) * acc * dt;
    dp.block<2, 2>(2, 5) = -r_theta * dt;
    dp.block<1, 1>(4, 7) = -Eigen::Matrix<float, 1, 1>::Identity() * dt;

    Mat8x6f dq = Mat8x6f::Zero();

    dq.block<2, 2>(2, 0) = r_theta * dt;
    dq.block<2, 2>(2, 3) = -r_theta * dt;

    dq.block<1, 1>(4, 2) = Eigen::Matrix<float, 1, 1>::Identity() * dt;
    dq.block<1, 1>(4, 5) = -Eigen::Matrix<float, 1, 1>::Identity() * dt;
    dq.block<2, 2>(5, 3) = Mat2f::Identity() * dt;
    dq.block<1, 1>(7, 6) = Eigen::Matrix<float, 1, 1>::Identity() * dt;

    P = dp * P * dp.transpose() + dq * Q * dq.transpose();
}