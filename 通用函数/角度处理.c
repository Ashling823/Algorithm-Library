/*************************
 * 角度归一化
 */

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * 将角度归一化到 [-π, π]
 * @param angle 任意角度（弧度）
 * @return 归一化后的角度
 */
double normalize_angle_pi(double angle)
{
    angle = fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0)
    {
        angle += 2.0 * M_PI;
    }
    return angle - M_PI;
}

/**
 * 将角度归一化到 [0, 2π)
 */
double normalize_angle_2pi(double angle)
{
    angle = fmod(angle, 2.0 * M_PI);
    if (angle < 0.0)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

/**
 * 将值归一化到 [min_val, max_val) 区间
 * 适用于任意周期量（角度、相位、编码器计数等）
 */
double normalize_to_range(double value, double min_val, double max_val)
{
    double range = max_val - min_val;
    if (range <= 0.0)
        return min_val; // 非法区间保护

    value = fmod(value - min_val, range);
    if (value < 0.0)
    {
        value += range;
    }
    return value + min_val;
}

/**
 * 计算两个角度之间的最短有符号差 (a - b)
 * 结果范围 [-π, π]
 */
double angle_diff(double a, double b)
{
    double d = a - b;
    return normalize_angle_pi(d);
}

/**
 * 由 (x, y) 求角度，范围 [-π, π]
 */
double angle_from_xy(double x, double y)
{
    return atan2(y, x);
}

/**
 * 角度转弧度
 * 弧度转角度
 */
inline double deg2rad(double d) { return d * PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / PI; }
