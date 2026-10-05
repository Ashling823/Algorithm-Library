#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "complementary_filter.h"

void comp_filter_init(Comp_Filter *filter, float alpha, float dt,float k)
{
    filter->alpha = alpha;
    filter->dt = dt;
    filter->k = k;
    filter->start_gyor_bias.pitch = 0.0f;
    filter->start_gyor_bias.roll = 0.0f;
    filter->start_gyor_bias.yaw = 0.0f;
    filter->running_gyor_bias.pitch = 0.0f;
    filter->running_gyor_bias.roll = 0.0f;
    filter->running_gyor_bias.yaw = 0.0f;
}

//上电时来进行零偏检查
void calibrate_bias_start(Comp_Filter *filter, uint16_t n, Gyor_Data*gyor_data)
{
    float sum_pitch = 0.0f;
    float sum_roll = 0.0f;
    float sum_yaw = 0.0f;
    for (int i = 0;i<n,i++)
    {
        sum_pitch += gyor_data->wx;
        sum_roll += gyor_data->wy;
        sum_yaw += gyor_data->wz;
    }
    filter->start_gyor_bias.pitch = sum_pitch / n;
    filter->start_gyor_bias.roll = sum_roll / n;
    filter->start_gyor_bias.yaw = sum_yaw / n;
}

// 运行时在线检测
bool is_imu_static(Accel_Data*accel_data,Gyor_Data*gyor_data)
{
    float ax = accel_data->ax;
    float ay = accel_data->ay;
    float az = accel_data->az;
    float wz = gyor_data->wz;
    float wy = gyor_data->wy;
    float wx = gyor_data->wx;

    float acc_mag = sqrtf(ax * ax + ay * ay + az * az);

    float gyro_mag = sqrtf(wx * wx + wy * wy + wz * wz);

    return (fabsf(acc_mag - 1.0f) < 0.05f) 
           && (gyro_mag < 2.0f);          
}

// 运行时在线检测
void calibrate_bias_running(Comp_Filter *filter, Accel_Data *accel_data, Gyor_Data *gyor_data)
{
    float running_pitch_bias = filter->running_gyor_bias.pitch;
    float running_roll_bias = filter->running_gyor_bias.roll;
    float running_yaw_bias = filter->running_gyor_bias.yaw;
    float k = filter->k;
    if (is_imu_static(accel_data, gyor_data))
    {
        running_pitch_bias += k*(gyor_data->wx - running_pitch_bias);
        running_roll_bias += k * (gyor_data->wy - running_roll_bias);
        running_yaw_bias += k * (gyor_data->wz - running_yaw_bias);
    }
}

float accel_get_angle(const Accel_Data*data,Oula_Angle*angle)
{
    float ax = data->ax;
    float ay = data->ay;
    float az = data->az;
    angle->roll = atan2f(ax, az) * RAD_TO_DEG;
    angle->pitch = atant2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    angle->yaw = 0.0f;
}

float gyro_get_angle(const Gyor_Data *data, Oula_Angle *angle, Comp_Filter *filter)
{
    float wx = data->wx;
    float wy = data->wy;
    float wz = data->wz;
    float dt = filter->dt;
    float pitch_bias = filter->start_gyor_bias.pitch+filter->running_gyor_bias.pitch;
    float roll_bias = filter->start_gyor_bias.roll+filter->running_gyor_bias.roll;
    float yaw_bias = filter->start_gyor_bias.yaw+filter->running_gyor_bias.yaw;

    angle->pitch += (wy-pitch_bias) * dt;
    angle->roll += (wx-roll_bias) * dt;
    angle->yaw += (wz-yaw_bias) * dt;
}

float comp_filter_updata(Comp_Filter *filter, Oula_Angle *accel_angle, Oula_Angle *gyor_angle, Oula_Angle*filtered_angle)
{
    float alpha = filter->alpha;

    filtered_angle->pitch = alpha * gyor_angle->pitch + (1 - alpha) * accel_angle->pitch;

    filtered_angle->roll = alpha * gyor_angle->roll + (1 - alpha) * accel_angle->roll;

    filtered_angle->yaw = alpha * gyor_angle->yaw + (1 - alpha) * accel_angle->yaw;
}