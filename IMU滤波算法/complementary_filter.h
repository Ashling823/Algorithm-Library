#ifndef COMPLEMENTARY_H
#define COMPLEMENTARY_H

#define PI 3.1415926535

#define DGE_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

typedef struct
{
    float ax;
    float ay;
    float az;
} Accel_Data;

typedef struct
{
    float wx;
    float wy;
    float wz;
} Gyor_Data;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
} Oula_Angle;

typedef struct
{
    float alpha;
    Oula_Angle start_gyor_bias;
    Oula_Angle running_gyor_bias;
    float dt;
    float k;
} Comp_Filter;


void comp_filter_init(Comp_Filter *filter, float alpha, float dt);
void calibrate_bias_start(Comp_Filter *filter, uint16_t n, Gyor_Data *gyor_data);
bool is_imu_static(Accel_Data *accel_data, Gyor_Data *gyor_data);
void calibrate_bias_running(Comp_Filter *filter, Accel_Data *accel_data, Gyor_Data *gyor_data);
float accel_get_angle(const Accel_Data *data, Oula_Angle *angle);
float gyro_get_angle(const Gyor_Data *data, Oula_Angle *angle, Comp_Filter *filter);
float comp_filter_updata(Comp_Filter *filter, Oula_Angle *accel_angle, Oula_Angle *gyor_angle, Oula_Angle *filtered_angle);


/*使用示例*/
/*
Accel_Data accel_sensor;
Gyor_Data gyor_sensor;
Oula_Angle accel_angle;
Oula_Angle gyor_angle;
Oula filtered_angle;
Comp_Filter comp_filter;


comp_filter_init(&comp_filter,0.3f,0.01,0.3);
get_imu_acc_data(&accel_sensor);
get_imu_gyor_data(&gyor_data);

calibrate_bias_start(&comp_filter, 200, &gyor_data);
calibrate_bias_running(&comp_filter, &accel_data, &gyor_data);

accel_get_angle(&accel_sensor,&accel_angle);
gyor_get_angle(&gyor_data,&gyor_angle);
comp_filter_updata(&comp_filter,&accel_angle,&gyor_angle,&filtered_angle);
//现在这里的这个filter_data里面就是已经经过互补滤波后的角度值

*/

#endif