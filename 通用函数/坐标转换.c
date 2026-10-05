#include <math.h>
#include <stdint.h>

// 世界坐标指令 → 车身坐标
void worldToBody(float world_x, float world_y, float *self_x, float *self_y,
                 float robot_x, float robot_y, float robot_yaw)
{
    float dx = world_x - robot_x;
    float dy = world_y - robot_y;
    float c = cosf(robot_yaw);
    float s = sinf(robot_yaw);

    *self_x = c * dx + s * dy;
    *self_y = -s * dx + c * dy;
}

// 把车身坐标系下的指令 (bx, by) 转到世界坐标系
// 参数来自里程计或定位模块
void bodyToWorld(float bx, float by, float *wx, float *wy,
                 float robot_x, float robot_y, float robot_yaw)
{
    float c = cosf(robot_yaw);
    float s = sinf(robot_yaw);
    *wx = c * bx - s * by + robot_x;
    *wy = s * bx + c * by + robot_y;
}

// 把传感器坐标系下的点 (sx, sy) 转到车身坐标系
// offset_x, offset_y: 传感器在车身上的安装位置
// offset_yaw: 传感器相对车身的安装角度
void sensorToBody(float sx, float sy, float *bx, float *by,
                  float offset_x, float offset_y, float offset_yaw)
{
    float c = cosf(offset_yaw);
    float s = sinf(offset_yaw);
    *bx = c * sx - s * sy + offset_x;
    *by = s * sx + c * sy + offset_y;
}
