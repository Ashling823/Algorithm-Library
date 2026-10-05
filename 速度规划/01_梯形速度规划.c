/**************************************************
 * 上位机来下发位置指令，将位置转换成速度
 * 并且将世界坐标系下的速度转换成车身坐标系下的三个速度
 **************************************************/

#include <stdint.h>
#include <math.h>

#define K_angle 1

typedef struct
{
    double v_start;
    double v_end;
    double v_max;
    double a;
} Tr_Speed_Para;

typedef struct
{
    double t_up;
    double t_const;
    double t_down;
    double t_total;
    double l_up;
    double l_const;
    double l_down;
} Tr_speed_plan;

typedef struct
{
    float x_current;
    float y_current;
    float x_target;
    float y_target;
    float yaw;
} Position;

float Calc_move_distance(float target_pos,float current_pos)
{
    float move_diatance=sqrt((pow(target_pos-current_pos,2));

    return move_distance;
}

void Tr_Speed_Plan(Tr_Speed_Para*cfg,float L, Tr_speed_plan *plan)
{
    double l_up = (pow(cfg->v_max, 2) - pow(cfg->v_start, 2)) / 2.0 * cfg->a;
    double l_down = (pow(cfg->v_max, 2) - pow(cfg->v_end, 2)) / 2.0 * cfg->a;

    double l_up_down = l_up + l_down;
    double v_peek;

    if (l_up_down > L)
    {
        v_peek = cfg->v_max;
        plan->t_up = (cfg->v_max - cfg->v_start) / cfg->a;
        plan->t_down = (cfg->v_max - cfg->v_end) / cfg->a;
        plan->t_const = (L - l_up - l_down) / cfg->v_max;
        plan->t_total = plan->t_up + plan->t_down + plan->t_const;
        plan->l_up = l_up;
        plan->l_down = l_down;
        plan->l_const = L - l_up - l_down;
    }
    else
    {
        v_peek = sqrt(2.0 * cfg->a * L + pow(cfg->v_start, 2) + pow(cfg->v_end, 2)) / 2.0;
        plan->t_up = (v_peek - cfg->v_start) / cfg->a;
        plan->t_down = (v_peek - cfg->v_end) / cfg->a;
        plan->t_const = 0;
        plan->t_total = plan->t_up + plan->t_down + plan->t_const;
        plan->l_up = l_up;
        plan->l_down = l_down;
        plan->l_const = 0;
    }

    return 0;
}

float Tr_Calc_Speed(Tr_Speed_Para *cfg, Tr_speed_plan *plan , double t)
{
    if (t < 0)
        t = 0;
    if (t > plan->t_total)
        t = plan->t_total;

    if (t < plan->t_up)
    {
        return cfg->v_start + cfg->a * t;
    }
    else if (plan->t_up < t < plan->t_const)
    {
        return cfg->v_max;
    }
    else
    {
        double dt = t - plan->t_up - plan->t_const;
        double v_peek = cfg->v_start + cfg->a * plan->t_up;
        return v_peek - cfg->a * dt;
    }
}

//使用车上的IMU得到是车身的朝向角，就是世界坐标系里面的角度
//将世界坐标系下的速度标量转换成车身坐标下的速度分量
//如果要来发送目标角度也是需要来对角速度做速度规划的
//发的是绝对角度，也就是目标朝向角
void Speed_World_To_Self(Position*pos,float world_speed, float *Vx, float*Vy, float *Vw)
{
    float dx = pos->x_target - pos->x_current;
    float dy = pos->y_target - pos->y_current;
    
    float yaw = IMU_GetYAW();
    // float d_angle = pos->angle_target - yaw;

    float ditance_x = Calc_move_distance(pos->x_target, pos->x_current);
    float ditance_y = Calc_move_distance(pos->y_target, pos->y_current);

    float world_vx = world_speed * dx / ditance_x;
    float world_vy = world_speed * dy /ditance_y;

    *Vx = cos(yaw) * world_vx + sin(yaw) * world_vy;
    *Vy = -sin(yaw) * world_vx + cos(yaw) * world_vy;

    *Vw = 0.0f;
    //*Vw = d_angle*K_angle;
}
