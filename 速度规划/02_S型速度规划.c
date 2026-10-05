#include <stdint.h>
#include <math.h>

typedef struct{
    double v_start;
    double v_end;
    double v_max;
    double a;
    double jerk;
} S_Speed_Para;

typedef struct
{
    double t_up_up;
    double t_up_const;
    double t_up_down;
    double t_const;
    double t_down_up;
    double t_down_const;
    double t_down_down;
    double t_total;
    double l_up;
    double l_const;
    double l_down;
} S_speed_plan;


//七段式
void S_Speed_Plan(S_Speed_Para*cfg, S_speed_plan *plan, float L)
{
    double v_start = cfg->v_start;
    double v_end = cfg->v_end;
    double v_max = cfg->v_max;
    double a = cfg->a;
    double jerk = cfg->jerk;

    double t_up_up=plan->t_up_up;
    double t_up_const=plan->t_up_const;
    double t_up_down=plan->t_up_down;
    double t_const=plan->t_const;
    double t_down_up=plan->t_down_up;
    double t_down_const=plan->t_down_const;
    double t_down_down=plan->t_down_down;
    double t_total=t_total;
    double l_up;
    double l_const;
    double l_down;

    t_up_up = a / jerk;
    double dv_up = v_max -v_start;

    if(dv_up<a*t_up_up)
    {
        t_up_up = sqrt(dv_up / jerk);
        
    }
}