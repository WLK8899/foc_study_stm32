#include "pid.h"

void PID_init(PID_t *pid, float Kp, float Ki, float Kd, float max, float min,float MAX_KI_Value)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->MAX_output = max;
    pid->MIN_output = min;
    pid->MAX_Ki_value = MAX_KI_Value;

    pid->error[0] = 0;
    pid->error[1] = 0;
    pid->error[2] = 0;

    pid->Kp_value = 0;
    pid->Ki_value = 0;
    pid->Kd_value = 0;
    pid->output = 0;
    
}


void PID_limit(PID_t *pid)
{
    if (pid->output >= pid->MAX_output)
        pid->output = pid->MAX_output;
    else if (pid->output <= pid->MIN_output)
        pid->output = pid->MIN_output;
}
void PID_KI_limit(PID_t *pid){
        if (pid->Ki_value >= pid->MAX_Ki_value)
        pid->Ki_value = pid->MAX_Ki_value;
    else if (pid->Ki_value <= -pid->MAX_Ki_value)
       pid->Ki_value = -pid->MAX_Ki_value;
}

void PID_calc(PID_t *pid,float speed,float speed_ref){
    pid->error[2]=pid->error[1];
    pid->error[1]=pid->error[0];
    pid->error[0]=speed_ref-speed;

    pid->Kp_value=pid->Kp*pid->error[0];
    pid->Ki_value+=pid->error[0]*pid->Ki;
    //Limit the Ki_value
    PID_KI_limit(pid);
    pid->Kd_value=pid->Kd*(pid->error[1]-pid->error[0]);

    pid->output=pid->Kp_value+pid->Ki_value+pid->Kd_value;

    PID_limit(pid);
}

// PI 控制器计算
void PI_Scalc(PID_t *pid, float speed, float speed_ref) {
    // 更新误差
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = speed_ref - speed;

    // 比例项
    pid->Kp_value = pid->Kp * pid->error[0];
    // 积分项
    pid->Ki_value += pid->Ki * pid->error[0];
    // 限制积分项
    PID_KI_limit(pid);

    // 计算输出
    pid->output = pid->Kp_value + pid->Ki_value;

    // 限制输出
    PID_limit(pid);
}