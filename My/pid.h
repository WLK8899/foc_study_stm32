#ifndef PID_H
#define PID_H

typedef struct _pid
{
    float Kp; 
    float Ki;
    float Kd;
   
    float error[3];

    float Kp_value;
    float Ki_value;
    float Kd_value;
    float output;

    float MAX_Ki_value;

    float MAX_output;
    float MIN_output;

} PID_t;

void PID_init(PID_t *pid ,float Kp, float Ki, float Kd ,float max, float min ,float MAX_KI_Value);

//Limit 
void PID_limit(PID_t *pid);
void PID_KI_limit(PID_t *pid);

void PID_Scalc(PID_t *pid,float speed,float speed_ref);

void PI_Scalc(PID_t *pid, float speed, float speed_ref) ;

#endif