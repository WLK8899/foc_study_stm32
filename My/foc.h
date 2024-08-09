
#ifndef FOC_H
#define FOC_H

#include <math.h>
#include <stdint.h>

typedef struct {
    float u_d;
    float u_q;
    float theta;

    float u_alpha;
    float u_beta;

    float t_a;
    float t_b;
    float t_c;

    float i_a;
    float i_b;
    float i_c;

    float i_alpha;
    float i_beta;

    float i_d;
    float i_q;

    float sine;
    float cosine;

    float k_svpwm;
} FOC_t;

void ipark(FOC_t *foc);
void ipark2(FOC_t *foc);
void clarke(FOC_t *foc);
void park(FOC_t *foc);
void svpwm(FOC_t *foc);
#endif // FOC_H