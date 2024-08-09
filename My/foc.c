#include "foc.h"

const float Udc = 12;

void ipark(FOC_t *foc)
{
    foc->sine = sin(foc->theta);
    foc->cosine = cos(foc->theta);
    foc->u_alpha = foc->u_d * foc->cosine - foc->u_q * foc->sine;
    foc->u_beta = foc->u_q * foc->cosine + foc->u_d * foc->sine;
}

void ipark2(FOC_t *foc)
{
    foc->u_alpha = foc->u_d * foc->cosine - foc->u_q * foc->sine;
    foc->u_beta = foc->u_q * foc->cosine + foc->u_d * foc->sine;
}

void clarke(FOC_t *foc)
{
    foc->i_alpha = foc->i_a;
    foc->i_beta = (foc->i_a + 2 * foc->i_b) * 0.5773502691896257;
}

void park(FOC_t *foc)
{
    foc->sine = sin(foc->theta);
    foc->cosine = cos(foc->theta);
    foc->i_d = foc->i_alpha * foc->cosine + foc->i_beta * foc->sine;
    foc->i_q = foc->i_beta * foc->cosine - foc->i_alpha * foc->sine;
}

#define UDC 12.0f
void svpwm(FOC_t *foc)
{
    const int ts = 4200;
    const float one_over_sqrt3 = 0.57735026919f;
    const float sqrt3 = 1.73205080757f;

    // 计算Uref1, Uref2, Uref3
    float Uref1 = foc->u_beta;
    float Uref2 = one_over_sqrt3 * foc->u_alpha - 0.5f * foc->u_beta;
    float Uref3 = -one_over_sqrt3 * foc->u_alpha - 0.5f * foc->u_beta;

    // 确定扇区
    uint8_t sector = (Uref1 > 0.0f) + ((Uref2 > 0.0f) << 1) + ((Uref3 > 0.0f) << 2);

    // 计算X, Y, Z
    float X = sqrt3 * ts / UDC * foc->u_beta;
    float Y = sqrt3 * ts / UDC * (foc->u_alpha * one_over_sqrt3 + 0.5f * foc->u_beta);
    float Z = sqrt3 * ts / UDC * (-one_over_sqrt3 * foc->u_alpha + 0.5f * foc->u_beta);

    float t1, t2, t3;
    switch (sector)
    {
    case 1: // 扇区1
        t1 = Z;
        t2 = Y;
        break;
    case 2: // 扇区2
        t1 = Y;
        t2 = -X;
        break;
    case 3: // 扇区3
        t1 = -Z;
        t2 = X;
        break;
    case 4: // 扇区4
        t1 = -X;
        t2 = Z;
        break;
    case 5: // 扇区5
        t1 = X;
        t2 = -Y;
        break;
    case 6: // 扇区6
        t1 = -Y;
        t2 = -Z;
        break;
    default:
        return; // 无效扇区，添加错误处理
    }

    t3 = t1 + t2;
    if (t3 > ts)
    {
        float scale = ts / t3;
        t1 *= scale;
        t2 *= scale;
    }

    uint32_t tcm1 = (ts - t1 - t2) / 4.0f;
    uint32_t tcm2 = tcm1 + t1 / 2.0f;
    uint32_t tcm3 = tcm2 + t2 / 2.0f;

    switch (sector)
    {
    case 1:
        foc->t_a = tcm2;
        foc->t_b = tcm1;
        foc->t_c = tcm3;
        break;
    case 2:
        foc->t_a = tcm1;
        foc->t_b = tcm3;
        foc->t_c = tcm2;
        break;
    case 3:
        foc->t_a = tcm1;
        foc->t_b = tcm2;
        foc->t_c = tcm3;
        break;
    case 4:
        foc->t_a = tcm3;
        foc->t_b = tcm2;
        foc->t_c = tcm1;
        break;
    case 5:
        foc->t_a = tcm3;
        foc->t_b = tcm1;
        foc->t_c = tcm2;
        break;
    case 6:
        foc->t_a = tcm2;
        foc->t_b = tcm3;
        foc->t_c = tcm1;
        break;
    }
}
