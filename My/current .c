#include "current .h"

int16_t adc_buff[4] = {0};

void adc_init(ADC_cur_t *adc)
{
    adc->adc_refer = 1.65;

    adc->Iu = 0.0;
    adc->Iv = 0.0;
    adc->Iw = 0.0;
    adc->Udc = 0.0;
    adc->adc_stand=0;
    adc->cnt=0;
    adc->Iu_sum_offer=0;
    adc->Iv_sum_offer=0;
    adc->Iw_sum_offer=0;
}


// void Adcpro(Motor_t *motor, Current_t *current,  uint16_t *Iraw)
// {
    // motor->adc_u->Iu_Sample = -Iraw[0]; // 反向，一般电流取流入电机方向为正
    // motor->adc_u->Iv_Sample = -Iraw[1];
    // motor->adc_u->Iw_Sample = -Iraw[2];
    // // ADC系数转化
    // motor->adc_u->Iu = ((float)motor->adc_u->Iu_Sample * 3.3f / 4096 - motor->adc_u->adc_refer) * motor->adc_u->gain_cur; // 100/16.5
    // motor->adc_u->Iv = ((float)motor->adc_u->Iv_Sample * 3.3f / 4096 - motor->adc_u->adc_refer) * motor->adc_u->gain_cur;
    // motor->adc_u->Iw = ((float)motor->adc_u->Iw_Sample * 3.3f / 4096 - motor->adc_u->adc_refer) * motor->adc_u->gain_cur;
    // // adcvalue->Udc = ((float)Iraw[3]*3.3f/4096.0f-adcvalue->adc_refer)*adcvalue->gain_udc;
    // motor->adc_u->Udc = ((float)Iraw[3] * 3.3f / 4096.0f - 0) * motor->adc_u->gain_udc; // 25.0
//     // 母线异常判断
//     if ((motor->adc_u->Udc > 12.0f - 5.0f) && (motor->adc_u->Udc < 12.0f + 5.0f)) // 母线电压正常
//     {

//        // motor->ERR_UDC = 0; // 电压警告标志

//         if (motor->adc_u->adcstateflag < 2) // 上电后adc电流校准未开启
//         {
//             // 电流校准零偏补偿
//             if (motor->adc_u->ADCcount < 5000) // 5000次=0.5s
//             {
//                 motor->adc_u->adcstateflag = 1;
//                 motor->adc_u->ADCcount++;
//                 motor->adc_u->Iu_Sample_offer = motor->adc_u->Iu_Sample_offer * 0.998f + (float)motor->adc_u->Iu * 0.002f;
//                 motor->adc_u->Iv_Sample_offer = motor->adc_u->Iv_Sample_offer * 0.998f + (float)motor->adc_u->Iv * 0.002f;
//                 motor->adc_u->Iw_Sample_offer = motor->adc_u->Iw_Sample_offer * 0.998f + (float)motor->adc_u->Iw * 0.002f;
//             }
//             else
//             {
//                 motor->adc_u->adcstateflag = 2; // 校准完成
//                 motor->adc_u->ADCcount = 0;
//             }
//         }

//         if (motor->adc_u->adcstateflag == 2) // adc电流校准完成
//         {
//             current->Ia = (motor->adc_u->Iu - motor->adc_u->Iu_Sample_offer);
//             current->Ib = (motor->adc_u->Iv - motor->adc_u->Iv_Sample_offer);
//             current->Ic = (motor->adc_u->Iw - motor->adc_u->Iw_Sample_offer);
//            // motor->Motor_EnableFlag = 1; // 母线电流正常，电机使能位打开

//             // 判断是否过流保护
//             if ((((current->Ia > 4.0f) || (current->Ia < -4.0f)) || ((current->Ib > 4.0f) || (current->Ib < -4.0f)) || ((current->Ic > 4.0f) || (current->Ic < -4.0f))))
//             {
//              //   motor->ERR_OVERCURRENT = 1;
//             }
//             // if (fabs(motor->adc_u->currentoffera - 1.0f) < 0.5f && fabs(motor->adc_u->currentofferb - 1.0f) < 0.5f && fabs(motor->adc_u->currentofferc - 1.0f) < 0.5f && motor->adc_u->currentofferEN == 1)
//             // {
//             //     current->Ia = current->Ia * motor->adc_u->currentoffera; // 软件校正
//             //     current->Ib = current->Ib * motor->adc_u->currentofferb;
//             //     current->Ic = current->Ic * motor->adc_u->currentofferc;
//             // }
//         }
//         else
//         {
//             current->Ia = motor->adc_u->Iu;
//             current->Ib = motor->adc_u->Iv;
//             current->Ic = motor->adc_u->Iw;
//         }
//     }
//     else // 母线电压异常
//     {
//         current->Ia = motor->adc_u->Iu;
//         current->Ib = motor->adc_u->Iv;
//         current->Ic = motor->adc_u->Iw;
//         motor->adc_u->ADCcount = 0;
//         motor->adc_u->adcstateflag = 0;
//      //   motor->Motor_EnableFlag = 0;
//        // motor->ERR_UDC = 1;
//     }
// }