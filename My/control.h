#ifndef CONTROL_H
#define CONTROL_H

//电流环
typedef struct __CURRENTSTRUCT
{
	float Kp;  //电流环比例系数
	float Ki;  //电流环积分系数    (10000*ki)/s
	float Iqr; //q轴电流给定
	float Iq;  //q轴电流反馈
	float Idr; //d轴电流给定
	float Id;	 //d轴电流反馈
	float e_id;//d轴电流误差
	float x_id;//d轴电流积分
	float e_iq;//q轴电流误差
	float x_iq;//q轴电流积分
	float Udbuf;//d轴PI输出
	float Uqbuf;//q轴PI输出
	float Ud;   //d轴电压输出(限幅后)
	float Uq;		//q轴电压输出(限幅后)
	float Currentout_MAX;  //电流环输出最大值，即给定电压最大值
	
	
	float Ia; 
	float Ib; 
	float Ic; 
	float Ialfa;
	float Ibeta;
	

	
	
}Current_t;



#endif