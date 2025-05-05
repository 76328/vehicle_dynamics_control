/*
 * @file    文件名:main.c
 * @brief   文件简要说明:VDC
 * @details 文件详细描述:车辆动力学建模与控制
 * 
 * @author  作者名:weifan.wang
 * @email   作者邮箱:763280032@qq.com
 * @date    创建日期:25-5-1
 * @version 版本号:na
 */

#include "stdio.h"
#include "string.h"
#include <unistd.h>
#include <math.h>
#include "controller.h"

#define UCHAR unsigned char
#define VOID void
#define FLOAT float
#define INT int
#define IN
#define OUT
#define INOUT

#define DT (0.001)
#define G (9.18)
#define U (1.0)

#define CHECK_PTR(x,...) 		\
do{					\
if(x == NULL){ 				\
printf(#x" is null\n");			\
return __VA_ARGS__; 			\
} 					\
} while(0) 
	
typedef struct BICYCLE_MODEL_TAG
{
	float m;
	float a;
	float b;
	float I;
	/*variable*/
	float X_abs;
	float dX_abs;
	float Y_abs;
	float dY_abs;
	float x; /*local lateral*/
	float dx;
	float ddx;
	float y; /*local longitudinal*/
	float dy;
	float ddy;
	float psi; /*heading angle */
	float dpsi;
	float ddpsi;
	float F_xf;
	float F_xr;
	float F_yf;
	float F_yr;
	/*wheel variable*/
	float v_lf;
	float v_cf;
	float v_xf;
	float v_yf;
	float v_lr;
	float v_cr;
	float v_xr;
	float v_yr;
	float delta_f;
	float delta_r;
	float F_lr;
	float F_cr;
	float F_lf;
	float F_cf;
	/*slip angle*/
	float alpha_f;
	float alpha_r;
	/*slip ratio*/
	float s_f;
	float s_r;
	/*wheel rotate speed*/
	float w_f;
	float w_r;
	float r;/*wheel radius*/
	/*vertical load*/
	float F_zf;
	float F_zr;
	/*time*/
	int t;
	/*param*/


} BICYCLE_MODEL_S;

#define ZB 0.0000001

VOID BicycleModel_Init(INOUT BICYCLE_MODEL_S *pstM)
{
	CHECK_PTR(pstM);
	memset(pstM,0,sizeof(BICYCLE_MODEL_S));
	pstM->m = 500;
	pstM->a = 1;
	pstM->b = 1;
	pstM->I = 166;

	pstM->r = 0.3;
	pstM->F_zf = (pstM->b*pstM->m*G)/(2*(pstM->a+pstM->b));
	pstM->F_zr = (pstM->a*pstM->m*G)/(2*(pstM->a+pstM->b));

	return;
}

float BicycleModel_CalcSlipRatio(float r,float w,float vl)
{
	float result = 0;
	if ((r*w)<vl)
	{
		/*breaking*/
		if (vl != 0)
		{
			result = (r*w/vl)-1;
		}
		else
		{
			result = (r*w/(vl+ZB))-1;
		}
	}
	else
	{
		if (w != 0)
		{
			result = 1-(vl/(r*w));
		}
		else
		{
			result = 1-(vl/(r*(w+ZB)));
		}

	}
	return result;
}

float BicycleModel_CalcLongitudinalForce(float slpr,float u,float Fz)
{
	float a1 = -21.3;
	float a2 = 1144;
	float a3 = 49.6;
	float a4 = 226;
	float a5 = 0.69;
	float a6 = -0.006;
	float a7 = 0.056;
	float a8 = 0.486;
	float Fzk = Fz * 0.001;
	float D = a1 * pow(Fzk,2) + a2 * Fzk;
	float C = 1.65;
	float B = (a3 * pow(Fzk,2) +a4*Fzk)/(C*D*exp(a5*Fzk)); 
	float E = a6 * pow(Fzk,2) +a7*Fzk +a8;

	float phi = (1-E)*slpr+(E/B)*atan(B*slpr);
	float result = D * sin(C*atan(B*phi));

	return result;
}


float BicycleModel_CalcLateralForce(float slpa,float u,float Fz)
{
	float Fzk = Fz*0.001;
	float a1 = -22.1;
	float a2 = 1011;
	float a3 = 1078;
	float a4 = 1.82;
	float a5 = 0.208;
	float a6 = 0;
	float a7 = -0.354;
	float a8 = 0.707;
	float a9 = 0.028;
	float a10 = 0;
	float a11 = 14.8;
	float a12 = 0.022;
	float a13 = 0;
	float Camber = 0.0;
	float C = 1.30;
	float D = a1*pow(Fzk,2)+a2*Fzk;
	float Dsh = a9*Camber;
	float Dsv = (a10*pow(Fzk,2)+a11*Fzk)*Camber;
	float E = a6*pow(Fzk,2)+a7*Fzk+a8;
	float B = ((a3*sin(a4*atan(a5*Fzk)))/(C*D))*(1-a12*fabs(Camber));
	float phi = (1-E)*(slpa+Dsh)+(E/B)*atan(B*(slpa+Dsh));

	float result = D*sin(C*atan(B*phi))+Dsv;
	return -result;//与仿真计算里的论文图像相反
}


float BicycleModel_CalcSlipAngle(float vc,float vl)
{
	float result = 0;
	float angle = 0;
	if(vl != 0)
	{
		result =atan2f(vc,vl);
		/*防止跳变*/
		angle=fmod(result+3.141,2*3.141);
		if(angle<0)
		{
			angle=angle+2*3.141;
		}
		result=angle-3.141;
	}
	else
	{
		//result=atan(vc/(vl+ZB));
	}
	return result;
}

#define PRT_FIELD_FLOAT(st,field) printf("%-1s:%.4f ",#field, st->field)

VOID BicycleModel_PrintState(IN BICYCLE_MODEL_S *pstM,int iter)
{
	CHECK_PTR(pstM);
	if(0!=(iter%10))
	{
		return;
	}
	printf("time: %.2f s ",iter*DT);
	PRT_FIELD_FLOAT(pstM,X_abs);
	PRT_FIELD_FLOAT(pstM,Y_abs);
	PRT_FIELD_FLOAT(pstM,F_yf);
	PRT_FIELD_FLOAT(pstM,F_yr);
	PRT_FIELD_FLOAT(pstM,delta_f);
	PRT_FIELD_FLOAT(pstM,psi);
	//PRT_FIELD_FLOAT(pstM,dpsi);
	//PRT_FIELD_FLOAT(pstM,alpha_f);
	//PRT_FIELD_FLOAT(pstM,s_f);
	//PRT_FIELD_FLOAT(pstM,F_lf);
	//PRT_FIELD_FLOAT(pstM,v_cf);
	//PRT_FIELD_FLOAT(pstM,v_lf);
	printf("\n");
}

VOID BicycleModel_CalcNextState(INOUT BICYCLE_MODEL_S *pstM)
{
	CHECK_PTR(pstM);
	/*slip angle*/
	pstM->alpha_f = BicycleModel_CalcSlipAngle(pstM->v_cf,pstM->v_lf);
	pstM->alpha_r = BicycleModel_CalcSlipAngle(pstM->v_cr,pstM->v_lr);
	pstM->s_f = BicycleModel_CalcSlipRatio(pstM->r,pstM->w_f,pstM->v_lf);
	pstM->s_r = BicycleModel_CalcSlipRatio(pstM->r,pstM->w_r,pstM->v_lr);
	pstM->F_lf = BicycleModel_CalcLongitudinalForce(pstM->s_f,U,pstM->F_zf);
	pstM->F_lr = BicycleModel_CalcLongitudinalForce(pstM->s_r,U,pstM->F_zr);
	pstM->F_cf = BicycleModel_CalcLateralForce(pstM->alpha_f,U,pstM->F_zf);
	pstM->F_cr = BicycleModel_CalcLateralForce(pstM->alpha_r,U,pstM->F_zr);

	pstM->F_yf = pstM->F_lf*sin(pstM->delta_f)+pstM->F_cf*cos(pstM->delta_f);
	pstM->F_xf = pstM->F_lf*cos(pstM->delta_f)-pstM->F_cf*sin(pstM->delta_f);

	pstM->F_yr = pstM->F_lr*sin(pstM->delta_r)+pstM->F_cr*cos(pstM->delta_r);
	pstM->F_xr = pstM->F_lr*cos(pstM->delta_r)-pstM->F_cr*sin(pstM->delta_r);

	/*wheel dynamics*/
	pstM->v_yf = pstM->dy + pstM->a * pstM->dpsi;
	pstM->v_yr = pstM->dy - pstM->b * pstM->dpsi;
	pstM->v_xf = pstM->dx;
	pstM->v_xr = pstM->dx;

	pstM->v_lf = pstM->v_yf*sin(pstM->delta_f) + pstM->v_xf*cos(pstM->delta_f);
	pstM->v_lr = pstM->v_yr*sin(pstM->delta_r) + pstM->v_xr*cos(pstM->delta_r);
	pstM->v_cf = pstM->v_yf*cos(pstM->delta_f) - pstM->v_xf*sin(pstM->delta_f);
	pstM->v_cr = pstM->v_yr*cos(pstM->delta_r) - pstM->v_xr*sin(pstM->delta_r);

	/*car dynamics*/
	pstM->x = pstM->x + DT * pstM->dx;
	pstM->dx = pstM->dx + DT * pstM->ddx;
	
	pstM->y = pstM->y + DT * pstM->dy;
	pstM->dy = pstM->dy + DT * pstM->ddy;

	pstM->psi = pstM->psi + DT * pstM->dpsi;
	pstM->dpsi = pstM->dpsi + DT * pstM->ddpsi;

	pstM->ddx = (pstM->m * pstM->dy * pstM->dpsi + 2 * pstM->F_xf + 2 * pstM->F_xr)/pstM->m;
	pstM->ddy = (-1 * pstM->m * pstM->dx * pstM->dpsi + 2 *pstM->F_yf + 2 * pstM->F_yr)/pstM->m;
	pstM->ddpsi = (2 * pstM->a * pstM->F_yf - 2 * pstM->b * pstM->F_yr)/pstM->I;	

	/*abs cordinate*/
	pstM->X_abs = pstM->X_abs + DT * pstM->dX_abs;
	pstM->Y_abs = pstM->Y_abs + DT * pstM->dY_abs;
	pstM->dX_abs = pstM->dx * cos(pstM->psi) - pstM->dy * sin(pstM->psi);
	pstM->dY_abs = pstM->dx * sin(pstM->psi) + pstM->dy * cos(pstM->psi);
}

int main()
{
	int iter = 0;
	BICYCLE_MODEL_S stCar;
	BicycleModel_Init(&stCar);
	printf("hello world v3\n");
	while(iter < 30000)
	{
		/*Accelerate to 54 km/h in 5 seconds*/
		if(iter < 5000)
		{
			stCar.w_f = stCar.w_f+0.01;
			stCar.w_r = stCar.w_f;
		}
		/*lateral controller（maintain Y_abs=2m）,step response*/
		stCar.delta_f = Controller_Delta(2.0,stCar.Y_abs,stCar.psi);
		/*calc next state*/
		BicycleModel_CalcNextState(&stCar);
		/*print vehicle state*/
		BicycleModel_PrintState(&stCar,iter);
		usleep(1000);
		iter++;
	}
	return 0;
}

