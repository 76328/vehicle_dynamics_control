#include "stdio.h"
#include "string.h"

#define UCHAR unsigned char
#define VOID void
#define FLOAT float
#define INT int
#define IN
#define OUT
#define INOUT

#define DT (0.01)


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
} BICYCLE_MODEL_S;


VOID BicycleModel_Init(INOUT BICYCLE_MODEL_S *pstM)
{
	CHECK_PTR(pstM);
	memset(pstM,0,sizeof(BICYCLE_MODEL_S));
	pstM->m = 500;
	pstM->a = 1;
	pstM->b = 1;
	pstM->I = 166;
	return;
}

VOID BicycleModel_CalcNextState(INOUT BICYCLE_MODEL_S *pstM)
{
	CHECK_PTR(pstM);
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

}

int main()
{
	BICYCLE_MODEL_S stCar;
	BicycleModel_Init(&stCar);
	printf("hello world v2\n");
	return 0;
}

