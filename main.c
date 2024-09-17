#include "stdio.h"
#include "string.h"

#define UCHAR unsigned char
#define VOID void
#define FLOAT float
#define INT int
#define IN
#define OUT
#define INOUT

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
};


int main()
{
	BICYCLE_MODEL_S stCar;
	BicycleModel_Init(&stCar);
	printf("hello world\n");
	return 0;
}

