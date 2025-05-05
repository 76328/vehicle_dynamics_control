/*
 * @file    文件名:controller.c
 * @brief   文件简要说明:
 * @details 文件详细描述:车辆横向控制器
 * 
 * @author  作者名:weifan.wang
 * @email   作者邮箱:763280032@qq.com
 * @date    创建日期:25-5-1
 * @version 版本号:na
 */
#include "controller.h"
#include "stdio.h"

float Controller_Delta(float target,float feedback, float psi)
{
    float err=0;
    static float err_1=0;
    float kp = 2.0;
    float kd = 2.5;
    float ki = 0.1;
    float output = 0;

    float err_psi = 0;
    float kp_psi = 5.0;
    float kd_psi = 2.5;
    
    static float err_psi_1 =0.0;

    err = target - feedback;
    output = kp*(err)+kd*(err-err_1);

    if (output>0.2)
    {
        output=0.2;
    }
    
    if(output<-0.2)
    {
        output=-0.2;
    }

    err_psi = output - psi;

    output = kp_psi*(err_psi)+kd_psi*(err_psi-err_psi_1);

    err_psi_1 = err_psi;
    err_1 = err;
    if (output>0.5)
    {
        output=0.5;
    }
    
    if(output<-0.5)
    {
        output=-0.5;
    }
    //printf("output %f\n",output);
    return output;
}