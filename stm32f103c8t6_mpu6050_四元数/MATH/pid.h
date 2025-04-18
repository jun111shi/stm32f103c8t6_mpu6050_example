#ifndef __PID_H
#define __PID_H

//PID参数结构体
typedef struct{
	float desired;			//期望值
	float kp;           //比例系数
	float ki;           //积分系数
	float kd;           //微分系数
}PID;
extern PID pid;



#endif
