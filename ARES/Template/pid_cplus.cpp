/******************** 声明 ************************/
/*
这是东北林业大学ARES机器人战队在Robomaster2018赛季关于计算机视觉部分
的完整代码，包含大小能量机关和视觉辅助瞄准的全部内容，应用在步兵机器人上。
（注：英雄机器人与哨兵机器人参考视觉辅助部分的代码，它们与步兵机器人在
这一块是共用的）。虽然我们的成果与一些强队相比还有很大的差距，但是我们
希望在某些技术点上我们的解决思路能给其他参赛队伍（特别是新参赛队伍）提供
一定的借鉴，哪怕只有一点，那么我们的工作也是值得的。最后希望我们的成果能
帮助到大家，祝愿Robomaster越办越红火！--- 来自一个为了机器人单身26年又即将
离开校园的老人！
*/

//时间：2018.09.02
//地点：东北林业大学成栋楼大学生科技创新实验室
//作者：刘祖基、莫冲、侯弘毅、赖建伟、李士尧等

#include "stdafx.h"
#include "pid_cplus.h"
#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <iostream> 
#include "Armor_Detector.h"
#include "SerialPort.h"  
#include "pid_cplus.h"
#include "omp.h"

extern Armor_Detector armor;


float pid_cplus::PID_Control_normal(pid_cplus *pid, float Expected_value, float Actual_value)//, bool Pid_type)
{

	/*float point_value = 0;
	if (Actual_value<-20)
	{
		point_value = 130;
	}
	else if (Actual_value > 20)
	{
		point_value = -130;
	}
	pid->E = point_value - Actual_value;*/
	pid->E = Expected_value - Actual_value;

	//if (Pid_type)//增量式
	//{

	//	U += P * (E - PreE) +
	//		I * (E) +
	//		D * (E - 2 * PreE + PrePreE);
	//}
	//else//位置式
	//{
	pid->Intergral += pid->E;
	if (pid->Intergral >= pid->Ilimit)
		pid->Intergral = pid->Ilimit;
	else if (pid->Intergral <= -pid->Ilimit)
		pid->Intergral = -pid->Ilimit;
	else
		pid->Intergral = pid->Intergral;
	pid->U = pid->P * pid->E +
		//pid->I * pid->Intergral +
		pid->D * (pid->E - pid->PreE);
	//}


	if (pid->U >= pid->Ulimit)
		pid->U = pid->Ulimit;
	else if (pid->U <= -pid->Ulimit)
		pid->U = -pid->Ulimit;
	else
		pid->U = pid->U;

	pid->PrePreE = pid->PreE;
	pid->PreE = pid->E;

	return pid->U;

}


void pid_cplus::PID_Change_Pitch(pid_cplus *pid, float Expected_value, float Actual_value)
{
	pid->E = Expected_value - Actual_value;
	if (abs(pid->E) > 300)
	{
		pid->P = (pid->E*pid->E / 8000000) + 0.030;
	}
	if (abs(pid->E) > 200)
	{
		pid->P = (pid->E*pid->E / 4000000) + 0.030;
	}
	else if (abs(pid->E) > 100)
	{
		pid->P = (pid->E*pid->E / 2500000) + 0.035;
	}
	else
	{
		pid->P = (pid->E*pid->E / 1500000) + 0.0455;
	}
	//printf("E:%f\n", pid->E);
}



void pid_cplus::PID_Change_Yaw(pid_cplus *pid, float Expected_value, float Actual_value)
{
	/*float point_value = 0;
	if (Actual_value<-20)
	{
	point_value = 130;
	}
	else if (Actual_value > 20)
	{
	point_value = -130;
	}
	pid->E = point_value - Actual_value;*/
	/*if (armor.Offset_Y_error>5)
	{
	Expected_value += armor.VRlt_last.size.width;
	}*/
	pid->E = Expected_value - Actual_value;

	if (abs(pid->E) > 500)
	{
		pid->P = (pid->E*pid->E / 8000000) + 0.01;
	}
	else if (abs(pid->E) > 400)
	{
		pid->P = (pid->E*pid->E / 7000000) + 0.01;
	}
	else if (abs(pid->E) > 300)
	{
		pid->P = (pid->E*pid->E / 5000000) + 0.035;
	}
	else if (abs(pid->E) > 200)
	{
		pid->P = (pid->E*pid->E / 3500000) + 0.04;
	}
	else if (abs(pid->E) > 100)
	{
		pid->P = (pid->E*pid->E / 2500000) + 0.045;
	}
	else
	{
		pid->P = (pid->E*pid->E / 1500000) + 0.045;
	}
	//printf("E——Y:%f\n", pid->E);
}

void pid_cplus::PID_init(pid_cplus *pid, bool PORY)
{
	if (PORY)//P
	{
		pid->P = 1;
		pid->I = 0;
		pid->D = 0.06;
		pid->E = 0;
		pid->PreE = 0;
		pid->PrePreE = 0;
		pid->U = 0;
		pid->Intergral = 200;
		pid->Ilimit = 0;
		pid->Ulimit = 100;
	}
	else//Y
	{
		pid->P = 1;
		pid->I = 0;
		pid->D = 0.06;
		pid->E = 0;//偏差
		pid->PreE = 0;//上一次偏差
		pid->PrePreE = 0;//上上次偏差
		pid->U = 0;//最终偏差
		pid->Intergral = 200;//积分偏差
		pid->Ilimit = 0;//积分限幅
		pid->Ulimit = 50;//输出限幅

	}


}

pid_cplus::pid_cplus()
{
}


pid_cplus::~pid_cplus()
{
}
