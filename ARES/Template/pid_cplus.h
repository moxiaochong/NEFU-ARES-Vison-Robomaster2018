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

#pragma once
class pid_cplus
{
public:
	//float Yaw_point=0;//Y原点；
	//float Pitch_point = 0;//P原点；
	//计算的偏差量
	/*int ang_P;
	int ang_Y;*/
	//pid参数
	float P;
	float I;
	float D;
	float E;
	float PreE;
	float PrePreE;
	float U;
	float Intergral;
	float Ilimit;
	float Ulimit;
	//积分分离
	float E_max;
	//pid模式选择
	bool Positional_PID = 0;
	bool Incremental_PID = 1;
	//pid函数
	float PID_Control_normal(pid_cplus *pid,float Expected_value, float Actual_value);//, bool Pid_type);
	void PID_Change_Pitch(pid_cplus *pid, float Expected_value, float Actual_value);
	void PID_Change_Yaw(pid_cplus *pid, float Expected_value, float Actual_value);
	void PID_init(pid_cplus *pid,bool PORY);

	pid_cplus();
	~pid_cplus();
};

