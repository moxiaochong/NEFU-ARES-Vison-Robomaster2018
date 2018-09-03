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
#include "stdafx.h"
#include <opencv2/opencv.hpp>

//#define T_ANGLE_THRE 10 //角度差值
//#define T_ANGLE_THRE180 6 //反向外八角度差值
#define T_SIZE_THRE 5   //



//extern int	brightness;
//extern int	contrast;
//extern int	saturation;
//extern int	hue;
//extern int	exposure;
//extern char key;

//历程
#define AimCenter_X 400 //图像中心x值
#define AimCenter_Y 300 //图像中心y值

#define T_ANGLE_THRE 5   //灯柱正向最大角度差
#define T_ANGLE_THRE180 3   //灯柱反向最大角度差
#define T_ANGLE_THREMIN 3   //灯柱正向最小角度差
#define T_ANGLE_THRE180MIN 2   //灯柱反向最小角度差

#define T_HIGH_RAT 0.2   //灯带最大高差比 灯柱高度差是最大灯柱的1/3
#define T_HIGH_RAT_ANGLE 0.34   //灯带角度符合最小角度高差比 灯柱高度差是最大灯柱的1/2


#define T_WHIDTH_RAT 0.4  //灯带最大宽差比   灯柱宽度差是最大灯柱的3/5
#define T_WHIDTH_RAT_ANGLE 0.55  //灯带最大宽差比

#define L_WH_RAT 0.8 // 灯条宽高比

#define  BallisticCal_Accuracy 0.1               //弹道解析精度 (度)
#define  BallisticCal_BallisticSpd 21.5        //子弹初速度 (m)

#define Yuntai_Y_P  0.003
#define Yuntai_Y_D  0.07

#define Yuntai_P_P  0.001
#define Yuntai_P_D  0.03

#define TeamBlue 0x00
#define TeamRed  0x01


class Armor_Detector
{

public:

	//hsv
	int BLowH = 80;
	int BHighH = 150;

	int BLowS = 40;
	int BHighS = 255;

	int BLowV = 100;
	int BHighV = 255;

	int RLowH = 150;
	int RHighH = 180;
	int RLowH_2 = 0;
	int RHighH_2 = 40;

	int RLowS = 40;
	int RHighS = 255;

	int RLowV = 100;
	int RHighV = 255;

	double solvepnp_x, solvepnp_y, solvepnp_z, Muzzle_compensation;//solvepnp计算的xyz，枪口补偿
	int ourteam;



	int ang_P, ang_Y, Yaw_deviation,lost_canter;
	//预测
	int prve_armor_center_x;
	int prve_armor_center_y;
	int Deviation_prove_now;
	int	Pitch_deviation ;
	int left_follow_time ;
	int left_time_clearn ;
	int right_follow_time ;
	int right_time_clearn ;
	int follow_direction ;
	//类构造函数
	Armor_Detector();
	//摄像头初始化
	cv::VideoCapture capture_m; //视觉辅助摄像头
	bool camaraInit(int device);//视觉辅助摄像头初始化
	//装甲识别
	cv::Mat srcimage;//原图
	cv::Mat dstimage;//hsv提取蓝色图
					 //solvepnp
	std::vector<cv::Point3f> corners;
	std::vector<cv::Point3f> corners_big;
	std::vector<cv::Point2f> observation_points;



	std::vector<cv::RotatedRect> VRlt;
	cv::RotatedRect VRlt_last;
	//历程
	cv::RotatedRect LastTargetArmor;
	
	//云台控制（历程）
	int AutoShoot;
	int Offset_Y;
	int Offset_P;
	double Distance;
	double YunTai_P_coreect = 0;//云台Pich轴矫正量
	double BallisticCalculation(double distance); //弹道计算
	void Yuntai_Control(const cv::RotatedRect& tg_armor, int* offset_y, int* offset_p, bool flag);//云台控制
	void Armor_Detector_Solvepnp(cv::RotatedRect dst);//solvepnp测距
	void Armor_Detector_pretreatment(cv::Mat src, cv::Mat& dst);
	void Armor_rio11_Detector(cv::Mat master, cv::Mat src, std::vector<cv::RotatedRect>& vRlt, cv::RotatedRect& last_vRlt);
	void autoShoot();
	~Armor_Detector();
};


class Armor_builded //装甲构成
{
public:
	cv::RotatedRect armorS;
	int build1_No = 0;
	int build2_No = 0;
	int build_features[4];//角度差，高度坐标差，高度差，宽度差
	int vot = 0;
};
