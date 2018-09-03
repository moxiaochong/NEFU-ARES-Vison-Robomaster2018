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

#include<opencv2/opencv.hpp>
#include"SerialPort.h"
#include <Eigen\Dense>
struct myRect
{
	cv::Rect boundRect;
	float similarValue;
};

struct mypswRect
{
	cv::Rect boundRect;
	std::vector<cv::Point> contours;
	int value;
	float similarValue;
};

struct myResult
{
	int value; //结果值
	int Index;//位置索引
	cv::Point position; //位置
};

struct myPoint
{
	cv::Point pt;
	int sumXY;
	int sumXY_inv;
};
							 
enum symbolModel
{
	numArab,
	numFire
};


class sbol
{
public:
	cv::Mat R, T;
	//solve pnp
	cv::Mat K_;
	cv::Mat D_;
	std::vector<cv::Point3f> corners;
	double Depth;
	double angleY;
	double angleP;
	double corectAngY;
	double corectAngP;

	//变换矩阵
	Eigen::Matrix4d cam_shot;

	//大小符模式
	symbolModel SymbolModel;

	const int trainSample = 1;				  
	const int trainClasses = 9;
	const int sizeX = 24;
	const int sizeY = 36;
	const int featureLen = sizeX * sizeY;
	cv::Mat train_data, train_classes;
	//识别手写数字系统
	cv::Ptr<cv::ml::KNearest> knn;
	cv::Ptr<cv::ml::SVM> svm;
	cv::Ptr<cv::ml::SVM> svm2;

	cv::VideoCapture Capture_BigPower;//摄像头

	cv::Mat sbolImage;//原图像
	cv::Mat preImage;//预处理图像
	
	cv::Rect digitronLightRect;//数码管所在区域
	std::vector<cv::Point2f> observationPoints;
	std::vector<cv::RotatedRect> calibrateRects; //数字所在区域
	std::vector<cv::Rect> calFireRects; //数字所在区域
	cv::Rect rectShow;

	std::vector<myResult> vrecResult; //识别结果
	std::vector<myResult> vlstrecResult; //上一次识别结果

	int meanGrayValue;

	//重置打击顺序
	int psWord[5]; //当前密码
	int lstPsWord[5]; //上一次的密码
	int hitCnt;
	int observeCnt; //观察计数
	int errCnt;
	bool shootFlag;
	int shootCnt;
	int shootCnt1;
	//识别换页
	//bool observeFlag;
	bool pChangeFlag;
	int stopChange_cnt;
	int ArrayResult[9];
	int ArrayResult1[9];
	int ArrayResult2[9];
	int ArrayResult3[9];
	//目标选择
	myResult tarGet;

	
	sbol();//系统初始化构造函数
	Eigen::Matrix4d EigenFrom3(Eigen::Vector3d transl, Eigen::Matrix3d Rmat);
	void sampleInit(cv::Mat& trainData, cv::Mat& trainClass);//KNN训练样本装填
	void trainpassword();//训练密码识别器
	void loadSVM();//加载SVM
	bool camaraInit(int device);//大符摄像头初始化
	void imgPretreatment(const cv::Mat& src, cv::Mat& dst); //图像预处理
	bool findDigitronLightRoi(const cv::Mat& src, cv::Rect& roiRect);//寻找数码管所在区域
	bool recPassWord(const cv::Mat& src, cv::Rect& dlRect, int* resultArray,std::vector<cv::Point2f>& observationPts,cv::Mat& r,cv::Mat& t,double& depth);//识别密码
	bool calGirdRect(const cv::Mat& src, std::vector<cv::Point2f>& pts,std::vector<cv::RotatedRect>& gridRect);//锁定手写数字识别区
	void EliminateEdge(cv::Mat &imgSrc, cv::Mat &dst); //消除边缘
	bool calFireRect(const cv::Mat& src, std::vector<cv::Point2f>& pts, std::vector<cv::Rect>& fireRect);//锁定数字识别区域
	void numRecARab(const cv::Mat& src, std::vector<cv::RotatedRect>& numRects, std::vector<myResult>& vResult);//手写数字识别
	void numRecFire(const cv::Mat& src, std::vector<cv::Rect>& numRects, std::vector<myResult>& vResult);
	void resetShootCnt(int* ps_array, int* ls_ps_array, int& count, int& err_count, int& cnt, bool& flag);//重置打击顺序
	//bool recPageChange(std::vector<myResult>& vResult, std::vector<myResult>& vlsResult, bool& flag); //换页识别
	bool recPageChange();
	void chooseTarget(std::vector<myResult>& vResult, int psValue, myResult& target);
	//计算云台角度
	cv::Point2f get_pattern_shot_3D(cv::Matx33d K, cv::Point2f target, Eigen::Matrix4d cam_shot, double depth);
	cv::Point2f calAngle(int Index, cv::Mat& r, cv::Mat& t);
	void BigPower();//主函数调用函数
	void debugShow();

	//=========== 大符 ==============//

	//端点映射
	void point_To_point(std::vector<cv::Point2f>& src, std::vector<cv::Point>& dst);

};



#pragma once
