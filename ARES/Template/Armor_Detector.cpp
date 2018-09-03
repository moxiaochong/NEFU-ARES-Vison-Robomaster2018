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
#include "Armor_Detector.h"
#include <opencv2/opencv.hpp>
#include <iostream> 
#include "SerialPort.h"  
#include "pid_cplus.h"
#include "omp.h"
using namespace cv;
using namespace std;


#define Pitcure_centring_X 598 //图像中心点X
#define Pitcure_centring_Y 473 //图像中心点Y
//solve pnp
double K[3][3] =
{
	2050.40525, 0, 485.13985,
	0, 2050.40525, 420.20901,
	0, 0, 1,
};
double D[1][5] =
{
	-0.32349, -4.82262,-0.00825,-0.00334,0.0,
};

cv::Mat K_(3, 3, CV_64FC1, K);
cv::Mat D_(1, 5, CV_64FC1, D);


//char key;
//int	brightness = 10.00;
//int	contrast = 100.00;
//int	saturation = 60.00;
//int	hue = 50.00;
//int	exposure = 7.00;

const double PI = 3.1415926;

//装甲点(已知道)
float cornersaa[6][2] = {
	0.0,0.0,
	0.0,0.0275,
	0.0,0.055,
	0.13,0.0,
	0.13,0.0275,
	0.13,0.055,
};
float cornersaa_big[6][2] = {
	0.0,0.0,
	0.0,0.0275,
	0.0,0.055,
	0.225,0.0,
	0.225,0.0275,
	0.225,0.055,
};

pid_cplus Pitch_pid;
pid_cplus Yaw_pid;

//比较函数
bool cmpx(cv::Point2f& s1, cv::Point2f& s2)
{
	return s1.x < s2.x;
}
//类构造函数
Armor_Detector::Armor_Detector()
{
	//初始化PID参数
	Pitch_pid.PID_init(&Pitch_pid, true);
	Yaw_pid.PID_init(&Yaw_pid, false);

	//初始化装甲点世界坐标
	for (int i = 0; i < 6; i++)
	{
		Point3f tmp;
		tmp.x = cornersaa[i][0];
		tmp.y = cornersaa[i][1];
		tmp.z = 0;
		corners.push_back(tmp);
	}
	for (int i = 0; i < 6; i++)
	{
		Point3f tmpp;
		tmpp.x = cornersaa_big[i][0];
		tmpp.y = cornersaa_big[i][1];
		tmpp.z = 0;
		corners_big.push_back(tmpp);
	}
	ang_P = 0;
	ang_Y = 0;
}

//摄像头初始化
bool Armor_Detector::camaraInit(int device)
{

	capture_m.open(device);
	if (!capture_m.isOpened())
	{
		printf("主摄像头打开失败！\n");
		return false;
	}
	else
	{
		//设置副摄像头分辨率
		capture_m.set(CV_CAP_PROP_EXPOSURE, (-9));//曝光 50  -7
		capture_m.set(CV_CAP_PROP_FRAME_WIDTH, 960);
		capture_m.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
		return true;
	}
	//if (key == 'a')
	//{
	//	capture_m.set(CV_CAP_PROP_FPS, 60);//帧数  30
	//	capture_m.set(CV_CAP_PROP_BRIGHTNESS, (-brightness));//亮度 1  -10
	//	capture_m.set(CV_CAP_PROP_CONTRAST, contrast);//对比度 40  100
	//	capture_m.set(CV_CAP_PROP_SATURATION, saturation);//饱和度 50  60
	//	capture_m.set(CV_CAP_PROP_HUE, hue);//色调 50   0
	//	capture_m.set(CV_CAP_PROP_EXPOSURE, (-exposure));//曝光 50  -7
	//} 
}

void Armor_Detector::Armor_Detector_Solvepnp(cv::RotatedRect dst)//solvepnp测距
{

	Point2f ptt[4];
	dst.points(ptt); //计算二维盒子顶点 
	vector<Point2f> aa;
	for (size_t tt = 0; tt < 4; tt++)
	{
		aa.push_back(ptt[tt]);

	}
	sort(aa.begin(), aa.end(), cmpx);
	Point2f mid;
	observation_points.clear();
	if (aa[0].y < aa[1].y)
	{
		observation_points.push_back(aa[0]);

		mid.x = (aa[0].x + aa[1].x) / 2;
		mid.y = (aa[0].y + aa[1].y) / 2;
		observation_points.push_back(mid);

		observation_points.push_back(aa[1]);
	}
	else
	{
		observation_points.push_back(aa[1]);

		mid.x = (aa[0].x + aa[1].x) / 2;
		mid.y = (aa[0].y + aa[1].y) / 2;
		observation_points.push_back(mid);

		observation_points.push_back(aa[0]);
	}

	if (aa[2].y < aa[3].y)
	{
		observation_points.push_back(aa[2]);

		mid.x = (aa[2].x + aa[3].x) / 2;
		mid.y = (aa[2].y + aa[3].y) / 2;
		observation_points.push_back(mid);

		observation_points.push_back(aa[3]);
	}
	else
	{
		observation_points.push_back(aa[3]);

		mid.x = (aa[2].x + aa[3].x) / 2;
		mid.y = (aa[2].y + aa[3].y) / 2;
		observation_points.push_back(mid);

		observation_points.push_back(aa[2]);
	}

	if ((dst.size.width / dst.size.height) > 4)//大装甲
	{
		cv::Mat rvec, tvec;
		cv::solvePnP(cv::Mat(corners_big), cv::Mat(observation_points), K_, D_, rvec, tvec, false);
		cout << tvec << endl;
	}
	else//小装甲
	{
		cv::Mat rvec, tvec;
		cv::solvePnP(cv::Mat(corners), cv::Mat(observation_points), K_, D_, rvec, tvec, false);
		//xx = 1000 * tvec.at<double>(0, 0);//1000精度为毫米
		//yy = 1000 * tvec.at<double>(1, 0);
		solvepnp_z = 10 * tvec.at<double>(2, 0);
		//cout << "X:" << xx << endl << endl;
		//cout << "Y:" << yy << endl << endl;
		cout << "Z:" << solvepnp_z << endl << endl;
		if (solvepnp_z > 2)
		{
			Muzzle_compensation = 5 * solvepnp_z * solvepnp_z / 484;//击打高度补偿（米）
																	 //cout << "Muzzle_compensation:" << Muzzle_compensation << endl << endl;
		}
	}
}


void Armor_Detector::Armor_Detector_pretreatment(cv::Mat src, cv::Mat& dst)//预处理
{

	Mat imgHSV, HSV_RIO;// , channels_HSV_out;
	Mat image, hist_out;
	vector<Mat> channels_HSV;
	//channels.clear();
	channels_HSV.clear();
	dst = 0;
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	//hsv
	cvtColor(src, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	split(imgHSV, channels_HSV);
	if (ourteam)
	{
		threshold(channels_HSV[2], channels_HSV[2], 200, 255, THRESH_BINARY);//最好设为动态阈值
		morphologyEx(channels_HSV[2], channels_HSV[2], MORPH_DILATE, element);
		//imshow("V", channels_HSV[2]);
		imgHSV.copyTo(image, channels_HSV[2]);
		inRange(image, Scalar(BLowH, BLowS, BLowV), Scalar(BHighH, BHighS, BHighV), dst); //Threshold the image返回的是二值化图像
	}
	else
	{
		threshold(channels_HSV[2], channels_HSV[2], 150, 255, THRESH_BINARY);//最好设为动态阈值
		morphologyEx(channels_HSV[2], channels_HSV[2], MORPH_DILATE, element);
		//imshow("V", channels_HSV[2]);
		imgHSV.copyTo(image, channels_HSV[2]);
		inRange(image, Scalar(RLowH, RLowS, RLowV), Scalar(RHighH, RHighS, RHighV), hist_out); //Threshold the image返回的是二值化图像
		inRange(image, Scalar(RLowH_2, RLowS, RLowV), Scalar(RHighH_2, RHighS, RHighV), HSV_RIO);
		dst = hist_out | HSV_RIO;

	}
	//imshow("HSV", dst);
	morphologyEx(dst, dst, MORPH_DILATE, element);
	/*morphologyEx(dst, dst, MORPH_DILATE, element);
	morphologyEx(dst, dst, MORPH_ERODE, element);*/
	//morphologyEx(dst, dst, MORPH_ERODE, element);
	//imshow("形态学处理", dst);

}

void Armor_Detector::Armor_rio11_Detector(cv::Mat master, cv::Mat src, std::vector<cv::RotatedRect>& vRlt, cv::RotatedRect& last_vRlt)
{

	cv::RotatedRect s, s_fitEllipse, s_minAreaRect;
	static float prve_armor_center_x;
	static float prve_armor_center_y;
	std::vector<cv::RotatedRect> ss;
	//清零
	last_vRlt.center.x = 0;
	last_vRlt.center.y = 0;
	last_vRlt.size.width = 0;
	last_vRlt.size.height = 0;
	last_vRlt.angle = 0;
	vRlt.clear();
	ss.clear();


	//寻找轮廓
	Mat RIO, src_contours, RIO_master, RIO_BW;

	src_contours = src.clone();

	vector<vector<Point>> contours;
	vector<Vec4i> white_hierarchy;
	Mat drawout = Mat::zeros(src.size(), CV_8UC1);
	//Mat drawing_out = Mat::zeros(src.size(), CV_8UC1);
	Mat drawing_out1 = Mat::zeros(src.size(), CV_8UC1);
	//Mat drawing_out2 = master.clone();

	Scalar color = Scalar(255);//随机设置颜色
							   //Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
							   //morphologyEx(src_contours, src_contours, MORPH_OPEN, element);
							   //imshow("处理图像",src_contours);
	findContours(src_contours, contours, white_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	//计算矩找灯条
	for (unsigned int ii = 0; ii < contours.size(); ii++)
	{
		if (contours[ii].size() >= 10)//&& white_contours[i].size()> 10
		{
			s_fitEllipse = fitEllipse(contours[ii]);//二维点集的椭圆拟合，用椭圆将二维点包含起来  点要大于6个
			s_minAreaRect = minAreaRect(contours[ii]);//最小旋转矩

			s.angle = s_fitEllipse.angle;
			s.center = s_fitEllipse.center;
			if (s_minAreaRect.size.width > s_minAreaRect.size.height)
			{
				s.size.height = s_minAreaRect.size.width;
				s.size.width = s_minAreaRect.size.height;
			}
			else
			{
				s.size.height = s_minAreaRect.size.height;
				s.size.width = s_minAreaRect.size.width;

			}
			//限幅
			if ((s.size.width / s.size.height) > L_WH_RAT)
				continue;
			int x = s.center.x - s.size.width;
			if (x < 0)
				continue;
			int y = s.center.y - s.size.height;
			if (y < 0)
				continue;
			int w = s.size.width + s.size.width;
			if (w > master.cols - x)
				continue;
			int h = s.size.height + s.size.width;
			if (h > master.rows - y)
				continue;

			//新的判别方法
			if ((s.angle < 45 || s.angle>135) &&
				(s.size.height > 10) && (s.size.height < 150)
				)
			{
				//ellipse(drawing_out1, s.center, s.size, s.angle, 0, 360, color, 1, 8);//画椭圆
				//string Numbers1 = to_string(ii);
				//putText(drawing_out1, Numbers1, s.center, FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255));
				//imshow("灯柱最小椭圆", drawing_out1);
				//waitKey(30);
				ss.push_back(s);
			}
		}

	}

	//判别装甲
	std::vector<RotatedRect> armors;
	vector<Armor_builded> armor_SECOND;//
	Armor_builded armor_FIRST; //定义装甲区域的旋转矩形

	armors.clear();
	armor_SECOND.clear();
	int nL, nW;

	if (ss.size() < 2) //如果检测到的旋转矩形个数小于2，则直接返回
	{
		last_vRlt.center.x = 0;
		last_vRlt.center.y = 0;
		last_vRlt.size.width = 0;
		last_vRlt.size.height = 0;
		last_vRlt.angle = 0;
		vRlt.push_back(last_vRlt);
		//存储上一次装甲中心点
		prve_armor_center_x = 0;
		prve_armor_center_y = 0;
	}
	else
	{

		for (unsigned int III = 0; III < ss.size() - 1; III++) //求任意两个旋转矩形的夹角
		{
			for (unsigned int JJJ = III + 1; JJJ < ss.size(); JJJ++)
			{
				double height_diff = abs(ss[III].size.height - ss[JJJ].size.height);//高度差
				double height_sum = ss[III].size.height + ss[JJJ].size.height;//高度和
				double width_diff = abs(ss[III].size.width - ss[JJJ].size.width);//宽度差
				double width_sum = ss[III].size.width + ss[JJJ].size.width;//宽度和
				double angle_diff = fabs(ss[III].angle - ss[JJJ].angle);//角度差
				double Y_diff = abs(ss[III].center.y - ss[JJJ].center.y);//纵坐标差值
				double MH_diff = (min(ss[III].size.height, ss[JJJ].size.height)) * 2 / 3;//高度差限幅
				double height_max = (max(ss[III].size.height, ss[JJJ].size.height));//最大高度
				double X_diff = abs(ss[III].center.x - ss[JJJ].center.x);//横坐标差值

				if (
					Y_diff < MH_diff && X_diff < height_max * 4 &&
					(angle_diff < T_ANGLE_THRE || 180 - angle_diff < T_ANGLE_THRE180) &&
					height_diff / height_sum < T_HIGH_RAT &&
					width_diff / width_sum < T_WHIDTH_RAT   //还可以加入高度差限制
					)

				{

					armor_FIRST.armorS.center.x = ((ss[III].center.x + ss[JJJ].center.x) / 2); //装甲中心的x坐标 
					armor_FIRST.armorS.center.y = ((ss[III].center.y + ss[JJJ].center.y) / 2); //装甲中心的y坐标
					armor_FIRST.armorS.angle = (ss[III].angle + ss[JJJ].angle) / 2;   //装甲所在旋转矩形的旋转角度
					if (180 - angle_diff < T_ANGLE_THRE180)
						armor_FIRST.armorS.angle += 90;
					nL = (ss[III].size.height + ss[JJJ].size.height) / 2; //装甲的高度
					nW = sqrt((ss[III].center.x - ss[JJJ].center.x) * (ss[III].center.x - ss[JJJ].center.x) + (ss[III].center.y - ss[JJJ].center.y) * (ss[III].center.y - ss[JJJ].center.y)); //装甲的宽度等于两侧LED所在旋转矩形中心坐标的距离
					if (nL < nW)
					{
						armor_FIRST.armorS.size.height = nL;
						armor_FIRST.armorS.size.width = nW;
					}
					else
					{
						armor_FIRST.armorS.size.height = nW;
						armor_FIRST.armorS.size.width = nL;
					}
					if (Y_diff < nW / 3)
					{
						armor_FIRST.build1_No = III;
						armor_FIRST.build2_No = JJJ;
						armor_FIRST.build_features[0] = angle_diff;
						armor_FIRST.build_features[1] = Y_diff;
						armor_FIRST.build_features[2] = height_diff;
						armor_FIRST.build_features[3] = width_diff;
						armor_SECOND.push_back(armor_FIRST); //将找出的装甲的旋转矩形保存到vector
					}
				}
				else if ((angle_diff < T_ANGLE_THREMIN || 180 - angle_diff < T_ANGLE_THRE180MIN) &&
					Y_diff < MH_diff * 3 / 2 && X_diff < height_max * 4 &&
					height_diff / height_sum < T_HIGH_RAT_ANGLE &&
					width_diff / width_sum < T_WHIDTH_RAT_ANGLE
					)
				{
					armor_FIRST.armorS.center.x = ((ss[III].center.x + ss[JJJ].center.x) / 2); //装甲中心的x坐标 
					armor_FIRST.armorS.center.y = ((ss[III].center.y + ss[JJJ].center.y) / 2); //装甲中心的y坐标
					armor_FIRST.armorS.angle = (ss[III].angle + ss[JJJ].angle) / 2;   //装甲所在旋转矩形的旋转角度
					if (180 - angle_diff < T_ANGLE_THRE180)
						armor_FIRST.armorS.angle += 90;
					nL = (ss[III].size.height + ss[JJJ].size.height) / 2; //装甲的高度
					nW = sqrt((ss[III].center.x - ss[JJJ].center.x) * (ss[III].center.x - ss[JJJ].center.x) + (ss[III].center.y - ss[JJJ].center.y) * (ss[III].center.y - ss[JJJ].center.y)); //装甲的宽度等于两侧LED所在旋转矩形中心坐标的距离
					if (nL < nW)
					{
						armor_FIRST.armorS.size.height = nL;
						armor_FIRST.armorS.size.width = nW;
					}
					else
					{
						armor_FIRST.armorS.size.height = nW;
						armor_FIRST.armorS.size.width = nL;
					}
					if (Y_diff < nW / 2)
					{
						armor_FIRST.build1_No = III;
						armor_FIRST.build2_No = JJJ;
						armor_FIRST.build_features[0] = angle_diff;
						armor_FIRST.build_features[1] = Y_diff;
						armor_FIRST.build_features[2] = height_diff;
						armor_FIRST.build_features[3] = width_diff;
						armor_SECOND.push_back(armor_FIRST); //将找出的装甲的旋转矩形保存到vector
					}

				}
				else if ((angle_diff < 3 || 180 - angle_diff < 2) &&
					Y_diff < MH_diff * 2 && X_diff < height_max * 4
					//height_diff / height_sum < T_HIGH_RAT_ANGLE 
					)
				{
					armor_FIRST.armorS.center.x = ((ss[III].center.x + ss[JJJ].center.x) / 2); //装甲中心的x坐标 
					armor_FIRST.armorS.center.y = ((ss[III].center.y + ss[JJJ].center.y) / 2); //装甲中心的y坐标
					armor_FIRST.armorS.angle = (ss[III].angle + ss[JJJ].angle) / 2;   //装甲所在旋转矩形的旋转角度
					if (180 - angle_diff < T_ANGLE_THRE180)
						armor_FIRST.armorS.angle += 90;
					nL = (ss[III].size.height + ss[JJJ].size.height) / 2; //装甲的高度
					nW = sqrt((ss[III].center.x - ss[JJJ].center.x) * (ss[III].center.x - ss[JJJ].center.x) + (ss[III].center.y - ss[JJJ].center.y) * (ss[III].center.y - ss[JJJ].center.y)); //装甲的宽度等于两侧LED所在旋转矩形中心坐标的距离
					if (nL < nW)
					{
						armor_FIRST.armorS.size.height = nL;
						armor_FIRST.armorS.size.width = nW;
					}
					else
					{
						armor_FIRST.armorS.size.height = nW;
						armor_FIRST.armorS.size.width = nL;
					}
					if ((abs(ss[III].center.y - ss[JJJ].center.y) < nW / 2))
					{
						armor_FIRST.build1_No = III;
						armor_FIRST.build2_No = JJJ;
						armor_FIRST.build_features[0] = angle_diff;
						armor_FIRST.build_features[1] = Y_diff;
						armor_FIRST.build_features[2] = height_diff;
						armor_FIRST.build_features[3] = width_diff;
						armor_SECOND.push_back(armor_FIRST); //将找出的装甲的旋转矩形保存到vector
					}

				}
				else if ((angle_diff < 3 || 180 - angle_diff < 2) &&
					Y_diff < MH_diff * 3 && X_diff < height_max * 5
					//height_diff / height_sum < T_HIGH_RAT_ANGLE 
					)
				{
					armor_FIRST.armorS.center.x = ((ss[III].center.x + ss[JJJ].center.x) / 2); //装甲中心的x坐标 
					armor_FIRST.armorS.center.y = ((ss[III].center.y + ss[JJJ].center.y) / 2); //装甲中心的y坐标
					armor_FIRST.armorS.angle = (ss[III].angle + ss[JJJ].angle) / 2;   //装甲所在旋转矩形的旋转角度
					if (180 - angle_diff < T_ANGLE_THRE180)
						armor_FIRST.armorS.angle += 90;
					nL = (ss[III].size.height + ss[JJJ].size.height) / 2; //装甲的高度
					nW = sqrt((ss[III].center.x - ss[JJJ].center.x) * (ss[III].center.x - ss[JJJ].center.x) + (ss[III].center.y - ss[JJJ].center.y) * (ss[III].center.y - ss[JJJ].center.y)); //装甲的宽度等于两侧LED所在旋转矩形中心坐标的距离
					if (nL < nW)
					{
						armor_FIRST.armorS.size.height = nL;
						armor_FIRST.armorS.size.width = nW;
					}
					else
					{
						armor_FIRST.armorS.size.height = nW;
						armor_FIRST.armorS.size.width = nL;
					}
					if (Y_diff < nW / 2)
					{
						armor_FIRST.build1_No = III;
						armor_FIRST.build2_No = JJJ;
						armor_FIRST.build_features[0] = angle_diff;
						armor_FIRST.build_features[1] = Y_diff;
						armor_FIRST.build_features[2] = height_diff;
						armor_FIRST.build_features[3] = width_diff;
						armor_SECOND.push_back(armor_FIRST); //将找出的装甲的旋转矩形保存到vector
					}

				}
			}
		}

		if (armor_SECOND.size() < 1)
		{
			int ss_width = 0;
			int ss_ID = 0;
			for (unsigned int SSS = 0; SSS < ss.size(); SSS++) //求任意两个旋转矩形的夹角
			{
				if (ss[SSS].size.width > ss_width && (ss[SSS].size.width / ss[SSS].size.height) < 0.4 && (ss[SSS].size.width / ss[SSS].size.height) > 0.15)
				{
					ss_width = ss[SSS].size.width;
					ss_ID = SSS;
				}

			}
			int WIDTH = 3 * ss[ss_ID].size.height;
			int HEIGHT = 3 * ss[ss_ID].size.height;
			int XX_RIGHT = ss[ss_ID].center.x;
			int XX_LEFT = ss[ss_ID].center.x - WIDTH;
			int YY = ss[ss_ID].center.y - ss[ss_ID].size.height * 3 / 2;
			//
			if (XX_RIGHT + WIDTH > 1024)
			{
				WIDTH = 1024 - XX_RIGHT;
			}
			if (XX_RIGHT < 0)
			{
				XX_RIGHT = 0;
			}
			if (XX_LEFT < 0)
			{
				XX_LEFT = 0;
			}
			if (XX_LEFT + WIDTH > 1024)
			{
				WIDTH = 1024 - XX_LEFT;
			}
			if (YY + HEIGHT > 768)
			{
				HEIGHT = 768 - YY;
			}
			if (YY < 0)
			{
				YY = 0;
			}

			if (ss[ss_ID].angle > 45)
			{

				Mat LEFT_rio = master(Rect(XX_RIGHT, YY, WIDTH, HEIGHT));
				Mat  Rio_out, Rio_out1;

				cvtColor(LEFT_rio, Rio_out, COLOR_BGR2GRAY);//转化为灰度图像

				equalizeHist(Rio_out, Rio_out);


				//imshow("rio", Rio_out);
				GaussianBlur(Rio_out, Rio_out, Size(3, 3), 0, 0);//高斯滤波
				threshold(Rio_out, Rio_out1, 0, 255, THRESH_OTSU);//二值化
																  //imshow("RIO_二值化", Rio_out1);

				vector<vector<Point>> last_contours;
				vector<Vec4i>last_hierarchy;
				cv::RotatedRect s_center;
				std::vector<cv::RotatedRect> ss_center;
				ss_center.clear();
				findContours(Rio_out1, last_contours, last_hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
				//drawContours(drawing_out, last_contours, -1, color, 1);//可以省去
				int	centerX_num = 0;

				for (size_t ii = 0; ii < last_contours.size(); ii++)
				{
					if (last_contours[ii].size() > 10)
					{
						s_center = minAreaRect(last_contours[ii]);
						ss_center.push_back(s_center);
					}


				}
				for (size_t iii = 0; iii < ss_center.size() - 1; iii++)
				{
					if (ss_center[iii].size.height > ss[ss_ID].size.height / 2 && ss_center[iii].size.width / ss_center[iii].size.width > 0.5)
					{
						for (size_t jjj = iii + 1; jjj < ss_center.size(); jjj++)
						{
							int centerX_sum = abs(ss_center[iii].center.x - ss_center[jjj].center.x);
							int centerY_sum = abs(ss_center[iii].center.y - ss_center[jjj].center.y);
							if (centerX_sum < 5 && centerY_sum < 5)
							{
								centerX_num++;
							}
						}
					}

				}
				if (centerX_num > 3)
				{

					ss[ss_ID].center.x = ss[ss_ID].center.x + cos((double)(180.0 - ss[ss_ID].angle) / 180.0*PI)*ss[ss_ID].size.height;
					ss[ss_ID].center.y = ss[ss_ID].center.y - sin((double)(180.0 - ss[ss_ID].angle) / 180.0*PI)*ss[ss_ID].size.height;

					last_vRlt.center = ss[ss_ID].center;
					last_vRlt.size.width = ss[ss_ID].size.height * 2;
					last_vRlt.size.height = ss[ss_ID].size.height;

					last_vRlt.angle = ss[ss_ID].angle;

					vRlt.push_back(ss[ss_ID]);
				}


			}
			else
			{
				Mat LEFT_rio = master(Rect(XX_LEFT, YY, WIDTH, HEIGHT));
				Mat  Rio_out, Rio_out1;

				cvtColor(LEFT_rio, Rio_out, COLOR_BGR2GRAY);//转化为灰度图像

				equalizeHist(Rio_out, Rio_out);
				//imshow("rio", Rio_out);
				GaussianBlur(Rio_out, Rio_out, Size(3, 3), 0, 0);//高斯滤波

				threshold(Rio_out, Rio_out1, 0, 255, THRESH_OTSU);//二值化

																  //imshow("RIO_二值化", Rio_out1);

				vector<vector<Point>> last_contours;
				vector<Vec4i>last_hierarchy;
				cv::RotatedRect s_center;
				std::vector<cv::RotatedRect> ss_center;
				ss_center.clear();
				findContours(Rio_out1, last_contours, last_hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
				//drawContours(drawing_out, last_contours, -1, color, 1);//可以省去
				int	centerX_num = 0;

				for (size_t ii = 0; ii < last_contours.size(); ii++)
				{
					if (last_contours[ii].size() > 10)
					{
						s_center = minAreaRect(last_contours[ii]);
						ss_center.push_back(s_center);
					}


				}
				for (size_t iii = 0; iii < ss_center.size() - 1; iii++)
				{
					if (ss_center[iii].size.height > ss[ss_ID].size.height / 2 && ss_center[iii].size.width / ss_center[iii].size.width > 0.5)
					{
						for (size_t jjj = iii + 1; jjj < ss_center.size(); jjj++)
						{
							int centerX_sum = abs(ss_center[iii].center.x - ss_center[jjj].center.x);
							int centerY_sum = abs(ss_center[iii].center.y - ss_center[jjj].center.y);
							if (centerX_sum < 5 && centerY_sum < 5)
							{
								centerX_num++;
							}
						}
					}

				}
				if (centerX_num > 3)
				{

					ss[ss_ID].center.x = ss[ss_ID].center.x - cos((double)(ss[ss_ID].angle) / 180.0*PI)*ss[ss_ID].size.height;
					ss[ss_ID].center.y = ss[ss_ID].center.y - sin((double)(ss[ss_ID].angle) / 180.0*PI)*ss[ss_ID].size.height;

					last_vRlt.center = ss[ss_ID].center;
					last_vRlt.size.width = ss[ss_ID].size.height * 2;
					last_vRlt.size.height = ss[ss_ID].size.height;

					last_vRlt.angle = ss[ss_ID].angle;

					vRlt.push_back(ss[ss_ID]);
				}

			}

		}
		else if (armor_SECOND.size() == 1)
		{
			last_vRlt = armor_SECOND[0].armorS;
			vRlt.push_back(armor_SECOND[0].armorS);
			//存储上一次装甲中心点
			prve_armor_center_x = last_vRlt.center.x;
			prve_armor_center_y = last_vRlt.center.y;
		}
		else
		{
			double min_feature = 9999999;
			for (int armor_i = 0; armor_i < armor_SECOND.size(); armor_i++)//对各个灯带进行遍历
			{
				armors.push_back(armor_SECOND[armor_i].armorS);
				//计算加权特征值
				double feature = armor_SECOND[armor_i].build_features[0] * 100 +
					armor_SECOND[armor_i].build_features[1] * 10 +
					armor_SECOND[armor_i].build_features[2] * 100 +
					//armor_SECOND[armor_i].build_features[3] * 0 +
					abs(armor_SECOND[armor_i].armorS.center.x - prve_armor_center_x) * 50 +
					abs(armor_SECOND[armor_i].armorS.center.y - prve_armor_center_y) * 50 -
					armor_SECOND[armor_i].armorS.size.height * 100 -
					armor_SECOND[armor_i].armorS.size.width * 100;
				if (feature < min_feature)//找到最小特征值
				{
					min_feature = feature;
					last_vRlt = armor_SECOND[armor_i].armorS;
				}

			}
			//存储上一次装甲中心点
			prve_armor_center_x = last_vRlt.center.x;
			prve_armor_center_y = last_vRlt.center.y;
			vRlt = armors;
		}



	}




}

//弹道计算
double Armor_Detector::BallisticCalculation(double distance)
{
	double angle;
	double v = BallisticCal_BallisticSpd;
	distance = distance / 1000;
	double diff = 9999999;
	for (double a = -10; a <= 45; a += BallisticCal_Accuracy)
	{
		double ang = a / 180 * PI;
		double tem_diff = abs(tan(ang) - 4.89*distance / v / v / cos(ang) / cos(ang));
		if (tem_diff < diff)
		{
			diff = tem_diff;
			angle = a;
		}
	}
	return angle;
}

//云台控制
void Armor_Detector::Yuntai_Control(const cv::RotatedRect& tg_armor, int* offset_y, int* offset_p, bool flag)
{
	int ang_Y = 0;
	int ang_P = 0;
	double ballist_angle;//弹道角度
	double err_Y = 0;
	double err_P = 0;
	if (flag)
	{
		if (tg_armor.size.width&&tg_armor.size.height)
		{
			//err_Y = tg_armor.center.x - (Stander_x - (*offset_y) / 2.0);
			//err_P = tg_armor.center.y - (Stander_y + (*offset_p) / 20.0);
			double Move_Y_P = fabs(err_Y);
			ang_Y = (int)((Move_Y_P*Move_Y_P*Yuntai_Y_P + Move_Y_P * Yuntai_Y_D) * 10);
			double Move_P_P = fabs(err_P);
			ang_P = (int)((Move_P_P*Move_P_P*Yuntai_P_P + Move_P_P * Yuntai_P_D) * 10);
			//LastTargetArmor = tg_armor;
		}
		/*else
		{
			ang_Y = (*offset_y) / 25;
			ang_P = (*offset_p) / 100;
		}*/
		/*char cmd = 'c';
		*(INT16*)&send_data[0] = ang_Y;
		*(INT16*)&send_data[2] = ang_P;
		if (com.UART_Send_Buff(cmd, send_data, 4))
		printf("成功发送 ===cmd:%c ===ang_Y:%d ===ang_P:%d \n", cmd, ang_Y, ang_P);*/
	}
}

//画出结果图
void drawBox(RotatedRect box, Mat img)
{
	Point2f ppt[4];
	//int i;
	/*for (i = 0; i < 4; i++)
	{
	pt[i].x = 0;
	pt[i].y = 0;
	}*/
	box.points(ppt); //计算二维盒子顶点 
	line(img, ppt[0], ppt[1], CV_RGB(255, 0, 0), 2, 8, 0);
	line(img, ppt[1], ppt[2], CV_RGB(0, 255, 0), 2, 8, 0);
	line(img, ppt[2], ppt[3], CV_RGB(0, 0, 255), 2, 8, 0);
	line(img, ppt[3], ppt[0], CV_RGB(255, 255, 255), 2, 8, 0);


}
void drawBox1(RotatedRect box, Mat img)
{
	Point2f pts[4];
	box.points(pts); //计算二维盒子顶点 
	line(img, pts[0], pts[1], CV_RGB(255, 255, 255), 2, 8, 0);
	line(img, pts[1], pts[2], CV_RGB(255, 255, 255), 2, 8, 0);
	line(img, pts[2], pts[3], CV_RGB(255, 255, 255), 2, 8, 0);
	line(img, pts[3], pts[0], CV_RGB(255, 255, 255), 2, 8, 0);


}
void Armor_Detector::autoShoot()
{
	Mat imgOriginal_out = srcimage;
	Armor_Detector_pretreatment(srcimage, dstimage);
	Armor_rio11_Detector(srcimage, dstimage, VRlt, VRlt_last);
	if (VRlt_last.size.height > 0)
	{
		//标定中心点（调试使用）
		/*cout << "XX"<<VRlt_last.center.x << endl << endl;
		cout << "YY" <<VRlt_last.center.y << endl << endl;*/
		//标定中心点end
		//sovlepnp测距计算pitch偏移
		//Armor_Detector_Solvepnp(VRlt_last);
		//sovlepnp测距end

		

		//pid begin
		//pitch（Y方向）
		//Pitch_pid.PID_Change_Pitch(&Pitch_pid, 398 + Muzzle_compensation, VRlt_last.center.y);
		//ang_P = Pitch_pid.PID_Control_normal(&Pitch_pid, 398 + Muzzle_compensation, VRlt_last.center.y);//, Pitch_pid.Positional_PID);
		Pitch_pid.PID_Change_Pitch(&Pitch_pid, Pitcure_centring_Y, VRlt_last.center.y);
		ang_P = Pitch_pid.PID_Control_normal(&Pitch_pid, Pitcure_centring_Y, VRlt_last.center.y);//, Pitch_pid.Positional_PID);
			
		
		Yaw_pid.PID_Change_Yaw(&Yaw_pid, Pitcure_centring_X, VRlt_last.center.x);
		ang_Y = Yaw_pid.PID_Control_normal(&Yaw_pid, Pitcure_centring_X, VRlt_last.center.x);//, Yaw_pid.Positional_PID);
		 //pid end
		 
		if (abs(ang_P)<5 && abs(ang_Y)<5)
		{
			AutoShoot = 1;
		}
		else
		{
			AutoShoot = 0;
		}



		//预测
		//prve_armor_center_x = Yaw_deviation;//存储上一次的X方向偏差
		////prve_armor_center_y = Pitch_deviation;//存储上一次的Y方向偏差
		//Yaw_deviation = (Pitcure_centring_X - VRlt_last.center.x);//计算当前目标中心相对图像中心的点偏差
		//Deviation_prove_now = prve_armor_center_x - Yaw_deviation;//上一次偏差和本次偏差的差值
		////左边
		//if (abs(Deviation_prove_now) < VRlt_last.size.width &&//偏差的差的绝对值小于一个值
		//	prve_armor_center_x < VRlt_last.size.width*2 &&//偏差小于一个值
		//	Yaw_deviation > VRlt_last.size.width / 2 &&
		//	//armor.V_Yaw > 5 &&//yaw轴速度大于一个值
		//	//armor.V_Yaw<10//yaw轴速度小于一个值
		//	left_follow_time < 20//累加限幅
		//	)
		//{
		//	left_follow_time++;//跟随计时
		//}
		//else if (prve_armor_center_x < -VRlt_last.size.width / 2 //&&//过调清零计时
		//									   //armor.V_Yaw<0 //&&
		//									   //armor.V_Yaw<-10
		//	)
		//{
		//	left_time_clearn++;//清零计时
		//}

		//if (prve_armor_center_x < -VRlt_last.size.width && left_time_clearn>10)
		//{
		//	right_follow_time = 0;
		//	right_time_clearn = 0;
		//	follow_direction = 0;
		//}



		//if (left_time_clearn > 10)//清零
		//{
		//	left_follow_time = 0;
		//	left_time_clearn = 0;
		//	follow_direction = 0;
		//}

		//
		//if (left_follow_time > 10 && //跟随计时符合条件
		//	prve_armor_center_x > VRlt_last.size.width/2 && //滤除抖动数据
		//	follow_direction<8)//前馈限幅
		//{
		//	follow_direction += prve_armor_center_x/80;
		//	//follow_direction += 1;
		//	//follow_direction += Yaw_deviation / 100;
		//}
		//

		//
		//cout << "left_follow_time:" << left_follow_time << endl;
		//cout << "left_time_clearn:" << left_time_clearn << endl;
		//


		////右边  
		//
		//if (abs(Deviation_prove_now) < VRlt_last.size.width &&
		//	prve_armor_center_x > -VRlt_last.size.width*2 &&
		//	Yaw_deviation < -VRlt_last.size.width / 2 &&
		//	//armor.V_Yaw > 5 &&
		//	//armor.V_Yaw<10
		//	right_follow_time < 20
		//	)
		//{
		//	right_follow_time++;
		//}
		//else if (prve_armor_center_x > VRlt_last.size.width / 2 //&&
		//																	 //armor.V_Yaw<0 //&&
		//																	 //armor.V_Yaw<-10
		//	)
		//{
		//	right_time_clearn++;
		//}
		//if (prve_armor_center_x > VRlt_last.size.width && right_follow_time>10)
		//{
		//	right_follow_time = 0;
		//	right_time_clearn = 0;
		//	follow_direction = 0;
		//}

		//if (right_time_clearn > 10)
		//{
		//	right_follow_time = 0;
		//	right_time_clearn = 0;
		//	follow_direction = 0;
		//}

		//if (right_follow_time > 10 &&
		//	prve_armor_center_x < -VRlt_last.size.width / 2 &&
		//	follow_direction > (-8))
		//{
		//	follow_direction += prve_armor_center_x/80;
		//	//follow_direction += Yaw_deviation /100;
		//	//follow_direction += 1;

		//}


		//
		//cout << "right_follow_time:" << right_follow_time << endl;
		//cout << "right_time_clearn:" << right_time_clearn << endl;
		//

		//cout << "follow_direction:" << follow_direction << endl;
		//cout << "Deviation_prove_now:" << Deviation_prove_now << endl;
		//cout << "armor.prve_armor_center_x:" << prve_armor_center_x << endl;
		//armor.Offset_P = *(INT16*)&Rbuff[6];
		
		
		for (unsigned int nI = 0; nI < VRlt.size(); nI++) //在当前图像中标出识别到所有的装甲的位置
		{

			drawBox(VRlt[nI], imgOriginal_out);

		}
		drawBox1(VRlt_last, imgOriginal_out);//击打目标
	}
	else
	{
		if (AutoShoot == 1)
		{
			lost_canter++;
		}
		if (lost_canter>10)
		{
			lost_canter = 0;
			AutoShoot = 0;
		}
		ang_P = 0;
		ang_Y = 0;
		Yaw_deviation = 0;
		follow_direction = 0;
	}
	imshow("momoDebug:", imgOriginal_out);
}

Armor_Detector::~Armor_Detector()
{
}