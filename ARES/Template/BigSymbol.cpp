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


#include"bigSymbol.h"
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

cv::Point3f posTarget[9] =
{
	cv::Point3f(-0.115,0.280,0),
	cv::Point3f(0.21,0.280,0),
	cv::Point3f(0.535,0.280,0),
	cv::Point3f(-0.115,0.53,0),
	cv::Point3f(0.21,0.53,0),
	cv::Point3f(0.535,0.53,0),
	cv::Point3f(-0.115,0.78,0),
	cv::Point3f(0.21,0.78,0),
	cv::Point3f(0.535,0.78,0),
};
//========== 用于排序的比较函数 =============//
//升序比较函数(distance)
bool lessmark(const cv::Rect& s1, const cv::Rect& s2)
{
	return s1.x < s2.x;
}
//升序比较函数(distance)
bool lessmark_myRect(const myRect& s1, const myRect& s2)
{
	return s1.similarValue < s2.similarValue;
}
//降序比较函数
bool lessmark_rotateArea(const cv::RotatedRect& s1, const cv::RotatedRect& s2)
{
	return s1.size.area()>s2.size.area();
}
bool sortUp(const cv::Rect& s1, const cv::Rect& s2)
{
	return (s1.y) < (s2.y);
}
//降序比较函数
bool sortDown(const cv::Rect& s1, const cv::Rect& s2)
{
	return (s1.width*s1.height) > (s2.width*s2.height);
}
//按相似度升序比较
bool lessmark_mypswRect(const mypswRect& s1, mypswRect& s2)
{
	return s1.similarValue < s2.similarValue;
}
//按X坐标升序比较
bool lessmark_mypswRect1(const mypswRect& s1, mypswRect& s2)
{
	return (s1.boundRect.x) < (s2.boundRect.x);
}
//降序比较函数 X+Y
bool lessmark_myPoint(const myPoint& s1, const myPoint& s2)
{
	return s1.sumXY > s2.sumXY;
}
//降序比较函数 X+(rows-Y)
bool lessmark_myPoint1(const myPoint& s1, const myPoint& s2)
{
	return s1.sumXY_inv > s2.sumXY_inv;
}
//升序比较函数
bool lessmark_myResult(const myResult& s1, const myResult& s2)
{
	return s1.Index < s2.Index;
}


//========== 构造函数-系统初始化 ===========//
sbol::sbol()
{
	SymbolModel = numArab; //默认情况下为小符模式
	//识别模型初始化
	knn = cv::ml::KNearest::create();
	svm = cv::ml::SVM::create();
	svm2 = cv::ml::SVM::create();
	trainpassword();//训练密码
	loadSVM();//加载SVM模型

	psWord[5] = {0}; //当前密码
	lstPsWord[5] = {0}; //上一次的密码
	hitCnt = 0;//击打计数

	shootFlag = true;
	//observeFlag = false;
    pChangeFlag = false; //换页标志位
	stopChange_cnt = 0;
	for (int i = 0; i < 9; i++)
	{
		ArrayResult[i] = 0;
		ArrayResult1[i] = 0;
		ArrayResult2[i] = 0;
		ArrayResult3[i] = 0;
	}

    shootCnt = 0;
	shootCnt1 = 0;
	//识别结果初始化
	myResult temp;
	temp.Index = 0;
	temp.value = 0;
	temp.position = Point(0, 0);
	for (int i = 0; i < 9; i++)
	{
		vrecResult.push_back(temp);
		vlstrecResult.push_back(temp);
	};

    //导入相机参数

	cv::FileStorage fs2("C:\\DATA\\Paragram\\camera.yml", cv::FileStorage::READ);
	fs2["camera_matrix"] >> K_;
	fs2["distortion_coefficients"] >> D_; 
	fs2.release();
	//cout << K_ << endl;
	//cout << D_ << endl;
	//数码管四个端点世界坐标点
	float cornersaa[4][2] = {
		0.0,0.0,
		-0.0130,0.098,
		0.416,0.0,
		0.403,0.098,
		/*0.0,0.0,
		-0.015,0.115,
		0.495,0.0,
		0.480,0.115,*/
	};
	for (int i = 0; i < 4; i++)
	{
		Point3f tmp;
		tmp.x = cornersaa[i][0];
		tmp.y = cornersaa[i][1];
		tmp.z = 0;
		corners.push_back(tmp);
	}
	Depth = 0;
	angleY = 0;
    angleP = 0;
	//云台矫正角度（由于摄像头坐标系与云台坐标系不一样，引入
	//一个矫正角度）
	corectAngY = -2.0;
	corectAngP = -13.0;

	/*Eigen::Vector3f rgb_origin_out(-0.0861561, 0.139061, -0.026996);
	Eigen::Matrix3f rgb_btm;
	rgb_btm << -0.0141455, -0.988972, 0.147423, 0.9997, -0.0169382, -0.0177054, 0.0200073, 0.147128, 0.988915;
    cam_shot = EigenFrom3(rgb_origin_out, rgb_btm);*/

}

Eigen::Matrix4d sbol::EigenFrom3(Eigen::Vector3d transl, Eigen::Matrix3d Rmat)
{

	Eigen::Matrix4d out;

	out.topLeftCorner<3, 3>() = Rmat;

	out.topRightCorner<3, 1>() = transl;

	out(3, 0) = 0;

	out(3, 1) = 0;

	out(3, 2) = 0;

	out(3, 3) = 1;

	return out;

}
//KNN训练样本装填
void sbol::sampleInit(cv::Mat& trainData, cv::Mat& trainClass)
{
	float *PtrainData = (float*)trainData.data;
	float *PtrainClass = (float*)trainClass.data;
	for (int i = 0; i < trainClasses; i++)
	{
		Mat img;
		char fileName[100];
		sprintf_s(fileName, "C:\\DATA\\M\\m%d.jpg", i + 1);
		img = imread(fileName, 0);
		if (!img.data)
		{
			printf("无法读取样本图片，样本初始化失败！\n");
			return;
		}
		resize(img, img, Size(sizeX, sizeY));//把图像变成一样的大小
		threshold(img, img, 125, 255, THRESH_BINARY);
		uchar *Pimage = img.data;
		for (int n = 0; n < featureLen; n++)//特征向量：此图像的像素值
		{
			PtrainData[i*featureLen + n] = (float)Pimage[n];//把图像的特征向量（像素值)变成一个行向量
		}
		PtrainClass[i] = (float)(i + 1);//贴标签
	}
}
//训练密码
void sbol::trainpassword()
{
	train_data.create(trainClasses*trainSample, featureLen, CV_32FC1);
	train_classes.create(trainClasses*trainSample, 1, CV_32FC1);
	sampleInit(train_data, train_classes);							   //加载训练数据
	knn->train(train_data, cv::ml::ROW_SAMPLE, train_classes);
}
//加载SVM
void sbol::loadSVM()
{
	svm->clear(); 
	svm = cv::ml::SVM::load("C:\\DATA\\SVM\\SvmModel.xml");		 //	 SVM_HOG.xml		DATA\\SvmModel			Svm1	
	svm2->clear();
	svm2 = cv::ml::SVM::load("C:\\DATA\\SVM\\biaoqian1.xml");
}

//============= 摄像头初始化 =================//
bool sbol::camaraInit(int device)
{

	Capture_BigPower.open(device);
	//Capture_BigPower.open("C:\\NOW\\2\\2.mp4");
	if (!Capture_BigPower.isOpened())
	{
		printf("大符摄像头打开失败！\n");
		return false;
	}
	else
	{
		//设置副摄像头分辨率
		//Capture_BigPower.set(CV_CAP_PROP_EXPOSURE, (-7));//曝光 50  -7
		Capture_BigPower.set(CV_CAP_PROP_FRAME_WIDTH, 960);
		Capture_BigPower.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
		return true;
	}
}
//图像预处理
void sbol::imgPretreatment(const cv::Mat& src, cv::Mat& dst)
{
	Mat imgRed;
	std::vector<Mat> imgChannels;
	split(src, imgChannels);
	imgRed = imgChannels.at(2);
	Scalar meanValue = mean(imgRed);
	imgRed = imgRed - meanValue[0];
	blur(imgRed, imgRed, Size(5, 5));
	double maxValue_gray;
	minMaxLoc(imgRed, 0, &maxValue_gray, 0, 0);
	Mat imgBin;
	threshold(imgRed, imgBin, maxValue_gray*0.7, 255, THRESH_BINARY);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));//构造核函数
	cv::dilate(imgBin, imgBin, element);
	cv::dilate(imgBin, imgBin, element);
	//cv::dilate(imgBin, imgBin, element);
	//cv::dilate(imgBin, imgBin, element);
	//imshow("预处理_imgBin:", imgBin);
	dst = imgBin;
}
//锁定数码管感兴趣区域
bool sbol::findDigitronLightRoi(const cv::Mat& src, cv::Rect& roiRect)
{
	roiRect = Rect(0, 0, 0, 0);
	std::vector<std::vector<cv::Point>> contours;//检测到的轮廓
	std::vector<cv::Vec4i> hierarchy;//轮廓层次
	Mat imgContours;
	src.copyTo(imgContours);//拷贝一份图像用于检测轮廓
	cv::findContours(imgContours, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
	//删除太小的轮廓 求轮廓的最外围包络矩形
	std::vector<cv::Rect> contoursRect;
	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() < 50)
			continue;
		Rect tempRect = boundingRect(contours[i]);
		contoursRect.push_back(tempRect);
	}
	if (contoursRect.size()<5)
	{
		cout << "error1   ";
		return false;
	}
	//根据矩形长宽比筛选矩形
	std::vector<cv::Rect> tempRect1;
	for (int i = 0; i < contoursRect.size(); i++)
	{
		float H = (float)contoursRect[i].height;
		float W = (float)contoursRect[i].width;
		float rateHW = H / W;
		if (rateHW>1.01&&rateHW < 2.5)
			tempRect1.push_back(contoursRect[i]);
	}
	if (tempRect1.size()<4)
	{
		cout << "error2   ";
		return false;
	}
	//根据大小相似度筛选矩形
	std::vector<cv::Rect> tempRect2;
	for (int i = 0; i < tempRect1.size(); i++)
	{
		int Index = i;
		int cnt = 0;
		int W = tempRect1[Index].width;
		int H = tempRect1[Index].height;
		for (int j = 0; j < tempRect1.size(); j++)
		{
			if (j == Index)
				continue;
			int tempW = tempRect1[j].width;
			int tempH = tempRect1[j].height;
			if ((abs(W - tempW) < 25) && (abs(H - tempH)<25))
				cnt++;
		}//筛选数码管轮廓
		if (cnt >= 3)
			tempRect2.push_back(tempRect1[Index]);
	}
	if (tempRect2.size() < 4)
	{
		cout << "error3   ";
		return false;
	}
	std::vector<myRect> rectVectors;
	for (int i = 0; i < tempRect2.size(); i++)
	{
		myRect tempRect;
		tempRect.boundRect = tempRect2[i];
		Mat imgRoi = src(tempRect.boundRect);
		Mat imgNum = imgRoi.clone();
		resize(imgNum, imgNum, Size(sizeX, sizeY));
		threshold(imgNum, imgNum, 200, 255, THRESH_BINARY);
		Mat testImg(1, featureLen, CV_32FC1);
		float* p = (float*)testImg.data;//p指向testImg的首地址
		uchar* PtestImg = imgNum.data;//PtestImg指向imgNum的首地址
		for (int n = 0; n < featureLen; n++)
		{
			p[n] = (float)PtestImg[n];//把imgNum中的像素值移植到testImg中、因为imgNum是Mat，K近邻样本必须是按行存储的
		}
		Mat dists(1, 1, CV_32FC1, Scalar(0));
		Mat A(1, 1, CV_32FC1, Scalar(0));
		Mat B(1, 1, CV_32FC1, Scalar(0));
		knn->findNearest(testImg, 1, A, B, dists);//testImg是按行存储的,单精度浮点向量，样本数量*K
		tempRect.similarValue = dists.at<float>(0, 0);//dists指的是当前矩形跟数据集比对，得到的最大临近值
		rectVectors.push_back(tempRect);
	}
	sort(rectVectors.begin(), rectVectors.end(), lessmark_myRect);//根据与样本的相似度进行排序
	//最像的三个区域里面取最上面的
	std::vector<cv::Rect> vtempResultRects;
	for (int i = 0; i < 3; i++)
		vtempResultRects.push_back(rectVectors[i].boundRect);
	sort(vtempResultRects.begin(), vtempResultRects.end(), sortUp);
	int rectX, rectY, rectW, rectH;
	//外扩数码管轮廓
	/*rectX = rectVectors[0].boundRect.x - 5 * rectVectors[0].boundRect.height;
	if (rectX < 0)
		rectX = 0;
	rectY = rectVectors[0].boundRect.y - rectVectors[0].boundRect.height / 2;
	if (rectY < 0)
		rectY = 0;
	rectW = 10 * rectVectors[0].boundRect.height;
	if ((rectX + rectW)>src.cols)
		rectW = src.cols - rectX;
	rectH = 2 * rectVectors[0].boundRect.height;
	if ((rectY + rectH)>src.rows)
		rectH = src.rows - rectY;*/
	rectX = vtempResultRects[0].x - 5 * vtempResultRects[0].height;
	if (rectX < 0)
	rectX = 0;
	rectY = vtempResultRects[0].y - vtempResultRects[0].height / 2;
	if (rectY < 0)
	rectY = 0;
	rectW = 10 * vtempResultRects[0].height;
	if ((rectX + rectW)>src.cols)
	rectW = src.cols - rectX;
	rectH = 2 * vtempResultRects[0].height;
	if ((rectY + rectH)>src.rows)
	rectH = src.rows - rectY;
	if ((rectW <= 0) || (rectH <= 0))
	{
		cout << "error4   ";
		return false;
	}
	roiRect.x = rectX;
	roiRect.y = rectY;
	roiRect.width = rectW;
	roiRect.height = rectH;
	return true;
}
//识别密码并确定四个端点
bool sbol::recPassWord(const cv::Mat& src, cv::Rect& dlRect, int* resultArray, std::vector<cv::Point2f>& observationPts, cv::Mat& r, cv::Mat& t,double& depth)
{
	for (int i = 0; i < 5; i++)
		resultArray[i] = 0;
	observationPts.clear();
	depth = 0;
	Mat img;
	img = src(dlRect);

	Mat imgRed;
	std::vector<Mat> imgChannels;
	split(img, imgChannels);
	imgRed = imgChannels.at(2);
	Scalar meanValue = mean(imgRed);
	imgRed = imgRed - meanValue[0];
	blur(imgRed, imgRed, Size(5, 5));
	double maxValue_gray;
	minMaxLoc(imgRed, 0, &maxValue_gray, 0, 0);
	Mat imgBin;
	threshold(imgRed, imgBin, maxValue_gray*0.7, 255, THRESH_BINARY);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));//构造核函数
	cv::dilate(imgBin, imgBin, element);
	cv::dilate(imgBin, imgBin, element);
	cv::dilate(imgBin, imgBin, element);
	//imshow("识别密码_imgBin:", imgBin);

	std::vector<std::vector<cv::Point>> contours;//检测到的轮廓
	std::vector<cv::Vec4i> hierarchy;//轮廓层次
	Mat imgContours;
	imgBin.copyTo(imgContours);//拷贝一份图像用于检测轮廓
	cv::findContours(imgContours, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
	if (contours.size() < 5)
	{
		cout << "error5   ";
		return false;
	}
	std::vector<mypswRect> pswRect;
	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() < 50)
			continue;
		mypswRect tempPswRect;
		tempPswRect.boundRect = boundingRect(contours[i]);
		tempPswRect.contours = contours[i];
		Mat imgRoi = imgBin(tempPswRect.boundRect);		    //通过数码管的轮廓抠图
		Mat imgNum = imgRoi.clone();
		resize(imgNum, imgNum, Size(sizeX, sizeY));
		threshold(imgNum, imgNum, 200, 255, THRESH_BINARY);
		Mat testImg(1, featureLen, CV_32FC1);
		float* p = (float*)testImg.data;
		uchar* PtestImg = imgNum.data;
		for (int n = 0; n < featureLen; n++)
		{
			p[n] = (float)PtestImg[n];
		}
		Mat dists(1, 1, CV_32FC1, Scalar(0));
		Mat C(1, 1, CV_32FC1, Scalar(0));
		Mat D(1, 1, CV_32FC1, Scalar(0));
		knn->findNearest(testImg, 1, C, D, dists);
		tempPswRect.similarValue = dists.at<float>(0, 0);
		tempPswRect.value =(int) C.at<float>(0, 0);
		pswRect.push_back(tempPswRect);
	}
	if (pswRect.size()<5)
	{
		cout << "error6   " << endl;
		return false;
	}
	sort(pswRect.begin(), pswRect.end(), lessmark_mypswRect);//按相似度升序比较
	std::vector<mypswRect> pswNumRects;
	for (int i = 0; i < 5; i++)
		pswNumRects.push_back(pswRect[i]);
	sort(pswNumRects.begin(), pswNumRects.end(), lessmark_mypswRect1);//按坐标升序比较

	//保存密码
	for (int i = 0; i < 5; i++)
		resultArray[i] = pswNumRects[i].value;

	//计算密码区的四个顶点(用于Solve pnp 测距和区域映射)
	for (int i = 0; i < pswNumRects.size(); i+=4)
	{
		std::vector<myPoint> vtempMyPoint;
		for (int j = 0; j < pswNumRects[i].contours.size(); j++)
		{
			myPoint tempMyPt;
			tempMyPt.pt = pswNumRects[i].contours[j];
			tempMyPt.sumXY = pswNumRects[i].contours[j].x + pswNumRects[i].contours[j].y;
			tempMyPt.sumXY_inv = pswNumRects[i].contours[j] .x+ (img.rows - pswNumRects[i].contours[j].y);
			vtempMyPoint.push_back(tempMyPt);
		}
		cv::Point2f tempPt;
	    //按SUM(X+(rows-Y))降序比较
		sort(vtempMyPoint.begin(), vtempMyPoint.end(), lessmark_myPoint1);
		tempPt = cv::Point2f(vtempMyPoint[0].pt.x + dlRect.x, vtempMyPoint[0].pt.y + dlRect.y);
		observationPts.push_back(tempPt);
		//按SUM(X+Y)降序比较
		sort(vtempMyPoint.begin(), vtempMyPoint.end(), lessmark_myPoint);
		tempPt = cv::Point2f(vtempMyPoint[0].pt.x + dlRect.x, vtempMyPoint[0].pt.y + dlRect.y);
		observationPts.push_back(tempPt);
	}
	Mat rvec, tvec;
	cv::solvePnP(cv::Mat(corners), cv::Mat(observationPts), K_, D_, rvec, tvec, false);
	Rodrigues(rvec, r);  //旋转向量转换为旋转矩阵
	t = tvec;//平移向量
	depth = tvec.at<double>(2, 0);
	/*cv::Point2f Angle;
	cv::Matx33d tempK(K_);
	Angle = get_pattern_shot_3D(tempK, observationPts[1], cam_shot, depth);
	angleY = Angle.x + corectAngY;
	angleP = Angle.y + corectAngP;*/
	//angleY -= angleY / 10;
	//angleP -= angleP / 3;
	return true;
}

//============= 重置打击顺序 ==============//
void sbol::resetShootCnt(int* ps_array, int* ls_ps_array, int& count, int& err_count, int& cnt, bool& flag)
{
	int tempCount = 0;
	for (int i = 0; i < 5; i++)
	{
		if (ps_array[i] != ls_ps_array[i])
			tempCount++;
	}
	if ((tempCount == 0) && (flag == true))
	{
		return;
	}
	else if ((tempCount > 0) && (tempCount < 3) && (flag == true))
	{
		for (int i = 0; i < 5; i++)
		{
			ps_array[i] = ls_ps_array[i];
		}
		return;
	}
	else if ((tempCount>=3)&&(flag == true))
	{
		flag = false;
		count = 0;
		err_count = 0;
		return;
	}
	else if (flag == false)
	{
		count++;
		if (tempCount >= 3)
			err_count++;
		if (count == 2)
		{
			if (err_count < 2)
			{
				flag = true;
				for (int i = 0; i < 5; i++)
				{
					ps_array[i] = ls_ps_array[i];
				}
			}
			else
			{
				flag = true;
				for (int i = 0; i < 5; i++)
				{
					ls_ps_array[i] = ps_array[i];
				}
				cnt = 0; //换密码了
				cout << "重置密码   ";
			}
		}
	}
}
//端点映射
void sbol::point_To_point(std::vector<cv::Point2f>& src, std::vector<cv::Point>& dst)
{
	double a;
	cv::Point tempPt[4];
	a = (src[2].x - src[0].x) / 416;
	tempPt[0] = cv::Point(src[0].x - 328.5*a, src[1].y + 60.2*a);
	tempPt[1] = cv::Point(src[0].x - 328.5*a, src[1].y + 720.2*a);
	tempPt[2] = cv::Point(src[0].x + 727.5*a, src[1].y + 60.2*a);
	tempPt[3] = cv::Point(src[0].x + 727.5*a, src[1].y + 720.2*a);
	for (int i = 0; i < 4; i++)
		dst.push_back(tempPt[i]);
}
//计算格子区域
bool sbol::calGirdRect(const cv::Mat& src, std::vector<cv::Point2f>& pts, std::vector<cv::RotatedRect>& gridRect)
{
	gridRect.clear();
	rectShow = cv::Rect(0, 0, 0, 0);
	//端点映射
	std::vector<cv::Point> tempPoints;
	point_To_point(pts, tempPoints);
	//判断点是否在图像中
	for (int i = 0; i < tempPoints.size(); i++)
	{
		/*if ((tempPoints[i].x < 0) || (tempPoints[i].x >= src.cols) || (tempPoints[i].y < 0) || (tempPoints[i].y >= src.rows))
		{
			cout << "error7   ";
			return false;
		}*/
		if (tempPoints[i].x < 1)
			tempPoints[i].x = 1;
		if(tempPoints[i].x > (src.cols - 2))
			tempPoints[i].x = src.cols-2;
		if (tempPoints[i].y< 1)
			tempPoints[i].y = 1;
		if (tempPoints[i].y > (src.rows - 2))
			tempPoints[i].y = src.rows - 2;
	}
	cv::Rect tempRect = boundingRect(tempPoints);
	//外扩1/8
	int rectX, rectY, rectW, rectH;
	rectX = tempRect.x - tempRect.width / 8;
    if (rectX < 0)
		rectX = 0;
	rectY = tempRect.y - tempRect.height / 8;
	if (rectY < 0)
		rectY = 0;
	rectW = tempRect.width + tempRect.width / 4;
	if ((rectX + rectW)>src.cols)
		rectW = src.cols - rectX;
	rectH = tempRect.height + tempRect.height / 4;
	if ((rectY + rectH)>src.rows)
		rectH = src.rows - rectY;
	if ((rectW <= 0) || (rectH <= 0))
	{
		cout << "error8   ";
		return false;
	}
	tempRect = cv::Rect(rectX, rectY, rectW, rectH);
	rectShow = tempRect;
	//感兴趣区域预处理
	Mat img;
	img = src(tempRect);
	Mat imgGray;
	cvtColor(img, imgGray, CV_BGR2GRAY);
	Mat imgBin;
	threshold(imgGray, imgBin, 0, 255, THRESH_OTSU);
	//imshow("二值化：", imgBin);
	//waitKey(0);
	std::vector<std::vector<cv::Point>> contours;//检测到的轮廓
	std::vector<cv::Vec4i> hierarchy;//轮廓层次
	Mat imgContours;
	imgBin.copyTo(imgContours);//拷贝一份图像用于检测轮廓
	cv::findContours(imgContours, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
	if (contours.size() < 9)
	{
		cout << "error9   ";
		return false;
	}
	std::vector<cv::RotatedRect> contoursRect;
	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() < 150)
			continue;
		cv::RotatedRect tempRotateRect = minAreaRect(contours[i]);
		if (tempRotateRect.size.width < tempRotateRect.size.height)//如果九宫格中一宫格感兴趣区域的大矩形的宽度小于高度，交换他们
		{
			float temp = tempRotateRect.size.width;
			tempRotateRect.size.width = tempRotateRect.size.height;
			tempRotateRect.size.height = temp;
			tempRotateRect.angle += 90;
		}
		contoursRect.push_back(tempRotateRect);
	}
	if (contoursRect.size()<9)
	{
		cout << "error10   ";
		return false;
	}
	sort(contoursRect.begin(), contoursRect.end(), lessmark_rotateArea);//按面积降序排
	cv::RotatedRect temp = contoursRect[8];
	for (int i = 0; i < contoursRect.size(); i++)
	{
		float W = contoursRect[i].size.width;
		float H = contoursRect[i].size.height;
		//把条件放宽一些 顶多会识别错一两个，但是不会一个都识别不到
		if ((W<temp.size.width*0.5) || (W>temp.size.width*1.5) || (H<temp.size.height*0.5) || (H>temp.size.height*1.5))
		{
			contoursRect.erase(contoursRect.begin() + i);
			i--;
		}
	}
	if (contoursRect.size() < 9)
	{
		cout << "error11   ";
		return false;
	}
	else if (contoursRect.size() > 9)
	{
		sort(contoursRect.begin(), contoursRect.end(), lessmark_rotateArea);//再次按面积降序
																			//选取面积最大的九个
		contoursRect.erase(contoursRect.begin() + 9, contoursRect.end());
	}
	for (int i = 0; i < contoursRect.size(); i++)
	{
		cv::RotatedRect temprotateRect = contoursRect[i];
		temprotateRect.center.x = temprotateRect.center.x + tempRect.x;
		temprotateRect.center.y = temprotateRect.center.y + tempRect.y;
		gridRect.push_back(temprotateRect);
	}
	return true;
}

//计算火焰数字区域
bool sbol::calFireRect(const cv::Mat& src, std::vector<cv::Point2f>& pts, std::vector<cv::Rect>& fireRect)
{
	fireRect.clear();
	rectShow = cv::Rect(0, 0, 0, 0);
	//端点映射
	//端点映射
	std::vector<cv::Point> tempPoints;
	point_To_point(pts, tempPoints);
	//判断点是否在图像中
	for (int i = 0; i < tempPoints.size(); i++)
	{
		/*if ((tempPoints[i].x < 0) || (tempPoints[i].x >= src.cols) || (tempPoints[i].y < 0) || (tempPoints[i].y >= src.rows))
		{
			cout << "Fire_error7   ";
			return false;
		}*/
		if (tempPoints[i].x < 1)
			tempPoints[i].x = 1;
		if (tempPoints[i].x >(src.cols - 2))
			tempPoints[i].x = src.cols - 2;
		if (tempPoints[i].y< 1)
			tempPoints[i].y = 1;
		if (tempPoints[i].y >(src.rows - 2))
			tempPoints[i].y = src.rows - 2;
	}
	cv::Rect tempRect = boundingRect(tempPoints);
	//外扩1/10
	int rectX, rectY, rectW, rectH;
	rectX = tempRect.x;
	rectW = tempRect.width;
	rectY = tempRect.y - tempRect.height / 10;
	if (rectY < 0)
		rectY = 0;
	rectH = tempRect.height + tempRect.height / 5;
	if ((rectY + rectH)>src.rows)
		rectH = src.rows - rectY;
	if (rectH <= 0)
	{
		cout << "Fire_error8   ";
		return false;
	}
	tempRect = cv::Rect(rectX, rectY, rectW, rectH);
	rectShow = tempRect;
	//感兴趣区域预处理
	Mat img;
	img = src(tempRect);
	Mat imgRed;
	std::vector<cv::Mat> imgChannels;
	split(img, imgChannels);
	imgRed = imgChannels.at(2);
	Mat imgBin;
   threshold(imgRed, imgBin, 0, 255, THRESH_OTSU);
	cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));//构造核函数
	cv::erode(imgBin, imgBin, element1);
	cv::dilate(imgBin, imgBin, element1);
	cv::dilate(imgBin, imgBin, element1);
	cv::dilate(imgBin, imgBin, element1);
	//imshow("imgBin:", imgBin);
	std::vector<std::vector<cv::Point>> contours;//检测到的轮廓
	std::vector<cv::Vec4i> hierarchy;//轮廓层次
	Mat imgContours;
	imgBin.copyTo(imgContours);//拷贝一份图像用于检测轮廓
	cv::findContours(imgContours, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
	if (contours.size() < 9)
	{
		cout << "Fire_error9   ";
		return false;
	}
	std::vector<cv::Rect> contoursRect;
	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() < 100)
			continue;
		cv::Rect tempRect1 = boundingRect(contours[i]);
		double rateHW = (double)(tempRect1.height) / (double)(tempRect1.width);
		if (rateHW > 0.7&&rateHW < 2.8)
		{
			contoursRect.push_back(tempRect1);
		}
	}
	if (contoursRect.size() < 9)
	{
		cout << "Fire_error10   ";
		return false;
	}
	else if (contoursRect.size() > 9)
	{
		//选取面积最大的九个
		sort(contoursRect.begin(), contoursRect.end(), sortDown);
	}
	for (int i = 0; i < 9; i++)
	{
		cv::Rect tempRect2 = contoursRect[i];
		tempRect2.x = tempRect2.x + tempRect.x;
		tempRect2.y = tempRect2.y + tempRect.y;
		fireRect.push_back(tempRect2);
	}
	//imshow("Red_image:", imgRed);
	//imshow("Fire_image:", imgBin);
	return true;
}
//============= 阿拉伯数字识别 =============//
void sbol::numRecARab(const cv::Mat& src, std::vector<cv::RotatedRect>& numRects, std::vector<myResult>& vResult)
{
	vResult.clear();
	for (int i = 0; i < numRects.size(); i++)
	{
		int x = numRects[i].center.x - numRects[i].size.width / 2;
		if (x < 0)
			x = 0;
		int y = numRects[i].center.y - numRects[i].size.height / 2;
		if (y < 0)
			y = 0;
		int w = numRects[i].size.width;
		if ((x + w)>src.cols)//如果向外扩的矩形宽度大于原图像的列数（行宽）
			w = src.cols - x - 1;
		int h = numRects[i].size.height;
		if ((y + h)>src.rows)//如果向外扩的矩形高度大于原图像的行数（列高）
			h = src.rows - y - 1;
		if ((w <= 0) || (h <= 0))
			return;
		Mat img = src(Rect(x, y, w, h));
		Mat img_gray;
		cvtColor(img, img_gray, CV_BGR2GRAY);
		//矫正
		Point center = Point(w / 2, h / 2);
		double angle = numRects[i].angle;//求倾斜的角度，为做旋转
		Mat rotmat = getRotationMatrix2D(center, angle, 1);//用于获得变换矩阵rotmat,进行旋转
		Mat img_rotate;
		warpAffine(img_gray, img_rotate, rotmat, img_gray.size());//旋转平移

		threshold(img_rotate, img_rotate, 0, 255, THRESH_OTSU);
		EliminateEdge(img_rotate, img_rotate);//去除黑色边缘
		img_rotate = ~img_rotate;//把数字变成白的，背景变成黑的
		if (img_rotate.cols <= 15 || img_rotate.rows <= 15)
		{
			cout << "error12   ";
			return;
		}
		Mat imgRoi = img_rotate(Rect(5, 5, img_rotate.cols - 10, img_rotate.rows - 10));//向里面缩小了10个像素																			//imshow("13423", imgRoi);
		cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));//构造核函数
		cv::dilate(imgRoi, imgRoi, element);
		std::vector<std::vector<cv::Point>> contours;//检测到的轮廓
		std::vector<cv::Vec4i> hierarchy;//轮廓层次
		cv::Mat imgContours;
		img_rotate.copyTo(imgContours);//拷贝一份图像用于检测轮廓
		//imshow("img_rotate:", img_rotate);
		cv::findContours(imgContours, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, cv::Point(0, 0));
		//一般只检测到了一个外围轮廓
		std::vector<cv::Rect> contoursRect;
		for (int i = 0; i < contours.size(); i++)
		{
			if (contours[i].size() > 30)
				contoursRect.push_back(boundingRect(contours[i]));
		}
		if (!contoursRect.size())
		{
			cout << "error13   ";
			return;
		}
		sort(contoursRect.begin(), contoursRect.end(), sortDown); //面积降序比较，其实只有一个
		Mat img_num = img_rotate(contoursRect[0]);
		Mat img_test;
		img_num.copyTo(img_test);
		resize(img_test, img_test, Size(sizeX, sizeY)); //变成统一大小
		threshold(img_test, img_test, 245, 1, THRESH_BINARY);
		uchar DataBuff[216] = { 0 };
		int adr = 0;
		for (int a = 0; a < sizeY; a += 2)
		{
			for (int b = 0; b < sizeX; b += 2)
			{
				char cnt = 0;
				for (int n = 0; n < 2; n++)
				{
					for (int m = 0; m < 2; m++)
					{
						if (img_test.at<uchar>(a + n, b + m) == 1)
							cnt++;
					}
				}
				DataBuff[adr] = cnt;
				adr++;
			}
		}
		Mat tempImg(1, featureLen / 4, CV_32FC1, Scalar(0));
		float* p = (float*)tempImg.data;
		for (int n = 0; n < featureLen / 4; n++)
		{
			p[n] = (float)DataBuff[n];
		}
		int result = svm->predict(tempImg);
		myResult tempResult;
		tempResult.value = result;
		tempResult.position = Point(numRects[i].center.x, numRects[i].center.y);
		vResult.push_back(tempResult);
	}
	//排序
	int sum_x = 0;
	int sum_y = 0;
	Point center_point;
	for (int i = 0; i < vResult.size(); i++)
	{
		sum_x += vResult[i].position.x;
		sum_y += vResult[i].position.y;
	}
	center_point.x = (int)(sum_x / vResult.size());
	center_point.y = (int)(sum_y / vResult.size());
	int w = numRects[0].size.width / 2;
	int h = numRects[0].size.width / 2;
	for (int i = 0; i < vResult.size(); i++)
	{
		int t_err_x = vResult[i].position.x - center_point.x;
		int t_err_y = vResult[i].position.y - center_point.y;
		
		if (t_err_x < 0 && t_err_y < 0 && abs(t_err_x) > w && abs(t_err_y) > h)
			vResult[i].Index = 1;
		else if (t_err_y < 0 && abs(t_err_x) < w&&abs(t_err_y) > h)
			vResult[i].Index = 2;
		else if (t_err_x > 0 && t_err_y < 0 && abs(t_err_x) > w && abs(t_err_y) > h)
			vResult[i].Index = 3;
		else if (t_err_x < 0 && abs(t_err_x) > w&&abs(t_err_y) < h)
			vResult[i].Index = 4;
		else if (abs(t_err_x) < w&&abs(t_err_y) < h)
			vResult[i].Index = 5;
		else if (t_err_x > 0 && abs(t_err_x) > w&&abs(t_err_y) < h)
			vResult[i].Index = 6;
		else if (t_err_x < 0 && t_err_y > 0 && abs(t_err_x) > w && abs(t_err_y) > h)
			vResult[i].Index = 7;
		else if (t_err_y > 0 && abs(t_err_x) < w&&abs(t_err_y) > h)
			vResult[i].Index = 8;
		else if (t_err_x > 0 && t_err_y > 0 && abs(t_err_x) > w && abs(t_err_y) > h)
			vResult[i].Index = 9;
	}
	sort(vResult.begin(), vResult.end(), lessmark_myResult);
	for (int i = 0; i < 9; i++)
		ArrayResult[i] = vResult[i].value;
	return;
}
//消除边缘
void sbol::EliminateEdge(Mat &imgSrc, Mat &dst)
{
	for (int i = 0; i < dst.rows; i++)   //行循环
	{
		for (int j = 0; j < dst.cols; j++)
		{
			uchar* data = dst.ptr<uchar>(i);  //获取第i行的首地址
			if (data[j] == 0)
			{
				data[j] = 255;
			}
			else
			{
				break;
			}
		}
		for (int j = dst.cols - 1; j > -1; j--)
		{
			uchar* data = dst.ptr<uchar>(i);  //获取第i行的首地址
			if (data[j] == 0)
			{
				data[j] = 255;
			}
			else
			{
				break;
			}
		}
	}

	for (int i = 0; i < dst.cols; i++)   //列循环
	{
		for (int j = 0; j < dst.rows; j++)
		{
			uchar* data = dst.ptr<uchar>(j);  //获取第i行的首地址
			if (data[i] == 0)
			{
				data[i] = 255;
			}
			else
			{
				break;
			}
		}
		for (int j = dst.rows - 1; j > -1; j--)
		{
			uchar* data = dst.ptr<uchar>(j);  //获取第i行的首地址
			if (data[j] == 0)
			{
				data[j] = 255;
			}
			else
			{
				break;
			}
		}
	}
}

//============= 火焰数字识别 ===============
void sbol::numRecFire(const cv::Mat& src, std::vector<cv::Rect>& numRects, std::vector<myResult>& vResult)
{
	vResult.clear();
	for (int i = 0; i < numRects.size(); i++)
	{
		Mat img = src(numRects[i]);
		Mat imgGray;
		cvtColor(img, imgGray, CV_BGR2GRAY);
		Scalar meaValue = mean(imgGray);//求灰度图像的均值
		imgGray = imgGray - meaValue[0];//灰度图像-红色通道的灰度均值
		double maxValue;
		cv::minMaxLoc(imgGray, 0, &maxValue, 0, 0);//求图像的最大像素值
		Mat imgCanny;
		Canny(imgGray, imgCanny, maxValue, maxValue/4);//设置动态阈值
		//imshow("imgCanny:", imgCanny);
		Mat imgTest;
		imgCanny.copyTo(imgTest);
		cv::resize(imgTest, imgTest, cv::Size(24, 36), (0, 0), (0, 0), cv::INTER_LINEAR); //变成统一大小
		threshold(imgTest, imgTest, 100, 1, THRESH_BINARY);
		uchar DataBuff[216] = { 0 };
		int adr = 0;
		for (int a = 0; a < sizeY; a += 2)
		{
			for (int b = 0; b < sizeX; b += 2)
			{
				char cnt = 0;
				for (int n = 0; n < 2; n++)
				{
					for (int m = 0; m < 2; m++)
					{
						if (imgTest.at<uchar>(a + n, b + m) == 1)
							cnt++;
					}
				}
				DataBuff[adr] = cnt;
				adr++;
			}
		}
		Mat tempImg(1, featureLen / 4, CV_32FC1, Scalar(0));
		float* p = (float*)tempImg.data;
		for (int n = 0; n < featureLen / 4; n++)
		{
			p[n] = (float)DataBuff[n];
		}
		int result = svm2->predict(tempImg);
		myResult tempResult;
		tempResult.value = result;
		tempResult.position = Point(numRects[i].x + numRects[i].width / 2, numRects[i].y + numRects[i].height / 2);
		vResult.push_back(tempResult);
	}
	//排序
	int sum_x = 0;
	int sum_y = 0;
	Point center_point;
	for (int i = 0; i < vResult.size(); i++)
	{
		sum_x += vResult[i].position.x;
		sum_y += vResult[i].position.y;
	}
	center_point.x = (int)(sum_x / vResult.size());
	center_point.y = (int)(sum_y / vResult.size());
	int w = numRects[0].height;
	int h = numRects[0].height;
	for (int i = 0; i < vResult.size(); i++)
	{
		int t_err_x = vResult[i].position.x - center_point.x;
		int t_err_y = vResult[i].position.y - center_point.y;

		if (t_err_x < 0 && t_err_y < 0 && abs(t_err_x) > w && abs(t_err_y) > h)
			vResult[i].Index = 1;
		else if (t_err_y < 0 && abs(t_err_x) < w&&abs(t_err_y) > h)
			vResult[i].Index = 2;
		else if (t_err_x > 0 && t_err_y < 0 && abs(t_err_x) > w && abs(t_err_y) > h)
			vResult[i].Index = 3;
		else if (t_err_x < 0 && abs(t_err_x) > w&&abs(t_err_y) < h)
			vResult[i].Index = 4;
		else if (abs(t_err_x) < w&&abs(t_err_y) < h)
			vResult[i].Index = 5;
		else if (t_err_x > 0 && abs(t_err_x) > w&&abs(t_err_y) < h)
			vResult[i].Index = 6;
		else if (t_err_x < 0 && t_err_y > 0 && abs(t_err_x) > w && abs(t_err_y) > h)
			vResult[i].Index = 7;
		else if (t_err_y > 0 && abs(t_err_x) < w&&abs(t_err_y) > h)
			vResult[i].Index = 8;
		else if (t_err_x > 0 && t_err_y > 0 && abs(t_err_x) > w && abs(t_err_y) > h)
			vResult[i].Index = 9;
	}
	sort(vResult.begin(), vResult.end(), lessmark_myResult);
	for (int i = 0; i < 9; i++)
		ArrayResult[i] = vResult[i].value;
	return;
}
//============= 识别换页 ==================//
/*bool sbol::recPageChange(std::vector<myResult>& vResult, std::vector<myResult>& vlsResult, bool& flag)
{
	bool tempFlag = false;
	if (vResult.size() != 9)
		return tempFlag;
	int tempCnt = 0;
	for (int i = 0; i < vResult.size(); i++)
	{
		if (vResult[i].value != vlsResult[i].value)
			tempCnt++;
	}
	if ((tempCnt >= 0) && (tempCnt < 5)&&(flag == false))
	{
		return tempFlag;
	}
	if ((tempCnt >= 5) && (flag == false))
	{
		flag = true;
		vlsResult.clear();
		for (int i = 0; i < vResult.size(); i++)
		{
			vlsResult.push_back(vResult[i]);
		}
		return tempFlag;
	}
	if ((tempCnt <= 3) && (flag == true))
	{
		flag = false;
		vlsResult.clear();
		for (int i = 0; i < vResult.size(); i++)
		{
			vlsResult.push_back(vResult[i]);
		}
		tempFlag = true;
		cout << "换页   ";
		return tempFlag;
	}
}*/

//=============目标选择 ==================//
void sbol::chooseTarget(std::vector<myResult>& vResult, int psValue, myResult& target)
{
	if (vResult.size() != 9)
		return;
	int cnt = 0;
	for (int i = 0; i < vResult.size(); i++)
	{
		if (vResult[i].value == psValue)
			cnt++;
	}
	if (cnt == 0)
	{
		for (int i = 1; i < 10; i++)
		{
			int cnt1 = 0;
			for (int j = 0; j < 9; j++)
			{
				if (i == vResult[j].value)
				{
					cnt1++;
					if (cnt1 == 2)
					{
						target = vResult[j];
						return;
					}
				}
			}
		}
	}
	else
	{
		for (int i = 0; i < vResult.size(); i++)
		{
			if (vResult[i].value == psValue)
			{
				target = vResult[i];
				return;
			}
		}
	}
}

cv::Point2f sbol:: get_pattern_shot_3D(cv::Matx33d K, cv::Point2f target, Eigen::Matrix4d cam_shot, double depth)
{
	Eigen::Vector4d points;
	const double cx = K(0, 2);
	const double cy = K(1, 2);
	const double fx = K(0, 0);
	const double fy = K(1, 1);

	//得到pattern-cam坐标
	int u = target.x;
	int v = target.y;
	Eigen::Vector4d point;
	point[3] = 1;
	point[2] = depth;
	point[0] = (u - cx)*point[2] / fx;
	point[1] = (v - cy)*point[2] / fy;
	point[0] += 0.14;
	point[1] += 0.135;
	point[2] += 0.105;
	points= point;
	//得到pattern-shot坐标点
	//points = cam_shot * points;
	//得到角度
	cv::Point2f angle;
	angle.x = (float)(atan(points(0) / points(2)) / 3.1415926 * 180);
	angle.y = (float)(atan(points(1) / points(2)) / 3.1415926 * 180);
	return angle;
}

cv::Point2f sbol::calAngle(int Index, cv::Mat& r, cv::Mat& t)
{
	Eigen::Vector4d pointW;
	pointW[0] = posTarget[Index].x;
	pointW[1] = posTarget[Index].y;
	pointW[2] = posTarget[Index].z;
	pointW[3] = 1;

	Eigen::Vector3d rgb_origin_out(t.at<double>(0,0), t.at<double>(1, 0), t.at<double>(2, 0));
	Eigen::Matrix3d rgb_btm;
	rgb_btm << r.at<double>(0,0), r.at<double>(0, 1), r.at<double>(0, 2), r.at<double>(1, 0), r.at<double>(1, 1), r.at<double>(1, 2), r.at<double>(2, 0), r.at<double>(2, 1), r.at<double>(2, 2);
	cam_shot = EigenFrom3(rgb_origin_out, rgb_btm);
	Eigen::Vector4d pointC;
	pointC = cam_shot * pointW;
	//将摄像头坐标系原点平移到云台坐标系原点
	pointC[0] += 0.14;
	pointC[1] += 0.135;
	pointC[2] += 0.105;
	//计算云台角度
	cv::Point2f angle;
	angle.x = (float)(atan(pointC(0) / pointC(2)) / 3.1415926 * 180);
	angle.y =(float)(atan(pointC(1) / pointC(2)) / 3.1415926 * 180);
	return angle;
}
//============= 主函数调用函数 ================//
void sbol::BigPower()
{
	Mat ImageGray;
	cvtColor(sbolImage, ImageGray, CV_BGR2GRAY);
	Scalar meanValue = mean(ImageGray);
	meanGrayValue = meanValue(0);
	cout << "&&&&&&&&&&&&&&&&&&&& meanGrayValue:" << meanGrayValue << endl;
	//图像预处理
	imgPretreatment(sbolImage, preImage);
	if (findDigitronLightRoi(preImage, digitronLightRect))
	{
		//KNN识别密码
		if (recPassWord(sbolImage, digitronLightRect, psWord, observationPoints, R, T,Depth))
		{
			//重置击打顺序
			resetShootCnt(psWord, lstPsWord, observeCnt, errCnt, hitCnt, shootFlag);//重置打击顺序
			if (shootFlag == true)
			{
				if (SymbolModel == numArab)
				{
					if (calGirdRect(sbolImage, observationPoints, calibrateRects))
						numRecARab(sbolImage, calibrateRects, vrecResult);
					if (vrecResult.size() != 9)
						Depth = 0;
				}
				else if (SymbolModel == numFire)
				{
					if (calFireRect(sbolImage, observationPoints, calFireRects))//锁定数字识别区域
						numRecFire(sbolImage, calFireRects, vrecResult);
					if (vrecResult.size() != 9)
						Depth = 0;
				}
				if (recPageChange())//识别到了换页
				{
					//目标数字选择(更新目标)
					chooseTarget(vrecResult, psWord[hitCnt], tarGet);
					cv::Point2f tg = cv::Point2f(tarGet.position.x, tarGet.position.y);
					cv::Point2f Angle;
					cv::Matx33d tempK(K_);
					Angle = get_pattern_shot_3D(tempK, tg, cam_shot, Depth);
					shootCnt1++;
					if (shootCnt1 > 1)
					{
						angleY = Angle.x + corectAngY;
						angleP = Angle.y + corectAngP;
						shootCnt++;
						//angleY -= angleY / 10;
						//angleP -= angleP / 3;
						hitCnt++;
						if (hitCnt == 5)
							hitCnt = 0;
					}
					//Angle = calAngle(tarGet.Index-1, R, T);
				
				}
			}
		}
	}
	debugShow();
	std::cout << endl;
}

void sbol::debugShow()
{
	if (digitronLightRect.x >= 0)
	{
		//cv::rectangle(sbolImage, digitronLightRect, Scalar(0, 0, 255), 2);

		char c_number[] = "00000";
		for (int i = 0; i< 5; i++)
		{
			c_number[i] = psWord[i] + '0';
		}
		int passWordValue = atoi(c_number);	
		char s[30];
		sprintf_s(s, "PassWord:%d", passWordValue);
		putText(sbolImage, s, Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255), 2);
	}
	if (rectShow.width > 0)
	{
		cv::Rect tempRect = cv::Rect(rectShow.x, rectShow.y + 25, rectShow.width, rectShow.height);
		cv::rectangle(sbolImage, tempRect, Scalar(255, 0, 0), 2);
	}
	for (int i = 0; i < calibrateRects.size(); i++)
	{
		cv::Rect tempRect = calibrateRects[i].boundingRect();
		cv::rectangle(sbolImage, tempRect, Scalar(0, 0, 255), 2);
		if (vrecResult.size() == 9)
		{
			char s1[5];
			sprintf_s(s1, "%d", vrecResult[i].value);
			putText(sbolImage, s1, Point(vrecResult[i].position.x-tempRect.width/2 - 5, vrecResult[i].position.y - tempRect.height / 2 - 5), cv::FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255), 2);
		}
	}
	for (int i = 0; i < calFireRects.size(); i++)
	{
		cv::Rect tempRect = calFireRects[i];
		cv::rectangle(sbolImage, tempRect, Scalar(0, 0, 255), 2);
		if (vrecResult.size() == 9)
		{
			char s1[5];
			sprintf_s(s1, "%d", vrecResult[i].value);
			putText(sbolImage, s1, Point(vrecResult[i].position.x - tempRect.width / 2 - 5, vrecResult[i].position.y - tempRect.height / 2 - 5), cv::FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255), 2);
		}
	}
	for (int i = 0; i < observationPoints.size(); i++)
		circle(sbolImage, observationPoints[i], 2, Scalar(255, 0, 0), 2);
	imshow("momoDebug:", sbolImage);
}
//识别换页
bool sbol::recPageChange()
{
	bool flag = false;
	if (!pChangeFlag)
	{
		stopChange_cnt++;
		if (stopChange_cnt > 3)
		{
			int err_cnt_max = 0;
			int err_array[3] = { 0 };
			for (int i = 0; i < 9; i++)
			{
				if (ArrayResult[i] != ArrayResult1[i])
				{
					err_array[0]++;
				}
			}
			for (int i = 0; i < 9; i++)
			{
				if (ArrayResult[i] != ArrayResult2[i])
				{
					err_array[1]++;
				}
			}
			for (int i = 0; i < 9; i++)
			{
				if (ArrayResult[i] != ArrayResult3[i])
				{
					err_array[2]++;
				}
			}
			for (int i = 0; i < 3; i++)
			{
				if (err_array[i]>err_cnt_max)
				{
					err_cnt_max = err_array[i];
				}
			}
			if (err_cnt_max > 4)
			{
				pChangeFlag = true;
			}
		}
	}
	else
	{
		int err_cnt = 0;
		for (int i = 0; i < 9; i++)
		{
			if (ArrayResult[i] != ArrayResult1[i])
			{
				err_cnt++;
			}
		}
		if (err_cnt <=3)
		{
			flag = true;
			pChangeFlag = false;
			stopChange_cnt = 0;
			cout<< "======================= 换页了！" << endl;
		}
	}
	for (int i = 0; i < 9; i++)
	{
		ArrayResult3[i] = ArrayResult2[i];
		ArrayResult2[i] = ArrayResult1[i];
		ArrayResult1[i] = ArrayResult[i];
	}
	return flag;
}








