#include <opencv2\opencv.hpp>
#include <vector>
#include <iostream>
#include <algorithm>
#include <windows.h>
#include <chrono>

//以图片为输入源
//#define PICTURE   

//以视频流为输入源
#define VIDEO_STREAM

//摄像头/采集卡输入
#define CAMERA

//从视频文件读入
//#define VIDEO_FILE

//摄像头/采集卡编号(一般情况下自带摄像头为0, 外接采集卡或摄像头为1)
#define CAMERA_NUM 1

//大于面积阈值(frame_width * width_height * AREA_THRESH)的颜色范围才会被当作可能的目标
#define AREA_THRESH   0.0003

#define RESIZE_BEFORE_PROCESS

using namespace std;
using namespace cv;

Mat ColorProcess(Mat _frame);
vector<Point> DetectContour(Mat src);
bool SortByContourArea(vector<int>& _idx, vector<float>& _areas, vector<vector<Point>>& _contours);

bool save = false;
const string detected_folder = "./detected/";
const string no_target_folder = "./none";
const string video_file = "test4.mp4";


struct AreaCmp
{
	AreaCmp(const vector<float>& _area) : area(&_area) {}
	const vector<float>* area;
	bool operator() (int a, int b) const
	{
		return (*area)[a] > (*area)[b];
	}
};

int main()
{
	Mat frame;
#ifdef PICTURE
	frame = imread("F:\\source\\repos\\opencv\\color_detect\\test4.jpg");
#endif

#ifdef VIDEO_STREAM
	VideoCapture cap;
	LARGE_INTEGER freq, start, end;
	QueryPerformanceFrequency(&freq);

#ifdef VIDEO_FILE
	cap.open(video_file);
	if (!cap.isOpened())
	{
		cout << "Can not open video file" << endl;
		exit(1);
	}
#endif

#ifdef CAMERA
	for (int cn = CAMERA_NUM; cn >= 0; cn--)
	{
		cap.open(cn);
		if (!cap.isOpened())
		{
			if (cn == 0)
			{
				cout << "can not find camera." << endl;
				exit(1);
			}
			continue;
		}
		else
		{
			cap.set(3, 1920);
			cap.set(4, 1080);
			break;
		}
	}

#endif

#ifdef VIDEO_STREAM
	VideoWriter outputVideo;
	cv::Size S = cv::Size((int)cap.get(CV_CAP_PROP_FRAME_WIDTH),
		(int)cap.get(CV_CAP_PROP_FRAME_HEIGHT));
	outputVideo.open("./save.mp4", CV_FOURCC('P', 'I', 'M', '1'), 30.0, S, true);
#endif

	while (1)
	{
		cap >> frame;
		outputVideo << frame;
		if (!frame.data) break;
#endif
		chrono::milliseconds timestamp = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
		QueryPerformanceCounter(&start);
#ifdef RESIZE_BEFORE_PROCESS
		resize(frame, frame, Size(0, 0), 0.5, 0.5);
#endif
		Mat ImageBinary = ColorProcess(frame);
		vector<Point> mainContour = DetectContour(ImageBinary);
		imshow("ori", frame);
		imshow("target", ImageBinary);
		if (save && mainContour.size() > 0)
		{

			static int cnt1 = 1;
			char imgName[50];
			long long count = timestamp.count();
			sprintf_s(imgName, (detected_folder + to_string(count) + ".jpg").c_str(), cnt1++);
			imwrite(imgName, frame);
			cout << "save " << imgName << " done" << endl;

#ifdef PICTURE
			imwrite("F:\\source\\repos\\opencv\\color_detect\\detected.jpg", frame);
#endif
		}
		QueryPerformanceCounter(&end);
		double consume = (end.QuadPart - start.QuadPart) / ((double)freq.QuadPart / 1000);
		cout << "process time: " << consume << "ms" << endl;
#ifdef VIDEO_STREAM
		int key = waitKey(1);
		if (key == 32)
		{
			if (!save)
				cout << "--------------开始保存----------------" << endl;
			else
				cout << "--------------停止保存----------------" << endl;
			save = !save;
		}
		if (key == 27)
			break;
#endif

#ifdef VIDEO_STREAM
	}
#endif
	getchar();
	return 0;
}

vector<Point> DetectContour(Mat input)
{
	vector<Point> contour;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	vector<int> sortIdx;
	vector<float> areas;
	//Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
	//morphologyEx(input, input, MORPH_CLOSE, kernel);
	findContours(input, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
	if (SortByContourArea(sortIdx, areas, contours))
	{
		double area = contourArea(contours[sortIdx[0]], false);
		cout << "target detected     area: " << area << endl;
		if(area > input.cols * input.rows * AREA_THRESH)
			contour = contours[sortIdx[0]];
	}
	return contour;
}

Mat ColorProcess(Mat _frame)
{
	Mat imgHsv;
	cvtColor(_frame, imgHsv, COLOR_BGR2HSV);
	vector<Mat> splitHSV;
	split(imgHsv, splitHSV);
	equalizeHist(splitHSV[2], splitHSV[2]);
	merge(splitHSV, imgHsv);
	Mat mask1, mask2, mask;
	Scalar low = Scalar(170, 43, 47);
	Scalar high = Scalar(180, 255, 255);
	inRange(imgHsv, low, high, mask1);
	low = Scalar(0, 43, 47);
	high = Scalar(3, 255, 255);
	inRange(imgHsv, low, high, mask2);
	mask = mask1 | mask2;
	return mask;
}

bool SortByContourArea(vector<int>& _idx, vector<float>& _areas, vector<vector<Point>>& _contours)
{
	int contours_num = _contours.size();
	if (contours_num == 0) return false;
	_idx.resize(contours_num);
	_areas.resize(contours_num);
	for (int i = 0; i < contours_num; i++)
	{
		_idx[i] = i;
		_areas[i] = contourArea(_contours[i], false);
	}
	sort(_idx.begin(), _idx.end(), AreaCmp(_areas));
	return true;
}