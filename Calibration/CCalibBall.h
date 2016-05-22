#ifndef CCalibBall_H
#define CCalibBall_H

#include <imgproc.hpp>
#include <highgui.hpp>
#include <core.hpp>
#include <calib3d.hpp>
#include <stdio.h>
#include <time.h>
#include <map>
#include <iostream>
#include <assert.h> 
#include "cvsba.h"
using namespace cv;
using namespace std;

#define square(x) ((x)*(x))

#if _DEBUG
	#pragma comment(lib, "opencv_lib/opencv_core245d.lib")
 	#pragma comment(lib, "opencv_lib/opencv_highgui245d.lib")
 	#pragma comment(lib, "opencv_lib/opencv_imgproc245d.lib")
	#pragma comment(lib, "opencv_lib/opencv_calib3d245d.lib")
	#pragma comment(lib, "cvsba_lib/libf2cd.lib")
	#pragma comment(lib, "cvsba_lib/blasd.lib")
	#pragma comment(lib, "cvsba_lib/lapackd.lib")
	#pragma comment(lib, "cvsba_lib/cvsbad.lib")
#else
	#pragma comment(lib, "opencv_lib/opencv_core245.lib")
 	#pragma comment(lib, "opencv_lib/opencv_highgui245.lib")
	#pragma comment(lib, "opencv_lib/opencv_imgproc245.lib")
	#pragma comment(lib, "opencv_lib/opencv_calib3d245.lib")
	#pragma comment(lib, "cvsba_lib/libf2c.lib")
	#pragma comment(lib, "cvsba_lib/blas.lib")
	#pragma comment(lib, "cvsba_lib/lapack.lib")
	#pragma comment(lib, "cvsba_lib/cvsba.lib")
#endif

struct Kinect
{
	string img_name;
	Mat DepthCameraIntrinsic;
	Mat ColorCameraIntrinsic;
	Mat ColorCameraExtrinsic;
	Mat R1t_times_R;
	Kinect(string name){img_name = name;}
};
class CCalibBall
{
public:
	int cam_num;
	int kinect_num;
	int cam_used_num;
	int code_num;
	string code;
	string kinect_calib_name, camera_calib_name;
	vector<int> cam_list;
	vector<Kinect> kinect;
	string filepath;
	int NPoints;
	vector<string> backgroundlist;
	vector<string> imagelist_vector;
	double radii;				//大球的直径
	double DF;					//角点间距
	vector<double> focal_length;		//初始焦距
	double init_cx, init_cy;
	string IsRotated;
	int IsAuto;

	CCalibBall(char config_name[]);
	~CCalibBall();
	void run(char config_name[]);
private:
	int run_once(const vector<string> &imagelist, vector< Point3d> &points, vector< vector < Point2d > > &imagePoints, vector< vector< int > > &visibility, vector< Mat > &R, vector< Mat > &T, vector<Mat> cameraMatrix);
	void FindPoints(const vector<string> &imagelist, vector< Mat > &spheres, vector< vector< vector<Point2f> > > &pointsImg, vector< vector< vector<Point3d> > > &points3D, vector< vector <Point2f> > &MarkerPosition, int *first_label);
	void FindBigCircle(Mat background, Mat &ref, Mat &BigMarker,Mat &mask_dst,  Vec3d &circle, int idx);
	void FindCorners(Mat diff_image, vector< vector<Point2f> > &pointsImg, int idx);
	void Cal3D(Vec3d circles, Mat &sphere, vector< vector<Point2f> > pointsImg, vector< vector<Point3d> > &points3D, double F);
	void RANSAC(vector< Matx33d > points3D_src, vector< Matx33d > &points3D_dst, Mat &R, vector<Point3d> &point3d_final, vector<int> &visibility_src, vector<int> &visibility_dst);
	void Two_Pass(const Mat& _binImg, Mat& _lableImg);
	void Points3DToMat(vector< vector<Point3d> > points3D, vector<Matx33d> &points);
	void ProcessForSba(vector<vector<Point2f>> pointImg_src, vector<Point2d> &pointImg_dst, vector< Point3d> points, vector< Matx33d > points3D_normed, vector<int> &visibility);
	void OutputParam(vector< Mat > cameraMatrix, vector< Mat > R, vector< Mat > T, vector< Mat > distCoeffs);
	void ProjectToImg(const vector<string> &imagelist, vector< Mat > cameraMatrix, vector< Mat > R, vector< Mat > T, vector< Point3d> points, vector<vector< Point2d >> imagePoints, vector<vector<int>> visibility);
	void ScaleToWorld(vector< Point3d> &points, vector< Mat > &T);
	void KinectMarker(Mat BigMarker, Mat mask, vector< Point2f > &MarkerPosition, int &first_label, int idx);
	bool RansacLine(vector<Point2f> src, vector<Point2f> &dst, double threshold);
	void FetchKinect(int idx, vector<Point3f> AllPosition, FileStorage fs);
	void MapCoordinate(vector< Mat > P, vector< vector <Point2f> > MarkerPosition, int *first_label, vector< Point3f > &AllPosition);
	
};
void fillEdgeImage(Mat edgesIn, Mat& filledEdgesOut);
void DetectCircle(Mat image, Vec3d &circle_, double minCircleRadius, double maxCircleRadius);
void AutoThres(Mat src, Mat &dst);
void imfillholes(Mat src, Mat &dst);

#endif
