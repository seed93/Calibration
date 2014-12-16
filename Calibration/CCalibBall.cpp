#include "CCalibBall.h"

#define IS_OUTPUT //是否输出投影结果
//#define IS_OUTPUT2 //是否输出三维点
//#define IS_OUTPUT_CIRCLE
#define SCALE 2
#define CODE_LEN 5
#define MAX_COUNT 1000
#define PI 3.14159265354

inline int strindex(const char *str1,char *str2)
{
	int end,i,j;
	end=int(strlen(str1)-strlen(str2));
	if(end>0)
	{
		for(i=0;i<=end;i++)
		{
			for(j=i;str1[j]==str2[j-i];j++)
				if(str2[j-i+1]=='\0')
					return i;
		}
	}
	return -1;
}

CCalibBall::CCalibBall(char config_name[])
{
	FileStorage fp(config_name, FileStorage::READ);
	if (fp.isOpened() == false)
	{
		printf("cannot open file %s\n", config_name);
		return ;
	}
	fp["filepath"]>>filepath;
	fp["code"]>>code;
	code_num = (int)code.length()-5;
	fp["auto"]>>IsAuto;
	fp["imagelist"]>>imagelist_vector;
	fp["backgroundlist"]>>backgroundlist;

	FileNodeIterator sn = fp["kinect_sn"].begin();
	Kinect temp_kinect;
	for (;sn!=fp["kinect_sn"].end(); sn++)
	{
		temp_kinect.sn = (int)*sn;
		kinect.push_back(temp_kinect);
	}
	kinect_num = (int)kinect.size();

	string line;
	fp["kinect_info"]>>line;
	FileStorage fs(line, FileStorage::READ);
	if (fs.isOpened() == false)
	{
		printf("cannot open file %s\n", line.c_str());
		return ;
	}
	for (int i=0; i<kinect_num; i++)
	{
		char pp[MAX_COUNT];
		sprintf(pp, "ColorCameraExtrinsic-%d", kinect[i].sn);
		fs[pp]>>kinect[i].ColorCameraExtrinsic;
		sprintf(pp, "ColorCameraIntrinsic-%d", kinect[i].sn);
		fs[pp]>>kinect[i].ColorCameraIntrinsic;
		sprintf(pp, "DepthCameraIntrinsic-%d", kinect[i].sn);
		fs[pp]>>kinect[i].DepthCameraIntrinsic;	
	}
	fp["kinect_calib_name"]>>kinect_calib_name;
	fp["camera_calib_name"]>>camera_calib_name;

	fp["isrotated"]>>IsRotated;
	fp["camPair"]>>line;
	cam_used_num = int(line.length());
	cam_list.resize(cam_used_num);
	for (int i=0; i<cam_used_num; i++)
	{
		cam_list[i] = line[i]-'0';
	}
	
	fp["radii"]>>radii;
	fp["DF"]>>DF;
	fp["init_cx"]>>init_cx;
	fp["init_cy"]>>init_cy;
	fp["initfc"]>>focal_length;
}

void CCalibBall::run(char config_name[])
{
	clock_t start = clock();
	FileStorage fp(config_name, FileStorage::READ);
	if (fp.isOpened() == false)
	{
		printf("cannot open file %s\n", config_name);
		return ;
	}
	vector< Mat > cameraMatrix(cam_used_num),distCoeffs(cam_used_num);
	for (int i=0; i<cam_used_num; i++)
	{
		cameraMatrix[i] = Mat::zeros(3,3,CV_64FC1);
		cameraMatrix[i].at<double>(0,2) = init_cx;
		cameraMatrix[i].at<double>(1,2) = init_cy;
		cameraMatrix[i].at<double>(0,0) = focal_length[cam_list[i]];
		cameraMatrix[i].at<double>(1,1) = focal_length[cam_list[i]];
		cameraMatrix[i].at<double>(2,2) = 1;
		distCoeffs[i] = Mat::zeros(5,1,CV_64FC1);
	}
	NPoints = 0;
	vector<Point3d> points;
	vector<vector<Point2d>> imagePoints(cam_used_num);
	vector<vector<int>> visibility(cam_used_num);
	vector<Mat> R(cam_used_num), T(cam_used_num);
	Mat T_base;
	for (int iv=0; iv<imagelist_vector.size(); iv++)
	{
		vector<string> current_imagelist(cam_used_num);
		fp[imagelist_vector[iv]]>>current_imagelist;
		vector<Point3d> current_points;
		vector<vector<Point2d>> current_imagePoints(cam_used_num);
		vector<vector<int>> current_visibility(cam_used_num);
		NPoints += run_once(current_imagelist, current_points, current_imagePoints, current_visibility, R, T, cameraMatrix);
		for (int i=0; i<cam_used_num; i++)
		{
			size_t current_npoints = current_points.size();
			for (int j=0; j<current_npoints; j++)
			{
				imagePoints[i].push_back(current_imagePoints[i][j]);
				visibility[i].push_back(current_visibility[i][j]);
			}
		}
		Point3d T_temp;
		if (iv == 0)
		{
			T_base = T[0].clone();
			T_temp.x = 0;
			T_temp.y = 0;
			T_temp.z = 0;
		}
		else
		{
			Mat T_new = T_base-T[0];
			T_temp.x = T_new.at<double>(0,0);
			T_temp.y = T_new.at<double>(1,0);
			T_temp.z = T_new.at<double>(2,0);
		}
		for (int i=0; i<current_points.size(); i++)
			points.push_back(current_points[i]+T_temp);
	}
	cvsba::Sba sba;
	cvsba::Sba::Params params ;
	params.type = cvsba::Sba::MOTIONSTRUCTURE;
	params.iterations = 200;
	params.minError = 1e-10;
	params.fixedIntrinsics = 1;
	params.fixedDistortion = 5;
	params.verbose = 0;
	sba.setParams(params);
	sba.run(points, imagePoints, visibility, cameraMatrix, R, T, distCoeffs);
	cout<<"Optimization. Initial error="<<sba.getInitialReprjError()<<" and Final error="<<sba.getFinalReprjError()<<std::endl;
	ScaleToWorld(points,T);
	OutputParam(cameraMatrix, R, T, distCoeffs);
	printf("time: %f\n", double(clock()-start)/1000);

	
// 	vector< Mat > P(cam_used_num);
// 	for (int i=0; i<cam_used_num; i++)
// 	{
// 		P[i] = Mat(3,4,CV_64FC1);
// 		R[i].copyTo(P[i].colRange(0,3));
// 		T[i].copyTo(P[i].col(3));
// 		P[i] = cameraMatrix[i]*P[i];
// 	}
// 	vector< Point3f > AllPosition;
// 	MapCoordinate(P, MarkerPosition, first_label, AllPosition);
// 	FILE *fp = fopen("position.txt", "w");
// 	for (int i=0; i<AllPosition.size(); i++)
// 	{
// 		fprintf(fp, "%f %f %f\n", AllPosition[i].x, AllPosition[i].y, AllPosition[i].z);
// 	}
// 	fclose(fp);
// 
// 	FileStorage fs(kinect_calib_name, FileStorage::WRITE);
// 	for (int i=0; i<kinect_num; i++)
// 	{
// 		FetchKinect(i, AllPosition, fs);
// 	}
// 

//	printf("time: %f\n", double(clock()-start)/1000);
}

int CCalibBall::run_once(const vector<string> &imagelist, vector<Point3d> &points, vector<vector<Point2d>> &imagePoints, vector<vector<int>> &visibility, vector<Mat> &R, vector<Mat> &T, vector<Mat> cameraMatrix)
{
	vector< vector< vector<Point2f> > > pointsImg(cam_used_num);
	vector< vector< vector<Point3d> > > points3D(cam_used_num);
	vector< vector <Point2f> > MarkerPosition(cam_used_num);
	int *first_label = new int [cam_used_num];
	FindPoints(imagelist, T, pointsImg, points3D, MarkerPosition, first_label);
	R[0] = Mat::eye(3, 3, CV_64FC1);
	vector<vector< Matx33d >> points3D_normed(cam_used_num);
	Points3DToMat(points3D[0], points3D_normed[0]);
	visibility[0].resize(points3D_normed[0].size());
	for (int i=0; i<visibility[0].size(); i++)
		visibility[0][i] = -1;
	for (int i=1; i<cam_used_num; i++)
	{
		Points3DToMat(points3D[i], points3D_normed[i]);
		RANSAC(points3D_normed[i-1], points3D_normed[i], R[i], points, visibility[i-1], visibility[i]);
	}
	for (int i=0; i<cam_used_num; i++)
		ProcessForSba(pointsImg[i], imagePoints[i], points, points3D_normed[i], visibility[i]);
	ProjectToImg(imagelist, cameraMatrix, R, T, points, imagePoints, visibility);
 	delete [] first_label;
	return int(points.size());
}


void CCalibBall::FindPoints(const vector<string> &imagelist, vector< Mat > &spheres, vector< vector< vector<Point2f> > > &pointsImg, vector< vector< vector<Point3d> > > &points3D, vector< vector <Point2f> > &MarkerPosition, int *first_label)
{
	vector< Vec3d > circles(cam_used_num);
#pragma omp parallel for
	for (int i=0; i<cam_used_num; i++)
	{
		Mat origin = imread(filepath+backgroundlist[cam_list[i]]);
		if (origin.rows == 0 || origin.cols == 0)
		{
			printf("%s read error\n", (filepath+backgroundlist[cam_list[i]]).c_str());
			exit(-1);
		}
		Mat ref = imread(filepath+imagelist[cam_list[i]]);
		if (IsRotated[cam_list[i]]=='1')
		{
			flip(origin, origin, -1);
			flip(ref, ref, -1);
		}
		Mat BigMarker, mask;
// 		imwrite("background.jpg", origin);
// 		imwrite("ref.jpg",ref);
		FindBigCircle(origin, ref, BigMarker, mask, circles[i], i);
//		KinectMarker(BigMarker, mask, MarkerPosition[i], first_label[i], i);
		FindCorners(ref, pointsImg[i], i);
		Cal3D(circles[i], spheres[i], pointsImg[i], points3D[i], focal_length[cam_list[i]]);
	}
}

void CCalibBall::FindBigCircle(Mat background, Mat &ref, Mat &BigMarker, Mat &mask_dst, Vec3d &circle_, int idx)
{
	int minR=700;
	int maxR=1200;
	Mat gray;
	absdiff(background,ref,gray);
	cvtColor(gray, gray, CV_BGR2GRAY);
//	medianBlur(gray,gray,3);
	GaussianBlur( gray, gray, Size(25, 25), 2, 2);
	gray = gray > 10;
	Mat element = getStructuringElement( MORPH_ELLIPSE, Size(20, 20));
	morphologyEx(gray, gray, MORPH_CLOSE, element, Point(-1,-1), 2);
	vector<vector<Point> > contours;
	findContours(gray, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0,0));
	double max_area = 0;
	int max_ID = -1;
	for (int i=0; i<contours.size(); i++)
	{
		double area = contourArea(contours[i]);
		if (area>max_area)
		{
			max_area = area;
			max_ID = i;
		}
	}
	gray.setTo(0);
	drawContours( gray, contours, max_ID, Scalar(255) );
	DetectCircle(gray, circle_, minR, maxR);
	Point2i bestCircleCenter(cvRound(circle_[0]), cvRound(circle_[1]));	
	Mat mask(ref.size(), CV_8UC1, Scalar(0));
	circle(mask, bestCircleCenter, cvRound(circle_[2])-120, Scalar(255), CV_FILLED);
	Mat dst;
	ref.copyTo(dst, mask);

	mask.setTo(0);
	drawContours( mask, contours, max_ID, Scalar(255), CV_FILLED );
	circle(mask, bestCircleCenter, cvRound(circle_[2]), Scalar(0), CV_FILLED);
	element = getStructuringElement( MORPH_ELLIPSE, Size(10, 10));
	erode(mask, mask, element);
	findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0,0));
	max_area = 0;
	max_ID = -1;
	for (int i=0; i<contours.size(); i++)
	{
		double area = contourArea(contours[i]);
		if (area>max_area)
		{
			max_area = area;
			max_ID = i;
		}
	}
	mask.setTo(0);
	drawContours( mask, contours, max_ID, Scalar(255), CV_FILLED );
 	ref.copyTo(BigMarker, mask);
	cvtColor(BigMarker, BigMarker, CV_BGR2GRAY);
	mask_dst = mask;
#ifdef IS_OUTPUT_CIRCLE
	char filename[512];
	sprintf(filename, "out//temp_%d.jpg", idx);
	imwrite(filename, gray);
	sprintf(filename, "out//out_%d.jpg", idx);
	Mat ref1 = ref.clone();
	circle(ref1, bestCircleCenter, cvRound(circle_[2]), Scalar(255,255,255), 5);
	imwrite(filename, ref1);
#endif
	ref = dst;
}

void CCalibBall::FindCorners(Mat image, vector< vector<Point2f> > &pointsImg, int idx)
{
	Mat element;
	Mat image_gray, image_small;
	resize(image, image_small, Size(), 1.0/SCALE, 1.0/SCALE);
	Mat image_back = image_small.clone();
	cvtColor(image_small, image_gray, CV_BGR2GRAY);
	cvtColor(image_small, image_small, CV_BGR2HSV);
	Mat mask;
	if (IsAuto)
	{
		mask = image_gray != 0;
		vector<Mat> hsv(image_small.channels());
		split(image_small, hsv);
		Mat gray = hsv[0];								//liuxiaoyang

		element = getStructuringElement( MORPH_ELLIPSE, Size(5, 5));

		GaussianBlur( gray, gray, Size(5, 5), 4, 4);
		morphologyEx(gray, gray, MORPH_OPEN, element, Point(-1,-1), 2);
		AutoThres(gray, mask);
		erode(mask, mask, element);
		fillEdgeImage(mask, mask);
		element = getStructuringElement( MORPH_ELLIPSE, Size(10, 10));
		erode(mask, mask, element);
	}
	else
	{
		char fname[100];
		sprintf(fname, "out//%d_mask.jpg", idx);		//liuxiaoyang
		mask = imread(fname,false);					//liuxiaoyang
		if (mask.rows == 0)
		{
			printf("unable to read image %s\n", fname);
			exit(-1) ;
		}
	}

 	
	
// 	element = getStructuringElement( MORPH_ELLIPSE, Size(10, 10));
// 	morphologyEx(mask, mask, MORPH_OPEN, element, Point(-1,-1), 2);
// 	medianBlur(image_gray,image_gray,3);
// 	medianBlur(image_gray,image_gray,3);
	vector<Point2f> corners;
	goodFeaturesToTrack(image_gray, corners, 100, 0.1, 60/SCALE, mask, 5);
	Mat labelImg;
	Mat mask1(mask.size(), CV_8UC1, Scalar(1));
	Mat temp4label;
	mask1.copyTo(temp4label, mask);
	mask.copyTo(mask1);
	element = getStructuringElement( MORPH_ELLIPSE, Size(8, 8));
	erode(mask1, mask1, element);
	vector<Point> corners_new;
	for (int i=0; i<corners.size(); i++)
		if (mask1.at<uchar>(int(corners[i].y), int(corners[i].x)) == 255)
			corners_new.push_back(corners[i]);
	corners.clear();
	Two_Pass(temp4label, labelImg);
	vector<vector<int>> corner_label;
	map<int,int> label;
	int k=0;
	for (int i=0; i<corners_new.size(); i++)
	{
		int temp_label = labelImg.at<int>(corners_new[i].y, corners_new[i].x);
		if (label.count(temp_label) <= 0)
		{
			label[temp_label] = k++;
			vector<int> temp_vec;
			temp_vec.push_back(i);
			corner_label.push_back(temp_vec);
		}
		else corner_label[label[temp_label]].push_back(i);
	}
	TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 ) ;
	Mat I;
	cvtColor(image, I, CV_BGR2GRAY);
	Point2f p(2.5, 2.5);
	float ratio = 0.1f;
	for (int i=0; i<corner_label.size(); i++)
	{
		if (corner_label[i].size() != 2)
			continue;
		vector<Point2f> temp(2);
		temp[0] = corners_new[corner_label[i][0]];
		temp[1] = corners_new[corner_label[i][1]];
		cornerSubPix(image_gray, temp , Size(7,7), Size(-1,-1), criteria);
		temp[0] = temp[0]*SCALE+p;
		temp[1] = temp[1]*SCALE+p;
		cornerSubPix(I, temp , Size(5,5), Size(-1,-1), criteria);
		Point2f temp_point = (1-ratio)*temp[0]+ratio*temp[1];
		Point2f temp_point2 = (temp[1]-temp[0])*ratio;
		Point2f point_new1(temp_point.x-temp_point2.y, temp_point.y+temp_point2.x);
		Point2f point_new2(temp_point.x+temp_point2.y, temp_point.y-temp_point2.x);
		if (I.at<uchar>(point_new1) > I.at<uchar>(point_new2))
		{
			Point2f temp_swap = temp[0];
			temp[0] = temp[1];
			temp[1] = temp_swap;
		}
		pointsImg.push_back(temp);
#ifdef IS_OUTPUT
		circle(I, temp[0], 3, Scalar(255), CV_FILLED);
		circle(I, temp[1], 3, Scalar(0), CV_FILLED);
#endif
	}
#ifdef IS_OUTPUT
	char filename[512];
	sprintf(filename, "out//%d.jpg", idx);
	imwrite(filename, I);
	if (IsAuto)
	{
		sprintf(filename, "out//%d_mask.jpg", idx);
		imwrite(filename, mask1);
	}
#endif
}

void CCalibBall::Cal3D(Vec3d circles, Mat &sphere, vector< vector<Point2f> > pointsImg, vector< vector<Point3d> > &points3D, double F)
{
	double scale = radii/circles[2];
	sphere = Mat(3,1,CV_64FC1);
	sphere.at<double>(0,0) = scale*(circles[0]-init_cx);
	sphere.at<double>(1,0) = scale*(circles[1]-init_cy);
	sphere.at<double>(2,0) = scale*F;
	points3D.resize(pointsImg.size());
	double r_square = square(radii);
	for (int i=0; i<pointsImg.size(); i++)
	{
		points3D[i].resize(2);
		for (int j=0; j<2; j++)
		{
			double px_img = pointsImg[i][j].x;
			double py_img =  pointsImg[i][j].y;
			points3D[i][j].z = - sqrt(r_square - square(scale) * ( square(px_img-circles[0]) + square(py_img-circles[1]) ) );
			points3D[i][j].x = (points3D[i][j].z + sphere.at<double>(2,0))*(px_img-init_cx)/F - sphere.at<double>(0,0);
			points3D[i][j].y = (points3D[i][j].z + sphere.at<double>(2,0))*(py_img-init_cy)/F - sphere.at<double>(1,0);
		}
	}
}

void CCalibBall::RANSAC(vector< Matx33d > points3D_src, vector< Matx33d > &points3D_dst, Mat &R, vector<Point3d> &point3d_final, vector<int> &visibility_src, vector<int> &visibility_dst)
{
	double threshold = 0.15;
	int src_size = int(points3D_src.size());
	int dst_size = int(points3D_dst.size());
	int max_inliers = -1, best_i, best_j;
	double min_total_error = 1e6;
	for (int i=0; i<src_size; i++)
	{
		for (int j=0; j<dst_size; j++)
		{
			Matx33d test =  points3D_dst[j] * points3D_src[i].t();
			Mat S,U,VT;
			SVD::compute(test,S,U,VT, SVD::FULL_UV);
			Matx33d R_test = Mat(U*VT);
			int inliers = 0;
			double total_error = 0;
			for (int l=0; l<src_size; l++)
			{
				if (l==i) continue;
				Matx33d new_src = R_test * points3D_src[l];
				double min_error = 1e6;
				for (int k=0; k<dst_size; k++)
				{
					if (k==j) continue;
					double error = MAX(norm(new_src.col(0)-points3D_dst[k].col(0)), norm(new_src.col(1)-points3D_dst[k].col(1)));
					if (min_error > error) min_error = error;
				}
				if (min_error < threshold)
				{
					inliers++;
					total_error+=min_error;
				}
			}
			if ((inliers == max_inliers && min_total_error > total_error) || inliers > max_inliers )
			{
				min_total_error = total_error;
				max_inliers = inliers;
				R = Mat(R_test);
				best_i = i;
				best_j = j;
			}
		}
	}
	if (max_inliers == -1)
	{
		printf("no inliers!!!\n");
		exit(0);
	}	
	Mat R_t = R.t();
	Matx33d R_temp = R_t;
	double total_error = 0;
	int points_end_idx = int(point3d_final.size())/2;
	visibility_dst.resize(dst_size);
	for (int k=0; k<dst_size; k++)
	{
		if (k==best_j)
		{
			if (visibility_src[best_i] == -1)
			{
				for (int i=0; i<2; i++)
				{
					Mat u = radii*Mat(points3D_src[best_i].col(i));
					Point3d p(u.at<double>(0,0), u.at<double>(1,0), u.at<double>(2,0));
					point3d_final.push_back(p);
				}
				visibility_src[best_i] = points_end_idx;
				points_end_idx++;
			}
			visibility_dst[best_j] = visibility_src[best_i];
			continue;
		}
		Matx33d new_dst = R_temp * points3D_dst[k];
		double min_error = 1e6;
		int fit_l = 0;
		for (int l=0; l<src_size; l++)
		{
			if (l==best_j) continue;
			double error = norm(new_dst.col(0)-points3D_src[l].col(0))+norm(new_dst.col(1)-points3D_src[l].col(1));
			if (min_error > error)
			{
				min_error = error;
				fit_l = l;
			}
		}
//		printf("min error: %f\n", min_error);
		if (min_error < threshold)
		{
			if (visibility_src[fit_l] == -1)//该点没有被添加过
			{
				for (int i=0; i<2; i++)
				{
					Mat u = radii*Mat(points3D_src[fit_l].col(i));
					Point3d p(u.at<double>(0,0), u.at<double>(1,0), u.at<double>(2,0));
					point3d_final.push_back(p);
				}
				visibility_src[fit_l] = points_end_idx;
				points_end_idx++;
			}
			visibility_dst[k] = visibility_src[fit_l];
		}
		else visibility_dst[k] = -1;
	}
	for (int k=0; k<dst_size; k++)
		points3D_dst[k] = R_temp * points3D_dst[k];
}

void CCalibBall::Two_Pass(const Mat& binImg, Mat& lableImg)    //两遍扫描法
{
	if (binImg.empty() ||
		binImg.type() != CV_8UC1)
	{
		return;
	}

	// 第一个通路

	lableImg.release();
	binImg.convertTo(lableImg, CV_32SC1);

	int label = 1; 
	vector<int> labelSet;
	labelSet.push_back(0);  
	labelSet.push_back(1);  

	int rows = binImg.rows - 1;
	int cols = binImg.cols - 1;
	for (int i = 1; i < rows; i++)
	{
		int* data_preRow = lableImg.ptr<int>(i-1);
		int* data_curRow = lableImg.ptr<int>(i);
		for (int j = 1; j < cols; j++)
		{
			if (data_curRow[j] == 1)
			{
				vector<int> neighborLabels;
				neighborLabels.reserve(2);
				int leftPixel = data_curRow[j-1];
				int upPixel = data_preRow[j];
				if ( leftPixel > 1)
				{
					neighborLabels.push_back(leftPixel);
				}
				if (upPixel > 1)
				{
					neighborLabels.push_back(upPixel);
				}

				if (neighborLabels.empty())
				{
					labelSet.push_back(++label);  // 不连通，标签+1
					data_curRow[j] = label;
					labelSet[label] = label;
				}
				else
				{
					std::sort(neighborLabels.begin(), neighborLabels.end());
					int smallestLabel = neighborLabels[0];  
					data_curRow[j] = smallestLabel;

					// 保存最小等价表
					for (size_t k = 1; k < neighborLabels.size(); k++)
					{
						int tempLabel = neighborLabels[k];
						int& oldSmallestLabel = labelSet[tempLabel];
						if (oldSmallestLabel > smallestLabel)
						{							
							labelSet[oldSmallestLabel] = smallestLabel;
							oldSmallestLabel = smallestLabel;
						}						
						else if (oldSmallestLabel < smallestLabel)
						{
							labelSet[smallestLabel] = oldSmallestLabel;
						}
					}
				}				
			}
		}
	}

	// 更新等价对列表
	// 将最小标号给重复区域
	for (size_t i = 2; i < labelSet.size(); i++)
	{
		int curLabel = labelSet[i];
		int preLabel = labelSet[curLabel];
		while (preLabel != curLabel)
		{
			curLabel = preLabel;
			preLabel = labelSet[preLabel];
		}
		labelSet[i] = curLabel;
	}

	for (int i = 0; i < rows; i++)
	{
		int* data = lableImg.ptr<int>(i);
		for (int j = 0; j < cols; j++)
		{
			int& pixelLabel = data[j];
			pixelLabel = labelSet[pixelLabel];	
		}
	}
}

void CCalibBall::Points3DToMat(vector< vector<Point3d> > points3D, vector<Matx33d> &points)
{
	int src_size = int(points3D.size());
	points.resize(src_size);
	Mat temp(3,3,CV_64FC1);
	for (int i=0; i<src_size; i++)
	{
		Mat(points3D[i][0] * (1/norm(points3D[i][0]))).copyTo(temp.col(0));
		Mat(points3D[i][1] * (1/norm(points3D[i][1]))).copyTo(temp.col(1));
		Mat(temp.col(1).cross(temp.col(0))).copyTo(temp.col(2));
		temp.col(2) /= norm(temp.col(2));
		temp.copyTo(points[i]);
	}
}

void CCalibBall::ProcessForSba(vector<vector<Point2f>> pointImg_src, vector<Point2d> &pointImg_dst,  vector< Point3d> points, vector< Matx33d > points3D_normed, vector<int> &visibility)
{
	size_t nPoints = points.size();
	pointImg_dst.resize(nPoints);
	vector<int> visibility_new(nPoints);
	Point2d p(0,0);
	for (int i=0; i< nPoints; i++)
	{
		visibility_new[i] = 0;
		pointImg_dst[i] = p;
	}
	int count = 0;
	double threshold = 0.15;
	printf("visibility: ");
	int vis_size = int(visibility.size());
	vector<Mat> points_normed(points.size());
	for (int i=0; i<points.size(); i++)
	{
		points_normed[i] = Mat(points[i]*(1/radii));
	}
	for (int i=0; i<vis_size;  i++)
	{
//		printf("%d ",visibility[i]);
		if (visibility[i] != -1)
		{
			int idx = visibility[i]<<1;
			pointImg_dst[idx] = pointImg_src[i][0];
			pointImg_dst[idx+1] = pointImg_src[i][1];
			visibility_new[idx] = 1;
			visibility_new[idx+1] = 1;
			count++;
		}
		else
		{
			double min_error = 1e6;
			int idx = 0;
			for (int j=0; j<nPoints; j+=2)
			{
				if (visibility_new[j]==0)
				{
					double error = MAX(norm(Mat(points3D_normed[i].col(0))-points_normed[j]), norm(Mat(points3D_normed[i].col(1))-points_normed[j+1]));
					if (min_error > error)
					{
						min_error = error;
						idx = j;
					}
				}
			}
			if (min_error < threshold)
			{
				pointImg_dst[idx] = pointImg_src[i][0];
				pointImg_dst[idx+1] = pointImg_src[i][1];
				visibility_new[idx] = 1;
				visibility_new[idx+1] = 1;
				count++;
			}
		}
	}
	printf("%d/%d\n", count, visibility.size());
	visibility = visibility_new;
}

void CCalibBall::OutputParam(vector< Mat > cameraMatrix, vector< Mat > R, vector< Mat > T, vector< Mat > distCoeffs)
{
	FileStorage fp(filepath+camera_calib_name, FileStorage::WRITE);
	Mat extrisic(3,4,CV_64FC1);
	for (int i=0; i<cam_used_num; i++)
	{
		string currentID = to_string((_Longlong)cam_list[i]);
		fp<<"intrinsic-"+currentID<<cameraMatrix[i];
		R[i].copyTo(extrisic.colRange(0,3));
		T[i].copyTo(extrisic.col(3));
		fp<<"extrinsic-"+currentID<<extrisic;
	}
	fp.release();
}

void CCalibBall::ProjectToImg(const vector<string> &imagelist, vector< Mat > cameraMatrix, vector< Mat > R, vector< Mat > T, vector< Point3d> points, vector<vector< Point2d >> imagePoints, vector<vector<int>> visibility)
{
	double error = 0;
	double mean_radii = 0;
	int nvis = 0;
	size_t nPoints = points.size();
	for (int i=0; i<cam_used_num; i++)
	{
#ifdef IS_OUTPUT
		bool rotate90 = IsRotated[cam_list[i]];
		Mat ref = imread(filepath+imagelist[cam_list[i]]);
		if (IsRotated[cam_list[i]]=='1')
		{
			flip(ref, ref, -1);
		}
#endif
		Mat P1 = cameraMatrix[i]*R[i];
		Mat P2 = cameraMatrix[i]*T[i];
		for (int j=0; j<nPoints; j++)
		{
			if (visibility[i][j] == 1)
			{
				Mat u = Mat(points[j]);
				u = P1*u+P2;
				Point2d center(u.at<double>(0,0)/u.at<double>(2,0), u.at<double>(1,0)/u.at<double>(2,0));
#ifdef IS_OUTPUT
				circle(ref, Point(center), 5, Scalar(0,255,0), CV_FILLED);
				circle(ref, Point(imagePoints[i][j]), 5, Scalar(255,0,0), CV_FILLED);
#endif
				error += norm(center-imagePoints[i][j]);
				
				nvis ++;
			}
		}
#ifdef IS_OUTPUT
		imwrite("projection//"+imagelist[cam_list[i]], ref);
#endif
	}
	printf("mean square error: %f of %d projections\n", error/nvis, nvis);
	for (int i=0; i<nPoints; i++)
		mean_radii += norm(points[i]);
	printf("mean radii: %f of %d points\n", mean_radii/nPoints, nPoints);
}

void CCalibBall::ScaleToWorld(vector< Point3d> &points, vector< Mat > &T)
{
	double mean_DF = 0;
	size_t nPoints = points.size();
	for (int i=0; i<nPoints; i+=2)
	{
		mean_DF += norm(points[i+1]-points[i]);
	}
	double scale = DF/(mean_DF/nPoints*2);
	for (int i=0; i<cam_used_num; i++)
	{
		T[i] *= scale;
	}
	for (int i=0; i<nPoints; i++)
	{
		points[i] *= scale;
	}
}

void CCalibBall::KinectMarker(Mat BigMarker, Mat mask, vector< Point2f > &final_corners, int &first_label, int idx)
{
	Mat image_gray;
	resize(BigMarker, image_gray, Size(), 1.0/SCALE, 1.0/SCALE);
	resize(mask, mask, Size(), 1.0/SCALE, 1.0/SCALE);
	vector<Point2f> corners;
//	equalizeHist(image_gray, image_gray);
//	for (int i=0; i<3; i++)
//		medianBlur(image_gray, image_gray, 3);
	goodFeaturesToTrack(image_gray, corners, 50, 0.02, 110/SCALE, mask, 3);
	TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 ) ;
	cornerSubPix(image_gray, corners, Size(7,7), Size(-1,-1), criteria);
	Mat element = getStructuringElement( MORPH_ELLIPSE, Size(100/SCALE, 100/SCALE));
	Mat mask_bk = mask.clone();
	erode(mask, mask, element);
	for (int i=0; i<corners.size(); i++)
	{
		if (mask.at<uchar>(corners[i]) == 255)
		{
			final_corners.push_back(corners[i]);
		}
	}
	vector<Point2f> line_(2);
	double threshold = 35./SCALE;
	if (RansacLine(final_corners, line_, threshold)==false)
	{
		printf("not enough points\n");
		return ;
	}
	final_corners.clear();
	element = getStructuringElement( MORPH_ELLIPSE, Size(100/SCALE, 100/SCALE));
	erode(mask_bk, mask_bk, element);
	Mat image_gray_bk = image_gray.clone();
	for (int i=0; i<corners.size(); i++)
	{
		Point2f temp = line_[0]-corners[i];
#ifdef IS_OUTPUT
		circle(image_gray_bk, corners[i], 10, Scalar(255));
#endif
		if (mask_bk.at<uchar>(corners[i]) == 255 && sqrt(square(norm(temp)) - square(temp.dot(line_[1]))+0.1) < threshold)
		{
			final_corners.push_back(corners[i]);
		}
	}
	//order
	int points_num = (int)final_corners.size();
	for (int i=0; i<points_num; i++)
	{
		float x = -1;
		int ID = -1;
		int j=0;
		for (; j<points_num-i; j++)
		{
			if (final_corners[j].x > x)
			{
				ID = j;
				x = final_corners[j].x;
			}
		}
		Point2f temp = final_corners[j-1];
		final_corners[j-1] = final_corners[ID];
		final_corners[ID] = temp;
	}
	//label
	Point2f direction = final_corners[1] - final_corners[0];
	float temp_dir = direction.x;
	direction.x = -direction.y;
	direction.y = temp_dir;
	Point2f center = (final_corners[1] + final_corners[0])*0.5;
	bool IsUpWhite = false;
	if (image_gray.at<uchar>(center+0.2*direction) > image_gray.at<uchar>(center-0.2*direction))
	{
		IsUpWhite = true;
	}
	char * marker_bool = new char [points_num];
	marker_bool[0] = '0' + (image_gray.at<uchar>(center+0.7*direction) > image_gray.at<uchar>(center-0.7*direction) ? !IsUpWhite:IsUpWhite);
	for (int i=2; i<points_num; i++)
	{
		IsUpWhite = !IsUpWhite;
		direction = final_corners[i] - final_corners[i-1];
		temp_dir = direction.x;
		direction.x = -direction.y;
		direction.y = temp_dir;
		center = (final_corners[i] + final_corners[i-1])*0.5;
		marker_bool[i-1] = '0' + (image_gray.at<uchar>(center+0.7*direction) > image_gray.at<uchar>(center-0.7*direction) ? !IsUpWhite:IsUpWhite);
	}
	marker_bool[points_num-1] = '\0';
	first_label = strindex(code.c_str(), marker_bool);
	printf("%d first label: %d,\tidx: %s\n", idx, first_label, marker_bool);
	Point2f p(.5f, .5f);
	for (int i=0; i<points_num; i++)
	{
		final_corners[i] = final_corners[i]*SCALE+p;
#ifdef IS_OUTPUT
		circle(BigMarker, final_corners[i], 10, Scalar(255));
#endif
	}
#ifdef IS_OUTPUT
	char filename[512];
	sprintf(filename, "out//%d_kinect.jpg", idx);
	imwrite(filename, BigMarker);
 	sprintf(filename, "kinect//%d_kinect.jpg", idx);
 	imwrite(filename, image_gray_bk);
#endif
}

bool CCalibBall::RansacLine(vector<Point2f> src, vector<Point2f> &line, double threshold)
{
	int total_points = (int)src.size();
	int max_inliers = 0;
	double min_error = 1e6;
	for (int i=0; i<total_points; i++)
	{
		for (int j=i+1; j<total_points; j++)
		{
			int inliers = 0;
			double total_error = 0;
			Point2f line_k = src[i]-src[j];
			line_k = 1/norm(line_k)*line_k;
			for (int k=0; k<total_points; k++)
			{
				if (k == i || k == j)
					continue;
				Point2f temp_pt = src[i]-src[k];
				double error = sqrt(square(norm(temp_pt)) - square(temp_pt.dot(line_k)));
				if ( error < threshold )
				{
					inliers++;
					total_error += error;
				}
			}
			if (inliers > max_inliers)
			{
				max_inliers = inliers;
				line[0] = src[i];
				line[1] = line_k;
			}
			else if ((inliers == max_inliers) && (total_error < min_error))
			{
				min_error = total_error;
				line[0] = src[i];
				line[1] = line_k;	
			}
		}
	}
//	printf("inliers %d\n", max_inliers);
	if (max_inliers == 0)
		return false;
	return true;
}

void CCalibBall::FetchKinect(int idx, vector<Point3f>	AllPosition, FileStorage fs)
{
	char filename[MAX_COUNT];
	sprintf(filename, "%s_%d_color.jpg", filepath.c_str(), kinect[idx].sn);//这有问题@@@@
	Mat origin = imread(filename);
	sprintf(filename, "%s_%d_color.jpg", filepath.c_str(), kinect[idx].sn);
	Mat ref = imread(filename);
	//cut circle
	int minR=115;
	int maxR=145;
	Mat gray, BigMarker;
	Vec3d circle_;
	absdiff(origin,ref,gray);
	cvtColor(gray, gray, CV_BGR2GRAY);
	GaussianBlur( gray, gray, Size(25, 25), 2, 2);
	gray = gray > 10;
	vector<vector<Point> > contours;
	findContours(gray, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0,0));
	double max_area = 0;
	int max_ID = -1;
	for (int i=0; i<contours.size(); i++)
	{
		double area = contourArea(contours[i]);
		if (area>max_area)
		{
			max_area = area;
			max_ID = i;
		}
	}
	Mat mask(gray.size(), CV_8UC1, Scalar(0));
	drawContours( mask, contours, max_ID, Scalar(255) );
	DetectCircle(mask, circle_, minR, maxR);
	drawContours( mask, contours, max_ID, Scalar(255), CV_FILLED );
	Point2i bestCircleCenter(cvRound(circle_[0]), cvRound(circle_[1]));	
	circle(mask, bestCircleCenter, cvRound(circle_[2])+5, Scalar(0), CV_FILLED);
	ref.copyTo(BigMarker, mask);
	cvtColor(BigMarker, BigMarker, CV_BGR2GRAY);
	//find corners
	vector<Point2f> corners, final_corners;
	goodFeaturesToTrack(BigMarker, corners, 50, 0.05, 15, mask, 3);
	TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 ) ;
	cornerSubPix(BigMarker, corners, Size(5,5), Size(-1,-1), criteria);
	Mat element = getStructuringElement( MORPH_ELLIPSE, Size(5, 5));
	Mat mask_bk = mask.clone();
	erode(mask, mask, element);
	for (int i=0; i<corners.size(); i++)
	{
		if (mask.at<uchar>(corners[i]) == 255)
		{
			final_corners.push_back(corners[i]);
		}
	}
	vector<Point2f> line_(2);
	double threshold = 4;
	if (RansacLine(final_corners, line_, threshold)==false)
	{
		printf("not enough points\n");
		return ;
	}
	final_corners.clear();
	element = getStructuringElement( MORPH_ELLIPSE, Size(5, 5));
	erode(mask_bk, mask_bk, element);
	for (int i=0; i<corners.size(); i++)
	{
		Point2f temp = line_[0]-corners[i];
		if (mask_bk.at<uchar>(corners[i]) == 255 && sqrt(square(norm(temp)) - square(temp.dot(line_[1]))+0.1) < threshold)
		{
			final_corners.push_back(corners[i]);
		}
	}
	//order
	int points_num = (int)final_corners.size();
	for (int i=0; i<points_num; i++)
	{
		float x = -1;
		int ID = -1;
		int j=0;
		for (; j<points_num-i; j++)
		{
			if (final_corners[j].x > x)
			{
				ID = j;
				x = final_corners[j].x;
			}
		}
		Point2f temp = final_corners[j-1];
		final_corners[j-1] = final_corners[ID];
		final_corners[ID] = temp;
	}
	//label
	Point2f direction = final_corners[1] - final_corners[0];
	float temp_dir = direction.x;
	direction.x = -direction.y;
	direction.y = temp_dir;
	Point2f center = (final_corners[1] + final_corners[0])*0.5;
	bool IsUpWhite = false;
	if (BigMarker.at<uchar>(center+0.2*direction) > BigMarker.at<uchar>(center-0.2*direction))
	{
		IsUpWhite = true;
	}
	char * marker_bool = new char [points_num];
	marker_bool[0] = '0' + (BigMarker.at<uchar>(center+0.6*direction) > BigMarker.at<uchar>(center-0.6*direction) ? !IsUpWhite:IsUpWhite);
	for (int i=2; i<points_num; i++)
	{
		IsUpWhite = !IsUpWhite;
		direction = final_corners[i] - final_corners[i-1];
		temp_dir = direction.x;
		direction.x = -direction.y;
		direction.y = temp_dir;
		center = (final_corners[i] + final_corners[i-1])*0.5;
		marker_bool[i-1] = '0' + (BigMarker.at<uchar>(center+0.6*direction) > BigMarker.at<uchar>(center-0.6*direction) ? !IsUpWhite:IsUpWhite);
	}
	marker_bool[points_num-1] = '\0';
	int first_label = strindex(code.c_str(), marker_bool);
	printf("%d first label: %d,\tidx: %s\n", idx, first_label, marker_bool);
#ifdef IS_OUTPUT
	for (int i=0; i<points_num; i++)
		circle(BigMarker, final_corners[i], 3, Scalar(255));
	sprintf(filename, "out//%d_kinect.jpg", idx);
	imwrite(filename, BigMarker);
#endif
	Mat left_mat(points_num, 3, CV_32FC1), right_mat(points_num, 2, CV_32FC1), P;
	for (int i=0; i<points_num; i++)
	{
		int current_idx = (first_label+i)%code_num;
		left_mat.at<float>(i,0) = AllPosition[current_idx].x;
		left_mat.at<float>(i,1) = AllPosition[current_idx].y;
		left_mat.at<float>(i,2) = AllPosition[current_idx].z;
		right_mat.at<float>(i,0) = 600+final_corners[i].y;
		right_mat.at<float>(i,1) = 809-final_corners[i].x;
	}
	Mat r,t;
	solvePnP(left_mat, right_mat, kinect[idx].ColorCameraIntrinsic, Mat(), r, t);
	Rodrigues(r, r);
	Mat temp_ex(3, 4, CV_32FC1);
	t.convertTo(t, CV_32FC1);
	r.convertTo(r, CV_32FC1);
	kinect[idx].ColorCameraExtrinsic.colRange(0,3).copyTo(temp_ex.colRange(0,3));
	Mat(kinect[idx].ColorCameraExtrinsic.col(3)-t).copyTo(temp_ex.col(3));
	kinect[idx].R1t_times_R = Mat(temp_ex.t()*r).clone();
	string currentID = std::to_string((_Longlong)kinect[idx].sn);
	fs<<"DepthCameraIntrinsic-"+currentID<<"["
		<<kinect[idx].DepthCameraIntrinsic.at<float>(0,0)
		<<kinect[idx].DepthCameraIntrinsic.at<float>(1,1)
		<<kinect[idx].DepthCameraIntrinsic.at<float>(0,2)
		<<kinect[idx].DepthCameraIntrinsic.at<float>(1,2)<<"]";
	fs<<"R1t_times_R-"+currentID<<kinect[idx].R1t_times_R;
}

void CCalibBall::MapCoordinate(vector< Mat > P, vector< vector <Point2f> > MarkerPosition, int *first_label, vector< Point3f > &AllPosition)
{
	AllPosition.resize(code_num);
	for (int i=0; i<code_num; i++)
		AllPosition[i] = Point3f(10000,0,0);

	int th = code_num/2;
	vector<int> id_list;
	for (int i=0; i<cam_used_num-1; i++)
	{
		int diff = first_label[i] - first_label[i+1];
		int start0,start1,common_firstlabel;
		switch((int)floor(double(diff)/th))
		{
		case -2:
			start0 = 0;
			start1 = 22+diff;
			common_firstlabel = first_label[i];
			break;
		case -1:
			start0 = -diff;
			start1 = 0;
			common_firstlabel = first_label[i+1];
			break;
		case 0:
			start0 = 0;
			start1 = diff;
			common_firstlabel = first_label[i];
			break;;
		case 1:
			start0 = 22-diff;
			start1 = 0;
			common_firstlabel = first_label[i+1];
		}
		int common_length = MIN((int)MarkerPosition[i].size()-start0, (int)MarkerPosition[i+1].size()-start1);
		vector< Point2f > pt0(common_length), pt1(common_length);
		for (int j=0; j<common_length; j++)
		{
			pt0[j] = MarkerPosition[i][j+start0];
			pt1[j] = MarkerPosition[i+1][j+start1];
		}
		Mat output;
		triangulatePoints(P[i], P[i+1], pt0, pt1, output);
		for (int j=0; j<common_length; j++)
		{
			int idx = (common_firstlabel+j)%code_num;
			if (AllPosition[idx].x == 10000)
			{
				float temp = output.at<float>(3,j);
				AllPosition[idx].x = output.at<float>(0,j)/temp;
				AllPosition[idx].y = output.at<float>(1,j)/temp;
				AllPosition[idx].z = output.at<float>(2,j)/temp;
				id_list.push_back(idx);
			}
		}
	}
	int count = (int)id_list.size();
	//fit plane
	Mat points_buffer(count, 3, CV_32FC1);
	int k = 0;
	bool flag = true;
	int first_idx = 0;
	for (int i=0; i<count; i++)
	{
		Mat(Mat(AllPosition[id_list[i]]).t()).copyTo(points_buffer.row(k++));
	}
	Mat plane_normal;
	Mat const_temp = Mat::ones(points_buffer.rows,1,CV_32FC1);
	solve(points_buffer, const_temp, plane_normal, DECOMP_SVD);
	double constValue = 1/norm(plane_normal);
	plane_normal *= constValue;
	//fit circle
	Mat s = repeat(points_buffer*plane_normal-constValue*const_temp, 1, 3);
	Mat proj_points = points_buffer - s.mul(repeat(Mat(plane_normal.t()), count, 1));
	Mat A(count, 3, CV_32FC1);
	Mat(-2*proj_points.col(0)+(2*plane_normal.at<float>(0,0)/plane_normal.at<float>(2,0))*proj_points.col(2)).copyTo(A.col(0));
	Mat(-2*proj_points.col(1)+(2*plane_normal.at<float>(1,0)/plane_normal.at<float>(2,0))*proj_points.col(2)).copyTo(A.col(1));
	A.col(2).setTo(1);
	Mat b = -proj_points.col(0).mul(proj_points.col(0))
			-proj_points.col(1).mul(proj_points.col(1))
			-proj_points.col(2).mul(proj_points.col(2))
			+ (2*constValue/plane_normal.at<float>(2,0))*proj_points.col(2);
	Mat dst;
	solve(A, b, dst, DECOMP_SVD);
	Point3f center;
	center.x = dst.at<float>(0,0);
	center.y = dst.at<float>(1,0);
	center.z = ((float)constValue - plane_normal.at<float>(0,0)*dst.at<float>(0,0)-plane_normal.at<float>(1,0)*dst.at<float>(1,0))/plane_normal.at<float>(2,0);
	double radii = sqrt(center.dot(center)-dst.at<float>(2,0));
// 	Vec3f ref_point(Mat(proj_points.row(0)-Mat(center).t()).ptr<float>(0,0));
// 	normalize(ref_point, ref_point);
// 	Point3f u = Point3f(ref_point)*radii;
// 	if (Mat(points_buffer.row(0).cross(points_buffer.row(1))).dot(plane_normal.t()) < 0)
// 		plane_normal = -plane_normal;
// 	Point3f v(Vec3f(plane_normal.cross(Mat(u))));
// 	for (int i=0; i<code_num; i++)
// 	{
// 		AllPosition[(i+first_idx)%code_num] = u*cos(double(i)/code_num*2*PI) + v*sin(double(i)/code_num*2*PI) + center;
// 	}
	for (int i=0; i<count; i++)
	{
		Vec3f ref_point(Mat(proj_points.row(i)-Mat(center).t()).ptr<float>(0));
		normalize(ref_point, ref_point);
		AllPosition[id_list[i]] = Point3f(ref_point)*radii+center;
	}
	int start_idx = id_list[count-1];
	Point3f u = AllPosition[start_idx]-center;
	double theta = acos((AllPosition[id_list[0]]-center).dot(u)/square(radii))/(code_num-count+1);
	if (Mat(points_buffer.row(0).cross(points_buffer.row(1))).dot(plane_normal.t()) < 0)
		plane_normal = -plane_normal;
	Point3f v(Vec3f(plane_normal.cross(Mat(u))));
	for (int i=1; i<(code_num-count+1); i++)
	{
		AllPosition[(start_idx+i)%code_num] = u*cos(i*theta) + v*sin(i*theta) + center;
	}
}

void fillEdgeImage(Mat edgesIn, Mat& filledEdgesOut)
{
	Mat edgesNeg = edgesIn.clone();

	floodFill(edgesNeg, Point(0,0), CV_RGB(255,255,255));
	bitwise_not(edgesNeg, edgesNeg);
	filledEdgesOut = (edgesNeg | edgesIn);

	return;
}

CCalibBall::~CCalibBall()
{
}