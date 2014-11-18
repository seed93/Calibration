#include "CCalibBall.h"

void AutoThres(Mat src, Mat &dst)
{
	Mat hist;
	int bins=256;
	int histSize[] = {bins};
	float range[] = {0,256};
	const float* ranges[] = {range};
	int channels[] = {0};
	Mat mask = dst.clone();
	calcHist(&src,1,channels,mask,hist,1,histSize,ranges,true,false);
	hist = hist.t();
	const float* hist_p = hist.ptr<float>(0);
	double T_old = 0, T_new = 100;
	double err = 0.5;
	float total_times[256], total_count[256];
	total_count[0] = 0;
	total_times[0] = hist_p[0];
	for (int i=1; i<256; i++)
	{
		total_count[i] = total_count[i-1] + hist_p[i];
		total_times[i] = total_times[i-1] + hist_p[i]*i;
	}
	do 
	{
		T_old = T_new;
		int thres_int = int(T_new);
		if (thres_int > 254)
		{
			printf("err");
			break;
		}
		
		T_new = (total_times[thres_int] / total_count[thres_int] + total_times[thres_int+1] / total_count[thres_int+1])/2;	
	} while (abs(T_old-T_new)<err);
	threshold(src,dst,T_new+5,255,CV_THRESH_BINARY);
}