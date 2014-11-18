#include "CCalibBall.h"

inline void getCircle(Point2d& p1,Point2d& p2,Point2d& p3, Point2d& center, double& radius)
{
	double x1 = p1.x;
	double x2 = p2.x;
	double x3 = p3.x;

	double y1 = p1.y;
	double y2 = p2.y;
	double y3 = p3.y;

	// PLEASE CHECK FOR TYPOS IN THE FORMULA :)
	center.x = (x1*x1+y1*y1)*(y2-y3) + (x2*x2+y2*y2)*(y3-y1) + (x3*x3+y3*y3)*(y1-y2);
	center.x /= ( 2*(x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2) );

	center.y = (x1*x1 + y1*y1)*(x3-x2) + (x2*x2+y2*y2)*(x1-x3) + (x3*x3 + y3*y3)*(x2-x1);
	center.y /= ( 2*(x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2) );

	radius = sqrt((center.x-x1)*(center.x-x1) + (center.y-y1)*(center.y-y1));
}

std::vector<Point2d> getPointPositions(Mat binaryImage)
{
	std::vector<Point2d> pointPositions;

	for(int y=0; y<binaryImage.rows; ++y)
	{
		uchar* rowPtr = binaryImage.ptr<uchar>(y);
		for(int x=0; x<binaryImage.cols; ++x)
		{
			if(rowPtr[x] > 0) pointPositions.push_back(Point2d(x,y));
		}
	}

	return pointPositions;
}

double evaluateCircle(Mat dt, Point2d center, double radius)
{

	double completeDistance = 0.0f;
	int counter = 0;

	double maxDist = 10.0f;   //TODO: this might depend on the size of the circle!

	double minStep = 0.001f;
	// choose samples along the circle and count inlier percentage

	//HERE IS THE TRICK that no minimum/maximum circle is used, the number of generated points along the circle depends on the radius.
	// if this is too slow for you (e.g. too many points created for each circle), increase the step parameter, but only by factor so that it still depends on the radius

	// the parameter step depends on the circle size, otherwise small circles will create more inlier on the circle
	double step = 2*3.14159265359f / (6.0f * radius);
	if(step < minStep) step = minStep; // TODO: find a good value here.

	//for(double t =0; t<2*3.14159265359f; t+= 0.05f) // this one which doesnt depend on the radius, is much worse!
	for(double t =0; t<2*3.14159265359f; t+= step)
	{
		int cX = int(radius*cos(t) + center.x + 0.5);
		int cY = int(radius*sin(t) + center.y + 0.5);

		if(cX < dt.cols)
			if(cX >= 0)
				if(cY < dt.rows)
					if(cY >= 0)
					{	
						float temp = dt.at<float>(cY,cX);
						if( temp <= maxDist)
						{
							completeDistance += temp;
							counter++;
						}
					}
	}

	return counter;
}


void DetectCircle(Mat image, Vec3d &circle_, double minCircleRadius, double maxCircleRadius)
{
	std::vector<Point2d> edgePositions;
	edgePositions = getPointPositions(image);

	// create distance transform to efficiently evaluate distance to nearest edge
	Mat dt;
	distanceTransform(255-image, dt, CV_DIST_L1, 3);

	Point2d bestCircleCenter;
	double bestCircleRadius;
	//double bestCVal = FLT_MAX;
	double bestCVal = -1;

	int iter = 100;
	//TODO: implement some more intelligent ransac without fixed number of iterations
	int edge_size = int(edgePositions.size());
	int width = image.cols;
	int height = image.rows;
	while(1)
	{
		//RANSAC: randomly choose 3 point and create a circle:
		//TODO: choose randomly but more intelligent,
		//so that it is more likely to choose three points of a circle.
		//For example if there are many small circles, it is unlikely to randomly choose 3 points of the same circle.
		unsigned int idx1 = rand()%edge_size;
		unsigned int idx2 = rand()%edge_size;
		unsigned int idx3 = rand()%edge_size;

		// we need 3 different samples:
		if(idx1 == idx2) continue;
		if(idx1 == idx3) continue;
		if(idx3 == idx2) continue;

		// create circle from 3 points:
		Point2d center; double radius;
		getCircle(edgePositions[idx1],edgePositions[idx2],edgePositions[idx3],center,radius);

		if(radius < minCircleRadius)continue;
		if(radius > maxCircleRadius)continue;
		if (center.x > (width-radius) || center.x < radius || center.y > (height- radius) || center.y < radius)
			continue;
		//verify or falsify the circle by inlier counting:
		//double cPerc = verifyCircle(dt,center,radius, inlierSet);
		double cVal = evaluateCircle(dt,center,radius);

		if(cVal > bestCVal)
		{
			bestCVal = cVal;
			bestCircleRadius = radius;
			bestCircleCenter = center;
		}
		iter--;
		if (iter == 0) break;
	}
	circle_[0] = bestCircleCenter.x;
	circle_[1] = bestCircleCenter.y;
	circle_[2] = bestCircleRadius;
}