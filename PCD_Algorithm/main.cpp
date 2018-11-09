#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types_c.h>
#include <opencv2/core/types.hpp>
#include <iostream>
#include <opencv2/core/cvdef.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h> 
#include <pcl/visualization/cloud_viewer.h>
#include "defs.h"
#include "Intrinsic.hpp"
#include "Distortion.hpp"
#ifdef debugPCD
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
#endif

using namespace std;


#define HIGH2LOW 0
#define LOW2HIGH 1
#define RISE 1
#define SET 2
#define INIT 0

bool relation(int a, int b, bool condition) //relation mark swaps in case of condition
{
	if (condition)
		return a < b;
	return a > b;
}

void gethistvalley(cv::Mat imgIn, int* idxOut, double* valueOut, int type, int valley_count)
{
	int imgCount = 1;
	cv::Mat hist;
	int histSize = 256;
	float range[] = { 0, 255 };
	const float* histRange = { range };

	cv::calcHist(&imgIn, imgCount, 0, cv::Mat(), hist, 1, &histSize, &histRange);

	int from, to, increment;
	switch (type)
	{
	case HIGH2LOW:from = histSize - 1; to = -1; increment = -1; break;
	case LOW2HIGH:from = 0; to = histSize; increment = 1; break;
	}
	int value_before = 0; int status = INIT; int valley_counter = 0;
	for (int i = from; relation(i, to, type); i += increment)
	{
		switch (status)
		{
		case RISE:	if (value_before > hist.at<float>(i, 0)) status = SET;
			break;
		case INIT:	if (value_before > hist.at<float>(i, 0)) status = SET;
					else status = RISE;
					break;
		case SET:	if (value_before < hist.at<float>(i, 0))
		{
			if (++valley_counter == valley_count)
			{
				*idxOut = i - 1;
				*valueOut = hist.at<float>(i - 1, 0);
				//cout << "van volgy" << endl;
				return;
			}
			status = RISE;
		}
					break;
		default:	cout << "error a gethistvalleyban" << endl;  break;
		}
		value_before = hist.at<float>(i, 0);
	}
	//cout << "nincs volgy" << endl;
	*idxOut = -1;
	*valueOut = -1;
}

void hist_debug(cv::Mat in)
{
	int imgCount = 1;
	const int chs = in.channels();
	std::vector<cv::Mat> channels;
	for (int i = 0; i < chs; i++)
	{
		channels.push_back(cv::Mat());
	}
	cv::split(in, channels);
	for (int i = 0; i < chs; i++)
	{
		cv::Mat inHist = channels[i];
		cv::Mat hist;
		int histSize = 256;
		float range[] = { 0, 255 };
		const float* histRange = { range };

		cv::calcHist(&inHist, imgCount, 0, cv::Mat(), hist, 1, &histSize, &histRange);
		for (int k = 0; k < histSize; k++)
		{
			cout << (int)(hist.at<float>(k, 0)) << "db, " << k << " val, " << i << " channel" << endl;
		}
	}

}

void type_debug(cv::Mat in)
{
	string r;
	uchar depth = in.type() & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (in.type() >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');
	cout << r << endl;
}

typedef pcl::PointXYZRGB PCDPoint;
typedef pcl::PointCloud<PCDPoint> PCD;
typedef PCD::Ptr PCDPtr;
typedef PCDPtr PCDDataBase;
PCDPtr DepthToPCD(cv::Mat in, Intrinsic intr, Distortion dist, std::string imageType, std::string coordinateSystem)
{
	//Intrinsic intr = popFrameAs<IntrinsicData>("cameraMatrix")->data(); //fill to intrinsic
	//Distortion dist = popFrameAs<DistortionData>("distCoeffs")->data(); //fill to distortion
	//std::string imageType = in.type();
	//std::string coordinateSystem = popFrameAs<StringData>("coordinateSystem")->data(); //fill coordinate sys

	if (in.type() != CV_32FC1)
	{
		return (PCDPtr)NULL;
	}// ("Only CV_32FC1 image is supported", "");

	//bool outZImageConnected = isPinConnected("outZ");

	double k1 = dist.radial(0);
	double k2 = dist.radial(1);
	double p1 = dist.tangential(0);
	double p2 = dist.tangential(1);
	double k3 = dist.radial(2);

	cv::Mat outZ;
	outZ = cv::Mat(in.rows, in.cols, CV_32FC1, cv::Scalar(0));

	PCDPtr pcd(new PCD(in.cols, in.rows));
	PCD::PointType p;
	p.r = 255;
	p.g = 255;
	p.b = 255;

	double x, y, x0, y0;
	Eigen::Vector3f normPos;
	Eigen::Vector3f imagePos;
	for (int j = 0; j < in.rows; ++j)
	{
		for (int i = 0; i < in.cols; ++i)
		{
			/// Normalize coordinates
			normPos = intr.backProject(i, j, 1);
			x0 = x = normPos.x();
			y0 = y = normPos.y();

			/// Copy from OpenCV undistortPoints
			const int maxIter = 5;
			for (int iter = 0; iter < maxIter; ++iter)
			{
				double r2 = x * x + y * y;
				double _2xy = 2 * x * y;
				double icdist = 1 / (1 + (k1 + (k2 + k3 * r2) * r2) * r2); /// Radial
				double deltaX = p1 * _2xy + p2 * (r2 + 2 * x*x);     /// X tangential
				double deltaY = p1 * (r2 + 2 * y*y) + p2 * _2xy;             /// Y tangential
				x = (x0 - deltaX) * icdist;
				y = (y0 - deltaY) * icdist;

				if (r2 >= 1)
					break;
			}

			float z = in.at<float>(j, i);

			if (z <= 0.01)
			{
				continue;
			}

			if (imageType == "DImage")
			{
				/// Distance to z
				z /= sqrt(x*x + y * y + 1);
			}

			/// Z to pcd
			p.x = x * z;
			p.y = y * z;
			p.z = z;

			if (coordinateSystem == "OpenGL")
			{
				p.y = -p.y;
				p.z = -p.z;
			}

			pcd->at(i, j) = p;

			//if (outZImageConnected)
			{
				imagePos = intr.project(Eigen::Vector3f(x, y, 1));

				if (imagePos.x() >= 0 && imagePos.x() < in.cols && imagePos.y() >= 0 && imagePos.y() < in.rows)
					outZ.at<float>(imagePos.y(), imagePos.x()) = z;
			}
		}
	}
	/*
	pushFrameAs<PCDData>("outPCD", pcd);
	if (outZImageConnected)
		pushFrameAs<ImageData>("outZ", outZ);*/
	return pcd;
}


#ifdef debugPCD

void PCDColorize(PCDPtr pcd, int R, int G, int B)
{
	PCDPtr outPCD(new PCD());
	outPCD->resize(pcd->size());

	for (size_t i = 0; i < pcd->size(); ++i)
	{
		PCD::PointType p = pcd->points[i];
		p.r = 255;
		p.g = 255;
		p.b = 255;
		outPCD->points[i] = p;
	}
	pcd = outPCD;
}

bool update = false;
boost::mutex updateModelMutex;
PCDPtr cloud(new PCD);
pcl::visualization::PointCloudColorHandlerGenericField<PCDPoint> colorHandler(cloud, "intensity");

void visualize()
{
	static pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
	viewer.addPointCloud(cloud, "sample cloud");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		// Get lock on the boolean update and check if cloud was updated
		boost::mutex::scoped_lock updateLock(updateModelMutex);
		if (update)
		{
			if (!viewer.updatePointCloud(cloud, "sample cloud"))
				viewer.addPointCloud(cloud, "sample cloud");
			update = false;
		}
		updateLock.unlock();
	}
}
#endif
void imgPath(std::string* workpath, std::string* depth_name, std::string* amplitude_name)
{

#ifdef Tanszekiervin
	*workpath = "D:/OneDrive/Visual studio/defkepek/Tanszekiervin/";
	*depth_name = "013654251447_depth_";
	*amplitude_name = "013654251447_rgb_";
#endif

#ifdef Ervinexport
	*workpath = "D:/OneDrive/Visual studio/defkepek/Ervinexport/";
	*depth_name = "depth_" + std::to_string(CAM_NUM) + "_";
	*amplitude_name = "ampli_" + std::to_string(CAM_NUM) + "_";
#endif

#ifdef Enexport
	*workpath = "D:/OneDrive/Visual studio/defkepek/Enexport/";
	*depth_name = "013654251447_depth_";
	*amplitude_name = "013654251447_rgb_";
#endif

#ifdef Zinemath
	*workpath = "D:/OneDrive/Visual studio/defkepek/Somi/";
	*depth_name = "image";
#endif
}

int main()
{
	std::string workpath, depth_name, amplitude_name;
	imgPath(&workpath, &depth_name, &amplitude_name);

	int loader_iter = ITERATOR_MIN;
	PCDPtr pcdThis,pcdBefore;
#ifdef debugPCD
	boost::thread workerThread(visualize);
#endif
	while (loader_iter < ITERATOR_MAX)
	{
		string image_name;
		if (loader_iter < 10) image_name = "000" + std::to_string(loader_iter) + ".png";
		else if (loader_iter<100) image_name = "00" + std::to_string(loader_iter) + ".png";
		else if (loader_iter<1000) image_name = "0" + std::to_string(loader_iter) + ".png";
		else  image_name = std::to_string(loader_iter);
		string camNumPath = "";
#ifdef CAM_NUM
		camNumPath = std::to_string(CAM_NUM) + "/";
#endif
		string workpath_dimg = workpath + "Depth/" + camNumPath + depth_name + image_name;
		string workpath_rgbimg = workpath + "RGB/" + camNumPath + amplitude_name + image_name;

		cv::Mat depth_image_u, depth_image, depth_image_1c, rgb_img; //depth_image_, rgb_img
		depth_image_u = cv::imread(workpath_dimg); // Read img to unsigned
		rgb_img = cv::imread(workpath_rgbimg); // Read img to unsigned
		if (depth_image_u.empty()) // Check for invalid input
		{
#ifdef debug
			cout << "Could not open or find the depth image" << std::endl;
			cout << "Iteration: " << loader_iter << "path: " << workpath_dimg << std::endl;
			cv::waitKey(0);
#endif
			return -1;
		}
		if(rgb_img.empty() & !amplitude_name.empty())
		{
#ifdef debug
			cout << "Could not open or find the RGB image" << std::endl;
			cout << "Iteration: " << loader_iter << "path: " << workpath_rgbimg << std::endl;
			cv::waitKey(0);
#endif
			return -1;
		}

		cv::cvtColor(depth_image_u, depth_image_1c, CV_BGR2GRAY);//depth_image was an RGB, with same RGBpixel values -> greyscale
		depth_image_1c.convertTo(depth_image, CV_32F);
		
		//SZURES
		cv::Mat median_res, gaussian_res;
		cv::medianBlur(depth_image, median_res, 3);
		cv::GaussianBlur(median_res, gaussian_res, cv::Size(3, 3), 1.8);

		//ELOZO PCD MENTES
		if (loader_iter - ITERATOR_MIN > 1)
		{
			pcdBefore = pcdThis; //"Before": all before, "this": from this depth img.
		}

		//ATLAGOLAS
		//??

		//PCD KESZITES
		Intrinsic intrinsic(418.920523, 418.285805, 137.401996, 122.136574, 320, 240);// <item key="intrinsic" type="Intrinsic" encoding="text">418.920523 418.285805 137.401996 122.136574 320 240</item>
		Distortion distortion(-0.40322, 0.706050, 0.001742, -0.002878, -1.370257);// <item key = "distorsion" type = "Distortion" encoding = "text">-0.40322 0.706050 0.001742 - 0.002878 - 1.370257< / item>
		std::string imgType = "DImage";
		std::string coordinateSys = "OpenGL";
		pcdThis = DepthToPCD(gaussian_res, intrinsic, distortion, imgType, coordinateSys);
		if (pcdThis == NULL)
		{
			cout << "PCD is nullpointer" << endl;
			cv::waitKey(0);
			return -1;
		}

		//PCD MEGJELENITES
#ifdef debugPCD
		boost::mutex::scoped_lock updateLock(updateModelMutex);
		//PCDColorize(pcdThis, 255, 255, 255);
		cloud = pcdThis;
		update = true;
#endif

		//KOZOS PCD
		
		//cv::resize(amplitude_image_, amplitude_image, cv::Size(1920, 1080), 0, 0, cv::INTER_LINEAR);
		//cv::Mat depth_image;
		//cv::resize(dpt_image, depth_image, cv::Size(amplitude_image.cols, amplitude_image.rows), 0, 0, cv::INTER_LINEAR);
		

		//TODO list ---------------------
		//Szures
			//melyseg gauss
			//melyseg median??
			//amplitudo hisztogram
		//PCD
			//pontfelho keszites
		//KozosPCD
			//Iterative closest point
		//
		


		//-------------kibaszott meres ------------------------//
		/*
		double freq = cvGetTickFrequency();
		int64 e1;
		int64 e2;
		double time;
		e1 = cvGetTickCount();



		//Insert algorithm here!




		e2 = cvGetTickCount();
		time = (e2 - e1) / freq;
		cout << time << " sec volt a futasi ido" << endl;
		*/


#ifdef debugPCD
		updateLock.unlock();
#endif
#ifdef debug
		//Visualization part
#endif
		cout << "Picture number: " << loader_iter-ITERATOR_MIN<< endl;
		loader_iter++;
	}
#ifdef debugPCD
	workerThread.join();
#endif
#ifdef debug
	cv::waitKey(0);
#endif
	return 0;
}