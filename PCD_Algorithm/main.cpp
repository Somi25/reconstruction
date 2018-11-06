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
#include "defs.h"
#include "Intrinsic.hpp"
#include "Distortion.hpp"
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>

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
typedef pcl::PointCloud<pcl::PointXYZRGB> PCD;
typedef boost::shared_ptr<PCD> PCDPtr;
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
/*
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "i only run once" << std::endl;

}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	//user_data++;
}
*/
/*
void graphCut(cv::Mat inAmpitude, cv::Mat inDepth, cv::Mat* outBin)
{
	int* maxidx = 0;
	int* minidx = 0;
	double maxval, minval;
	cv::minMaxIdx(inDepth, &minval, &maxval, minidx, maxidx, cv::noArray());

	//PFGD create
	cv::Mat hist;
	int histSize = 256;
	float range[] = { 0, 255 };
	const float* histRange = { range };
	cv::calcHist(&inDepth, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);

	int thresh;
	double sum = 0.0;
	for (int i = 0; i < histSize; i++)
	{
		sum += hist.at<float>(i, 0);

		if (sum >= inDepth.cols*inDepth.rows*0.4)
		{
			thresh = i;
			break;
		}
	}
	cv::Mat maskPFGD, _maskPFGD;
	cv::threshold(inDepth, maskPFGD, thresh, 3, cv::THRESH_BINARY_INV);
	cv::dilate(maskPFGD, _maskPFGD, cv::Mat(), cv::Point(-1, -1), 3);
	cv::erode(_maskPFGD, maskPFGD, cv::Mat(), cv::Point(-1, -1), 3);
	//PFGD kész

	//PBGD create
	cv::Mat maskPBGD;
	cv::bitwise_not(maskPFGD, maskPBGD);
	maskPBGD -= 253; //(~(uchar)3)+1
					 //PBGD kész

					 //FGD create
#ifdef Tanszekiervin
	cv::Mat thresholdedFRONT, thresholdedERROR_, thresholdedERROR;
	cv::threshold(inDepth, thresholdedERROR_, minval, 1, cv::THRESH_BINARY_INV);
	cv::dilate(thresholdedERROR_, thresholdedERROR, cv::Mat(), cv::Point(-1, -1), 5);
	cv::threshold(inDepth, thresholdedFRONT, minval + 6, 1, cv::THRESH_BINARY_INV);
	cv::Mat maskFGD_;
	maskFGD_ = thresholdedFRONT - thresholdedERROR;
#else
	cv::Mat maskFGD_;
	sum = 0.0;
	for (int i = 0; i < histSize; i++)
	{
		sum += hist.at<float>(i, 0);

		if (sum >= inDepth.cols*inDepth.rows*0.04)
		{
			thresh = i;
			break;
		}
	}
	cv::threshold(inDepth, maskFGD_, thresh, 1, cv::THRESH_BINARY_INV);
#endif

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::Mat maskFGD;
	cv::dilate(maskFGD_, maskFGD, cv::Mat(), cv::Point(-1, -1), 5);
	cv::erode(maskFGD, maskFGD_, cv::Mat(), cv::Point(-1, -1), 5);
	cv::findContours(maskFGD_, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	maskFGD_ -= 1;

	for (int i = 0; i < contours.size(); i++)
		if (cv::contourArea(contours[i]) > 1000)
			cv::drawContours(maskFGD_, contours, i, 2, CV_FILLED);

	cv::erode(maskFGD_, maskFGD, cv::Mat(), cv::Point(-1, -1), 10);
	//FGD kész

	//BGD create
	cv::Mat maskBGD, maskBGD_;
	sum = 0.0;
	for (int i = 0; i < histSize; i++)
	{
		sum += hist.at<float>(i, 0);

		if (sum >= inDepth.cols*inDepth.rows*0.96)
		{
			thresh = i;
			break;
		}
	}
	cv::threshold(inDepth, maskBGD_, thresh, 1, cv::THRESH_BINARY);
	cv::dilate(maskBGD_, maskBGD, cv::Mat(), cv::Point(-1, -1), 5);
	cv::erode(maskBGD, maskBGD_, cv::Mat(), cv::Point(-1, -1), 15);

	//BGD kész

	//Final sums
	cv::Mat maskF;
	maskF = maskPFGD - maskFGD;

	cv::Mat maskB;
	maskB = maskPBGD - maskBGD;

	cv::Mat mask = maskB + maskF;
	//--------------------------------------------------------------------------------------
	cv::MatIterator_<uchar> m_it, m_end;
#pragma omp for
	for (m_it = mask.begin<uchar>(), m_end = mask.end<uchar>(); m_it != m_end; ++m_it)
	{
		if (*m_it != 1)
			*m_it = 0;
	}
	*outBin = mask;
	return;
	//------------------------------------------------------------------------------------------
	//GRABCUT
	cv::Mat bgdModel, fgdModel;
	cv::grabCut(inAmpitude, mask, cv::Rect(), bgdModel, fgdModel, 1, cv::GrabCutModes::GC_INIT_WITH_MASK);

	//	cv::MatIterator_<uchar> m_it, m_end;
#pragma omp for
	for (m_it = mask.begin<uchar>(), m_end = mask.end<uchar>(); m_it != m_end; ++m_it)
	{
		if (*m_it != 1)
			*m_it = 0;
	}
	cv::MatIterator_<cv::Vec3b> it, end;
#pragma omp for
	for (m_it = mask.begin<uchar>(), it = inAmpitude.begin<cv::Vec3b>(), end = inAmpitude.end<cv::Vec3b>(); it != end; ++it, ++m_it)
	{
		(*it)[0] *= *m_it;
		(*it)[1] *= *m_it;
		(*it)[2] *= *m_it;
	}
	inAmpitude.copyTo(*outBin);
	return;
}

void waterShed(cv::Mat inAmpitude, cv::Mat inDepth, cv::Mat* outImg)
{
	cv::Mat img_a, img_b;
	cv::medianBlur(inDepth, img_a, 3);
	cv::GaussianBlur(img_a, img_b, cv::Size(3, 3), 0.6);
	cv::adaptiveThreshold(img_b, img_a, 255, cv::AdaptiveThresholdTypes::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 21, 0);
	cv::distanceTransform(img_a, img_b, cv::DistanceTypes::DIST_L2, cv::DistanceTransformMasks::DIST_MASK_3, 5);

	int* maxidx = 0;
	int* minidx = 0;
	double maxval, minval;
	cv::minMaxIdx(img_b, &minval, &maxval, minidx, maxidx, cv::noArray());

	cv::threshold(img_b, img_a, maxval*0.4, 255, cv::THRESH_BINARY);

	img_a.convertTo(img_b, CV_8U);
	cv::dilate(img_b, img_a, cv::Mat(), cv::Point(-1, -1), 3);
	cv::erode(img_a, img_b, cv::Mat(), cv::Point(-1, -1), 5);
	std::vector<std::vector<cv::Point> > contours, valid_contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(img_b, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	img_b *= 0;
	int to = contours.size() < 3 ? contours.size() : 3;
	for (int are = 0; are < to; are++)
	{
		valid_contours.push_back(contours[0]);
		for (int i = 0; i < contours.size(); i++)
		{
			if (are == 0)
			{
				if (cv::contourArea(valid_contours[are]) < cv::contourArea(contours[i]))
				{
					valid_contours[are] = contours[i];
				}
			}
			else if (are == 1)
			{
				if (cv::contourArea(valid_contours[are]) < cv::contourArea(contours[i]) && cv::contourArea(valid_contours[0]) != cv::contourArea(contours[i]))
				{
					valid_contours[are] = contours[i];
				}
			}
			else
			{
				if (cv::contourArea(valid_contours[are]) < cv::contourArea(contours[i]) && cv::contourArea(valid_contours[0]) != cv::contourArea(contours[i]) && cv::contourArea(valid_contours[1]) != cv::contourArea(contours[i]))
				{
					valid_contours[are] = contours[i];
				}
			}
		}
		drawContours(img_b, valid_contours, are, cv::Scalar({ are + 1.0,are + 1.0,are + 1.0 }), CV_FILLED, 8, cv::noArray(), 0, cv::Point(0, 0));
	}

	*outImg = inAmpitude;
	img_b.convertTo(img_a, CV_32SC1);
	cv::watershed(*outImg, img_a);
	img_a.convertTo(img_b, CV_8UC1);
	img_b -= 1;
	cv::MatIterator_<uchar> m_it;
	cv::MatIterator_<cv::Vec3b> it, end;
#pragma omp for
	for (m_it = img_b.begin<uchar>(), it = outImg->begin<cv::Vec3b>(), end = outImg->end<cv::Vec3b>(); it != end; ++it, ++m_it)
	{
		(*it)[0] *= *m_it;
		(*it)[1] *= *m_it;
		(*it)[2] *= *m_it;
	}
}
*/

int main()
{
	string workpath, depth_name, amplitude_name;

#ifdef Tanszekiervin
	workpath = "D:/OneDrive/Visual studio/defkepek/Tanszekiervin/";
	depth_name = "013654251447_depth_";
	amplitude_name = "013654251447_rgb_";
#endif

#ifdef Ervinexport
	workpath = "D:/OneDrive/Visual studio/defkepek/Ervinexport/";
	depth_name = "depth_" + std::to_string(CAM_NUM) + "_";
	amplitude_name = "ampli_" + std::to_string(CAM_NUM) + "_";
#endif

#ifdef Enexport
	workpath = "D:/OneDrive/Visual studio/defkepek/Enexport/";
	depth_name = "013654251447_depth_";
	amplitude_name = "013654251447_rgb_";
#endif

#ifdef Zinemath
	workpath = "D:/OneDrive/Visual studio/defkepek/Somi/";
	depth_name = "image";
#endif
	int loader_iter = ITERATOR_MIN;
	PCDPtr pcdThis,pcdBefore;

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

		cv::Mat depth_image_u, depth_image_f, depth_image_1c, rgb_img; //depth_image_, rgb_img
		depth_image_u = cv::imread(workpath_dimg); // Read the file
		rgb_img = cv::imread(workpath_rgbimg); // Read the file
		if (depth_image_u.empty()) // Check for invalid input
		{
			cout << "Could not open or find the depth image" << std::endl;
			return -1;
		}
		if(rgb_img.empty() & !amplitude_name.empty())
		{
			cout << "Could not open or find the RGB image" << std::endl;
			return -1;
		}
		type_debug(depth_image_u);
		depth_image_u.convertTo(depth_image_f, CV_32F);
		type_debug(depth_image_f);
		cv::cvtColor(depth_image_f, depth_image_1c, CV_BGR2GRAY);//depth_image was an RGB, with same RGBpixel values -> greyscale
		type_debug(depth_image_1c);
		
#ifdef debug
		//type_debug(rgb_img);
		cv::imshow("Depth", depth_image_1c); // Show our image inside it.
		//cv::waitKey(0); // Wait for a keystroke in the window
#endif
		//SZURES
		cv::Mat median_res, gaussian_res;
		cv::medianBlur(depth_image_1c, median_res, 3);
		cv::GaussianBlur(median_res, gaussian_res, cv::Size(3, 3), 1.8);

		//ELOZO PCD KITERJESZTES
		if (loader_iter - ITERATOR_MIN > 1)
		{
			pcdBefore = pcdThis; //"Before": all before, "this": from this depth img.
		}

		//ATLAGOLAS
		//PCD KESZITES
			/*
				<item key="distorsion" type="Distortion" encoding="text">-0.40322 0.706050 0.001742 -0.002878 -1.370257</item>
				<item key="intrinsic" type="Intrinsic" encoding="text">418.920523 418.285805 137.401996 122.136574 320 240</item>
			*/
		Intrinsic intrinsic(418.920523, 418.285805, 137.401996, 122.136574, 320, 240);
		Distortion distortion(-0.40322, 0.706050, 0.001742, -0.002878, -1.370257);
		std::string imgType = "DImage";
		std::string coordinateSys = "OpenGL";
		pcdThis = DepthToPCD(gaussian_res, intrinsic, distortion, imgType, coordinateSys);

		//KOZOS PCD

		PCDColorize(pcdThis, 255, 255, 255);
		pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
		viewer.showCloud(pcdThis);

		///*
		while (!viewer.wasStopped())
		{
		}
		 //*/
		/*checkthis
		viewer.runOnVisualizationThreadOnce(viewerOneOff);
		viewer.runOnVisualizationThread(viewerPsycho);
		while (!viewer.wasStopped())
		{
			//you can also do cool processing here
			//FIXME: Note that this is running in a separate thread from viewerPsycho
			//and you should guard against race conditions yourself...
			//user_data++;
		}
		*/
		//cv::resize(amplitude_image_, amplitude_image, cv::Size(1920, 1080), 0, 0, cv::INTER_LINEAR);
		//cv::Mat depth_image;
		//cv::resize(dpt_image, depth_image, cv::Size(amplitude_image.cols, amplitude_image.rows), 0, 0, cv::INTER_LINEAR);
		

		//TODO list ---------------------
		//Szures
			//melyseg gauss
			//melyseg median??
		//Atlagolas
			//amplitudo hisztogram
			//melyseg atlagolas
		//PDC [kerdezni]
			//pontfelho keszites
		//KozosPCD [olvasni]
			//kozos pontfelhobe helyezes
		//PCD ATLAGOLAS!!! (magic) [olvasni]
			//kozos pontfelho igazitasa


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







		//--------------melyseglyukasztas----------------------//
		/*
		//cv::Mat grabCut_out = cv::Mat(amplitude_image.cols, amplitude_image.rows, CV_8UC1);
		//e1 = cvGetTickCount();
		//graphCut(amplitude_image, depth_image, &grabCut_out);
		//e2 = cvGetTickCount();
		//time = (e2 - e1) / freq;
		//cout << time << " sec volt a futasi ido GC" << endl;
		//cv::namedWindow("grabCut_out", cv::WINDOW_NORMAL); // Create a window for display.
		//cv::imshow("grabCut_out", grabCut_out); // Show our image inside it.
		//cv::Mat mask;
		//cv::resize(grabCut_out, mask, cv::Size(amplitude_image_.cols, amplitude_image_.rows), 0, 0, cv::INTER_LINEAR);
		//cv::imwrite(workpath + "mask" + image_name, mask);
		//cv::waitKey(0); // Wait for a keystroke in the window
		
		cv::Mat watershedOut = cv::Mat(amplitude_image.cols, amplitude_image.rows, CV_8UC3);
		e1 = cvGetTickCount();
		waterShed(amplitude_image, depth_image, &watershedOut);
		e2 = cvGetTickCount();
		time = (e2 - e1) / freq;
		cout << time << " sec volt a futasi ido WS" << endl;
		cv::imshow("Watershed Out", watershedOut); // Show our image inside it.
		cv::waitKey(0); // Wait for a keystroke in the window
		//cv::imwrite(workpath + "watershed" + image_name, watershedOut);
		*/
		//--------------melyseglyukasztas----------------------//
#ifdef debug
		cv::imshow("Depth", depth_image_1c); // Show our image inside it.
#endif
		loader_iter++;
	}
	return 0;
}