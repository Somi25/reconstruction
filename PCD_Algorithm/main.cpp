#include "defs.h"
#include "DepthToPCD.hpp"

using namespace std;

void cout_messages(std::string message)
{
#ifdef showStatus
	cout << message << endl;
#endif
}

//All the functions for showingPCDs
#ifdef debugPCD
#if shownWindowsNum > 0
boost::mutex updateAModelMutex;
PCDcPtr cloudShowA(new PCDc);
bool updateA = false;
#endif
#if shownWindowsNum > 1
boost::mutex updateBModelMutex;
PCDcPtr cloudShowB(new PCDc);
bool updateB = false;
#endif
#if shownWindowsNum > 2
boost::mutex updateCModelMutex;
PCDcPtr cloudShowC(new PCDc);
bool updateC = false;
#endif
#if shownWindowsNum > 3
boost::mutex updateDModelMutex;
PCDcPtr cloudShowD(new PCDc);
bool updateD = false;
#endif
#if shownWindowsNum > 4
boost::mutex updateEModelMutex;
PCDcPtr cloudShowE(new PCDc);
bool updateE = false;
#endif
#if shownWindowsNum > 5
boost::mutex updateFModelMutex;
PCDcPtr cloudShowF(new PCDc);
bool updateF = false;
#endif

#if shownWindowsNum > 0
void visualizeA()
{
	static pcl::visualization::PCLVisualizer viewerA("Show A");
	viewerA.addPointCloud(cloudShowA, "Cloud A");

	while (!viewerA.wasStopped())
	{
		viewerA.spinOnce(100);
		// Get lock on the boolean update and check if cloud was updated
		boost::mutex::scoped_lock updateLockA(updateAModelMutex);
		if (updateA)
		{
			if (!viewerA.updatePointCloud(cloudShowA, "Cloud A"))
				viewerA.addPointCloud(cloudShowA, "Cloud A");
			updateA = false;
		}
		updateLockA.unlock();
	}
}
#endif
#if shownWindowsNum > 1
void visualizeB()
{
	static pcl::visualization::PCLVisualizer viewerB("Show B");
	viewerB.addPointCloud(cloudShowB, "Cloud B");

	while (!viewerB.wasStopped())
	{
		viewerB.spinOnce(100);
		// Get lock on the boolean update and check if cloud was updated
		boost::mutex::scoped_lock updateLockB(updateBModelMutex);
		if (updateB)
		{
			if (!viewerB.updatePointCloud(cloudShowB, "Cloud B"))
				viewerB.addPointCloud(cloudShowB, "Cloud B");
			updateB = false;
		}
		updateLockB.unlock();
	}
}

#endif
#if shownWindowsNum > 2
void visualizeC()
{
	static pcl::visualization::PCLVisualizer viewerC("Show C");
	viewerC.addPointCloud(cloudShowC, "Cloud C");

	while (!viewerC.wasStopped())
	{
		viewerC.spinOnce(100);
		// Get lock on the boolean update and check if cloud was updated
		boost::mutex::scoped_lock updateLockC(updateCModelMutex);
		if (updateC)
		{
			if (!viewerC.updatePointCloud(cloudShowC, "Cloud C"))
				viewerC.addPointCloud(cloudShowC, "Cloud C");
			updateC = false;
		}
		updateLockC.unlock();
	}
}

#endif
#if shownWindowsNum > 3
void visualizeD()
{
	static pcl::visualization::PCLVisualizer viewerD("Show D");
	viewerD.addPointCloud(cloudShowD, "Cloud D");

	while (!viewerD.wasStopped())
	{
		viewerD.spinOnce(100);
		// Get lock on the boolean update and check if cloud was updated
		boost::mutex::scoped_lock updateLockD(updateDModelMutex);
		if (updateD)
		{
			if (!viewerD.updatePointCloud(cloudShowD, "Cloud D"))
				viewerD.addPointCloud(cloudShowD, "Cloud D");
			updateD = false;
		}
		updateLockD.unlock();
	}
}

#endif
#if shownWindowsNum > 4
void visualizeE()
{
	static pcl::visualization::PCLVisualizer viewerE("Show E");
	viewerE.addPointCloud(cloudShowE, "Cloud E");

	while (!viewerE.wasStopped())
	{
		viewerE.spinOnce(100);
		// Get lock on the boolean update and check if cloud was updated
		boost::mutex::scoped_lock updateLockE(updateEModelMutex);
		if (updateE)
		{
			if (!viewerE.updatePointCloud(cloudShowE, "Cloud E"))
				viewerE.addPointCloud(cloudShowE, "Cloud E");
			updateE = false;
		}
		updateLockE.unlock();
	}
}

#endif
#if shownWindowsNum > 5
void visualizeF()
{
	static pcl::visualization::PCLVisualizer viewerF("Show F");
	viewerF.addPointCloud(cloudShowF, "Cloud F");

	while (!viewerF.wasStopped())
	{
		viewerF.spinOnce(100);
		// Get lock on the boolean update and check if cloud was updated
		boost::mutex::scoped_lock updateLockF(updateFModelMutex);
		if (updateF)
		{
			if (!viewerF.updatePointCloud(cloudShowF, "Cloud F"))
				viewerF.addPointCloud(cloudShowF, "Cloud F");
			updateF = false;
		}
		updateLockF.unlock();
	}
}
#endif

void PCDColorize(PCDPtr inPcd, PCDcPtr outPtr, bool autoColor, uint8_t R = 255, uint8_t G = 255, uint8_t B = 255)
{
	outPtr->resize(inPcd->size());
	if (autoColor)
	{
		for (size_t i = 0; i < inPcd->size(); ++i)
		{
			PCDc::PointType p;
			p.x = inPcd->points[i].x;
			p.y = inPcd->points[i].y;
			p.z = p.r = p.g = p.b = inPcd->points[i].z;
			outPtr->points[i] = p;
		}
	}
	else
	{
		for (size_t i = 0; i < inPcd->size(); ++i)
		{
			PCDc::PointType p;
			p.x = inPcd->points[i].x;
			p.y = inPcd->points[i].y;
			p.z = inPcd->points[i].z;
			p.r = R;
			p.g = G;
			p.b = B;
			outPtr->points[i] = p;
		}
	}
}
void PCDColorize(PCDPtr inPcd, PCDcPtr outPtr, uint8_t R, uint8_t G, uint8_t B)
{
	PCDColorize(inPcd, outPtr, false, R, G, B);
}
void showPCD(PCDPtr inPcd, int viewerNum) // viewer num is from 1 - shownWindowsNum
{
	if (viewerNum > shownWindowsNum || viewerNum < 1)
	{
#ifdef showStatus
		cout << " viewerNum is not valid in showPCD" << endl;
#endif
		return;
	}

#if shownWindowsNum > 0
	if (viewerNum == 1)
	{
		boost::mutex::scoped_lock updateLockA(updateAModelMutex);
		updateA = false;
		PCDColorize(inPcd, cloudShowA, false);
		updateA = true;
		updateLockA.unlock();
	}
#endif

#if shownWindowsNum > 1
	if (viewerNum == 2)
	{
		boost::mutex::scoped_lock updateLockB(updateBModelMutex);
		updateB = false;
		PCDColorize(inPcd, cloudShowB, false);
		updateB = true;
		updateLockB.unlock();
	}
#endif

#if shownWindowsNum > 2
	if (viewerNum == 3)
	{
		boost::mutex::scoped_lock updateLockC(updateCModelMutex);
		updateC = false;
		PCDColorize(inPcd, cloudShowC, false);
		updateC = true;
		updateLockC.unlock();
	}
#endif

#if shownWindowsNum > 3
	if (viewerNum == 4)
	{
		boost::mutex::scoped_lock updateLockD(updateDModelMutex);
		updateD = false;
		PCDColorize(inPcd, cloudShowD, false);
		updateD = true;
		updateLockD.unlock();
	}
#endif

#if shownWindowsNum > 4
	if (viewerNum == 5)
	{
		boost::mutex::scoped_lock updateLockE(updateEModelMutex);
		updateE = false;
		PCDColorize(inPcd, cloudShowE, false);
		updateE = true;
		updateLockE.unlock();
	}
#endif

#if shownWindowsNum > 5
	if (viewerNum == 6)
	{
		boost::mutex::scoped_lock updateLockF(updateFModelMutex);
		updateF = false;
		PCDColorize(inPcd, cloudShowF, false);
		updateF = true;
		updateLockF.unlock();
	}
#endif
}
#endif

void type_debug(cv::Mat in);
void saturation(cv::Mat& in, float alpha);
int hist_max(cv::Mat in);
void hist_debug(cv::Mat in, bool interestInHoleFill);
void HoleFill(cv::Mat& image, int iterations = 1);

void HoleFill(cv::Mat& image, int iterations)
{
	if (image.type() != CV_32FC1)
	{
		cout_messages("Only CV_32FC1 is supported");
		return;
	}
	int kernelSize = holeFillKERNEL; //3,5,7..
	double threshold = 1.0;

	iterations = std::max(iterations, 0);


	for (int i = 0; i < iterations; ++i)
	{
		int XX = image.cols;
		int YY = image.rows;
		if (i == 0)
		{
			for (int y = 0; y < YY; y++)
			{
				for (int x = 0; x < XX; x++)
				{
					float tmp = image.at<float>(y, x);
					if (tmp <= threshold || std::isnan(tmp) || std::isinf(tmp))
					{
						float res = 0.0f;
						int found = 0;
						//define kernel var
						int kernel_var = (kernelSize - 1) / 2;
						//calculate limits
						int lim_dx_b = -kernel_var;
						int lim_dx_t = kernel_var;
						int lim_dy_b = -kernel_var;
						int lim_dy_t = kernel_var;
						for (int k = 0; k < kernel_var; k++)
						{
							if (x == k)
								lim_dx_b = -k;
							if (XX - 1 - x == k)
								lim_dx_t = k;
							if (y == k)
								lim_dy_b = -k;
							if (YY - 1 - y == k)
								lim_dy_t = k;
						}

						for (int dx = lim_dx_b; dx <= lim_dx_t; dx++)
						{
							for (int dy = lim_dy_b; dy <= lim_dy_t; dy++)
							{
								if (x + dx >= 0 && y + dy >= 0 && x + dx < XX && y + dy < YY)
								{
									float val = image.at<float>(y + dy, x + dx);
									if (val > threshold && !std::isnan(val) && !std::isinf(val))
									{
										res += val;
										found++;
									}
								}
							}
						}
						int area = (lim_dx_t - lim_dx_b)*(lim_dy_t - lim_dy_b);
						if (found >= (area - 1) / 2)
							image.at<float>(y, x) = res / found;
						else if (iterations > 1)
						{
							static vector<cv::Point3i> missing;
							missing.push_back(cv::Point3i(x, y, 1));
						}
					}
				}
			}
		}
		else
		{
			static vector<cv::Point3i> missing; //against compile errors
			for (int counter = 0; counter < missing.size(); counter++)
			{
				if (missing[counter].z) //0 if found
				{
					int x = missing[counter].x, y = missing[counter].y;
					float tmp = image.at<float>(y, x);
					if (tmp <= threshold || std::isnan(tmp) || std::isinf(tmp))
					{
						float res = 0.0f;
						int found = 0;
						//define kernel var
						int kernel_var = (kernelSize - 1) / 2;
						//calculate limits
						int lim_dx_b = -kernel_var;
						int lim_dx_t = kernel_var;
						int lim_dy_b = -kernel_var;
						int lim_dy_t = kernel_var;
						for (int k = 0; k < kernel_var; k++)
						{
							if (x == k)
								lim_dx_b = -k;
							if (XX - 1 - x == k)
								lim_dx_t = k;
							if (y == k)
								lim_dy_b = -k;
							if (YY - 1 - y == k)
								lim_dy_t = k;
						}

						for (int dx = lim_dx_b; dx <= lim_dx_t; dx++)
						{
							for (int dy = lim_dy_b; dy <= lim_dy_t; dy++)
							{
								if (x + dx >= 0 && y + dy >= 0 && x + dx < XX && y + dy < YY)
								{
									float val = image.at<float>(y + dy, x + dx);
									if (val > threshold && !std::isnan(val) && !std::isinf(val))
									{
										res += val;
										found++;
									}
								}
							}
						}
						int area = (lim_dx_t - lim_dx_b)*(lim_dy_t - lim_dy_b);
						if (found >= (area - 1) / 2)
						{
							image.at<float>(y, x) = res / found;
							missing[counter].z = 0;
						}
					}
				}
			}
		}
	}
}

void saturation(cv::Mat& in, float alpha)
{
	if (in.type() != CV_32FC1)
	{
		cout_messages("Only CV_32FC1 is supported");
		return;
	}
	for (int y = 0; y < in.rows; y++)
	{
		for (int x = 0; x < in.cols; x++)
		{
			in.at<float>(y, x) *= alpha;
		}
	}
}

void downsamplePCD(PCDPtr pcdIO,float size = 0.05)
{
	PCDPtr pcd(new PCD);
	Eigen::Vector3i min;
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(pcdIO);
	grid.setLeafSize(size, size, size);
	grid.filter(*pcd);
	pcdIO = pcd;
}

void filterDepth(cv::Mat& depth_image)
{
	cv::Mat temp;
#ifdef useHoleFill
	HoleFill(depth_image);
#endif
#ifdef useMedian
	cv::medianBlur(depth_image, temp, medianKERNEL);
	depth_image = temp;
#endif
#ifdef useGaussian
	cv::GaussianBlur(depth_image, temp, cv::Size(5, 5), 1.8);
	depth_image = temp;
#endif
}

void preparingDepth(cv::Mat& depth_image)
{
	cv::Mat temp, temp2, temp3;
#ifdef resizeSRC
	cv :: Size newSize(1366,768);
	cv::resize(depth_image, temp, newSize,0,0, cv::INTER_LANCZOS4);
	depth_image = temp;
#endif
#ifdef useCrop
	cv::Rect myROI(60, 60, depth_image.cols - 60, depth_image.rows - 60);
	cv::Mat newDepth = depth_image(myROI);
	depth_image = newDepth;
#endif
	uchar chans = 1 + (depth_image.type() >> CV_CN_SHIFT);
	if (chans>1)
	{
		//depth_image was an RGB, with same RGBpixel values -> greyscale
		cv::cvtColor(depth_image, temp2, CV_BGR2GRAY);	//rgb->gray
	}
	uchar depth = depth_image.type() & CV_MAT_DEPTH_MASK;
	switch (depth) {
	case CV_8U:;
	case CV_8S:  depth = 8; break;
	case CV_16U:;
	case CV_16S: depth = 16;  break;
	case CV_32S: depth = 32; break;
	case CV_32F: depth = 0; break;
	case CV_64F: depth = 1; break;
	default:    return; //throw error
	}
	if (depth)
	{
		temp3 = depth_image;
		temp3.convertTo(depth_image, CV_32FC1);			//unsigned16->float32
	}
}

void imgPath(std::string* workpath, std::string* depth_name, std::string* amplitude_name)
{

#ifdef Tanszekiervin
	*workpath = (string)getenv("OneDrive") + "/Visual studio/defkepek/Tanszekiervin/";
	*depth_name = "013654251447_depth_";
	*amplitude_name = "013654251447_rgb_";
#endif

#ifdef Ervinexport
	*workpath = (string)getenv("OneDrive") + "/Visual studio/defkepek/Ervinexport/";
	*depth_name = "depth_" + std::to_string(CAM_NUM) + "_";
	*amplitude_name = "ampli_" + std::to_string(CAM_NUM) + "_";
#endif

#ifdef Enexport
	*workpath = (string)getenv("OneDrive") + "/Visual studio/defkepek/Enexport/";
	*depth_name = "013654251447_depth_";
	*amplitude_name = "013654251447_rgb_";
#endif

#ifdef Zinemath
	*workpath = (string)getenv("OneDrive") + "/Visual studio/defkepek/Somi/";
	*depth_name = "image";
#endif

#ifdef TanszekMunkaasztal1
	*workpath = (string)getenv("OneDrive") + "/Visual studio/defkepek/TanszekMunkaasztal1/";
	*depth_name = "depth_";
#endif
}

inline int loadImages(cv::Mat& depth, cv::Mat& rgb, std::string& path, std::string& depth_name, std::string& amplitude_name, int imgNumber)
{
	string image_name;
#if (defined(Tanszekiervin) | defined(Ervinexport) | defined(Enexport) | defined(Zinemath))
	if (imgNumber < 10) image_name = "000" + std::to_string(imgNumber) + ".png";
	else if (imgNumber < 100) image_name = "00" + std::to_string(imgNumber) + ".png";
	else if (imgNumber < 1000) image_name = "0" + std::to_string(imgNumber) + ".png";
#endif
#if defined(TanszekMunkaasztal1)
	if (imgNumber < 10) image_name = "00" + std::to_string(imgNumber) + ".png";
	else if (imgNumber < 100) image_name = "0" + std::to_string(imgNumber) + ".png";
	else if (imgNumber < 1000) image_name = std::to_string(imgNumber) + ".png";
#endif
	else  image_name = std::to_string(imgNumber);
	string camNumPath = "";
#ifdef CAM_NUM
	camNumPath = std::to_string(CAM_NUM) + "/";
#endif
	string workpath_dimg = path + "Depth/" + camNumPath + depth_name + image_name;
	string workpath_rgbimg = path + "RGB/" + camNumPath + amplitude_name + image_name;
	
	depth = cv::imread(workpath_dimg, cv::IMREAD_ANYDEPTH); // Read img to unsigned
	rgb = cv::imread(workpath_rgbimg); // Read img to unsigned
	if (depth.empty()) // Check for invalid input
	{
		cout_messages("Could not open or find the depth image");
		//cout << "Iteration: " << imgNumber << "path: " << workpath_dimg << std::endl;
		return 0;
	}
	if (rgb.empty() & !amplitude_name.empty())
	{
		cout_messages("Could not open or find the RGB image");
		//cout << "Iteration: " << imgNumber << "path: " << workpath_rgbimg << std::endl;
		return 0;
	}
	cout_messages("Images loaded");
	return 1;
}

inline void smoothingPCD(PCDPtr in,float rad)
{
	pcl::search::KdTree<PCDPoint>::Ptr tree(new pcl::search::KdTree<PCDPoint>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	PCDn mls_points;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<PCDPoint, PCDnPoint> mls;

	mls.setComputeNormals(true);

	// Set parameters
	mls.setInputCloud(in);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(rad);

	// Reconstruct
	mls.process(mls_points);
	pcl::copyPointCloud(mls_points, *in);
}

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PCDnPoint>
{
	using pcl::PointRepresentation<PCDnPoint>::nr_dimensions_;
public:
	MyPointRepresentation()
	{
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PCDnPoint &p, float * out) const
	{
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

inline void alignPCD(const PCDPtr cloud_src, const PCDPtr cloud_tgt, PCDPtr output, Eigen::Matrix4f &final_transform)
{
	// Compute surface normals and curvature
	PCDnPtr points_with_normals_src(new PCDn);
	PCDnPtr points_with_normals_tgt(new PCDn);

	//normal estimation
	pcl::NormalEstimation<PCDPoint, PCDnPoint> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);
		//for source
	norm_est.setInputCloud(cloud_src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*cloud_src, *points_with_normals_src);
		//for target
	norm_est.setInputCloud(cloud_tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*cloud_tgt, *points_with_normals_tgt);

	//
	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	// Align
	pcl::IterativeClosestPointNonLinear<PCDnPoint, PCDnPoint> reg;
	reg.setTransformationEpsilon(1e-6);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(0.1);
	// Set the point representation
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);

	//Optimize
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	reg.setMaximumIterations(30);//30

	reg.align(*points_with_normals_src);
	//accumulate transformation between each Iteration
	Ti = reg.getFinalTransformation() * Ti;
	//
	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	// Transform target back in source frame
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

	//add the source to the transformed target
	*output += *cloud_src;

	final_transform = targetToSource;
}

//main--------------------------------------------------------------------------------------------------------------------------------------------------------

int main()
{
	std::string workpath, depth_name, amplitude_name;
	imgPath(&workpath, &depth_name, &amplitude_name);

	int loader_iter = ITERATOR_MIN;
	PCDPtr pcdThis(new PCD());
	PCDPtr pcdBefore(new PCD());
	PCDPtr finalResult(new PCD());

	//Open visualizing threads
#ifdef debugPCD
#if shownWindowsNum > 0
	boost::thread workerThreadA(visualizeA);
#endif
#if shownWindowsNum > 1
	boost::thread workerThreadB(visualizeB);
#endif
#if shownWindowsNum > 2
	boost::thread workerThreadC(visualizeC);
#endif
#if shownWindowsNum > 3
	boost::thread workerThreadD(visualizeD);
#endif
#if shownWindowsNum > 4
	boost::thread workerThreadE(visualizeE);
#endif
#if shownWindowsNum > 5
	boost::thread workerThreadF(visualizeF);
#endif
#endif

	while (loader_iter < ITERATOR_MAX)
	{
		cv::Mat depth_image, rgb_img;
		//Load
		if (!loadImages(depth_image, rgb_img, workpath, depth_name, amplitude_name, loader_iter))
		{
			return -1;
		}

		//Conversion
		preparingDepth(depth_image);
		cout_messages("preparingDepth done");

		//Filtering
#ifdef addFilters
		filterDepth(depth_image);
		cout_messages("filtering done");
#endif
		//Bring up the last PCD
		if (loader_iter - ITERATOR_MIN > 0)
		{
			pcdBefore = pcdThis;
		}

		//Making new PCD
		pcdThis = DepthToPCD(depth_image, 100.0);
		if (pcdThis == NULL)
		{
			cout_messages("PCD is nullpointer");
			return -1;
		}
		cout_messages("DepthToPCD done");

#ifdef removeNAN
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*pcdThis, *pcdThis, indices);
		cout_messages("removeNaN done");
#endif
#ifdef downsampleSRC
		downsamplePCD(pcdThis, downsampleRate);
		cout_messages("downsamplePCD done");
#endif
#ifdef smoothingSRC
		smoothingPCD(pcdThis, searchRAD_SRC);
		cout_messages("smoothingPCD done");
#endif
#ifdef downsampleSRC
		downsamplePCD(pcdThis, downsampleRate_SRC);
		cout_messages("downsamplePCD done");
#endif
		//Align PCD
#ifdef addAlign
		if ((loader_iter - ITERATOR_MIN) > 0)
		{
			//Register PCD
			PCDPtr temp(new PCD), result(new PCD);
			Eigen::Matrix4f pairTransform;
			Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
			alignPCD(pcdBefore, pcdThis, temp, pairTransform);
			cout_messages("alignPCD done");
			//transform current pair into the global transform
			pcl::transformPointCloud(*temp, *result, GlobalTransform);
			//update the global transform
			GlobalTransform = GlobalTransform * pairTransform;
			//Visualize align result
			*finalResult += *result;
			cout_messages("finalResult done");
#ifdef downsampleRES
			downsamplePCD(finalResult, downsampleRate_RES);
			cout_messages("downsamplePCD done");
#endif
#ifdef smoothingRES
			smoothingPCD(finalResult, searchRAD_RES);
			cout_messages("smoothingPCD done");
#endif
			//show result
#ifdef debugPCD
			showPCD(finalResult, 1);
#endif
		}
#endif
		//Show some pcds
#ifdef debugPCD
		cout_messages("show..");
#endif

			//-------------kibaszott meres ------------------------//
#ifdef addMeasurement
		double freq = cvGetTickFrequency();
		int64 e1;
		int64 e2;
		double time;
		e1 = cvGetTickCount();


		//Insert algorithm here!


		e2 = cvGetTickCount();
		time = (e2 - e1) / freq;
		cout << time << " sec volt a futasi ido" << endl;
#endif

		std::cout << "Picture number: " << loader_iter-ITERATOR_MIN<< std::endl;
		loader_iter++;
	}

	//join visualizing threads
#ifdef debugPCD
#if shownWindowsNum > 0
	workerThreadA.join();
#endif
#if shownWindowsNum > 1
	workerThreadB.join();
#endif
#if shownWindowsNum > 2
	workerThreadC.join();
#endif
#if shownWindowsNum > 3
	workerThreadD.join();
#endif
#if shownWindowsNum > 4
	workerThreadE.join();
#endif
#if shownWindowsNum > 5
	workerThreadF.join();
#endif
#endif

	return 0;
}


//Functions from former projects----------------------------------------------------------------------------------------------------------


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

void hist_debug(cv::Mat in, bool interestInHoleFill)
{
	int imgCount = 1;
	const int chs = in.channels();
	std::vector<cv::Mat> channels;
	for (int i = 0; i < chs; i++)
	{
		channels.push_back(cv::Mat());
	}
	cv::split(in, channels);
	const uchar depth = in.type() & CV_MAT_DEPTH_MASK; 

	for (int i = 0; i < chs; i++)
	{
		cv::Mat inHist = channels[i];
		cv::Mat hist;
		int histSize = 2048;
		float range[] = { 0, 65535 };
		const float* histRange = { range };

		cv::calcHist(&inHist, imgCount, 0, cv::Mat(), hist, 1, &histSize, &histRange);
		for (int k = 0; interestInHoleFill ? k==0 : (k < histSize) ; k++)
		{
			switch (depth) 
			{
			case CV_8U:
			case CV_8S:
			case CV_16U:
			case CV_16S: 
			case CV_32S:cout << (int)hist.at<float>(k, 0) << "db, " << k << " val, " << i << " channel" << endl; break;
			case CV_32F:cout << (int)hist.at<float>(k, 0) << "db, " << k << " val, " << i << " channel" << endl; break;
			case CV_64F:cout << (int)hist.at<float>(k, 0) << "db, " << k << " val, " << i << " channel" << endl; break;
			default: break;
			}
			
		}
	}
}

int hist_max(cv::Mat in)
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
		for (int k = histSize-1; k > -1; k--)
		{
			if ((int)(hist.at<float>(k, 0)) != 0)
				return k;
		}
	}
	return -1;
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