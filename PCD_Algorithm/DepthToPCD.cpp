#include "DepthToPCD.hpp"



PCDPtr DepthToPCD(cv::Mat in)
{
#ifdef Zinemath
	Intrinsic intr(418.920523, 418.285805, 137.401996, 122.136574, 320, 240);// <item key="intrinsic" type="Intrinsic" encoding="text">418.920523 418.285805 137.401996 122.136574 320 240</item>
	Distortion dist(-0.40322, 0.706050, 0.001742, -0.002878, -1.370257);// <item key = "distorsion" type = "Distortion" encoding = "text">-0.40322 0.706050 0.001742 - 0.002878 - 1.370257< / item>
#endif
#ifdef TanszekMunkaasztal1
	//<item key = "intrinsic" type = "Intrinsic">164.343 163.483 83.0361 59.1976 160 120 < / item >
	Intrinsic intr(164.343, 163.483, 83.0361, 59.1976, 160, 120);
	//<item key = "distortion" type = "Distortion">-0.331462551773313 -0.055347852432151 -0.000475008698636328 -0.00144602003297141 0.473202830799858< / item>
	Distortion dist(-0.331462551773313, -0.055347852432151, -0.000475008698636328, -0.00144602003297141, 0.473202830799858);
#endif
	std::string imageType = "DImage";
	std::string coordinateSystem = "OpenGL";

	if (in.type() != CV_32FC1)
	{
		return (PCDPtr)NULL;
	}// ("Only CV_32FC1 image is supported", "");

	double k1 = dist.radial(0);
	double k2 = dist.radial(1);
	double p1 = dist.tangential(0);
	double p2 = dist.tangential(1);
	double k3 = dist.radial(2);

	cv::Mat outZ;
	outZ = cv::Mat(in.rows, in.cols, CV_32FC1, cv::Scalar(0));

	PCDPtr pcd(new PCD(in.cols, in.rows));
	PCD::PointType p;

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

	return pcd;
}