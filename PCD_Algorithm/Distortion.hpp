#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Distortion
{
public:
	Distortion();

	Distortion(double r1, double r2, double t1, double t2, double r3 = 0);

	explicit Distortion(const Eigen::Affine3f& affine);

	explicit Distortion(const cv::Mat& dist);

	/// Returns CV_64FC1 Mat (Nx1) from the coeffs
	cv::Mat toCvMat() const;

	/// Returns the radial component "i" indexed from 0
	/// Throws exception if invalid index is given
	double radial(unsigned i) const;

	/// Returns the tangential component "i" indexed from 0
	/// Throws exception if invalid index is given
	double tangential(unsigned i) const;

	bool operator==(const Distortion& rhs) const;

	bool operator!=(const Distortion& rhs) const;

private:
	/// Similar to OpenCV distorsion storing (size is always 5)
	/// Radial1, Radial2, Tangential1, Tangential2, Radial3
	std::vector<double> coeffs;
};

class DistortionData
{
public:
	DistortionData(const Distortion& distortion = Distortion());

	DistortionData(const std::string& distortion);

	virtual void set(const std::string& value);

	virtual std::string get() const;

};
typedef boost::shared_ptr<DistortionData> DistortionDataPtr;