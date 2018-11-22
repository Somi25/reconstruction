#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Intrinsic
{
public:
	enum FlipMode
	{
		FlipNone = 0x0,
		FlipHorizontal = 0x1,
		FlipVertical = 0x2,
		FlipBoth = 0x3
	};

	Intrinsic(float fx = 1, float fy = 1, float cx = 0, float cy = 0, int width = 0, int height = 0);

	Intrinsic(const Eigen::Affine3f& intrinsic, int w = 0, int h = 0);

	void fromFOV(double fovXDeg, double fovYDeg, int width, int height);

	Eigen::Affine3f toMatrix() const;

	Eigen::Projective3f toGLMatrix(float near_, float far_) const;

	cv::Mat toCvMat() const;

	/// Input x,y is in pixel coordinates (y top-to-bottom, x left-to-right)
	/// Result is 3D point.
	Eigen::Vector3f backProject(float x, float y, float z) const;

	/// Input is 3D point.
	/// Result is in pixel coordinates (y top-to-bottom, x left-to-right)
	Eigen::Vector3f project(Eigen::Vector3f p) const;

	void scale(float scale);

	/// \return (1 - alpha) * this + alpha * right
	Intrinsic interpolate(Intrinsic right, double alpha) const;

	void flip(FlipMode mode);

	void calibrationMatrixValues(double apertureWidth, double apertureHeight, double &fovXDeg, double &fovYDeg, double &focalLength);

	bool operator==(const Intrinsic& rhs);

	bool operator!=(const Intrinsic& rhs);

	float fx, fy, cx, cy;
	int width, height;

	
};

class  IntrinsicData
{
public:
	IntrinsicData(const Intrinsic& intrinsic = Intrinsic());

	IntrinsicData(const Eigen::Affine3f& intrinsic, int width = 0, int height = 0);

	IntrinsicData(const std::string& intrinsic);

	virtual void set(const std::string& value);

	virtual std::string get() const;
};
typedef boost::shared_ptr<IntrinsicData> IntrinsicDataPtr;