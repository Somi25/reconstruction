#include "Intrinsic.hpp"
#include "defs.h"
#include <opencv2/calib3d/calib3d.hpp>

	Intrinsic::Intrinsic(float fx, float fy, float cx, float cy, int width, int height) : fx(fx)
		, fy(fy)
		, cx(cx)
		, cy(cy)
		, width(width)
		, height(height)
	{}

	Intrinsic::Intrinsic(const Eigen::Affine3f& intrinsic, int w, int h)
	{
		fx = intrinsic(0, 0);
		fy = intrinsic(1, 1);
		cx = intrinsic(0, 2);
		cy = intrinsic(1, 2);
		width = w;
		height = h;
	}

	void Intrinsic::fromFOV(double fovXDeg, double fovYDeg, int w, int h)
	{
		width = w;
		height = h;
		fx = width / (2 * tan(fovXDeg / 180 * M_PI / 2));
		fy = height / (2 * tan(fovYDeg / 180 * M_PI / 2));
		cx = width / 2 - 0.5;
		cy = height / 2 - 0.5;
	}

	Eigen::Affine3f Intrinsic::toMatrix() const
	{
		Eigen::Affine3f tr(Eigen::Affine3f::Identity());
		tr(0, 0) = fx;
		tr(1, 1) = fy;
		tr(0, 2) = cx;
		tr(1, 2) = cy;
		return tr;
	}

	Eigen::Projective3f Intrinsic::toGLMatrix(float near_, float far_) const
	{
		Eigen::Projective3f tr(Eigen::Projective3f::Identity());
		tr(0, 0) = 2 * fx / width;
		tr(0, 2) = 1 - 2 * cx / width;
		tr(1, 1) = 2 * fy / height;
		tr(1, 2) = 2 * cy / height - 1;
		tr(2, 2) = -(far_ + near_) / (far_ - near_);
		tr(2, 3) = -2 * far_ * near_ / (far_ - near_);
		tr(3, 2) = -1;
		tr(3, 3) = 0;
		return tr;
	}

	cv::Mat Intrinsic::toCvMat() const
	{
		cv::Mat ret(3, 3, CV_32FC1, cv::Scalar(0));
		ret.at<float>(0, 0) = fx;
		ret.at<float>(1, 1) = fy;
		ret.at<float>(0, 2) = cx;
		ret.at<float>(1, 2) = cy;
		ret.at<float>(2, 2) = 1;
		return ret;
	}

	Eigen::Vector3f Intrinsic::backProject(float x, float y, float z) const
	{
		Eigen::Vector3f v;
		v.x() = (x - cx) * z / fx;
		v.y() = (y - cy) * z / fy;
		v.z() = z;
		return v;
	}

	Eigen::Vector3f Intrinsic::project(Eigen::Vector3f p) const
	{
		p.x() = p.x() * fx / p.z() + cx;
		p.y() = p.y() * fy / p.z() + cy;
		return p;
	}

	void Intrinsic::scale(float scale)
	{
		fx *= scale;
		fy *= scale;
		cx *= (scale * width - 1) / (width - 1);
		cy *= (scale * height - 1) / (height - 1);
		width = cv::saturate_cast<int>(width * scale);
		height = cv::saturate_cast<int>(height * scale);
	}

	Intrinsic Intrinsic::interpolate(Intrinsic right, double alpha) const
	{
		right.fx = fx + (right.fx - fx) * alpha;
		right.fy = fy + (right.fy - fy) * alpha;
		right.cx = cx + (right.cx - cx) * alpha;
		right.cy = cy + (right.cy - cy) * alpha;
		return right;
	}

	void Intrinsic::flip(FlipMode mode)
	{
		if (width <= 0 || height <= 0)
		{
			//throw std::runtime_error("Intrinsic::flip: The intrinsic width/height is invalid");
			//TODO
		}

		if (mode == FlipVertical || mode == FlipBoth)
		{
			cy = height - cy;
		}
		if (mode == FlipHorizontal || mode == FlipBoth)
		{
			cx = width - cx;
		}
	}

	void Intrinsic::calibrationMatrixValues(double apertureWidth, double apertureHeight, double &fovXDeg, double &fovYDeg, double &focalLength)
	{
		if (width == 0 || height == 0)
			return; //invalid intrinsic image size

		if (fx == 0 || fy == 0)
			return;  //invalid intrinsic value

		cv::Mat intrinsicMatrix = toCvMat();

		double aspectRatio;
		cv::Point2d principalPoint;
		cv::calibrationMatrixValues(intrinsicMatrix, cv::Size(width, height), apertureWidth, apertureHeight, fovXDeg, fovYDeg, focalLength, principalPoint, aspectRatio);
	}

	bool Intrinsic::operator==(const Intrinsic& rhs)
	{
		return fx == rhs.fx
			&& fy == rhs.fy
			&& cx == rhs.cx
			&& cy == rhs.cy
			&& width == rhs.width
			&& height == rhs.height;
	}

	bool Intrinsic::operator!=(const Intrinsic& rhs)
	{
		return !(*this == rhs);
	}

	////////////////////////
	//// IntrinsicData
	////////////////////////

	IntrinsicData::IntrinsicData(const Intrinsic& intrinsic)
	{
	}

	IntrinsicData::IntrinsicData(const Eigen::Affine3f& intrinsic, int width, int height)
	{
	}

	IntrinsicData::IntrinsicData(const std::string& intrinsic)
	{
		IntrinsicData::set(intrinsic);
	}

	void IntrinsicData::set(const std::string& value)
	{
		Intrinsic intr;
		std::istringstream ss(value);
		ss >> intr.fx;
		ss >> intr.fy;
		ss >> intr.cx;
		ss >> intr.cy;
		ss >> intr.width;
		ss >> intr.height;

		if (!ss)
		{
			throw std::runtime_error("IntrinsicData::set(): Error setting intrinsic value from string: '" + value + "'. Expected format is 6 numbers (fx, fy, cx, cy, width, height)");
		}
		//TODO
		//data_ = intr;
	}

	std::string IntrinsicData::get() const
	{
		std::ostringstream ss;
		//TODO
		/*
		ss << data_.fx << " "
			<< data_.fy << " "
			<< data_.cx << " "
			<< data_.cy << " "
			<< data_.width << " "
			<< data_.height;
		*/
		return ss.str();
	}