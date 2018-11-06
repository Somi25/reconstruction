#include "Distortion.hpp"

Distortion::Distortion() : coeffs(5, 0)
{
}

Distortion::Distortion(double r1, double r2, double t1, double t2, double r3)
{
	coeffs.push_back(r1);
	coeffs.push_back(r2);
	coeffs.push_back(t1);
	coeffs.push_back(t2);
	coeffs.push_back(r3);
}

Distortion::Distortion(const Eigen::Affine3f& affine)
{
	for (int i = 0; i < 4; ++i)
		coeffs.push_back(affine(0, i));

	coeffs.push_back(0);
}

Distortion::Distortion(const cv::Mat& dist)
{
	int type = dist.type();

	if (type != CV_32FC1 && type != CV_64FC1)
		//throw std::runtime_error("Distortion from cv::Mat: Invalid cv::Mat type: " + getOpenCVTypeName(dist));
		return;
	// OpenCV distortion mat sizes are 1xN or Nx1
	if (!((dist.rows == 1 && dist.cols >= 4 && dist.cols <= 5) || (dist.cols == 1 && dist.rows >= 4 && dist.rows <= 5)))
		//throw std::runtime_error(QString("Unable to set Distortion from cv::Mat(%1 x %2)").arg(dist.cols).arg(dist.rows).toStdString());
		return;
	int numParams = std::max(dist.rows, dist.cols);

	int rowInc = dist.rows > 1;
	int colInc = dist.cols > 1;

	for (int i = 0; i < numParams; ++i)
	{
		double value = 0;

		if (type == CV_32FC1)
			value = dist.at<float>(i * rowInc, i * colInc);
		else
			value = dist.at<double>(i * rowInc, i * colInc);

		coeffs.push_back(value);
	}

	if (coeffs.size() < 5)
		coeffs.push_back(0);
}


cv::Mat Distortion::toCvMat() const
{
	cv::Mat distortionCoeffs;
	distortionCoeffs.create(1, coeffs.size(), CV_64FC1);

	for (unsigned i = 0; i < coeffs.size(); ++i)
		distortionCoeffs.at<double>(0, i) = coeffs[i];

	return distortionCoeffs;
}

double Distortion::radial(unsigned i) const
{
	if (i > 2)
		//throw std::runtime_error("Distortion::radial(): Invalid index is given: " + toString(i));
		return -1;
	i = (i > 1) ? i + 2 : i;

	return coeffs.at(i);
}

double Distortion::tangential(unsigned i) const
{
	if (i >= 2)
		//throw std::runtime_error("Distortion::tangential(): Invalid index is given: " + toString(i));
		return -1;
	return coeffs.at(i + 2);
}

bool Distortion::operator==(const Distortion& rhs) const
{
	for (unsigned i = 0; i < coeffs.size(); ++i)
		if (coeffs[i] != rhs.coeffs[i])
			return false;

	return true;
}

bool Distortion::operator!=(const Distortion& rhs) const
{
	return !(*this == rhs);
}

////////////////////////
//// DistortionData
////////////////////////

DistortionData::DistortionData(const Distortion& distortion)
{
}

DistortionData::DistortionData(const std::string& data)
{
	DistortionData::set(data);
}

void DistortionData::set(const std::string& value)
{
	std::istringstream ss(value);

	std::vector<double> d;
	double val;
	while (ss >> val)
		d.push_back(val);

	if (d.size() < 4 || d.size() > 5)
		//throw std::runtime_error("DistortionData::set(): Error setting value from string: Expected 4 or 5 number separated by spaces");
		return;
	if (d.size() < 5)
		d.push_back(0);
	//TODO
	//data_ = Distortion(d[0], d[1], d[2], d[3], d[4]);
}

std::string DistortionData::get() const
{
	//TODO
	
	std::ostringstream ostr;
	/*
	ostr << std::setprecision(std::numeric_limits<double>::digits10);

	ostr << data_.radial(0) << " ";
	ostr << data_.radial(1) << " ";
	ostr << data_.tangential(0) << " ";
	ostr << data_.tangential(1) << " ";
	ostr << data_.radial(2);
	*/
	return ostr.str();
}