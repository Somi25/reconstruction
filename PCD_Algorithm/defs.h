#pragma once

//BUG ELIMINATION----------
typedef unsigned long long pop_t;
#define BOOST_TYPEOF_EMULATION
#define _CRT_SECURE_NO_WARNINGS
//-------------------------

#include "External.hpp"
#include "Intrinsic.hpp"
#include "Distortion.hpp"

//FUNCTION OPTIONS---------
#define addFilters
#define addAlign
#define removeNAN
//#define smoothingSRC
//#define downsampleSRC
//#define smoothingRES
//#define downsampleRES
//#define resizeSRC
//#define addMeasurement
#define saveInputPCD
#define saveAlignedPCD

//functions followups
#ifdef addFilters
	//#define useHoleFill
	//#define useMedian
	//#define useGaussian	
	//#define useCrop
#endif
#ifdef downsampleSRC //not working
	#define downsampleRate_SRC 2
#endif
#ifdef downsampleRES //not working
	#define downsampleRate_RES 2
#endif
#ifdef smoothingSRC
	#define searchRAD_SRC 0.03 //works with 0.05
#endif
#ifdef smoothingRES
	#define searchRAD_RES 0.04 
#endif
#ifdef useMedian
	#define medianKERNEL 5
#endif
#ifdef useHoleFill
	#define holeFillKERNEL 5
#endif
#ifdef addAlign
	#define ICP_IterNum 10
#endif
#ifdef saveAlignedPCD //SAP
	#define SAP_saveOriginal
	#if (defined smoothingRES || defined downsampleRES)
		#define SAP_saveFiltered
	#endif
#endif
//-------------------------


//DEBUG OPTIONS------------
//#define showStatus
#define debugPCD

//debug followups 
#ifdef debugPCD
#define shownWindowsNum 1	//to change
//--------
#if shownWindowsNum < 1
	#undef debugPCD
#endif
#if shownWindowsNum > 6
#undef shownWindowsNum
#define shownWindowsNum 6
#endif
#endif
//-------------------------


//IMAGE PATHS--------------
//#define Tanszekiervin
//#define Enexport
//#define Ervinexport
//#define Zinemath
#define TanszekMunkaasztal1


#if !(defined(Tanszekiervin) | defined(Ervinexport) | defined(Enexport) | defined(Zinemath) | defined(TanszekMunkaasztal1))
#error You have to define one input path
#endif

#ifdef Tanszekiervin
#define ITERATOR_MAX 492
#define ITERATOR_MIN 466
#endif

#ifdef Zinemath
#define ITERATOR_MAX 1939
#define ITERATOR_MIN 319
#endif

#ifdef TanszekMunkaasztal1
#define ITERATOR_MAX 205
#define ITERATOR_MIN 0
#endif

#ifdef Ervinexport
#define CAM 1
#define ITERATOR_MAX 694
#define ITERATOR_MIN 0


#if CAM 1
#define CAM_NUM 916056915768
#else
#if CAM 2
#define CAM_NUM 916060915777
#else
#define CAM_NUM 916073915802
#endif
#endif
#endif
//-------------------------

//TYPEDEFS-----------------
typedef pcl::PointXYZ PCDPoint;
typedef pcl::PointCloud<PCDPoint> PCD;		//simple
typedef PCD::Ptr PCDPtr;

typedef pcl::PointXYZRGB PCDcPoint;
typedef pcl::PointCloud<PCDcPoint> PCDc;	//colored
typedef PCDc::Ptr PCDcPtr;

typedef pcl::PointNormal PCDnPoint;
typedef pcl::PointCloud<PCDnPoint> PCDn;	//normals
typedef PCDn::Ptr PCDnPtr;
//-------------------------