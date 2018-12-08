#pragma once

//BUG ELIMINATION----------

typedef unsigned long long pop_t;
#define BOOST_TYPEOF_EMULATION
#define _CRT_SECURE_NO_WARNINGS
//-------------------------

//INCLUDES-----------------

#include "External.hpp"
#include "Intrinsic.hpp"
#include "Distortion.hpp"
//-------------------------

//FUNCTION OPTIONS---------

//#define TESTING_ON
//#define resizeSRC
#define addFilters
//#define SIP_saveOriginal
#define removeNAN
#define outlierRemovalSRC
//#define downsampleSRC
//#define smoothingSRC
//#define SIP_saveFiltered
#define addAlign
#define addMeasurement

//functions followups
#if 1
#ifdef addFilters
	#define useHoleFill
	//#define useMedian
	//#define useGaussian	
	//#define useCrop

#ifdef useMedian
#define medianKERNEL 5
#endif
#ifdef useHoleFill
#define holeFillKERNEL 5
#endif
#endif
#ifdef downsampleSRC
#define downsampleRate_SRC 0.06
#endif
#ifdef smoothingSRC
#define searchRAD_SRC 0.04
#endif
#ifdef addAlign
	#define ICP_IterNum 60
	#define ICP_Precision 1e-8
	//#define SAP_saveResult //result of matching the 2 pcds
	//#define SAP_saveOriginal
	#define downsampleRES
	//#define outlierRemovalRES
	//#define smoothingRES
	//#define SAP_saveFiltered

#if (!(defined smoothingRES) && !(defined downsampleRES))
#undef SAP_saveFiltered
#endif
#ifdef downsampleRES
	#define downsampleRate_RES 0.03
#endif
#ifdef smoothingRES
	#define searchRAD_RES 0.5
#endif
#endif

#ifdef TESTING_ON
#undef resizeSRC
#undef addFilters
#undef SIP_saveOriginal
#undef removeNAN
#undef downsampleSRC
#undef smoothingSRC
#undef SIP_saveFiltered
#undef addAlign
#undef addMeasurement
#endif
#endif
//-------------------------


//DEBUG OPTIONS------------

#define showStatus
#define debugPCD

//debug followups 
#ifdef debugPCD
#define shownWindowsNum 2	//to change
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

//image paths followups
#if 1
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