#pragma once

//#define debug

//#define Tanszekiervin
//#define Enexport
//#define Ervinexport
#define Zinemath


#if !(defined(Tanszekiervin) | defined(Ervinexport) | defined(Enexport) | defined(Zinemath))
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