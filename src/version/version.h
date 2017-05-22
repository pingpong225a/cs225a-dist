#ifndef VERSION_H
#define VERSION_H

//#define USING_KUKA_REDIS_KEYS


#ifdef USING_KUKA_REDIS_KEYS

#define REDIS_SET_EIGEN_MATRIX setEigenMatrixDerived
#define REDIS_GET_EIGEN_MATRIX getEigenMatrixDerived

#else

#define REDIS_SET_EIGEN_MATRIX setEigenMatrixDerivedString
#define REDIS_GET_EIGEN_MATRIX getEigenMatrixDerivedString

#endif // USING_KUKA_REDIS_KEYS

#endif // VERSION_H
