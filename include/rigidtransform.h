//using Eigen's SVD to fastly compute the rigid transformation between two point clouds.
#ifndef RIGIDTRANSFORM
#define RIGIDTRANSFORM

#include <iostream>
#include <ctime>

#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> TransformType;
typedef std::vector<Eigen::Vector3d> PointsType;

TransformType computeRigidTransform(const PointsType& src, const PointsType& dst);
#endif
