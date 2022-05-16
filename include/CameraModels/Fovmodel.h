#ifndef CAMERAMODELS_FOVMODEL_H
#define CAMERAMODELS_FOVMODEL_H
#include <assert.h>

#include "GeometricCamera.h"

#include "TwoViewReconstruction.h"

namespace ORB_SLAM3 {
    class Fovmodel : public GeometricCamera {
    friend class boost::serialization::access;

        template <class Archieve>
        void serialize(Archieve& ar, const unsigned int version)
        {
            ar & boost::serialization::base_object<GeometricCamera>(*this);
            ar & const_cast<float &>(precision);
        }
        public:
        Fovmodel() : precision(1e-6){
            mvParameters.resize (5);
            mnId = nNextId++;
            mnType = CAM_FOVMODEL;
        }
        Fovmodel(const std::vector<float> _vParameters) : GeometricCamera(_vParameters), precision(1e-6), mvLappingArea(2,0), tvr(nullptr){
            assert(mvParameter.size() == 5);
            mnId = nNextId++;
            mnType = CAM_FOVMODEL;
        } 

        Fovmodel(const std::vector<float> _vParameters, const float _precision) : GeometricCamera(_vParameters), precision(_precision), mvLappingArea(2,0){
            assert(mvParameter.size() == 5);
            mnId = nNextId++;
            mnType = CAM_FOVMODEL;
        }

        Fovmodel(Fovmodel* fov):GeometricCamera(fov->mvParameters),precision(fov->precision), mvLappingArea(2,0),tvr(nullptr){
            assert(mvParameter.size() == 5);
            mnId = nNextId++;
            mnType = CAM_FOVMODEL;
        }

        cv::Point2f project(const cv::Point3f & p3D);
        Eigen::Vector2f project(const Eigen::Vector3f &v3D);
        Eigen::Vector2d project(const Eigen::Vector3d &v3D);
        Eigen::Vector2f projectMat(const cv::Point3f & p3D);

        float uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D);

        Eigen::Vector3f unprojectEig(const cv::Point2f &p2D);
        cv::Point3f unproject(const cv::Point2f &p2D);

        Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d & v3D);

        bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
                                 Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated);
        cv::Mat toK();
        Eigen::Matrix3f toK_();

        bool epipolarConstrain(GeometricCamera* pCamera2, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12, const float sigmaLevel, const float unc);

        float TriangulateMatches(GeometricCamera* pCamera2, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,  const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12, const float sigmaLevel, const float unc, Eigen::Vector3f& p3D);

        std::vector<int> mvLappingArea;

        bool matchAndtriangulate(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, GeometricCamera* pOther,
                                 Sophus::SE3f& Tcw1, Sophus::SE3f& Tcw2,
                                 const float sigmaLevel1, const float sigmaLevel2,
                                 Eigen::Vector3f& x3Dtriangulated);

        friend std::ostream& operator<<(std::ostream& os, const Fovmodel& fov);
        friend std::istream& operator>>(std::istream& is, Fovmodel& fov);

        float GetPrecision(){ return precision;}

        bool IsEqual(GeometricCamera* pCam);
    private:
        const float precision;

        //Parameters vector corresponds to
        //[fx, fy, cx, cy, k0, k1, k2, k3]

        TwoViewReconstruction* tvr;

        void Triangulate(const cv::Point2f &p1, const cv::Point2f &p2, const Eigen::Matrix<float,3,4> &Tcw1,
                         const Eigen::Matrix<float,3,4> &Tcw2, Eigen::Vector3f &x3D);
    };
}


#endif