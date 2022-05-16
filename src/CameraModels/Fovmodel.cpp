#include<Fovmodel.h>
#include <boost/serialization/export.hpp>
namespace ORB_SLAM3
{
/*********************************************************************************************** 
 * @brief 投影
 * xc​ = Xc/Zc, yc = Yc/Zc
 * r^2 = x*x + y*y
 * rd = (1/w) * atan(2 * r * tanhalfw)
 * xd = (rd/r)*xc, yd = (rd/r)*yc
 * u = fx*xd + cx  v = fy*yd + cy
 * @param p3D 三维点
 * @return 像素坐标
 **********************************************************************************************/
    cv::Point2f Fovmodel::project(const cv::Point3f &p3D)
    {
        float x = p3D.x;
        float y = p3D.y;
        float z = p3D.z;
        float w = mvParameters[4];

        float xc = x / z;
        float yc = y / z;

        float r = sqrt(x*x + y*y);
        float rd = (1 / w) * atan(2 * r * tan(w / 2));

        float xd = (rd / r) * xc;
        float yd = (rd / r) * yc;

        return cv::Point2f(mvParameters[0] * xd + mvParameters[2],
                           mvParameters[1] * yd + mvParameters[3]);
    }
/***********************************************************************************************
 * @brief 投影
 * @param v3D 三维点
 * @return 像素坐标 输入采用 Eigen::Vector3d,输出采用 Eigen::Vector2d
 **********************************************************************************************/
   
    Eigen::Vector2f Fovmodel::project(const Eigen::Vector3f &v3D)
    {
        float x = v3D[0];
        float y = v3D[1];
        float z = v3D[2];
        float w = mvParameters[4];

        float xc = x / z;
        float yc = y / z;

        float r = sqrt(x*x + y*y);
        float rd = (1 / w) * atan(2 * r * tan(w / 2));

        float xd = (rd / r) * xc;
        float yd = (rd / r) * yc;

        Eigen::Vector2f res;

        res[0] = mvParameters[0] * xd + mvParameters[2];
        res[1] = mvParameters[1] * yd + mvParameters[3];
        return Eigen::Vector2f(res[0],res[1]);
    }
/***********************************************************************************************
 * @brief 投影
 * @param v3D 三维点
 * @return 像素坐标,重构，输入采用Eigen::Vector3f,输出Eigen::Vector2f
 **********************************************************************************************/
    Eigen::Vector2d Fovmodel::project(const Eigen::Vector3d &v3D)
    {
        double x = v3D[0];
        double y = v3D[1];
        double z = v3D[2];
        double w = mvParameters[4];

        double xc = x / z;
        double yc = y / z;

        double r = sqrt(x*x + y*y);
        double rd = (1 / w) * atan(2 * r * tan(w / 2));

        double xd = (rd / r) *  xc;
        double yd = (rd / r) * yc;
        Eigen::Vector2d res;

        res[0] = mvParameters[0] * xd + mvParameters[2];
        res[1] = mvParameters[1] * yd + mvParameters[3];
        return Eigen::Vector2d(res[0],res[1]);
    }
/***********************************************************************************************
 * @brief 投影
 * @param p3D 三维点
 * @return 像素坐标，重构，输入cv::point3f,输出采用Eigen::Vector2f
 **********************************************************************************************/
    Eigen::Vector2f Fovmodel::projectMat(const cv::Point3f &p3D)
    {
        cv::Point2f point = this->project(p3D);
        return Eigen::Vector2f(point.x, point.y);
    }


    float Fovmodel::uncertainty2(const Eigen::Matrix<double,2,1> &p2D)
    {
        return 1.f;
    }
/***********************************************************************************************
 * @brief 反投影,以Eigen::Vector3f形式返回，Fovmodel::TriangulateMatches中调用
 * @param p2D 特征点
 * @return 
 **********************************************************************************************/
    Eigen::Vector3f Fovmodel::unprojectEig(const cv::Point2f &p2D)
    {
        cv::Point3f ray = this->unproject(p2D);
        return Eigen::Vector3f(ray.x, ray.y, ray.z);
    }

/*********************************************************************************************** 
 * @brief 反投影
 * 投影过程
 * xc​ = Xc/Zc, yc = Yc/Zc
 * r^2 = x*x + y*y
 * rd = (1/w) * atan(2 * r * tanhalfw)
 * xd = (rd/r)*xc, yd = (rd/r)*yc
 * u = fx*xd + cx  v = fy*yd + cy
 * 
 * 
 * 已知u与v 未矫正的特征点像素坐标
 * xd = (u - cx) / fx;
 * yd = (v - cy) / fy;
 * rd = sqrt(xd * xd + yd * yd);
 * 待求的 r = tan(rd * w) / (2 * tanhalfw);
 *  
 * 待求的 x = xd * (r/rd) * fx + cx;
 * 待求的 y = yd * (r/rd) * fy + cy;
 *
 * 其中 r的算法如下：
 *    r = tan(rd * w) / (2 * tanhalfw);
 * 最后x = xd * (r/rd) * fx + cx;
 *    y = yd * (r/rd) * fy + cy;
 * @return 
 **********************************************************************************************/
cv::Point3f Fovmodel::unproject(const cv::Point2f &p2D)
{
   
    float u = p2D.x, v = p2D.y;
    float fx = mvParameters[0];
    float fy = mvParameters[1];
    float cx = mvParameters[2];
    float cy = mvParameters[3];
    float w  = mvParameters[4];
    float tanhalfw = tan(w / 2);

    float x_distort_fov = (u - cx) / fx;
    float y_distort_fov = (v - cy) / fy;

    float rd = sqrt(x_distort_fov * x_distort_fov + y_distort_fov * y_distort_fov);
    float r = tan(rd * w) / (2 * tanhalfw);
    
    float x = x_distort_fov * (r/rd) * fx + cx;
    float y = y_distort_fov * (r/rd) * fy + cy;

    return cv::Point3f(x, y, 1.f);
}
/***********************************************************************************************
 * @brief 求解二维像素坐标关于三维点的雅克比矩阵
 * u =fx*((1/w)*atan(2*sqrt((x/z)^2 + (y/z)^2)*tanhalfw)*x/(sqrt((x/z)^2+(y/z)^2)))+cx
 * v =fy*((1/w)*atan(2*sqrt((x/z)^2 + (y/z)^2)*tanhalfw)*y/(sqrt((x/z)^2+(y/z)^2)))+cy
 * 
 * r^2 = x*x + y*y
 * rd = (1/w) * atan(2 * r * tanhalfw)
 * 
 * xc = rd/r * (x/z)
 * yc = rd/r * (y/z)
 * 
 * 这两个式子分别对 xyz 求导

 * ∂u/∂x =

 * ∂u/∂y =(2*fx*x*y*tanhalfw)/(w*zz*rr*(4*tan2*rr + 1))
          - (fx*x*y*atan(2*tanhalfw*r))/(w*zz*sqrt(rrr))


 * ∂u/∂z = (fx*x*atan(2*tanhalfw*r)*((2*xx)/zzz + (2*yy)/zzz))/(2*w*sqrt(rrr))
          - (fx*x*tanhalfw*((2*xx)/zzz + (2*yy)/zzz))/(w*rr*(4*tan2*rr
          + 1))

 * ∂v/∂x = (2*fy*x*y*tanhalfw)/(w*zz*rr*(4*tan2*rr + 1))
          - (x*fy*y*atan(2*tanhalfw*r))/(w*zz*sqrt(rrr))

 * ∂v/∂y = fy*atan(2*tanhalfw*r)/(w*r)
          - (fy*yy*atan(2*tanhalfw*r))/(w*zz*sqrt(rrr))
          + (2*fy*yy*tanhalfw)/(w*zz*rr*(4*tan2*rr
           + 1))

 * ∂v/∂z = (fy*y*atan(2*tanhalfw*r)*((2*xx)/zzz + (2*yy)/zzz))/(2*w*sqrt(rrr))
          - (fy*y*tanhalfw*((2*xx)/zzz + (2*yy)/zzz))/(w*rr*(4*tan2*rr
          + 1))

 * @param p3D 三维点
 * @return
 **********************************************************************************************/
Eigen::Matrix<double, 2, 3>  Fovmodel::projectJac(const Eigen::Vector3d &p3D)
{
    double x = p3D[0], y = p3D[1], z = p3D[2];
    double xx = x * x, yy = y * y, zz = z * z;
    double zzz = z*z*z, xxx = x*x*x, yyy = y*y*y;
    double rr = xx / zz + yy / zz;

    double fx = mvParameters[0];
    double fy = mvParameters[1];
    double cx = mvParameters[2];
    double cy = mvParameters[3];
    double w  = mvParameters[4];
    double r = sqrt(rr);

    double rrr = r * r * r;

    double tanhalfw = tan(w / 2);
    double tan2 = tanhalfw * tanhalfw;

    Eigen::Matrix<double, 2, 3> JacGood;

    JacGood(0, 0) = fx * atan(2 * tanhalfw * r) / (w * r) 
                     - (fx * xx * atan(2 * tanhalfw * r)) / (w * zz * sqrt(rrr)) 
                     + (2 * fx * xx * tanhalfw) / (w * zz * rr * (4 * tan2 * rr + 1));//∂u/∂x

    JacGood(1, 0) = (2 * fy * x * y * tanhalfw) / (w * zz * rr * (4 * tan2 * rr + 1)) 
                     - (x * fy * y * atan(2 * tanhalfw * r)) / (w * zz * sqrt(rrr)); // ∂v/∂x

    JacGood(0, 1) = (2 * fx * x * y * tanhalfw) / (w * zz * rr * (4 * tan2 * rr + 1)) 
                     - (x * fx * y * atan(2 * tanhalfw * r)) / (w * zz * sqrt(rrr)); // ∂u/∂y

    JacGood(1, 1) =  fy * atan(2 * tanhalfw * r) / (w * r)
                     - (fy * yy * atan(2 * tanhalfw * r)) / (w * zz * sqrt(rrr))
                     + (2 * fy * yy * tanhalfw) / (w * zz * rr * (4 * tan2 * rr + 1));//  ∂v/∂y
 
    JacGood(0, 2) = (fx * x * atan(2 * tanhalfw * r) * ((2 * xx) / zzz + (2 * yy) / zzz)) / (2 * w * sqrt(rrr))
                     - (fx * x * tanhalfw * ((2 * xx) / zzz + (2 * yy) / zzz)) / (w * rr * (4 * tan2 * rr + 1)); //∂u/∂z


    JacGood(1, 2) = (fy * y * atan(2 * tanhalfw * r) * ((2 * xx) / zzz + (2 * yy) / zzz)) / (2 * w * sqrt(rrr))
                     -(fy * y * tanhalfw * ((2 * xx) / zzz + (2 * yy) / zzz)) / (w * rr * (4 * tan2 * rr + 1)); // ∂v/∂z

    return JacGood;
}
/*********************************************************************************************** 
 * 三角化恢复三维点，但要提前做去畸变 单目初始化中使用
 * @param vKeys1 第一帧的关键点
 * @param vKeys2 第二帧的关键点
 * @param vMatches12 匹配关系，长度与vKeys1一样，对应位置存放vKeys2中关键点的下标
 * @param T21 顾名思义
 * @param vP3D 恢复出的三维点
 * @param vbTriangulated 是否三角化成功，用于统计匹配点数量
 **********************************************************************************************/
    bool Fovmodel::ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
                                 Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated){
        if(!tvr){
            Eigen::Matrix3f K = Fovmodel::toK_();
            tvr = new TwoViewReconstruction(K);
        }

        return tvr->Reconstruct(vKeys1,vKeys2,vMatches12,T21,vP3D,vbTriangulated);
    }

/***********************************************************************************************
 * @brief 返回内参矩阵
 * @return K
 ***********************************************************************************************/
cv::Mat Fovmodel::toK()
{ //cv::Mat_才能包含模板参数
    cv::Mat K = (cv::Mat_<float> (3, 3) << 
        mvParameters[0], 0.f,             mvParameters[2],
        0.f,             mvParameters[1], mvParameters[3],
        0.f,             0.f,             1.f);
    return K;
}

Eigen::Matrix3f Fovmodel::toK_()
{
    Eigen::Matrix3f K;
    K << mvParameters[0], 0.f,             mvParameters[2],
         0.f,             mvParameters[1], mvParameters[3],
         0.f,             0.f,             1.f;
    return K;
}


bool Fovmodel::epipolarConstrain(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                                        const Eigen::Matrix3f &R12, const Eigen::Vector3f &t12, const float sigmaLevel, const float unc)
{
    Eigen::Vector3f p3D;
    return this->TriangulateMatches(pCamera2, kp1, kp2, R12, t12, sigmaLevel, unc, p3D) > 0.0001f;
}

/*********************************************************************************************** 
 * @brief 通过三角化后的重投影误差判断点匹配的情况，如果比较好则返回深度值
 * @param pCamera2 右相机
 * @param kp1 左相机特征点
 * @param kp2 右相机特征点
 * @param R12 2->1的旋转
 * @param t12 2->1的平移
 * @param sigmaLevel 特征点1的尺度的平方
 * @param unc 特征点2的尺度的平方
 * @param p3D 恢复的三维点
 * @return 点的深度值
 ***********************************************************************************************/
bool Fovmodel::matchAndtriangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, GeometricCamera *pOther,
                                            Sophus::SE3f &Tcw1, Sophus::SE3f &Tcw2,
                                            const float sigmaLevel1, const float sigmaLevel2,
                                            Eigen::Vector3f &x3Dtriangulated)
{
    Eigen::Matrix<float, 3, 4> eigTcw1 = Tcw1.matrix3x4();
    Eigen::Matrix3f Rcw1 = eigTcw1.block<3, 3>(0, 0);
    Eigen::Matrix3f Rwc1 = Rcw1.transpose();
    Eigen::Matrix<float, 3, 4> eigTcw2 = Tcw2.matrix3x4();
    Eigen::Matrix3f Rcw2 = eigTcw2.block<3, 3>(0, 0);
    Eigen::Matrix3f Rwc2 = Rcw2.transpose();

    cv::Point3f ray1c = this->unproject(kp1.pt);
    cv::Point3f ray2c = pOther->unproject(kp2.pt);

    // 获得点1在帧1的归一化坐标
    Eigen::Vector3f r1(ray1c.x, ray1c.y, ray1c.z);
    // 获得点2在帧2的归一化坐标
    Eigen::Vector3f r2(ray2c.x, ray2c.y, ray2c.z);

    // Check parallax between rays
    // 射线在世界坐标系下
    Eigen::Vector3f ray1 = Rwc1 * r1;
    Eigen::Vector3f ray2 = Rwc2 * r2;

    const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());

    // If parallax is lower than 0.9998, reject this match
    // 夹角几乎为0时返回，因为表示这个点过远，三角化会带来大量误差
    if (cosParallaxRays > 0.9998)
    {
        return false;
    }

    // Parallax is good, so we try to triangulate
    cv::Point2f p11, p22;

    p11.x = ray1c.x;
    p11.y = ray1c.y;

    p22.x = ray2c.x;
    p22.y = ray2c.y;

    Eigen::Vector3f x3D;

    // 三角化
    Triangulate(p11, p22, eigTcw1, eigTcw2, x3D);

    // Check triangulation in front of cameras
    // 查看点是否位于相机前面
    float z1 = Rcw1.row(2).dot(x3D) + Tcw1.translation()(2);
    if (z1 <= 0)
    { // Point is not in front of the first camera
        return false;
    }

    float z2 = Rcw2.row(2).dot(x3D) + Tcw2.translation()(2);
    if (z2 <= 0)
    { // Point is not in front of the first camera
        return false;
    }

    // Check reprojection error in first keyframe
    //   -Transform point into camera reference system
    // 查看投影误差
    Eigen::Vector3f x3D1 = Rcw1 * x3D + Tcw1.translation();
    Eigen::Vector2f uv1 = this->project(x3D1);

    float errX1 = uv1(0) - kp1.pt.x;
    float errY1 = uv1(1) - kp1.pt.y;

    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaLevel1)
    { // Reprojection error is high
        return false;
    }

    // Check reprojection error in second keyframe;
    //   -Transform point into camera reference system
    Eigen::Vector3f x3D2 = Rcw2 * x3D + Tcw2.translation(); // avoid using q
    Eigen::Vector2f uv2 = pOther->project(x3D2);

    float errX2 = uv2(0) - kp2.pt.x;
    float errY2 = uv2(1) - kp2.pt.y;

    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaLevel2)
    { // Reprojection error is high
        return false;
    }

    // Since parallax is big enough and reprojection errors are low, this pair of points
    // can be considered as a match
    x3Dtriangulated = x3D;

    return true;
}


/*********************************************************************************************** 
 * @brief 通过三角化后的重投影误差判断点匹配的情况，如果比较好则返回深度值
 * @param pCamera2 右相机
 * @param kp1 左相机特征点
 * @param kp2 右相机特征点
 * @param R12 2->1的旋转
 * @param t12 2->1的平移
 * @param sigmaLevel 特征点1的尺度的平方
 * @param unc 特征点2的尺度的平方
 * @param p3D 恢复的三维点
 * @return 点的深度值
 ***********************************************************************************************/
float Fovmodel::TriangulateMatches(
    GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
    const Eigen::Matrix3f &R12, const Eigen::Vector3f &t12, const float sigmaLevel,
    const float unc, Eigen::Vector3f &p3D)
{
    // 1. 得到对应特征点的相平面坐标
    Eigen::Vector3f r1 = this->unprojectEig(kp1.pt);
    Eigen::Vector3f r2 = pCamera2->unprojectEig(kp2.pt);

    // Check parallax
    // 2. 查看射线夹角
    // 这里有点像极线约束，但并不是，将r2通过R12旋转到与r1同方向的坐标系
    // 然后计算他们的夹角，看其是否超过1.14° 
    Eigen::Vector3f r21 = R12 * r2;

    const float cosParallaxRays = r1.dot(r21) / (r1.norm() * r21.norm());

    if (cosParallaxRays > 0.9998)
    {
        return -1;
    }

    // Parallax is good, so we try to triangulate
    cv::Point2f p11, p22;

    p11.x = r1[0];
    p11.y = r1[1];

    p22.x = r2[0];
    p22.y = r2[1];

    Eigen::Vector3f x3D;
    Eigen::Matrix<float, 3, 4> Tcw1;

    // 3. 设定变换矩阵用于三角化
    Tcw1 << Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero();

    Eigen::Matrix<float, 3, 4> Tcw2;

    Eigen::Matrix3f R21 = R12.transpose();
    Tcw2 << R21, -R21 * t12;

    // 4. 三角化
    Triangulate(p11, p22, Tcw1, Tcw2, x3D);
    // cv::Mat x3Dt = x3D.t();

    // 深度值是否正常
    float z1 = x3D(2);
    if (z1 <= 0)
    {
        return -2;
    }

    float z2 = R21.row(2).dot(x3D) + Tcw2(2, 3);
    if (z2 <= 0)
    {
        return -3;
    }

    // Check reprojection error
    // 5. 做下投影计算重投影误差
    Eigen::Vector2f uv1 = this->project(x3D);

    float errX1 = uv1(0) - kp1.pt.x;
    float errY1 = uv1(1) - kp1.pt.y;

    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaLevel)
    { // Reprojection error is high
        return -4;
    }

    Eigen::Vector3f x3D2 = R21 * x3D + Tcw2.col(3);
    Eigen::Vector2f uv2 = pCamera2->project(x3D2);

    float errX2 = uv2(0) - kp2.pt.x;
    float errY2 = uv2(1) - kp2.pt.y;

    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * unc)
    { // Reprojection error is high
        return -5;
    }

    p3D = x3D;

    return z1;
}

std::ostream &operator<<(std::ostream &os, const Fovmodel &fov)
{
    os << fov.mvParameters[0] << " " << fov.mvParameters[1] << " " << fov.mvParameters[2] << " " << fov.mvParameters[3] << " "
        << fov.mvParameters[4];
    return os;
}

std::istream &operator>>(std::istream &is, Fovmodel &fov)
{
    float nextParam;
    for (size_t i = 0; i < 8; i++)
    {
        assert(is.good()); // Make sure the input stream is good
        is >> nextParam;
        fov.mvParameters[i] = nextParam;
    }
    return is;
}

/**************************************************************************************
 * @brief 通过三角化恢复归一化坐标的三维点坐标，该三维点为在左相机坐标系下的点
 * @param p1 右相机
 * @param p2 左相机特征点
 * @param Tcw1 3×4 单位矩阵
 * @param Tcw2 3×4 T21
 * @param x3D 恢复的三维点
 **************************************************************************************/


void Fovmodel::Triangulate(
    const cv::Point2f &p1, const cv::Point2f &p2, const Eigen::Matrix<float, 3, 4> &Tcw1,
    const Eigen::Matrix<float, 3, 4> &Tcw2, Eigen::Vector3f &x3D)
{
    Eigen::Matrix<float, 4, 4> A;
    // 代码中有用到三角化的地方有：TwoViewReconstruction::Triangulate LocalMapping::CreateNewMapPoints KannalaBrandt8::Triangulate Initializer::Triangulate
    // 见Initializer.cpp的Triangulate函数,A矩阵构建的方式类似，不同的是乘的反对称矩阵那个是像素坐标构成的，而这个是归一化坐标构成的
    // Pc = Tcw*Pw， 左右两面乘Pc的反对称矩阵 [Pc]x * Tcw *Pw = 0 构成了A矩阵，中间涉及一个尺度a，因为都是归一化平面，但右面是0所以直接可以约掉不影响最后的尺度
    //  0 -1 y    Tcw.row(0)     -Tcw.row(1) + y*Tcw.row(2)
    //  1 0 -x *  Tcw.row(1)  =   Tcw.row(0) - x*Tcw.row(2) 
    // -y x  0    Tcw.row(2)    x*Tcw.row(1) - y*Tcw.row(0)
    // 发现上述矩阵线性相关，所以取前两维，两个点构成了4行的矩阵，就是如下的操作，求出的是4维的结果[X,Y,Z,A]，所以需要除以最后一维使之为1，就成了[X,Y,Z,1]这种齐次形式
    A.row(0) = p1.x * Tcw1.row(2) - Tcw1.row(0);
    A.row(1) = p1.y * Tcw1.row(2) - Tcw1.row(1);
    A.row(2) = p2.x * Tcw2.row(2) - Tcw2.row(0);
    A.row(3) = p2.y * Tcw2.row(2) - Tcw2.row(1);

    Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4f x3Dh = svd.matrixV().col(3);
    x3D = x3Dh.head(3) / x3Dh(3);
}

bool Fovmodel::IsEqual(GeometricCamera *pCam)
{
    if (pCam->GetType() != GeometricCamera::CAM_FISHEYE)
        return false;

    Fovmodel *pFovCam = (Fovmodel *)pCam;

    if (abs(precision - pFovCam->GetPrecision()) > 1e-6)
        return false;

    if (size() != pFovCam->size())
        return false;

    bool is_same_camera = true;
    for (size_t i = 0; i < size(); ++i)
    {
        if (abs(mvParameters[i] - pFovCam->getParameter(i)) > 1e-6)
        {
            is_same_camera = false;
            break;
        }
    }
    return is_same_camera;
}

}

