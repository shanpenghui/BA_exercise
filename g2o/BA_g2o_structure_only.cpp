/****************************
 * 题目：给定一组世界坐标系下的3D点(p3d.txt)以及它在相机中对应的坐标(p2d.txt)，以及相机的内参矩阵。
 * 使用bundle adjustment 方法（g2o库实现）来估计相机的位姿T。初始位姿T为单位矩阵。
 *
 * 本程序学习目标：
 * 熟悉g2o库编写流程，熟悉顶点定义方法。
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2019.02
****************************/

/****************************
 * 头文件
****************************/
#include <vector>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <glog/logging.h>
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#if defined G2O_HAVE_CHOLMOD
G2O_USE_OPTIMIZATION_LIBRARY(cholmod);
#else
G2O_USE_OPTIMIZATION_LIBRARY(eigen);
#endif

G2O_USE_OPTIMIZATION_LIBRARY(dense);

using namespace Eigen;
using namespace cv;
using namespace std;

string p3d_file = "../../data/p3d.txt";
string p2d_file = "../../data/p2d.txt";

/****************************
 * 自定义模型的顶点，需要自己定义的内容是：
 * 1.优化变量维度，这里是指g2o::BaseVertex<6,g2o::SE3Quat>里面的6
 * 2.数据类型，这里是指g2o::BaseVertex<6,g2o::SE3Quat>里面的SE3Quat
 * 3.函数read，
 * 4.函数write
 * 5.函数setToOriginImpl，用来设定初值，一定要重写
 * 6.函数oplusImpl，加法定义，用来计算估计值的加法运算，一定要重写
****************************/
// 为什么相机位姿顶点类VertexSE3Expmap使用了李代数表示相机位姿，而不是使用旋转矩阵和平移矩阵？
// 这是因为旋转矩阵是有约束的矩阵，它必须是正交矩阵且行列式为1。使用它作为优化变量就会引入额外的约束条件，
// 从而增大优化的复杂度。而将旋转矩阵通过李群-李代数之间的转换关系转换为李代数表示，
// 就可以把位姿估计变成无约束的优化问题，求解难度降低
class myVertex : public g2o::BaseVertex<6, g2o::SE3Quat> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    myVertex() : BaseVertex<6, g2o::SE3Quat>() {}

    virtual bool read(std::istream &is) {
        g2o::Vector7 est;
        g2o::internal::readVector(is, est);
        setEstimate(g2o::SE3Quat(est).inverse());
        return true;
    }

    virtual bool write(std::ostream &os) const {
        return g2o::internal::writeVector(os, estimate().inverse().toVector());
    }

    virtual void setToOriginImpl() {
        _estimate = g2o::SE3Quat();
    }

    virtual void oplusImpl(const double *update_) {
        Eigen::Map<const g2o::Vector6> update(update_);
        // 不能直接加，原因是变换矩阵不满足加法封闭
        setEstimate(g2o::SE3Quat::exp(update) * estimate());
    }
};

/****************************
 * 自定义边，根据边的模板定义，template <int D, typename E, typename VertexXi, typename VertexXj>
 * 其中
 *    D: 误差自由度，这里是比较像素坐标差值，一共2个自由度，x,y
 *    E: 误差数据类型，这里是像素坐标值，用g2o::Vector2表示
 * 2.数据类型，这里是指g2o::BaseVertex<6,g2o::SE3Quat>里面的SE3Quat
 * 3.函数read，
 * 4.函数write
 * 5.函数setToOriginImpl，用来设定初值，一定要重写
 * 6.函数oplusImpl，加法定义，用来计算估计值的加法运算，一定要重写
****************************/
class myEdgeProjectXYZ2UV
        : public g2o::BaseBinaryEdge<2, g2o::Vector2, g2o::VertexPointXYZ, g2o::VertexSE3Expmap> {
public:
    myEdgeProjectXYZ2UV() {
        _cam = 0;
        resizeParameters(1);
        installParameter(_cam, 0);
    }

    virtual bool read(std::istream &is) {}

    virtual bool write(std::ostream &os) const {}

    // computeError函数是使用当前顶点的值计算的测量值与真实的测量值之间的误差
    virtual void computeError() {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexPointXYZ *v2 = static_cast<const g2o::VertexPointXYZ *>(_vertices[0]);
        const g2o::CameraParameters *cam = static_cast<const g2o::CameraParameters *>(parameter(0));
        // v1->estimate().map(v2->estimate()) 就是
        // |Xc|       |Xw|
        // |Yc| = R * |Yw| + t
        // |Zc|       |Zw|
        // cam_map 就是把
        // |u|   |1/dx   0   u0|   |f * Xc/Zc|
        // |v| = | 0   1/dy  v0| * |f * Yc/Zc|
        // |1|   | 0     0    1|   |    1    |
        _error = measurement() - cam->cam_map(v1->estimate().map(v2->estimate()));
        LOG(INFO) << "Err: " << _error(0) << " " << _error(1);
    }

    // linearizeOplus函数是在当前顶点的值下，该误差对优化变量的偏导数，即jacobian
    virtual void linearizeOplus() {
        // SE3 -> vj
        g2o::VertexSE3Expmap *vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T(vj->estimate());
        //  Xw -> vi
        g2o::VertexPointXYZ *vi = static_cast<g2o::VertexPointXYZ *>(_vertices[0]);
        g2o::Vector3 xyz = vi->estimate();
        //  Xc = R * Xw + t
        g2o::Vector3 xyz_trans = T.map(xyz);

        // Camera coordinate
        number_t x = xyz_trans[0];
        number_t y = xyz_trans[1];
        number_t z = xyz_trans[2];
        number_t z_2 = z * z;

        const g2o::CameraParameters *cam = static_cast<const g2o::CameraParameters *>(parameter(0));

        bool choose_my_jacobian = false;
        Eigen::Matrix<number_t, 2, 3, Eigen::ColMajor> tmp;
        tmp(0, 0) = choose_my_jacobian ? (cam->focal_length / z) : (cam->focal_length);
        tmp(0, 1) = 0;
        tmp(0, 2) = choose_my_jacobian ? (-x / z_2 * cam->focal_length) : (-x / z * cam->focal_length);

        tmp(1, 0) = 0;
        tmp(1, 1) = choose_my_jacobian ? (cam->focal_length / z) : (cam->focal_length);
        tmp(1, 2) = choose_my_jacobian ? (-y / z_2 * cam->focal_length) : (-y / z * cam->focal_length);

        _jacobianOplusXi = (choose_my_jacobian ? (tmp) : (-1. / z * tmp)) * T.rotation().toRotationMatrix();

        _jacobianOplusXj(0, 0) = choose_my_jacobian ? (1. / z * cam->focal_length) :
                                 (x * y / z_2 * cam->focal_length);
        _jacobianOplusXj(0, 1) = choose_my_jacobian ? 0 :
                                 (-(1 + (x * x / z_2)) * cam->focal_length);
        _jacobianOplusXj(0, 2) = choose_my_jacobian ? (-x / z_2 * cam->focal_length) :
                                 (y / z * cam->focal_length);
        _jacobianOplusXj(0, 3) = choose_my_jacobian ? (-x * y / z_2 * cam->focal_length) :
                                 (-1. / z * cam->focal_length);
        _jacobianOplusXj(0, 4) = choose_my_jacobian ? ((1 + (x * x / z_2)) * cam->focal_length) : 0;
        _jacobianOplusXj(0, 5) = choose_my_jacobian ?
                                 (-y / z * cam->focal_length) :
                                 (x / z_2 * cam->focal_length);

        _jacobianOplusXj(1, 0) = choose_my_jacobian ? 0 :
                                 ((1 + y * y / z_2) * cam->focal_length);
        _jacobianOplusXj(1, 1) = choose_my_jacobian ?
                                 (1. / z * cam->focal_length) :
                                 (-x * y / z_2 * cam->focal_length);
        _jacobianOplusXj(1, 2) = choose_my_jacobian ?
                                 (-y / z_2 * cam->focal_length) :
                                 (-x / z * cam->focal_length);
        _jacobianOplusXj(1, 3) = choose_my_jacobian ?
                                 (-(1 + y * y / z_2) * cam->focal_length) : 0;
        _jacobianOplusXj(1, 4) = choose_my_jacobian ?
                                 (x * y / z_2 * cam->focal_length) :
                                 (-1. / z * cam->focal_length);
        _jacobianOplusXj(1, 5) = choose_my_jacobian ?
                                 (x / z * cam->focal_length) :
                                 (y / z_2 * cam->focal_length);

        auto &&log = COMPACT_GOOGLE_LOG_INFO;
        log.stream() << "JacXi: [";
        for (int i = 0; i < _jacobianOplusXi.rows(); ++i) {
            for (int j = 0; j < _jacobianOplusXi.cols(); ++j) {
                log.stream() << _jacobianOplusXi(i, j) << " ";
            }
        }
        log.stream() << "]\n";
        log.stream() << "JacXj: [";
        for (int i = 0; i < _jacobianOplusXj.rows(); ++i) {
            for (int j = 0; j < _jacobianOplusXj.cols(); ++j) {
                log.stream() << _jacobianOplusXj(i, j) << " ";
            }
        }
        log.stream() << "]\n";
    }

public:
    g2o::CameraParameters *_cam;
};

void bundleAdjustment(
        const vector<Point3f> points_3d,
        const vector<Point2f> points_2d,
        Mat &K);

int main(int argc, char **argv) {
    // Input
    if (argc < 2) {
        cout << endl;
        cout << "Please type: " << endl;
        cout << "ba_demo [PIXEL_NOISE] [OUTLIER RATIO] [ROBUST_KERNEL] [STRUCTURE_ONLY] [DENSE]" << endl;
        cout << endl;
        cout << "PIXEL_NOISE: noise in image space (E.g.: 1)" << endl;
        cout << "OUTLIER_RATIO: probability of spuroius observation  (default: 0.0)" << endl;
        cout << "ROBUST_KERNEL: use robust kernel (0 or 1; default: 0==false)" << endl;
        cout
                << "STRUCTURE_ONLY: performe structure-only BA to get better point initializations (0 or 1; default: 0==false)"
                << endl;
        cout << "DENSE: Use dense solver (0 or 1; default: 0==false)" << endl;
        cout << endl;
        cout << "Note, if OUTLIER_RATIO is above 0, ROBUST_KERNEL should be set to 1==true." << endl;
        cout << endl;
        exit(0);
    }
    double PIXEL_NOISE = atof(argv[1]);
    double OUTLIER_RATIO = 0.0;
    if (argc > 2) {
        OUTLIER_RATIO = atof(argv[2]);
    }
    bool ROBUST_KERNEL = false;
    if (argc > 3) {
        ROBUST_KERNEL = atoi(argv[3]) != 0;
    }
    bool STRUCTURE_ONLY = false;
    if (argc > 4) {
        STRUCTURE_ONLY = atoi(argv[4]) != 0;
    }
    bool DENSE = false;
    if (argc > 5) {
        DENSE = atoi(argv[5]) != 0;
    }
    cout << "PIXEL_NOISE: " << PIXEL_NOISE << endl;
    cout << "OUTLIER_RATIO: " << OUTLIER_RATIO << endl;
    cout << "ROBUST_KERNEL: " << ROBUST_KERNEL << endl;
    cout << "STRUCTURE_ONLY: " << STRUCTURE_ONLY << endl;
    cout << "DENSE: " << DENSE << endl;

    // Google log
    google::InitGoogleLogging(argv[0]);
    google::SetLogDestination(google::GLOG_INFO, "../../log/g2o_log_");

    // Initialize
    vector<Point3f> p3d;
    vector<Point2f> p2d;
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    // 导入3D点和对应的2D点
    ifstream fp3d(p3d_file);
    if (!fp3d) {
        cout << "No p3d.text file" << endl;
        return -1;
    } else {
        while (!fp3d.eof()) {
            double pt3[3] = {0};
            for (auto &p:pt3) {
                fp3d >> p;
            }
            p3d.push_back(Point3f(pt3[0], pt3[1], pt3[2]));
        }
    }
    ifstream fp2d(p2d_file);
    if (!fp2d) {
        cout << "No p2d.text file" << endl;
        return -1;
    } else {
        while (!fp2d.eof()) {
            double pt2[2] = {0};
            for (auto &p:pt2) {
                fp2d >> p;
            }
            Point2f p2(pt2[0], pt2[1]);
            p2d.push_back(p2);
        }
    }
    assert(p3d.size() == p2d.size());
    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    // 创建稀疏优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    // 创建求解器solver，这里和平时用的GN, LM, DogLeg不太一样，因为要考虑structure only情况
    // 所以用另外的一种方式来定义求解器，其实也可以一样，只是包装了一下而已，因为
    // G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix6_3_cholmod, new CholmodSolverCreator(
    // OptimizationAlgorithmProperty("lm_fix6_3_cholmod", "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 6, 3)));
    // 可以看到，其实这里用的 lm_fix6_3_cholmod 就是用cholmod的LM算法，然后固定的6×3矩阵形式来求解
    string solverName = "lm_fix6_3";
    if (DENSE) {
        solverName = "lm_dense6_3";
    } else {
#ifdef G2O_HAVE_CHOLMOD
        solverName = "lm_fix6_3_cholmod";
#else
        solverName = "lm_fix6_3";
#endif
    }
    cout << "solverName = " << solverName << endl;

    // 定义稀疏优化器的算法、矩阵维度
    g2o::OptimizationAlgorithmProperty solverProperty;
    optimizer.setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct(solverName, solverProperty));

    // 设置相机参数
    g2o::CameraParameters *cam_params = new g2o::CameraParameters(K.at<double>(0, 0),
                                                                  Eigen::Vector2d(K.at<double>(0, 2),
                                                                                  K.at<double>(1, 1)), 0.);
    cam_params->setId(0);
    if (!optimizer.addParameter(cam_params)) {
        assert(false);
    }

    // Vertex id should increase
    int vertex_id = 0;

    // Add camera pose into vertex
    int N_pose = 3;
    for (int i = 0; i < N_pose; ++i) {
        g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
        pose->setId(vertex_id);
        if (i < 2) {
            pose->setFixed(true);
        }
        pose->setEstimate(g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));
        optimizer.addVertex(pose);
        vertex_id++;
    }


    for (int i = 0; i < p3d.size(); ++i) {
        // 定义当前三维点为节点
        g2o::VertexPointXYZ *point = new g2o::VertexPointXYZ();
        point->setId(vertex_id);
        point->setMarginalized(true);
        point->setEstimate(Eigen::Vector3d(p3d[i].x, p3d[i].y, p3d[i].z));
//        point->setFixed(index++);
        optimizer.addVertex(point);
        // Add points into vertex
        // 循环不能放外面！！
        for (int j = 0; j < N_pose; ++j) {
            // 添加边
            g2o::EdgeProjectXYZ2UV *e = new g2o::EdgeProjectXYZ2UV();
            // 边的一头连接当前三维点
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(point));
            // 边的另一头连接第j个位姿，可以认为在位姿上相机可以观测到该三维点，所以需要连接该位姿
            // 确定是位姿，因为 optimizer.vertices().find(j) 返回的是 VertexIDMap 类型
            // 而该类型定义是 typedef std::unordered_map<int, Vertex*> VertexIDMap;
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>
            (optimizer.vertices().find(j)->second));
            // 把当前观测值复制给该边，用来计算误差
            e->setMeasurement(Eigen::Vector2d(p2d[i].x, p2d[i].y));
            // 信息矩阵是单位矩阵，没有任何偏向
            e->information() = Matrix2d::Identity();
            // 设置鲁邦核函数
            if (ROBUST_KERNEL) {
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
            }
            // 第二个参数是优化器内添加的参数的id。当你调用addEdge来添加这条边时，
            // 会根据第二个参数的id，把相应的参数地址给边，以后边内的成员函数，就根据第一个参数，拿到这个地址。
            // TODO:并不是太理解
            e->setParameterId(0, 0);
            optimizer.addEdge(e);
            vertex_id++;
        }
    }

    optimizer.initializeOptimization();
    optimizer.setVerbose(true);

    {
        // 定义StructureOnlySolver求解器
        g2o::StructureOnlySolver<3> structure_only_ba;
        cout << "Performing structure-only BA:" << endl;
        // 定义并新增三维点作为图的节点，并给到StructureOnlySolver求解器
        g2o::OptimizableGraph::VertexContainer points;
        LOG(INFO) << __FUNCTION__ << " " << __LINE__;
        int n = 0;
        // 原来的三维点都在optimizer中，因此需要遍历optimizer来获取原来的三维点
        for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin();
             it != optimizer.vertices().end(); ++it) {
            // Add point3d to vertex
            g2o::OptimizableGraph::Vertex *v = static_cast<g2o::OptimizableGraph::Vertex *>(it->second);
            // TODO:不知道为什么要判断维度为3？为什么会出现不是3的情况？
            // 因为节点里面还有位姿，维度是6，要把位姿去掉才行
            if (v->dimension() == 3) {
//            cout << n++ << endl;
                points.push_back(v);
            }
        }
        cout << "Add point3d end" << endl;
        LOG(INFO) << __FUNCTION__ << " " << __LINE__;
        // 进行structure only优化
        structure_only_ba.calc(points, 10);
    }

    optimizer.optimize(10);
//    bundleAdjustment(p3d, p2d, K);
    return 0;
}

void bundleAdjustment(
        const vector<Point3f> points_3d,
        const vector<Point2f> points_2d,
        Mat &K) {
    // creat g2o
    // new g2o version. Ref:https://www.cnblogs.com/xueyuanaichiyu/p/7921382.html

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3> > Block;  // pose 维度为 6, landmark 维度为 3

    // 第1步：创建一个线性求解器LinearSolver
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();

    // 第2步：创建BlockSolver。并用上面定义的线性求解器初始化
    Block *solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));

    // 第3步：创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
            std::unique_ptr<Block>(solver_ptr));

    // 第4步：创建稀疏优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

//    // old g2o version
//    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
//    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
//    Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
//    g2o::SparseOptimizer optimizer;
//    optimizer.setAlgorithm ( solver );

    // 第5步：定义图的顶点和边。并添加到SparseOptimizer中

    // ----------------------开始你的代码：设置并添加顶点，初始位姿为单位矩阵
    myVertex *pose = new myVertex(); // camera pose
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));
    optimizer.addVertex(pose);

    int index = 1;
    for (const Point3f p:points_3d)   // landmarks
    {
        g2o::VertexPointXYZ *point = new g2o::VertexPointXYZ();
        point->setId(index);
        point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
        point->setMarginalized(true);
        point->setFixed(index++);
        optimizer.addVertex(point);
    }
    // ----------------------结束你的代码

    // 设置相机内参
    g2o::CameraParameters *camera = new g2o::CameraParameters(
            K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0);
    camera->setId(0);
    optimizer.addParameter(camera);

    // 设置边
    index = 1;
    for (const Point2f p:points_2d) {
        myEdgeProjectXYZ2UV *edge = new myEdgeProjectXYZ2UV();
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexPointXYZ *> ( optimizer.vertex(index)));
        edge->setVertex(1, pose);
        edge->setMeasurement(Eigen::Vector2d(p.x, p.y));  //设置观测值
        edge->setParameterId(0, 0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }

    // 第6步：设置优化参数，开始执行优化
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(100);

    // 输出优化结果
    cout << endl << "After optimization:" << endl;
    cout << "T = " << endl << Eigen::Isometry3d(pose->estimate()).matrix() << endl;
}