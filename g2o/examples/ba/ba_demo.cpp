// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <stdint.h>

#include <iostream>
#include <unordered_set>

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
using namespace std;

class Sample {
public:
    static int uniform(int from, int to) { return static_cast<int>(g2o::Sampler::uniformRand(from, to)); }
};

int main(int argc, const char *argv[]) {
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

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
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

    g2o::OptimizationAlgorithmProperty solverProperty;
    optimizer.setAlgorithm(
            g2o::OptimizationAlgorithmFactory::instance()->construct(solverName, solverProperty));

    vector<Vector3d> true_points;
    //打开输出文件
    ofstream outf;
    outf.open("ba_demo_points3d.txt");
    outf << "x  y  z " << "\n";
    for (size_t i = 0; i < 500; ++i) {
        // X = (-1.5 ~ 1.5)
        double x = (g2o::Sampler::uniformRand(0., 1.) - 0.5) * 3;
        // Y = (-0.5 ~ 0.5)
        double y = g2o::Sampler::uniformRand(0., 1.) - 0.5;
        // Z = (3 ~ 4)
        double z = g2o::Sampler::uniformRand(0., 1.) + 3;
        outf << x << " " << y << " " << z << " " << "\n";
        true_points.push_back(Vector3d((g2o::Sampler::uniformRand(0., 1.) - 0.5) * 3,
                                       g2o::Sampler::uniformRand(0., 1.) - 0.5,
                                       g2o::Sampler::uniformRand(0., 1.) + 3));
    }
    outf.close();

    double focal_length = 1000.;
    Vector2d principal_point(320., 240.);

    vector<g2o::SE3Quat,
            aligned_allocator<g2o::SE3Quat> > true_poses;
    g2o::CameraParameters *cam_params
            = new g2o::CameraParameters(focal_length, principal_point, 0.);
    cam_params->setId(0);

    if (!optimizer.addParameter(cam_params)) {
        assert(false);
    }

    int vertex_id = 0;
    for (size_t i = 0; i < 15; ++i) {
        // 平移矩阵从 [-1,0,0] 到 [-0.44,0,0]
        Vector3d trans(i * 0.04 - 1., 0, 0);
        // 旋转矩阵以四元素形式保存，为单位矩阵
        Eigen::Quaterniond q;
        q.setIdentity();
        // 利用上面的平移矩阵trans和旋转矩阵q来构造位姿李群 pose
        g2o::SE3Quat pose(q, trans);
        // 初始化图优化用的节点 v_se3
        g2o::VertexSE3Expmap *v_se3
                = new g2o::VertexSE3Expmap();
        // 设置节点ID
        v_se3->setId(vertex_id);
        // 前2个位姿固定，不进行优化
        // TODO:所以structure_only的条件是至少要2个位姿？
        if (i < 2) {
            v_se3->setFixed(true);
        }
        // 节点 v_se3 的估计值设置为上面计算得到的位姿 pose
        v_se3->setEstimate(pose);
        // 给优化器添加节点 v_se3
        optimizer.addVertex(v_se3);
        // 保存当前位姿
        true_poses.push_back(pose);
        // 更新节点ID
        vertex_id++;
    }
    // 在节点后面添加点，因为图的节点不仅仅是有位姿，还有三维点（优不优化再说）
    int point_id = vertex_id;
    // 就只是计数，并没有什么用
    int point_num = 0;
    // 三维点的总误差
    double sum_diff2 = 0;

    //
    cout << endl;
    unordered_map<int, int> pointid_2_trueid;
    unordered_set<int> inliers;

    // 遍历三维点，添加图优化中的三维点的节点
    for (size_t i = 0; i < true_points.size(); ++i) {
        g2o::VertexPointXYZ *v_p
                = new g2o::VertexPointXYZ();
        v_p->setId(point_id);
        v_p->setMarginalized(true);
        // 估计值要添加高斯噪声，否则优化体现不出来，和真值都一样了
        v_p->setEstimate(true_points.at(i)
                         + Vector3d(g2o::Sampler::gaussRand(0., 1),
                                    g2o::Sampler::gaussRand(0., 1),
                                    g2o::Sampler::gaussRand(0., 1)));
        // 添加观测值，即三维点投影到平面上的二维坐标值，是用来计算误差的
        int num_obs = 0;
        // 遍历位姿，true_poses.size()就是位姿的总数，来计算三维点通过相机模型投影到平面上的二维点，即观测值
        for (size_t j = 0; j < true_poses.size(); ++j) {
            Vector2d z = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));
            // 二维点的坐标值范围需要在 u ∈ [0, 640], v ∈ [0, 480]
            if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480) {
                // 如果范围合理，这个观测值才有效，否则无效
                ++num_obs;
            }
        }
        // 在所有位姿的投影下，至少要有2组有效观测值，因为这里是模拟数据，所以有效观测值是指投影坐标在范围内的观测值
        // 这里是对每个三维坐标点进行投影，用的是所有位姿，来检测是否能满足2组有效观测值的条件
        if (num_obs >= 2) {
            // 证明当前三维点符合"至少要有2组有效观测值"的条件，可以添加到图中，作为节点优化
            optimizer.addVertex(v_p);
            // TODO:这里为什么要重复遍历？
            // 是因为添加N个三维点的过程中，需要针对第i个三维点添加第j个位姿，需要通过遍历方式来明确他们之间的关系
            bool inlier = true;
            for (size_t j = 0; j < true_poses.size(); ++j) {
                Vector2d z = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));
                if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480) {
                    double sam = g2o::Sampler::uniformRand(0., 1.);
                    // 检测输入的离群值比率是否大于默认的(0,1)范围，如果是则重新计算观测值，且该点判断为outlier
                    if (sam < OUTLIER_RATIO) {
                        z = Vector2d(Sample::uniform(0, 640),
                                     Sample::uniform(0, 480));
                        inlier = false;
                    }
                    // 如果该点是内点,则观测值添加观测噪声，该噪声由手动输入的PIXEL_NOISE值决定
                    z += Vector2d(g2o::Sampler::gaussRand(0., PIXEL_NOISE),
                                  g2o::Sampler::gaussRand(0., PIXEL_NOISE));
                    // 添加该三维点的边
                    // TODO:这里不是太理解？
                    // 看完边的两头好像就理解了
                    g2o::EdgeProjectXYZ2UV *e = new g2o::EdgeProjectXYZ2UV();
                    // 边的一头连接当前三维点
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(v_p));
                    // 边的另一头连接第j个位姿，可以认为在位姿上相机可以观测到该三维点，所以需要连接该位姿
                    // 确定是位姿，因为 optimizer.vertices().find(j) 返回的是 VertexIDMap 类型
                    // 而该类型定义是 typedef std::unordered_map<int, Vertex*> VertexIDMap;
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>
                    (optimizer.vertices().find(j)->second));
                    // 把当前观测值复制给该边，用来计算误差
                    e->setMeasurement(z);
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
                }
            }

            // 只计算内点的误差，外点误差不计算，否则可以会导致发散？
            if (inlier) {
                // 把内点搜集到inliers容器里面
                inliers.insert(point_id);
                // 计算三维点的误差
                Vector3d diff = v_p->estimate() - true_points[i];
                sum_diff2 += diff.dot(diff);
            }
            pointid_2_trueid.insert(make_pair(point_id, i));
            ++point_id;
            ++point_num;
        }
    }
    cout << endl;
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    // 如果只优化三维点的话，需要另外定义 StructureOnlySolver 求解器来求解
    // 原来的 optimizer 是 SparseOptimizer 类型的，并不能直接对点进行优化
    if (STRUCTURE_ONLY) {
        // 定义StructureOnlySolver求解器
        g2o::StructureOnlySolver<3> structure_only_ba;
        cout << "Performing structure-only BA:" << endl;
        // 定义并新增三维点作为图的节点，并给到StructureOnlySolver求解器
        g2o::OptimizableGraph::VertexContainer points;
        // 原来的三维点都在optimizer中，因此需要遍历optimizer来获取原来的三维点
        for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin();
             it != optimizer.vertices().end(); ++it) {
            //
            g2o::OptimizableGraph::Vertex *v = static_cast<g2o::OptimizableGraph::Vertex *>(it->second);
            // TODO:不知道为什么要判断维度为3？为什么会出现不是3的情况？
            if (v->dimension() == 3)
                points.push_back(v);
        }
        // 进行structure only优化
        structure_only_ba.calc(points, 10);
    }
    //optimizer.save("test.g2o");
    cout << endl;
    cout << "Performing full BA:" << endl;
    optimizer.optimize(10);
    cout << endl;
    cout << "Point error before optimisation (inliers only): " << sqrt(sum_diff2 / inliers.size()) << endl;
    point_num = 0;
    sum_diff2 = 0;
    // 计算内点的误差，所以要遍历内点容器 pointid_2_trueid，要注意的是
    // pointid_2_trueid.insert(make_pair(point_id, i));
    // it->first 是图节点中的ID(连续的), v_it->second 是三维点容器的ID(大概率不是连续的)
    for (unordered_map<int, int>::iterator it = pointid_2_trueid.begin();
         it != pointid_2_trueid.end(); ++it) {
        // 找到图节点对应的二维点 v_it
        g2o::HyperGraph::VertexIDMap::iterator v_it = optimizer.vertices().find(it->first);
        // 判断其合法性
        if (v_it == optimizer.vertices().end()) {
            cerr << "Vertex " << it->first << " not in graph!" << endl;
            exit(-1);
        }
        // 找到对应的三维点
        g2o::VertexPointXYZ *v_p = dynamic_cast< g2o::VertexPointXYZ * > (v_it->second);
        // 判断其合法性
        if (v_p == 0) {
            cerr << "Vertex " << it->first << "is not a PointXYZ!" << endl;
            exit(-1);
        }
        // 计算误差
        Vector3d diff = v_p->estimate() - true_points[it->second];
        // 因为it->first大概率比it->second小，所以it->first更容易先遍历完，如果遍历完了剩下的就不用管了
        if (inliers.find(it->first) == inliers.end())
            continue;
        sum_diff2 += diff.dot(diff);
        ++point_num;
    }
    cout << "Point error after optimisation (inliers only): " << sqrt(sum_diff2 / inliers.size()) << endl;
    cout << endl;
}
