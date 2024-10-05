#pragma once

#include "../include/initial_ex_rotation.h"
#include "../include/g2o_tools.h"
#include "ros/package.h"
#include <unistd.h>
#include <Python.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#define PBWIDTH 30
#define BOLDCYAN  "\033[1m\033[36m"
#define BOLDYELLOW "\033[1m\033[33m"
#define RESET "\033[0m"
#define BOLDGREEN "\033[1m\033[32m" 
#define GREEN  "\033[32m"
#define PBSTR  "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */ 
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#include <numeric>
#include <Eigen/Dense>
#include <vector>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <unordered_set>
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/core/auto_differentiation.h"
#include <algorithm>

#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0
namespace Eigen{
    Eigen::Matrix3d skew(Eigen::Vector3d vec){
        return (Eigen::Matrix3d() << 0, vec(2), -vec(1), -vec(2), 0, vec(0), vec(1), -vec(0), 0).finished();
    }
}

void printProgress_svd(double percentage, int axis_ascii) {
    
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
//    # extern int flag_init;
    printf(BOLDCYAN "[旋转激励程度] ");
    if (percentage < 1) {
        // printf(BOLDYELLOW "%c Axis: ", char(axis_ascii));
        printf(YELLOW "%3d%% [%.*s%*s]\n", val, lpad, PBSTR, rpad, "");
        std::cout << RESET;
        // flag_init=1;
    } else {  

        printf(BOLDGREEN "%c Axis: ", char(axis_ascii));
        printf(GREEN "%3d%% [%.*s%*s]\n", val, lpad, PBSTR, rpad, "");
       std::cout << RESET;
    }
}
InitialEXRotation::InitialEXRotation(){
    frame_count = 0;
    Rc.push_back(Matrix3d::Identity());
    Rc_g.push_back(Matrix3d::Identity());
    Rimu.push_back(Matrix3d::Identity());
    ric = Matrix3d::Identity();
}

void InitialEXRotation::Reset()
{
    frame_count = 0;
    Rc.clear();
    Rc_g.clear();
    Rimu.clear();
    Rc.push_back(Matrix3d::Identity());
    Rc_g.push_back(Matrix3d::Identity());
    Rimu.push_back(Matrix3d::Identity());
    ric = Matrix3d::Identity();
}
void InitialEXRotation::clear() {
    // CSI[2J clears screen, CSI[H moves the cursor to top-left corner
    cout << "\x1B[2J\x1B[H";
}
void InitialEXRotation::printProgress(double percentage, int axis_ascii) {
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
//    # extern int flag_init;
    printf(BOLDCYAN "[Initialization] ");
    if (percentage < 1) {
        printf(BOLDYELLOW "Rotation around IMU %c Axis: ", char(axis_ascii));
        printf(YELLOW "%3d%% [%.*s%*s]\n", val, lpad, PBSTR, rpad, "");
        cout << RESET;
        // flag_init=1;
    } else {  

        printf(BOLDGREEN "Rotation around IMU %c Axis: ", char(axis_ascii));
        printf(GREEN "%3d%% [%.*s%*s]\n", val, lpad, PBSTR, rpad, "");
        cout << RESET;
    }
}
class EdgeHE : public g2o::BaseUnaryEdge<3, g2o::Vector3, VertexSim3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeHE(const Eigen::Isometry3d &Ta, const Eigen::Isometry3d &Tb, const double weight): _Ta(Ta), _Tb(Tb), _weight(weight){}
    virtual void computeError() override {
        const VertexSim3 *v = static_cast<VertexSim3 *> (_vertices[0]);
        Eigen::Matrix3d Rab;
        Eigen::Vector3d tab;
        double s;
        std::tie(Rab, tab, s) = Sim3Exp<double>(v->estimate().data());
        Eigen::Vector4d tsab;  // [tab, s]
        tsab.block<3, 1>(0, 0) = tab;
        tsab(3) = s;
        Eigen::AngleAxisd axisTa(_Ta.rotation()), axisTb(_Tb.rotation());
        Eigen::Vector3d errRotVec = Rab * (axisTb.angle() * axisTb.axis()) - (axisTa.angle() * axisTa.axis()); 
        Eigen::Matrix<double, 3, 4> A;
        Eigen::Vector3d b;
        A.block<3, 3>(0, 0) = _Ta.rotation() - Eigen::Matrix3d::Identity();
        A.block<3, 1>(0, 3) = _Ta.translation();
        b = Rab * _Tb.translation();
        Eigen::Vector3d errTran = A * tsab - b;
        _error << _weight * (errRotVec + errTran);
    }

    virtual void linearizeOplus() override {
        const VertexSim3 *v = static_cast<VertexSim3 *> (_vertices[0]);
        Eigen::Matrix3d Rab;
        Eigen::Vector3d tab;
        double s;
        std::tie(Rab, tab, s) = Sim3Exp<double>(v->estimate().data());
        Eigen::Vector4d tsab;  // [tab, s]
        tsab.block<3, 1>(0, 0) = tab;
        tsab(3) = s;
        Eigen::AngleAxisd axisTb(_Tb.rotation());
        Eigen::Matrix3d JacobianRotVec = -1.0 * Eigen::skew(Rab * (axisTb.angle() * axisTb.axis())); // -(Rp)^ or (-Rp)^
        Eigen::Matrix<double, 3, 4> JacobianTran;
        JacobianTran.block<3, 3>(0, 0) = _Ta.rotation() - Eigen::Matrix3d::Identity();
        JacobianTran.block<3, 1>(0, 3) = _Ta.translation();
        _jacobianOplusXi << _weight * JacobianRotVec, _weight * JacobianTran;  // (3,3) (3,4) -> concat -> (3,7)
    }

    void updateWeight(double weight){
        _weight = weight;
    }

    double returnSquaredWeight() const{
        return _weight * _weight;
    }

    virtual bool read(std::istream &in) override {return false;}

    virtual bool write(std::ostream &out) const override {return false;}
private:
    Eigen::Isometry3d _Ta, _Tb;
    double _weight;
};


class EdgeRegulation : public g2o::BaseUnaryEdge<3, g2o::Vector3, VertexSim3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeRegulation(){}
    // virtual void computeError() override {
    //     const VertexSim3 *v = static_cast<VertexSim3 *> (_vertices[0]);
    //     g2o::Sim3 Tab(v->estimate());
    //     Eigen::Vector3d translation = Tab.translation();
    //     _error << translation;
    // }

    // virtual void linearizeOplus() override {
    //     const VertexSim3 *v = static_cast<VertexSim3 *> (_vertices[0]);
    //     Eigen::Matrix<double, 3, 7> Jacobian;
    //     Jacobian.setZero();
    //     Jacobian.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    //     _jacobianOplusXi << Jacobian;  // (3,3) (3,4) -> concat -> (3,7)
    // }

    virtual bool read(std::istream &in) override {return false;}

    virtual bool write(std::ostream &out) const override {return false;}
    template <typename T>
    bool operator()(const T* params, T* error) const {
        error[0] = params[3];
        error[1] = params[4];
        error[2] = params[5];
        return true;
    }
    G2O_MAKE_AUTO_AD_FUNCTIONS
};
std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double> HECalib(const std::vector<Eigen::Isometry3d> &TaList, const std::vector<Eigen::Isometry3d> &TbList){
    assert(TaList.size()==TbList.size());
    std::vector<Eigen::Vector3d> alphaList, betaList;
    Eigen::MatrixX4d A(3*TaList.size(), 4);  // 3N, 4
    Eigen::VectorXd b(3*TaList.size());  // 3N
    Eigen::Vector3d alpha_mean, beta_mean;
    alpha_mean.setZero();
    beta_mean.setZero();
    for(std::size_t i = 0; i < TaList.size(); ++i){
        // Ta
        Eigen::AngleAxisd ax;
        ax.fromRotationMatrix(TaList[i].rotation());
        Eigen::Vector3d alpha = ax.angle() * ax.axis();
        alphaList.push_back(alpha);
        // Tb
        alpha_mean += alpha;
        ax.fromRotationMatrix(TbList[i].rotation());
        Eigen::Vector3d beta = ax.angle() * ax.axis();
        betaList.push_back(beta);
        beta_mean += beta;
    }
    alpha_mean /= TaList.size();
    beta_mean /= TbList.size();
    // Decentralization and Compute Covariance Matrix
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for(std::vector<Eigen::Vector3d>::const_iterator ita=alphaList.begin(), itb=betaList.begin(); ita!=alphaList.end() ;++ita, ++itb){
        H += (*itb - beta_mean) * (*ita - alpha_mean).transpose(); // (3,1) x (1,3) -> (3,3)
    }
    // Computate Rotation Part RAB
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d Ut = svd.matrixU().transpose(), Vt = svd.matrixV().transpose();
    Eigen::Matrix3d RAB = Vt.transpose() * Ut;
    if(RAB.determinant() < 0){
        Vt.row(2) *= -1;
        RAB = Vt.transpose() * Ut;
    }
    // Compute Translation Part tAB
    for(std::size_t i = 0; i < TaList.size(); ++i){
        A.block<3, 3>(3*i, 0) = TaList[i].rotation() - Eigen::Matrix3d::Identity();
        A.block<3, 1>(3*i, 3) = TaList[i].translation();
        b.block<3, 1>(3*i, 0) = RAB * TbList[i].translation();
    }
    Eigen::Vector4d res = (A.transpose() * A).ldlt().solve(A.transpose() * b);  // use cholesky decompostion to solve this problem with large
    return std::make_tuple(RAB, res.block<3, 1>(0, 0), res(3));

}
std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double> HECalibRobustKernelg2o(const std::vector<Eigen::Isometry3d> &vTa, const std::vector<Eigen::Isometry3d> &vTb,
    const Eigen::Matrix3d &initialRotation, const Eigen::Vector3d &initialTranslation, const double &initialScale, const double &robust_kernel_size=0.1,
    const bool regulation=true, const double regulation_ratio= 0.005, const bool verbose=true){

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<7, 3>> BlockSolverType; 
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
    auto solver = new g2o::OptimizationAlgorithmDogleg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);   // 设置求解器
    optimizer.setVerbose(verbose);       // 打开调试输出
    VertexSim3 *v(new VertexSim3);
    Eigen::AngleAxisd ax(initialRotation);
    Eigen::Vector3d rotvec = ax.angle() * ax.axis();
    g2o::Vector7 initialValue;
    initialValue.head<3>() = rotvec;
    initialValue.segment<3>(3, 3) = initialTranslation;
    initialValue(6) = initialScale;
    v->setEstimate(initialValue);
    v->setId(0);
    optimizer.addVertex(v);
    for(std::size_t i = 0; i < vTa.size(); ++i){
        EdgeHE* edge = new EdgeHE(vTa[i], vTb[i], 1.0);
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setInformation(Eigen::Matrix3d::Identity());
        g2o::RobustKernelHuber* rk(new g2o::RobustKernelHuber);
        rk->setDelta(robust_kernel_size);
        edge->setRobustKernel(rk);
        optimizer.addEdge(edge);
    }
    if(regulation){
        EdgeRegulation* regedge = new EdgeRegulation();
        regedge->setId(vTa.size());
        regedge->setVertex(0, v);
        double ratio = vTa.size() * regulation_ratio;
        regedge->setInformation(Eigen::Matrix3d::Identity() * ratio);
        optimizer.addEdge(regedge);
    }
    optimizer.initializeOptimization();
    optimizer.optimize(20);
    Eigen::Matrix3d RCL;
    Eigen::Vector3d tCL;
    double scale;
    std::tie(RCL, tCL, scale) = Sim3Exp<double>(v->estimate().data());
    if(verbose){
        std::vector<double> chi2List;
        chi2List.reserve(optimizer.edges().size());
        double chi2Mean = 0, RegChi2 = 0;
        for(auto it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it){
            if((*it)->id() < (int) vTa.size()){
                EdgeHE* edge = dynamic_cast<EdgeHE*> (*it);
                chi2List.push_back((double)edge->chi2());
                chi2Mean += (double)edge->chi2();
            }else{
                EdgeRegulation* edge = dynamic_cast<EdgeRegulation*> (*it);
                RegChi2 = (double)edge->chi2();
            }
        }
        std::sort(chi2List.begin(),chi2List.end());
        std::cout << "Squared Error:\n";
        std::cout << "Max: " << chi2List[chi2List.size()-1] << std::endl;
        std::cout << "Min: " << chi2List[0] << std::endl;
        std::cout << "Median: " << chi2List[chi2List.size()/2] << std::endl;
        std::cout << "Mean: " << chi2Mean/chi2List.size() << std::endl;
        if(regulation){
            std::cout << "Regulation Squared Error: " << RegChi2 << std::endl;
        }
    }
    return {RCL, tCL, scale};
}
std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double> HECalibLineProcessg2o(const std::vector<Eigen::Isometry3d> &vTa, const std::vector<Eigen::Isometry3d> &vTb,
    const Eigen::Matrix3d &initialRotation, const Eigen::Vector3d &initialTranslation, const double &initialScale, const int in_max_iter = 5,
    const double mu0 = 64, const double divid_factor = 1.4, const double min_mu = 1e-1, const int ex_max_iter = 20,
    const bool regulation=true, const double regulation_ratio= 0.005, const bool verbose=true){

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<7, 3>> BlockSolverType; 
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);   // 设置求解器
    optimizer.setVerbose(verbose);       // 打开调试输出
    VertexSim3 *v(new VertexSim3);
    Eigen::AngleAxisd ax(initialRotation);
    Eigen::Vector3d rotvec = ax.angle() * ax.axis();
    g2o::Vector7 initialValue;
    initialValue.head<3>() = rotvec;
    initialValue.segment<3>(3, 3) = initialTranslation;
    initialValue(6) = initialScale;
    v->setEstimate(initialValue);
    v->setId(0);
    optimizer.addVertex(v);
    EdgeRegulation* regedge = new EdgeRegulation();
    for(std::size_t i = 0; i < vTa.size(); ++i){
        EdgeHE* edge(new EdgeHE(vTa[i], vTb[i], 1.0));
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setInformation(Eigen::Matrix<double, 3, 3>::Identity());
        optimizer.addEdge(edge);
    }
    if(regulation){
        regedge->setId(vTa.size());
        regedge->setVertex(0, v);
        double ratio = vTa.size() * regulation_ratio;
        regedge->setInformation(Eigen::Matrix3d::Identity() * ratio);
        optimizer.addEdge(regedge);
    }
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    double mu = mu0;
    for(unsigned short exiter = 0; exiter < ex_max_iter; ++exiter){
        double total_weight = 0;
        for(auto it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it){
            if((*it)->id() < (int) vTa.size()){
                EdgeHE* edge = dynamic_cast<EdgeHE*> (*it);
                double e2 = (double)(edge->chi2());
                double w = mu/(mu+e2);
                double w2 = w*w;
                total_weight += w2;
                edge->setInformation(w2*Eigen::Matrix3d::Identity());
            }   
        }
        if(regulation)
            regedge->setInformation(total_weight*regulation_ratio*Eigen::Matrix3d::Identity());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        mu /= divid_factor;
        if(mu < min_mu)
            break;
    }
    Eigen::Matrix3d RCL;
    Eigen::Vector3d tCL;
    double scale;
    std::tie(RCL, tCL, scale) = Sim3Exp<double>(v->estimate().data());
    if(verbose){
        std::vector<double> chi2List;
        chi2List.reserve(optimizer.edges().size());
        double chi2Mean = 0, RegChi2 = 0;
        for(auto it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it){
            if((*it)->id() < (int) vTa.size()){
                EdgeHE* edge = dynamic_cast<EdgeHE*> (*it);
                chi2List.push_back((double)edge->chi2());
                chi2Mean += (double)edge->chi2();
            }else{
                EdgeRegulation* edge = dynamic_cast<EdgeRegulation*> (*it);
                RegChi2 = (double)edge->chi2();
            }
        }
        std::sort(chi2List.begin(),chi2List.end());
        std::cout << "Squared Error:\n";
        std::cout << "Max: " << chi2List[chi2List.size()-1] << std::endl;
        std::cout << "Min: " << chi2List[0] << std::endl;
        std::cout << "Median: " << chi2List[chi2List.size()/2] << std::endl;
        std::cout << "Mean: " << chi2Mean/chi2List.size() << std::endl;
        if(regulation){
            std::cout << "Regulation Squared Error: " << RegChi2 << std::endl;
        }
    }
    return {RCL, tCL, scale};
}
Eigen::Vector4d InitialEXRotation::VI_data_sufficiency_assess(MatrixXd &Jacobian_rot, int &frame_num, Eigen::Vector3d &lidar_omg, int &orig_odom_freq,
                                      int &cut_frame_num) {
    Vector4d sufficiency;
    //Calculation of Rotation Jacobian
    // frame_num=frame_count;
    Eigen::Matrix3d lidar_omg_skew;
    // cout<<"w-----------"<<lidar_omg<<"f"<<frame_num<<endl;
    lidar_omg_skew << SKEW_SYM_MATRX(lidar_omg);
    Jacobian_rot.block<3, 3>(3 * frame_num, 0) = lidar_omg_skew;
    int data_sufficient = 0;
    int data_accum_length = 300;
    //Give a Data Appraisal every second
    if (frame_num %50!=0) {
         Eigen::Matrix3d Hessian_rot = Jacobian_rot.transpose() * Jacobian_rot;
        EigenSolver< Matrix3d> es(Hessian_rot);
         Eigen::Vector3d EigenValue = es.eigenvalues().real();
         Eigen::Matrix3d EigenVec_mat = es.eigenvectors().real();

          Eigen::Matrix3d EigenMatCwise = EigenVec_mat.cwiseProduct(EigenVec_mat);
        std::vector<double> EigenMat_1_col{EigenMatCwise(0, 0), EigenMatCwise(1, 0), EigenMatCwise(2, 0)};
        std::vector<double> EigenMat_2_col{EigenMatCwise(0, 1), EigenMatCwise(1, 1), EigenMatCwise(2, 1)};
        std::vector<double> EigenMat_3_col{EigenMatCwise(0, 2), EigenMatCwise(1, 2), EigenMatCwise(2, 2)};

        int maxPos[3] = {0};
        maxPos[0] = max_element(EigenMat_1_col.begin(), EigenMat_1_col.end()) - EigenMat_1_col.begin();
        maxPos[1] = max_element(EigenMat_2_col.begin(), EigenMat_2_col.end()) - EigenMat_2_col.begin();
        maxPos[2] = max_element(EigenMat_3_col.begin(), EigenMat_3_col.end()) - EigenMat_3_col.begin();
        EigenValue[0] = ( Hessian_rot(0,0));
        EigenValue[1] = (Hessian_rot(1,1));
        EigenValue[2] = (Hessian_rot(2,2));  //
        //保存Hessian矩阵到txt文件/home/wyw/Desktop/Hessian.txt
       std::ofstream outfile;
         outfile.open("/home/wyw/Desktop/Hessian.txt", std::ios::app);
         Eigen::Matrix3d Hessian_rot_t;
         Hessian_rot_t <<Hessian_rot(0,0), 0, 0,
                                       0, Hessian_rot(1,1), 0,
                                       0, 0, Hessian_rot(2,2);
         outfile << Hessian_rot_t.norm()/Hessian_rot.norm() << std::endl;
         outfile.close();
        //计算Hessian矩阵是否接近对角阵
        // cout<<Hessian_rot<<endl;
        Eigen::Vector3d Scaled_Eigen = EigenValue / data_accum_length;   //the larger data_accum_length is, the more data is needed
         Eigen::Vector3d Rot_percent(Scaled_Eigen[1] * Scaled_Eigen[2],
                        Scaled_Eigen[0] * Scaled_Eigen[2],
                        Scaled_Eigen[0] * Scaled_Eigen[1]);
        //  Rot_percent(abs(Scaled_Eigen[1]+ Scaled_Eigen[2]- Scaled_Eigen[0]),
        //                 abs(Scaled_Eigen[0] + Scaled_Eigen[2]- Scaled_Eigen[1]),
        //                 abs(Scaled_Eigen[0] + Scaled_Eigen[1]- Scaled_Eigen[2]));
         Eigen::Vector3d Rot_percent_scaled(Rot_percent[0] < 0.99 ? Rot_percent[0] : 1,
                               Rot_percent[1] < 0.99 ? Rot_percent[1] : 1,
                               Rot_percent[2] < 0.99 ? Rot_percent[2] : 1);

        int axis[3];
        axis[2] =2;// max_element(maxPos, maxPos + 3) - maxPos;
        axis[0] =0;// min_element(maxPos, maxPos + 3) - maxPos;
        axis[1] =1;// 3 - (axis[0] + axis[2]);


        // clear(); //clear the screen
        printf("\033[3A\r");
     

        printProgress(Rot_percent_scaled[axis[0]], 88);
        printProgress(Rot_percent_scaled[axis[1]], 89);
        printProgress(Rot_percent_scaled[axis[2]], 90);

        fflush(stdout);
        if (Rot_percent[0] > 0.99 && Rot_percent[1] > 0.99 && Rot_percent[2] > 0.99) {
            printf(BOLDCYAN "[Initialization] Data accumulation finished, Lidar IMU initialization begins.\n\n" RESET);
            printf(BOLDBLUE"============================================================ \n\n" RESET);
            data_sufficient = 1;
        }
       
        
        else if(Rot_percent[0] > 0.2&& Rot_percent[1] > 0.2 && Rot_percent[2] > 0.2)
        data_sufficient = 2;
       else
        data_sufficient = 3;   
        if(Rot_percent(0)>1)
           Rot_percent(0)=1;
        if(Rot_percent(1)>1)
           Rot_percent(1)=1;
        if(Rot_percent(2)>1)
           Rot_percent(2)=1; 
        
        sufficiency << Rot_percent[0], Rot_percent[1], Rot_percent[2], data_sufficient;
    }
    return  sufficiency;
    // if (data_sufficient)
    //     return true;
    // else
    //     return false;
}
template<typename T>
void mat2RPY(const Eigen::Matrix<T, 3, 3>& m, T& roll, T& pitch, T& yaw)
{
    roll = atan2(m(2,1), m(2,2));
    pitch = atan2(-m(2,0), sqrt(m(2,1) * m(2,1) + m(2,2) * m(2,2)));
    yaw = atan2(m(1,0), m(0,0));
}
 bool  SolveConstraintqyx(const Eigen::Vector4d t1, const Eigen::Vector4d t2, double& x1, double& x2)
  {
    double a = t1(0) * t1(1)+ t1(2)*t1(3);
    double b = t1(0) * t2(1)+ t1(1)*t2(0)+t1(2) * t2(3)+ t1(3)*t2(2);
    double c =  t2(0) * t2(1)+ t2(2)*t2(3);

    if ( std::fabs(a) < 1e-10)
    {
      x1 = x2 = -c/b;
      return true;
  }
  double delta2 = b*b - 4.0 * a * c;

  if(delta2 < 0.0) return false;

  double delta = sqrt(delta2);

  x1 = (-b + delta)/(2.0 * a);
  x2 = (-b - delta)/(2.0 * a);

  return true;

}



bool InitialEXRotation::CalibrationTime(const double frame_time, vector<pair<Vector3d, Vector3d>> &corres,vector<Eigen::Vector3d>& angular_velocity_buf, double &tdtime){
    frameT_count++;
    if(angular_velocity_buf.size()>30||angular_velocity_buf.size()<10) return false;
    Eigen::Vector3d Rc_ypr = Utility::R2ypr(solveRelativeR(corres));
    Rc_ypr(0) = 0;
    // Rc.push_back(solveRelativeR(corres));
    Eigen::Vector3d wc_t =Vector3d(Rc_ypr(2)/frame_time,Rc_ypr(1)/frame_time,Rc_ypr(0)/frame_time);
    Wc_t.push_back(wc_t);
    Frame_time.push_back(frame_time);
    Wimu_t_buf.push_back(angular_velocity_buf); // TODO: not fixed length   1   26  51 
    if(frameT_count>210){
        Eigen::Vector3d wc_t_mean;
        wc_t_mean = std::accumulate(Wc_t.begin()+10,Wc_t.end()-1,Vector3d(0,0,0));
        wc_t_mean = wc_t_mean/(Wc_t.size()-11);
        for(auto it = Wimu_t_buf.begin()+9;it!=Wimu_t_buf.end()-2;it++){
                int sz = it->size();
                auto itnext = it+1; 
                Wimu.push_back(((*itnext)[sz-1]+(*it)[sz-1])/2);
            }
        Eigen::Vector3d wimu_mean = std::accumulate(Wimu.begin(),Wimu.end(),Vector3d(0,0,0));
        wimu_mean /= Wimu.size();
        double frame_time_mean = std::accumulate(Frame_time.begin(),Frame_time.end(),0.0);
        frame_time_mean /= Frame_time.size();
        double trmax=0;
        for(int i=-25;i<25;++i){
            if(i<=0){
                for(auto it = Wimu_t_buf.begin()+9;it!=Wimu_t_buf.end()-2;it++){
                    int sz = it->size();
                    auto itnext = it+1; 
                    Wimu_t.push_back(((*itnext)[sz-1+i]+(*it)[sz-1+i])/2);
                }
            }
            else{
                for(auto it = Wimu_t_buf.begin()+10;it!=Wimu_t_buf.end()-1;it++){
                    int sz = it->size();
                    auto itnext = it+1; 
                    Wimu_t.push_back(((*itnext)[i]+(*it)[i])/2);
                }
            }
            Eigen::Vector3d wimu_t_mean=accumulate(Wimu_t.begin(),Wimu_t.end(),Vector3d(0,0,0));
            wimu_t_mean/=(Wimu_t.size());
            Eigen::Matrix3d sigmaci(Eigen::Matrix3d::Zero()),sigmacc(Eigen::Matrix3d::Zero()),
                        sigmaii(Eigen::Matrix3d::Zero()),sigmaic(Eigen::Matrix3d::Zero());
            for(int j=0;j<Wimu_t.size();++j){
                sigmacc += (Wc_t[j]-wc_t_mean)*(Wc_t[j]-wc_t_mean).transpose();
                sigmaci += (Wc_t[j]-wc_t_mean)*(Wimu_t[j]-wimu_t_mean).transpose();
                sigmaii += (Wimu[j]-wimu_mean)*(Wimu_t[j]-wimu_t_mean).transpose();
                sigmaic += (Wimu[j]-wimu_mean)*(Wc_t[j]-wc_t_mean).transpose();
            }
            Eigen::Matrix3d target = sigmacc.inverse()*sigmaci*sigmaic*sigmaii.inverse();
            if(target.trace()>trmax){
                tdtime = i*frame_time_mean/25;
                trmax = target.trace();
            }
        }
        return true;
    }
    return false;

}
 Eigen::Matrix3d  InitialEXRotation::CalibrationExRotation( Quaterniond delta_q_lidar, Quaterniond delta_q_camera )
{   
    std::string filename = "/home/wyw/ROS1_PROJECT/2023/camera_lidar_new/result/1.txt";
     //生成ROS时间戳
    std::string time = std::to_string(ros::Time::now().toSec());
    
    // 打开文件，使用追加模式，如果文件不存在则创建新文件
    std::ofstream outfile(filename, std::ios::app);

    if (!outfile) {
        std::cerr << "无法打开文件" << std::endl;
         
    }
    outfile <<Utility::R2ypr(ric).transpose()<<" "<<time<< std::endl;
    int frame_count_svd;
   Matrix4d e_L_R;
    frame_count++;
//     ...
// 相机帧之间匹配点得到本质矩阵，分解得到旋转矩阵R_ck+1^ck
// IMU之间预积分得到旋转矩阵R_bk+1^bk
// 每次迭代之前先用上一次估计的ric将R_bk+1^bk转换为R_ck+1^ck，为了下面求解核函数。

//     ......
    Rc.push_back( delta_q_camera.toRotationMatrix());           //Rcij
    Rimu.push_back(delta_q_lidar.toRotationMatrix()); //qij
    Rc_g.push_back(ric.inverse() * delta_q_lidar * ric);
   
   Eigen::MatrixXd A(frame_count * 4, 4);
    A.setZero();
    int sum_ok = 0;
    for (int i = 1; i <= frame_count; i++)
    {
        Quaterniond r1(Rc[i]);
        Quaterniond r2(Rc_g[i]);

        double angular_distance = 180 / M_PI * r1.angularDistance(r2);
        ROS_DEBUG(
            "%d %f", i, angular_distance);

        double huber = angular_distance > 5 ? 5 / angular_distance : 1.0;
        ++sum_ok;
        Matrix4d L, R;

        double w = Quaterniond(Rc[i]).w();
        Vector3d q = Quaterniond(Rc[i]).vec();
        L.block<3, 3>(0, 0) = w * Matrix3d::Identity() + Utility::skewSymmetric(q);
        L.block<3, 1>(0, 3) = q;
        L.block<1, 3>(3, 0) = -q.transpose();
        L(3, 3) = w;

        Quaterniond R_ij(Rimu[i]);
        w = R_ij.w();
        q = R_ij.vec();
        R.block<3, 3>(0, 0) = w * Matrix3d::Identity() - Utility::skewSymmetric(q);
        R.block<3, 1>(0, 3) = q;
        R.block<1, 3>(3, 0) = -q.transpose();
        R(3, 3) = w;

        A.block<4, 4>((i - 1) * 4, 0) = huber * (L - R);
    }

    JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
    Matrix<double, 4, 1> x = svd.matrixV().col(3);
    Quaterniond estimated_R(x);
    ric = estimated_R.toRotationMatrix().inverse();
    // cout << svd.singularValues().transpose() << endl;
    // cout <<Utility::R2ypr( ric.inverse()).transpose() << endl;
    Vector3d ric_cov;
    ric_cov = svd.singularValues().tail<3>();
   
   return ric;
    // if (frame_count >= WINDOW_SIZE && ric_cov(1) > 0.25)
    // {
         
    //     return true;
    // }
    // else 
    // return false;
    
}
bool InitialEXRotation::No_CalibrationExRotation(bool judge_motion,vector<pair<Vector3d, Vector3d> > corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result)
{   int frame_count_svd;
   Matrix4d e_L_R;
    frame_count++;
    Rc.push_back(solveRelativeR(corres));           //Rcij
    Rimu.push_back(delta_q_imu.toRotationMatrix()); //qij
    Rc_g.push_back(ric.inverse() * delta_q_imu * ric);
    if(frame_count>= WINDOW_SIZE+2)
    {
        return false;
    }
    //temp_ypr
    Vector3d temp_ypr;
    for (int i = 1; i <= frame_count; i++)
    {
        temp_ypr += Utility::R2ypr(Rimu[i]).cwiseAbs();
    }
    cout << "tem_pyr=" << temp_ypr.transpose() << endl;
    if (frame_count >= 30 && (temp_ypr[0] < 1.0 || temp_ypr[1] < 1.0 || temp_ypr[2] < 1.0))
    {
        ROS_WARN("Too long resting time, retry!!!");
        Reset();
        return false;
    }
    if (frame_count >= 50 && (temp_ypr[0] < 30&&temp_ypr[1] < 30 &&temp_ypr[2] < 30))
    {
        ROS_WARN("Too long resting time, retry!!!");
        Reset();
        return false;
    }
    if (frame_count < 5 || temp_ypr[0] < 1.0 || temp_ypr[1] < 1.0 || temp_ypr[2] < 1.0)
    {
        return false;
    }
    return true;
 
}

// Matrix3d InitialEXRotation::solveRelativeR(const vector<pair<Vector3d, Vector3d> > &corres)
// {
//     if (corres.size() >= 9)
//     {
//         vector<cv::Point2f> ll, rr;
//         for (int i = 0; i < int(corres.size()); i++)
//         {
//             ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
//             rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
//         }
//         cv::Mat E = cv::findFundamentalMat(ll, rr);
//         cv::Mat_<double> R1, R2, t1, t2;
//         decomposeE(E, R1, R2, t1, t2);

//         if (determinant(R1) + 1.0 < 1e-09)
//         {
//             E = -E;
//             decomposeE(E, R1, R2, t1, t2);
//         }
//         double ratio1 = max(testTriangulation(ll, rr, R1, t1), testTriangulation(ll, rr, R1, t2));
//         double ratio2 = max(testTriangulation(ll, rr, R2, t1), testTriangulation(ll, rr, R2, t2));
//         cv::Mat_<double> ans_R_cv = ratio1 > ratio2 ? R1 : R2;

//         Matrix3d ans_R_eigen;
//         for (int i = 0; i < 3; i++)
//             for (int j = 0; j < 3; j++)
//                 ans_R_eigen(j, i) = ans_R_cv(i, j);
//         return ans_R_eigen;
//     }
//     return Matrix3d::Identity();
// }

// bool InitialEXRotation::CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result)
// {
//     frame_count++;
//     Rc.push_back(solveRelativeR(corres));
//     Rimu.push_back(delta_q_imu.toRotationMatrix());
//     Rc_g.push_back(ric.inverse() * delta_q_imu * ric);

//     Eigen::MatrixXd A(frame_count * 4, 4);
//     A.setZero();
//     int sum_ok = 0;
//     for (int i = 1; i <= frame_count; i++)
//     {
//         Quaterniond r1(Rc[i]);
//         Quaterniond r2(Rc_g[i]);

//         double angular_distance = 180 / M_PI * r1.angularDistance(r2);
//         ROS_DEBUG(
//             "%d %f", i, angular_distance);

//         double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;
//         ++sum_ok;
//         Matrix4d L, R;

//         double w = Quaterniond(Rc[i]).w();
//         Vector3d q = Quaterniond(Rc[i]).vec();
//         L.block<3, 3>(0, 0) = w * Matrix3d::Identity() + Utility::skewSymmetric(q);
//         L.block<3, 1>(0, 3) = q;
//         L.block<1, 3>(3, 0) = -q.transpose();
//         L(3, 3) = w;

//         Quaterniond R_ij(Rimu[i]);
//         w = R_ij.w();
//         q = R_ij.vec();
//         R.block<3, 3>(0, 0) = w * Matrix3d::Identity() - Utility::skewSymmetric(q);
//         R.block<3, 1>(0, 3) = q;
//         R.block<1, 3>(3, 0) = -q.transpose();
//         R(3, 3) = w;

//         A.block<4, 4>((i - 1) * 4, 0) = huber * (L - R);
//     }

//     JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
//     Matrix<double, 4, 1> x = svd.matrixV().col(3);
//     Quaterniond estimated_R(x);
//     ric = estimated_R.toRotationMatrix().inverse();
//     //cout << svd.singularValues().transpose() << endl;
//     //cout << ric << endl;
//     Vector3d ric_cov;
//     ric_cov = svd.singularValues().tail<3>();
//     if (frame_count >= WINDOW_SIZE && ric_cov(1) > 0.25)
//     {
//         calib_ric_result = ric;
//         return true;
//     }
//     else
//         return false;
// }

Matrix3d InitialEXRotation::solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres)
{
    if (corres.size() >= 9)
    {
        vector<cv::Point2f> ll, rr;
        for (int i = 0; i < int(corres.size()); i++)
        {
            ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
            rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
        }
        cv::Mat E = cv::findFundamentalMat(ll, rr);
        cv::Mat_<double> R1, R2, t1, t2;
        decomposeE(E, R1, R2, t1, t2);

        if (determinant(R1) + 1.0 < 1e-09)
        {
            E = -E;
            decomposeE(E, R1, R2, t1, t2);
        }
        double ratio1 = max(testTriangulation(ll, rr, R1, t1), testTriangulation(ll, rr, R1, t2));
        double ratio2 = max(testTriangulation(ll, rr, R2, t1), testTriangulation(ll, rr, R2, t2));
        cv::Mat_<double> ans_R_cv = ratio1 > ratio2 ? R1 : R2;

        Matrix3d ans_R_eigen;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ans_R_eigen(j, i) = ans_R_cv(i, j);
        return ans_R_eigen;
    }
    return Matrix3d::Identity();
}

double InitialEXRotation::testTriangulation(const vector<cv::Point2f> &l,
                                          const vector<cv::Point2f> &r,
                                          cv::Mat_<double> R, cv::Mat_<double> t)
{
    cv::Mat pointcloud;
    cv::Matx34f P = cv::Matx34f(1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0);
    cv::Matx34f P1 = cv::Matx34f(R(0, 0), R(0, 1), R(0, 2), t(0),
                                 R(1, 0), R(1, 1), R(1, 2), t(1),
                                 R(2, 0), R(2, 1), R(2, 2), t(2));
    cv::triangulatePoints(P, P1, l, r, pointcloud);
    int front_count = 0;
    for (int i = 0; i < pointcloud.cols; i++)
    {
        double normal_factor = pointcloud.col(i).at<float>(3);

        cv::Mat_<double> p_3d_l = cv::Mat(P) * (pointcloud.col(i) / normal_factor);
        cv::Mat_<double> p_3d_r = cv::Mat(P1) * (pointcloud.col(i) / normal_factor);
        if (p_3d_l(2) > 0 && p_3d_r(2) > 0)
            front_count++;
    }
    ROS_DEBUG("MotionEstimator: %f", 1.0 * front_count / pointcloud.cols);
    return 1.0 * front_count / pointcloud.cols;
}

void InitialEXRotation::decomposeE(cv::Mat E,
                                 cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                                 cv::Mat_<double> &t1, cv::Mat_<double> &t2)
{
    cv::SVD svd(E, cv::SVD::MODIFY_A);
    cv::Matx33d W(0, -1, 0,
                  1, 0, 0,
                  0, 0, 1);
    cv::Matx33d Wt(0, 1, 0,
                   -1, 0, 0,
                   0, 0, 1);
    R1 = svd.u * cv::Mat(W) * svd.vt;
    R2 = svd.u * cv::Mat(Wt) * svd.vt;
    t1 = svd.u.col(2);
    t2 = -svd.u.col(2);
}

string camera_odom;
string lidar_odom;
Eigen::Matrix3d odom_rotation_matrix_old;
Eigen::Matrix3d pose_rotation_matrix_old;
int init_rotaion=0;
double s=1.0;
std::vector<Eigen::Isometry3d> vmTwc_sum, vmTwl_sum;
InitialEXRotation calib_LC;

void pose2Motion(std::vector<Eigen::Isometry3d> &vAbsPose, std::vector<Eigen::Isometry3d> &vRelPose){
    for(std::size_t i = 0; i < vAbsPose.size()-1; ++i){
        Eigen::Isometry3d mTwc(vAbsPose[i+1] * vAbsPose[i].inverse());  // T(i+1) * T(i).inverse()
        vRelPose.push_back(mTwc);
    }
}

void pose2Motion_refine( Eigen::Matrix3d RLC,std::vector<Eigen::Isometry3d> &vAbsPose, std::vector<Eigen::Isometry3d> &vRelPose,std::vector<Eigen::Isometry3d> &vAbsPose2, std::vector<Eigen::Isometry3d> &vRelPose2){
   

        

       
   
    for(std::size_t i = 0; i < vAbsPose.size()-1; ++i){
      Quaterniond r1(RLC*(vAbsPose[i+1].rotation() * vAbsPose[i].rotation().inverse())*RLC.inverse());
      Quaterniond r2(vAbsPose2[i+1].rotation() * vAbsPose2[i].rotation().inverse());
     double angular_distance = 180 / M_PI * r1.angularDistance(r2);
    //    cout<<"angular_distance:"<<angular_distance<<endl;
   
        Eigen::Isometry3d mTwc(vAbsPose[i+1] * vAbsPose[i].inverse());  // T(i+1) * T(i).inverse()
        Eigen::Isometry3d mTwl(vAbsPose2[i+1] * vAbsPose2[i].inverse());  // T(i+1) * T(i).inverse()

         if (angular_distance<5)
      {    vRelPose.push_back(mTwc);
           vRelPose2.push_back(mTwl);}

    }
}
void callback( const nav_msgs::OdometryConstPtr &odom_msg,
       const geometry_msgs::PoseStampedConstPtr &pose_msg) {
  
    // Process the messages from the three topics
 
   std::string package_path = ros::package::getPath("lcc_motion");
    std::string folder_path = package_path + "/results";
cout<<folder_path<<endl;
  std::string file_path = folder_path + "/calibration_result.txt";
std::ofstream calib_file(file_path, std::ios::app);
    // Extract position and orientation from the Pose message
    //将odom_msg转化为Eigen::Isometry3d vmTwl
    Eigen::Vector3d odom_position(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
    Eigen::Quaterniond odom_orientation(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    Eigen::Isometry3d vmTwl = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d vmTwc = Eigen::Isometry3d::Identity();

    vmTwl.rotate(odom_orientation);
    vmTwl.pretranslate(odom_position);
    vmTwl = vmTwl.inverse();
 
   
    //请将pose_msg转化为Eigen::Isometry3d vmTwc

    Eigen::Vector3d pose_position(pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z);
    Eigen::Quaterniond pose_orientation(pose_msg->pose.orientation.w, pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);
    vmTwc.rotate(pose_orientation);
    vmTwc.pretranslate(pose_position);
    vmTwc = vmTwc.inverse();
    vmTwl_sum.push_back(vmTwl);
    vmTwc_sum.push_back(vmTwc);
//////////////////solver
    std::vector<Eigen::Isometry3d> vmTwc2, vmTwl2;
    // vmTwc2.reserve(vmTwc_sum.size()-1);
    // vmTwl2.reserve(vmTwl_sum.size()-1);
 if(init_rotaion<300)
  {  pose2Motion(vmTwc_sum, vmTwc2);
    pose2Motion(vmTwl_sum, vmTwl2);
}
    Eigen::Matrix3d RCL;
   
    Eigen::Vector3d tCL;
     Eigen::Matrix3d pose_rotation_matrix = pose_orientation.toRotationMatrix();
    Eigen::Matrix3d odom_rotation_matrix = odom_orientation.toRotationMatrix();
   
    //保存旋转矩阵到txt文件/home/wyw/Desktop/rotation.txt
   init_rotaion=init_rotaion+1;
 
     if(init_rotaion>=1)
    {cout<<"calib_start---------------"<<endl;
          
       Eigen::Matrix3d RL_k1_k= odom_rotation_matrix.inverse()*odom_rotation_matrix_old;
        Eigen::Quaterniond q_RL_k1_k(RL_k1_k);
        Eigen::Matrix3d RC_k1_k= pose_rotation_matrix.inverse()*pose_rotation_matrix_old;
        Eigen::Quaterniond q_RC_k1_k(RC_k1_k); 
       RCL=calib_LC.CalibrationExRotation(q_RL_k1_k,q_RC_k1_k).inverse();
       std::cout << "LIDAR_CAMERA Calibration:\n";
    std::cout << "Rotation:\n " <<RCL << std::endl;
      
        if (calib_file.is_open()) {
            calib_file << "LIDAR to CAMERA Extrinsic Calibration Results (Camera as reference frame):\n";
            calib_file << " Hand-eye Calibration (only rotation based VINS-Mono)\n"<< "\n";

            calib_file << "Rotation Matrix (LiDAR to Camera):\n" << RCL << "\n";
            calib_file << "Translation Vector (LiDAR to Camera):\n" << tCL.transpose() << "\n";
            calib_file << "Scale:\n" << s << "\n";
            
            std::cout << "Calibration result saved to " << file_path << std::endl;
        } else {
            std::cerr << "Error opening file for writing calibration result\n";
        }
   
    //    cout<<RCL<<endl;
    }
    
    odom_rotation_matrix_old=odom_rotation_matrix;
    pose_rotation_matrix_old=pose_rotation_matrix;
  bool regulation=true;
  double robust_kernel_size=0.3;
  double regulation_weight=1.0E-3;
  bool verborse=false;
  if(init_rotaion>=200)
    {
  pose2Motion_refine(RCL,vmTwc_sum, vmTwc2,vmTwl_sum, vmTwl2);
  std::tie(RCL,tCL,s) = HECalib(vmTwc2, vmTwl2);
    std::cout << "Ordinary Hand-eye Calibration:\n";
    std::cout << "Rotation:\n " <<RCL << std::endl;
    std::cout << "Translation:  " << tCL << std::endl;
    std::cout << "s :" << s << std::endl;
    if (calib_file.is_open()) {
            calib_file << "LIDAR to CAMERA Extrinsic Calibration Results (Camera as reference frame):\n";
            calib_file << " Ordinary Hand-eye Calibration\n"<< "\n";

            calib_file << "Rotation Matrix (LiDAR to Camera):\n" << RCL << "\n";
            calib_file << "Translation Vector (LiDAR to Camera):\n" << tCL.transpose() << "\n";
            calib_file << "Scale:\n" << s << "\n";
            
            std::cout << "Calibration result saved to " << file_path << std::endl;
         
        } else {
            std::cerr << "Error opening file for writing calibration result\n";
        }
        tCL.setZero(); // Translation is too bad
     std::tie(RCL,tCL,s) = HECalibRobustKernelg2o(vmTwc2, vmTwl2, RCL, tCL, s, robust_kernel_size, regulation, regulation_weight, verborse);
    std::cout << "Robust Kernel Hand-eye Calibration with Regulation:\n";
     if (calib_file.is_open()) {
            calib_file << "LIDAR to CAMERA Extrinsic Calibration Results (Camera as reference frame):\n";
            calib_file << " Robust Kernel Hand-eye Calibration with Regulation\n" <<  "\n";

            calib_file << "Rotation Matrix (LiDAR to Camera):\n" << RCL << "\n";
            calib_file << "Translation Vector (LiDAR to Camera):\n" << tCL.transpose() << "\n";
            calib_file << "Scale:\n" << s << "\n";
            
            std::cout << "Calibration result saved to " << file_path << std::endl;
                calib_file.close();
        } else {
            std::cerr << "Error opening file for writing calibration result\n";
        }
 std::cout << "Rotation: \n" << RCL << std::endl;
     std::cout << "Translation: " << tCL << std::endl;
    std::cout << "scale :" << s << std::endl; 
    int inner_iter=10;
  int ex_iter= 20;
 int  init_mu= 64;
 double divide_factor=1.4;
  double min_mu=0.01;
    //  std::tie(RCL,tCL,s) = HECalibLineProcessg2o(vmTwc2, vmTwl2, RCL, tCL, s, inner_iter, init_mu, divide_factor, min_mu, ex_iter, regulation, regulation_weight, verborse);
    // std::cout << "Line Process Hand-eye Calibration:\n";
    // std::cout << "Rotation: \n" << RCL << std::endl;
    // std::cout << "Translation: \n" << tCL << std::endl;
    // std::cout << "s :" << s << std::endl;
    }
    

 
  


    // Print or use the extracted information
    // ROS_INFO("Received Pose message: x=%f, y=%f, z=%f", pose_position(0), pose_position(1), pose_position(2));
    // ROS_INFO("Received Odometry message: x=%f, y=%f, z=%f", odom_position(0), odom_position(1), odom_position(2));

    // Convert quaternions to rotation matrices
   

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "LCcalib");
    ros::NodeHandle nh;
    // ////////////////////////////////////////////////////////////
    lidar_odom="/aft_mapped_to_init";
    camera_odom="/orb_slam3/camera_pose";
    std::string lidar_odom = "/aft_mapped_to_init";
    std::string camera_odom = "/orb_slam3/camera_pose";

    // 从ROS参数服务器获取参数，如果有传入的参数则使用传入值，否则使用默认值
    nh.param<std::string>("lidar_odom", lidar_odom, lidar_odom);
    nh.param<std::string>("camera_odom", camera_odom, camera_odom);
     message_filters::Subscriber<nav_msgs::Odometry> lidar_odom_sub(nh, lidar_odom, 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> camera_odom_sub(nh, camera_odom, 1);
    
    // 定义时间同步策略
  
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::PoseStamped > SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(100), lidar_odom_sub, camera_odom_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
 
//    std::cout << "Result of Hand-eye Calibration saved to " << resFileName << std::endl;
 
    ros::spin();
    // cout << "lidar_type: " << lidar_type << endl;
    cout << "LiDAR-only odometry starts." << endl;
 
    return 0;
}
