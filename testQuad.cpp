#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

int main(int argc, char* argv[]){
    Eigen::Quaterniond qvec(0, 1, 0, 0);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R(1,1) = -1;
    std::cout<<"R: \n"<<R<<std::endl;
    std::cout<<"R_inv: \n"<<R.inverse()<<std::endl;
    qvec = R* qvec * R.inverse();
    std::cout<<qvec.w()<<qvec.x()<<qvec.y()<<qvec.z()<<std::endl;


    Eigen::Matrix3d Det;
    Det(0,0) = 26.297000; Det(0,1) = 2.425800; Det(0,2) = 1;
    Det(1,0) = 10.377800; Det(1,1) = 19.145900; Det(1,2) = 1;
    Det(2,0) = 4.578900; Det(2,1) = 29.229900; Det(2,2) = 1;
    std::cout<<Det.determinant()<<std::endl;
    return 0;
}