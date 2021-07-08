// test_intersection.cpp
#include <iostream>
#include "./core/polygon.h"
#include "./core/polygon_intersecting.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace polygon;
template <class T>
bool CalRealIntersectionPosition1(const extreme_edge_t<T> &edge0, const extreme_edge_t<T> &edge1, point2d_t<T> *inter)
{
    const point2d_t<T> *pa = edge0.p_start;
    const point2d_t<T> *pb = edge0.p_end;
    const point2d_t<T> *pc = edge1.p_start;
    const point2d_t<T> *pd = edge1.p_end;

    // |x_ab, -x_cd||k|    |x_ac|
    // |y_ab, -y_cd||l| =  |y_ac|
    Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
    A(0, 0) = pb->x - pa->x;
    A(0, 1) = -1 * (pd->x - pc->x);
    A(1, 0) = pb->y - pa->y;
    A(1, 1) = -1 * (pd->y - pc->y);
    std::cout<<A<<std::endl;
    // if (A.determinant() <= 0)
    // {
    //     // 就没有可用的结果。
    //     printf("A determinant is <= zero %lf.\n", A.determinant());
    //     return false;
    // }
    
    
    Eigen::Vector2d b(pc->x - pa->x, pc->y - pa->y);
    std::cout<<b<<std::endl;
    Eigen::Vector2d x = A.fullPivLu().solve(b);

    std::cout<<"x is: "<<x.transpose()<<std::endl;
    if (x[0] >= -1e-9  && x[0] - 1 <= 1e-9 && x[1] >= -1e-9 && x[1] -1 <= 1e-9)
    {
        printf("has intersection.\n");
        inter->x = pa->x + x[0] * (pb->x - pa->x);
        inter->y = pa->y + x[0] * (pb->y - pa->y);
        return true;
    }
    else
    {
        printf("no intersection.\n");
        return false;
    }
}

int main(int argc, char* argv[]){

    point2d_t<double> pa(44.458300, 40.640500);
    point2d_t<double> pb(16.644600, 46.646000);

    point2d_t<double> pc(41.099100, 45.097600);
    point2d_t<double> pd(35.426600, 34.938700);

    cv::Mat img(cv::Size(640, 640), CV_8UC1, cv::Scalar(0));

    point2d_t<double> inter(0,0);
    cv::line(img, cv::Point2d(10*pa.x, 550-10*pa.y), cv::Point2d(10*pb.x, 550-10*pb.y), 255, 2);
    cv::line(img, cv::Point2d(10*pc.x, 550-10*pc.y), cv::Point2d(10*pd.x, 550-10*pd.y), 100, 1);
    extreme_edge_t<double> e0(&pa, &pb);
    extreme_edge_t<double> e1(&pc, &pd);
    bool hasinter = CalRealIntersectionPosition1<double>(e0, e1, &inter);
    
    if(hasinter){
        printf("has inter");
        cv::circle(img, cv::Point2d(10*inter.x, 550-10*inter.y), 3, 255, 1);
    }
    else{
        printf("hasn't inter.\n");
    }
    cv::imshow("img", img);
    cv::waitKey(0);
    return 0;
}