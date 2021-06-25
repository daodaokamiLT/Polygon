#include "core/polygon.h"
#include "core/polygon_intersecting.h"
#include "test/random_point_creator.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
int main(int argc, char* argv[]){
    std::vector<polygon::point2d_t<double>> points, points1;
    polygon::CreateRandomPoints2d(20, std::pair<double, double>(0, 0), std::pair<double, double>(50, 50), points);    
    polygon::CreateRandomPoints2d(30, std::pair<double, double>(0, 0), std::pair<double, double>(50, 50), points1);    

    cv::Mat img(600, 600, CV_8UC1);
    cv::Mat img1(600, 600, CV_8UC1);

    for(auto& p : points){
        cv::circle(img, cv::Point2d(10*p.x+50, 600-(10*p.y+50)), 5, 255);
    }
    cv::imshow("img", img);
    cv::waitKey(30);
    polygon::Polygon<double> polygon(points);
    polygon::Polygon<double> polygon1(points1);

    polygon.SorttoStarPolygon();
    const polygon::point2d_t<double>* pminx = polygon.GetMinX_y();
    const polygon::point2d_t<double>* pmaxx = polygon.GetMaxX_y();
    const polygon::point2d_t<double>* pminy = polygon.GetMinY_x();
    const polygon::point2d_t<double>* pmaxy = polygon.GetMaxY_x();

    polygon::point2d_t<double> pminx_cpy(pminx->x, pminx->y+2);
    polygon::point2d_t<double> pmaxx_cpy(pmaxx->x, pmaxx->y+2);
    polygon::point2d_t<double> pminy_cpy(pminy->x+2, pminy->y);
    polygon::point2d_t<double> pmaxy_cpy(pmaxy->x+2, pmaxy->y);

    cv::circle(img, cv::Point2d(pminx_cpy.x*10+50, 600-(pminx_cpy.y*10+50)), 3, 255);
    cv::circle(img, cv::Point2d(pmaxx_cpy.x*10+50, 600-(pmaxx_cpy.y*10+50)), 3, 255);
    cv::circle(img, cv::Point2d(pminy_cpy.x*10+50, 600-(pminy_cpy.y*10+50)), 3, 255);
    cv::circle(img, cv::Point2d(pmaxy_cpy.x*10+50, 600-(pmaxy_cpy.y*10+50)), 3, 255);

    polygon.AddNodeForXminsameX(&pminx_cpy);
    polygon.AddNodeForXmaxsameX(&pmaxx_cpy);
    polygon.AddNodeForYminsameY(&pminy_cpy);
    polygon.AddNodeForYmaxsameY(&pmaxy_cpy);

    polygon.SorttoStarPolygon();


    polygon1.SorttoStarPolygon();

    std::vector<polygon::point2d_t<double>*> resort_points;
    std::vector<polygon::point2d_t<double>*> resort_points1;

    polygon.GetTempPoints(resort_points);
    polygon1.GetTempPoints(resort_points1);

    polygon::point2dd_t* p0 = resort_points[0];
    polygon::point2dd_t* p1 = resort_points1[0];

    cv::Mat cimg = img.clone();
    for(auto p : resort_points){
        cv::line(img, cv::Point2d((10*p0->x+50), 600-(10*p0->y+50)), cv::Point2d((10*p->x+50), 600-(10*p->y+50)), 255);
        cv::imshow("img", img);
        cv::waitKey(30);
    }

    polygon.CreateExtremeEdges();
    polygon1.CreateExtremeEdges();
    std::vector<polygon::extreme_edge_t<double>> exedges;
    std::vector<polygon::extreme_edge_t<double>> exedges1;
    polygon.GetExtremeEdges(exedges);
    polygon1.GetExtremeEdges(exedges1);

    for(int i=0; i<exedges.size(); ++i){
        cv::line(cimg, cv::Point2d(exedges[i].p_start->x*10+50, 600-(exedges[i].p_start->y*10+50)), cv::Point2d(exedges[i].p_end->x*10+50, 600-(exedges[i].p_end->y*10+50)), 255, 2);
        cv::imshow("cimg", cimg);
        cv::waitKey(30);
    }
    for(int i=0; i<exedges1.size(); ++i){
        cv::line(img1, cv::Point2d(exedges1[i].p_start->x*10+50, 600-(exedges1[i].p_start->y*10+50)), cv::Point2d(exedges1[i].p_end->x*10+50, 600-(exedges1[i].p_end->y*10+50)), 255, 2);
        cv::imshow("cimg1", img1);
        cv::waitKey(30);
    }
    // cv::waitKey(0);

    polygon::PolygonIntersecting<double> polygonIntersecting(&polygon, &polygon1);
    polygonIntersecting.SplitPolygon2MonotoneChain();


    std::vector<polygon::point2d_t<double>*> leftfirstchain, rightfirstchain;
    polygonIntersecting.GetFirstMonoChain(leftfirstchain, rightfirstchain);
    std::vector<polygon::point2d_t<double>*> leftsecondchain, rightsecondchain;
    polygonIntersecting.GetSecondMonoChain(leftsecondchain, rightsecondchain);

    
    cv::Mat img2 = img.clone();
    // std::cout<<"leftchain: "<<leftfirstchain.size()<<std::endl;
    for(int i=0; i<leftfirstchain.size()-1; ++i){
        cv::line(img2, cv::Point2d(leftfirstchain[i]->x*10+50, 600-(leftfirstchain[i]->y*10+50)), cv::Point2d(leftfirstchain[i+1]->x*10+50, 600-(leftfirstchain[i+1]->y*10+50)), 255, 3);            
    }   
    // cv::imshow("firstleftchain", img2);

    // cv::Mat img3 = img.clone();   
    for(int i=0; i<rightfirstchain.size()-1; ++i){
        cv::line(img2, cv::Point2d(rightfirstchain[i]->x*10+50, 600-(rightfirstchain[i]->y*10+50)), cv::Point2d(rightfirstchain[i+1]->x*10+50, 600-(rightfirstchain[i+1]->y*10+50)), 100, 2);    
    }   
    cv::imshow("firstleft+rightchain", img2);

    // cv::Mat img4 = img.clone();   
    // for(int i=0; i<leftsecondchain.size()-1; ++i){
    //     cv::line(img4, cv::Point2d(leftsecondchain[i]->x*10+50, 600-(leftsecondchain[i]->y*10+50)), cv::Point2d(leftsecondchain[i+1]->x*10+50, 600-(leftsecondchain[i+1]->y*10+50)), 255, 2);    
    // }   
    // cv::imshow("secondleftchain", img4);

    // cv::Mat img5 = img.clone();   
    // for(int i=0; i<rightsecondchain.size()-1; ++i){
    //     cv::line(img5, cv::Point2d(rightsecondchain[i]->x*10+50, 600-(rightsecondchain[i]->y*10+50)), cv::Point2d(rightsecondchain[i+1]->x*10+50, 600-(rightsecondchain[i+1]->y*10+50)), 255, 2);    
    // }   
    // cv::imshow("secondrightchain", img5);
    cv::waitKey(0);
    cv::destroyAllWindows();
    points.clear();
    return 0;
}