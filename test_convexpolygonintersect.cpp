#include <iostream>
#include <vector>
#include "test/random_point_creator.h"
#include "core/polygon.h"
#include "core/polygon_intersecting.h"

int main(int argc, char* argv[]){
    // create two polygon that has interface.
    std::vector<polygon::point2d_t<double>> created_points;
    std::pair<double, double> range_min(0, 0);
    std::pair<double, double> range_max(50, 50); 
    polygon::CreateRandomPoints2d(30, range_min, range_max, created_points);
    std::vector<polygon::point2d_t<double>> copyshift_points;

    for(int i=0; i<created_points.size(); ++i){
        copyshift_points.push_back(polygon::point2d_t<double>(created_points[i].x+30, created_points[i].y));
    }
    cv::Mat img(600, 1200, CV_8UC1, cv::Scalar(0));

    for(int i=0; i<created_points.size(); ++i){
        cv::circle(img, cv::Point2d(10*created_points[i].x+50, 550-10*created_points[i].y), 3, 255, 1);
        cv::circle(img, cv::Point2d(10*copyshift_points[i].x+50, 550-10*copyshift_points[i].y), 3, 255, 1);
    }

    
    polygon::Polygon<double> polygon(created_points);
    polygon.SorttoStarPolygon();
    std::vector<polygon::point2d_t<double>*> resort_points;
    polygon.GetTempPoints(resort_points);
    polygon::point2dd_t* p0 = resort_points[0];

    for(auto p : resort_points){
        cv::line(img, cv::Point2d((10*p0->x+50), 550-(10*p0->y)), cv::Point2d((10*p->x+50), 550-10*p->y), 255);
        cv::imshow("img", img);
        cv::waitKey(30);
    }

    polygon::Polygon<double> polygon1(copyshift_points);
    polygon1.SorttoStarPolygon();
    std::vector<polygon::point2d_t<double>*> resort_points1;
    polygon1.GetTempPoints(resort_points1);
    polygon::point2dd_t* p1 = resort_points1[0];

    for(auto p : resort_points1){
        cv::line(img, cv::Point2d((10*p1->x+50), 550-(10*p1->y)), cv::Point2d((10*p->x+50), 550-10*p->y), 255);
        cv::imshow("img", img);
        cv::waitKey(30);
    }


    polygon.CreateExtremeEdges();
    std::vector<polygon::extreme_edge_t<double>> exedges;
    polygon.GetExtremeEdges(exedges);
    for(int i=0; i<exedges.size(); ++i){
        cv::line(img, cv::Point2d(50+10*exedges[i].p_start->x, 550-10*exedges[i].p_start->y), cv::Point2d(50+10*exedges[i].p_end->x, 550-10*exedges[i].p_end->y), 255, 1);
    }

    polygon1.CreateExtremeEdges();
    std::vector<polygon::extreme_edge_t<double>> exedges1;
    polygon1.GetExtremeEdges(exedges1);
    for(int i=0; i<exedges1.size(); ++i){
        cv::line(img, cv::Point2d(50+10*exedges1[i].p_start->x, 550-10*exedges1[i].p_start->y), cv::Point2d(50+10*exedges1[i].p_end->x, 550-10*exedges1[i].p_end->y), 255, 1);
    }
    cv::imshow("img", img);
    cv::waitKey(0);

    polygon::PolygonIntersecting<double> polygonIntersecting(&polygon, &polygon1);
    polygonIntersecting.SplitPolygon2MonotoneChain();
    

    // polygonIntersecting

    created_points.clear();
    copyshift_points.clear();
    return 0;
}