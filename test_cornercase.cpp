#include <iostream>
#include "core/polygon.h"
#include "core/polygon_intersecting.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
int main(int argc, char* argv[]){
    std::vector<polygon::point2dd_t> points;
    points.emplace_back(polygon::point2dd_t(10, 10));
    points.emplace_back(polygon::point2dd_t(40, 45));
    points.emplace_back(polygon::point2dd_t(40, 47));
    points.emplace_back(polygon::point2dd_t(42, 45));
    
    polygon::Polygon<double> mpolygon(points);

    cv::Mat img(600, 600, CV_8UC1);

    for(auto& p : points){
        cv::circle(img, cv::Point2d(10*p.x+50, 600-(10*p.y+50)), 5, 255);
    }
    cv::imshow("img", img);
    cv::waitKey(30);

    mpolygon.SorttoStarPolygon();


    std::vector<polygon::point2dd_t*> resort_points;

    mpolygon.GetTempPoints(resort_points);
    polygon::point2dd_t* p0 = resort_points[0];

    for(auto p : resort_points){
        cv::line(img, cv::Point2d((10*p0->x+50), 600-(10*p0->y+50)), cv::Point2d((10*p->x+50), 600-(10*p->y+50)), 255);
        cv::imshow("img", img);
        cv::waitKey(30);
    }


    mpolygon.CreateExtremeEdges();
    std::vector<polygon::extreme_edge_t<double>> exedges;
    mpolygon.GetExtremeEdges(exedges);



    for(int i=0; i<exedges.size(); ++i){
        cv::line(img, cv::Point2d(10*exedges[i].p_start->x+50, 600 - (10*exedges[i].p_start->y+50)), cv::Point2d(10*exedges[i].p_end->x+50, 600-(10*exedges[i].p_end->y+50)), 255, 1);
    }
    cv::imshow("img", img);
    cv::waitKey(0);



    return 0;
}