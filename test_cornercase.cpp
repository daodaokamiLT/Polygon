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

// create p x, y (42.764500, 18.490700).
// create p x, y (50.530900, 7.875600).
// create p x, y (22.352400, 41.136600).
// create p x, y (13.167300, 37.623100).
// create p x, y (43.504000, 10.521500).
// create p x, y (36.193800, 15.290100).
// create p x, y (22.051800, 7.044800).
// create p x, y (15.399400, 26.405400).
// create p x, y (18.531300, 7.584800).
// create p x, y (40.460400, 23.788800).
// create p x, y (14.925400, 21.108400).
// create p x, y (7.731500, 19.940200).
// create p x, y (29.097000, 37.750700).
// create p x, y (2.040900, 37.290400).
// create p x, y (18.970400, 38.824200).
// create p x, y (39.864800, 45.004300).
// create p x, y (26.589200, 18.425800).
// create p x, y (15.214700, 5.641500).
// create p x, y (27.385100, 33.720400).
// create p x, y (1.660700, 45.565000).
// create p x, y (42.764500, 18.490700).
// create p x, y (50.530900, 7.875600).
// create p x, y (22.352400, 41.136600).
// create p x, y (13.167300, 37.623100).
// create p x, y (43.504000, 10.521500).
// create p x, y (36.193800, 15.290100).
// create p x, y (22.051800, 7.044800).
// create p x, y (15.399400, 26.405400).
// create p x, y (18.531300, 7.584800).
// create p x, y (40.460400, 23.788800).
// create p x, y (14.925400, 21.108400).
// create p x, y (7.731500, 19.940200).
// create p x, y (29.097000, 37.750700).
// create p x, y (2.040900, 37.290400).
// create p x, y (18.970400, 38.824200).
// create p x, y (39.864800, 45.004300).
// create p x, y (26.589200, 18.425800).
// create p x, y (15.214700, 5.641500).
// create p x, y (27.385100, 33.720400).
// create p x, y (1.660700, 45.565000).
// create p x, y (31.315800, 31.244300).
// create p x, y (42.534700, 1.985600).
// create p x, y (13.445100, 50.561300).
// create p x, y (18.565700, 30.949700).
// create p x, y (22.375600, 6.070200).
// create p x, y (36.346900, 9.878200).
// create p x, y (42.598700, 21.504200).
// create p x, y (40.069200, 2.377000).
// create p x, y (7.621400, 40.606400).
// create p x, y (41.227200, 7.367200).