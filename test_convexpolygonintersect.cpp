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
        // cv::imshow("img", img);
        // cv::waitKey(30);
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
    // draw the split result.
    std::vector<polygon::point2d_t<double>*> firstleftchain_xbase, firstrightchain_xbase;
    polygonIntersecting.GetFirstMonoChainXbase(firstleftchain_xbase, firstrightchain_xbase);
    std::vector<polygon::point2d_t<double>*> firstleftchain_ybase, firstrightchain_ybase;
    polygonIntersecting.GetFirstMonoChainYbase(firstleftchain_ybase, firstrightchain_ybase);

    std::vector<polygon::point2d_t<double>*> secondleftchain_xbase, secondrightchain_xbase;
    polygonIntersecting.GetSecondMonoChainXbase(secondleftchain_xbase, secondrightchain_xbase);
    std::vector<polygon::point2d_t<double>*> secondleftchain_ybase, secondrightchain_ybase;
    polygonIntersecting.GetSecondMonoChainYbase(secondleftchain_ybase, secondrightchain_ybase);

    cv::Mat img_xbase = img.clone();
    printf("firstleft chain xbase size is %d.\n", (int)firstleftchain_xbase.size());
    for(int i=0; i<firstleftchain_xbase.size()-1; ++i){
        printf("point is: %lf %lf -- %lf %lf.\n", firstleftchain_xbase[i]->x, firstleftchain_xbase[i]->y, firstleftchain_xbase[i+1]->x, firstleftchain_xbase[i+1]->y);
        cv::line(img_xbase, cv::Point2d(50+10*firstleftchain_xbase[i]->x, 550-10*firstleftchain_xbase[i]->y), cv::Point2d(50+10*firstleftchain_xbase[i+1]->x, 550-10*firstleftchain_xbase[i+1]->y), 255, 3);
    }
    printf("firstright chain xbase size is %d.\n", (int)firstrightchain_xbase.size());
    for(int i=0; i<firstrightchain_xbase.size()-1; ++i){
        printf("point is: %lf %lf -- %lf %lf.\n", firstrightchain_xbase[i]->x, firstrightchain_xbase[i]->y, firstrightchain_xbase[i+1]->x, firstrightchain_xbase[i+1]->y);
        cv::line(img_xbase, cv::Point2d(50+10*firstrightchain_xbase[i]->x, 550-10*firstrightchain_xbase[i]->y), cv::Point2d(50+10*firstrightchain_xbase[i+1]->x, 550-10*firstrightchain_xbase[i+1]->y), 100, 2);
    }
    for(int i=0; i<secondleftchain_xbase.size()-1; ++i){
        cv::line(img_xbase, cv::Point2d(50+10*secondleftchain_xbase[i]->x, 550-10*secondleftchain_xbase[i]->y), cv::Point2d(50+10*secondleftchain_xbase[i+1]->x, 550-10*secondleftchain_xbase[i+1]->y), 255, 3);
    }
    for(int i=0; i<secondrightchain_xbase.size()-1; ++i){
        cv::line(img_xbase, cv::Point2d(50+10*secondrightchain_xbase[i]->x, 550-10*secondrightchain_xbase[i]->y), cv::Point2d(50+10*secondrightchain_xbase[i+1]->x, 550-10*secondrightchain_xbase[i+1]->y), 100, 2);
    }
    cv::imshow("xbase split", img_xbase);

    cv::Mat img_ybase = img.clone();
    printf("firstleft chain ybase size is %d.\n", (int)firstleftchain_ybase.size());
    for(int i=0; i<firstleftchain_ybase.size()-1; ++i){
        printf("point is: %lf %lf -- %lf %lf.\n", firstleftchain_ybase[i]->x, firstleftchain_ybase[i]->y, firstleftchain_ybase[i+1]->x, firstleftchain_ybase[i+1]->y);
        cv::line(img_ybase, cv::Point2d(50+10*firstleftchain_ybase[i]->x, 550-10*firstleftchain_ybase[i]->y), cv::Point2d(50+10*firstleftchain_ybase[i+1]->x, 550-10*firstleftchain_ybase[i+1]->y), 255, 3);
    }
    printf("firstright chain ybase size is %d.\n", (int)firstrightchain_ybase.size());
    for(int i=0; i<firstrightchain_ybase.size()-1; ++i){
        printf("point is: %lf %lf -- %lf %lf.\n", firstrightchain_ybase[i]->x, firstrightchain_ybase[i]->y, firstrightchain_ybase[i+1]->x, firstrightchain_ybase[i+1]->y);
        cv::line(img_ybase, cv::Point2d(50+10*firstrightchain_ybase[i]->x, 550-10*firstrightchain_ybase[i]->y), cv::Point2d(50+10*firstrightchain_ybase[i+1]->x, 550-10*firstrightchain_ybase[i+1]->y), 100, 2);
    }
    // printf("secondleft chain ybase size is %d.\n", (int)secondleftchain_ybase.size());
    for(int i=0; i<secondleftchain_ybase.size()-1; ++i){
        // printf("point is: %lf %lf -- %lf %lf.\n", secondleftchain_ybase[i]->x, secondleftchain_ybase[i]->y, secondleftchain_ybase[i+1]->x, secondleftchain_ybase[i+1]->y);
        cv::line(img_ybase, cv::Point2d(50+10*secondleftchain_ybase[i]->x, 550-10*secondleftchain_ybase[i]->y), cv::Point2d(50+10*secondleftchain_ybase[i+1]->x, 550-10*secondleftchain_ybase[i+1]->y), 255, 3);
    }
    // printf("secondright chain ybase size is %d.\n", (int)secondrightchain_ybase.size());
    for(int i=0; i<secondrightchain_ybase.size()-1; ++i){
        // printf("point is: %lf %lf -- %lf %lf.\n", secondrightchain_ybase[i]->x, secondrightchain_ybase[i]->y, secondrightchain_ybase[i+1]->x, secondrightchain_ybase[i+1]->y);
        cv::line(img_ybase, cv::Point2d(50+10*secondrightchain_ybase[i]->x, 550-10*secondrightchain_ybase[i]->y), cv::Point2d(50+10*secondrightchain_ybase[i+1]->x, 550-10*secondrightchain_ybase[i+1]->y), 100, 2);
    }
    // cv::imshow("xbase split", img_xbase);
    cv::imshow("ybase split", img_ybase);
    cv::waitKey(0);
    bool hasintersect = polygonIntersecting.HasIntersection();
    if(hasintersect){
        printf("has intersections.\n");
        polygonIntersecting.CalIntersectionBetweenTwoConvexPolygon();
    }
    else{
        printf("hasn't polygon intersected.\n");
    }
    std::vector<polygon::point2d_t<double>*> interpoints;
    polygonIntersecting.GetIntersections(interpoints);
    printf("intersection size is %d.\n", (int)interpoints.size());
    for(auto ip : interpoints){
        printf("x y is %lf, %lf.\n", ip->x, ip->y);
        cv::circle(img_xbase, cv::Point2d(50+10*ip->x, 550-10*ip->y), 5, cv::Scalar(255), 5, 2);
    }
    cv::destroyAllWindows();
    cv::imshow("xbase split", img_xbase);
    cv::waitKey(0);
    // polygonIntersecting
    
    created_points.clear();
    copyshift_points.clear();
    return 0;
}