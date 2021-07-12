#include <iostream>
#include <vector>
#include "test/random_point_creator.h"
#include "core/polygon.h"
#include "core/polygon_intersecting.h"
#include <chrono>


template <class T>
void DrawPolygon(polygon::Polygon<T>& polygon, cv::Mat& img);


int main(int argc, char* argv[]){
    // create two polygon that has interface.
    for(int testcounter =0; testcounter < 100000; ++testcounter){
        PolygonPrintf("==========================================================\n");
        std::vector<polygon::point2d_t<double>> created_points;
        std::pair<double, double> range_min(0, 0);
        std::pair<double, double> range_max(50, 50);
        polygon::CreateRandomPoints2d(4, range_min, range_max, created_points);
        std::vector<polygon::point2d_t<double>> copyshift_points;
        double xmin = created_points[0].x;
        double xmax = created_points[0].x;
        for(int i=1; i<created_points.size(); ++i){
            if(xmin > created_points[i].x)
                xmin = created_points[i].x;
            if(xmax < created_points[i].x)
                xmax = created_points[i].x;
        }
        double delta = (xmax - xmin)/2;
        for (int i = 0; i < created_points.size(); ++i)
        {
            copyshift_points.push_back(polygon::point2d_t<double>(created_points[i].x + delta, created_points[i].y));
        }
        cv::Mat img(600, 1200, CV_8UC1, cv::Scalar(0));

        #if debug
        for (int i = 0; i < created_points.size(); ++i)
        {
            cv::circle(img, cv::Point2d(10 * created_points[i].x + 50, 550 - 10 * created_points[i].y), 3, 255, 1);
            cv::circle(img, cv::Point2d(10 * copyshift_points[i].x + 50, 550 - 10 * copyshift_points[i].y), 3, 255, 1);
        }
        #endif 

        auto start_createpolygon = std::chrono::steady_clock::now();
        polygon::Polygon<double> polygon(created_points);
        polygon.SorttoStarPolygon();
        std::vector<polygon::point2d_t<double> *> resort_points;
        polygon.GetTempPoints(resort_points);
        auto end_createpolygon = std::chrono::steady_clock::now();

        polygon::point2dd_t *p0 = resort_points[0];
        
        #if debug
        for (auto p : resort_points)
        {
            cv::line(img, cv::Point2d((10 * p0->x + 50), 550 - (10 * p0->y)), cv::Point2d((10 * p->x + 50), 550 - 10 * p->y), 255);
            // cv::imshow("img", img);
            // cv::waitKey(30);
        }
        #endif 
        auto start_sort = std::chrono::steady_clock::now();
        polygon::Polygon<double> polygon1(copyshift_points);
        polygon1.SorttoStarPolygon();
        std::vector<polygon::point2d_t<double> *> resort_points1;
        polygon1.GetTempPoints(resort_points1);
        auto end_sort = std::chrono::steady_clock::now();
        polygon::point2dd_t *p1 = resort_points1[0];

        #if debug
        for (auto p : resort_points1)
        {
            cv::line(img, cv::Point2d((10 * p1->x + 50), 550 - (10 * p1->y)), cv::Point2d((10 * p->x + 50), 550 - 10 * p->y), 255);
            cv::imshow("img", img);
            // cv::waitKey(30);
        }
        #endif

        auto start_createexedge = std::chrono::steady_clock::now();
        polygon.CreateExtremeEdges();
        std::vector<polygon::extreme_edge_t<double>> exedges;
        polygon.GetExtremeEdges(exedges);        
        polygon1.CreateExtremeEdges();
        std::vector<polygon::extreme_edge_t<double>> exedges1;
        polygon1.GetExtremeEdges(exedges1);
        auto end_createexedge = std::chrono::steady_clock::now();
        #if debug
        for (int i = 0; i < exedges.size(); ++i)
        {
            cv::line(img, cv::Point2d(50 + 10 * exedges[i].p_start->x, 550 - 10 * exedges[i].p_start->y), cv::Point2d(50 + 10 * exedges[i].p_end->x, 550 - 10 * exedges[i].p_end->y), 255, 1);
        }

        for (int i = 0; i < exedges1.size(); ++i)
        {
            cv::line(img, cv::Point2d(50 + 10 * exedges1[i].p_start->x, 550 - 10 * exedges1[i].p_start->y), cv::Point2d(50 + 10 * exedges1[i].p_end->x, 550 - 10 * exedges1[i].p_end->y), 255, 1);
        }
        cv::imshow("img", img);
        // cv::waitKey(0);
        #endif

        auto start_calintersec = std::chrono::steady_clock::now();
        polygon::PolygonIntersecting<double> polygonIntersecting(&polygon, &polygon1);
        polygonIntersecting.SplitPolygon2MonotoneChain();
        // draw the split result.
        std::vector<polygon::point2d_t<double> *> firstleftchain_xbase, firstrightchain_xbase;
        polygonIntersecting.GetFirstMonoChainXbase(firstleftchain_xbase, firstrightchain_xbase);
        if(firstleftchain_xbase.empty() || firstrightchain_xbase.empty()){
            printf("firstleft or right split error.\n");
            cv::Mat mpolygon(cv::Size(640, 1200), CV_8UC1, cv::Scalar(0));
            DrawPolygon<double>(polygon, mpolygon);
            cv::imshow("segmentfault img0", mpolygon);
            cv::waitKey(0);
            exit(-1);
        }
        std::vector<polygon::point2d_t<double> *> firstleftchain_ybase, firstrightchain_ybase;
        polygonIntersecting.GetFirstMonoChainYbase(firstleftchain_ybase, firstrightchain_ybase);
        if(firstleftchain_ybase.empty() || firstrightchain_ybase.empty()){
            printf("firstleft or right split error.\n");
            cv::Mat mpolygon(cv::Size(640, 1200), CV_8UC1, cv::Scalar(0));
            DrawPolygon<double>(polygon, mpolygon);
            cv::imshow("segmentfault img0", mpolygon);
            exit(-1);
        }
        std::vector<polygon::point2d_t<double> *> secondleftchain_xbase, secondrightchain_xbase;
        polygonIntersecting.GetSecondMonoChainXbase(secondleftchain_xbase, secondrightchain_xbase);
        if(secondleftchain_xbase.empty() || secondrightchain_xbase.empty()){
            printf("secondleft or right split error.\n");
            cv::Mat mpolygon(cv::Size(640, 1200), CV_8UC1, cv::Scalar(0));
            DrawPolygon<double>(polygon, mpolygon);
            cv::imshow("segmentfault img0", mpolygon);
            exit(-1);
        }
        std::vector<polygon::point2d_t<double> *> secondleftchain_ybase, secondrightchain_ybase;
        polygonIntersecting.GetSecondMonoChainYbase(secondleftchain_ybase, secondrightchain_ybase);
        if(secondleftchain_ybase.empty() || secondrightchain_ybase.empty()){
            printf("secondleft or right split error.\n");
            cv::Mat mpolygon(cv::Size(640, 1200), CV_8UC1, cv::Scalar(0));
            DrawPolygon<double>(polygon, mpolygon);
            cv::imshow("segmentfault img0", mpolygon);
            exit(-1);
        }
        bool hasintersect = polygonIntersecting.HasIntersection();
        if (hasintersect)
        {
            PolygonPrintf("has intersections.\n");
            polygonIntersecting.CalIntersectionBetweenTwoConvexPolygon();
        }
        else
        {
            PolygonPrintf("hasn't polygon intersected.\n");
        }
        std::vector<polygon::point2d_t<double> *> interpoints;
        polygonIntersecting.GetIntersections(interpoints);
        std::vector<polygon::intersec_extremeedge_t<double>> interexedgepairs;
        polygonIntersecting.GetIntersectionEdges(interexedgepairs);
        auto end_calintersec = std::chrono::steady_clock::now();

        // std::cout<<std::chrono::duration<double,std::micro>(end_createpolygon-start_createpolygon+ end_sort - start_sort + end_createexedge - start_createexedge + end_calintersec - start_calintersec).count()<<std::endl;
        PolygonPrintf("intersection size is %d.\n", (int)interpoints.size());

        cv::Mat img_xbase = img.clone();
        PolygonPrintf("firstleft chain xbase size is %d.\n", (int)firstleftchain_xbase.size());
        for (int i = 0; i < firstleftchain_xbase.size() - 1; ++i)
        {
            PolygonPrintf("point is: %lf %lf -- %lf %lf.\n", firstleftchain_xbase[i]->x, firstleftchain_xbase[i]->y, firstleftchain_xbase[i + 1]->x, firstleftchain_xbase[i + 1]->y);
            cv::line(img_xbase, cv::Point2d(50 + 10 * firstleftchain_xbase[i]->x, 550 - 10 * firstleftchain_xbase[i]->y), cv::Point2d(50 + 10 * firstleftchain_xbase[i + 1]->x, 550 - 10 * firstleftchain_xbase[i + 1]->y), 255, 3);
        }
        PolygonPrintf("firstright chain xbase size is %d.\n", (int)firstrightchain_xbase.size());
        for (int i = 0; i < firstrightchain_xbase.size() - 1; ++i)
        {
            PolygonPrintf("point is: %lf %lf -- %lf %lf.\n", firstrightchain_xbase[i]->x, firstrightchain_xbase[i]->y, firstrightchain_xbase[i + 1]->x, firstrightchain_xbase[i + 1]->y);
            cv::line(img_xbase, cv::Point2d(50 + 10 * firstrightchain_xbase[i]->x, 550 - 10 * firstrightchain_xbase[i]->y), cv::Point2d(50 + 10 * firstrightchain_xbase[i + 1]->x, 550 - 10 * firstrightchain_xbase[i + 1]->y), 100, 2);
        }
        for (int i = 0; i < secondleftchain_xbase.size() - 1; ++i)
        {
            cv::line(img_xbase, cv::Point2d(50 + 10 * secondleftchain_xbase[i]->x, 550 - 10 * secondleftchain_xbase[i]->y), cv::Point2d(50 + 10 * secondleftchain_xbase[i + 1]->x, 550 - 10 * secondleftchain_xbase[i + 1]->y), 255, 3);
        }
        for (int i = 0; i < secondrightchain_xbase.size() - 1; ++i)
        {
            cv::line(img_xbase, cv::Point2d(50 + 10 * secondrightchain_xbase[i]->x, 550 - 10 * secondrightchain_xbase[i]->y), cv::Point2d(50 + 10 * secondrightchain_xbase[i + 1]->x, 550 - 10 * secondrightchain_xbase[i + 1]->y), 100, 2);
        }

        cv::circle(img_xbase, cv::Point2d(50+10*p0->x, 550-10*p0->y), 6, 255, 1);
        cv::circle(img_xbase, cv::Point2d(50+10*p1->x, 550-10*p1->y), 6, 255, 1);
        cv::imshow("xbase split", img_xbase);
        #if debug
        cv::Mat img_ybase = img.clone();
        PolygonPrintf("firstleft chain ybase size is %d.\n", (int)firstleftchain_ybase.size());
        for (int i = 0; i < firstleftchain_ybase.size() - 1; ++i)
        {
            PolygonPrintf("point is: %lf %lf -- %lf %lf.\n", firstleftchain_ybase[i]->x, firstleftchain_ybase[i]->y, firstleftchain_ybase[i + 1]->x, firstleftchain_ybase[i + 1]->y);
            cv::line(img_ybase, cv::Point2d(50 + 10 * firstleftchain_ybase[i]->x, 550 - 10 * firstleftchain_ybase[i]->y), cv::Point2d(50 + 10 * firstleftchain_ybase[i + 1]->x, 550 - 10 * firstleftchain_ybase[i + 1]->y), 255, 3);
        }
        PolygonPrintf("firstright chain ybase size is %d.\n", (int)firstrightchain_ybase.size());
        for (int i = 0; i < firstrightchain_ybase.size() - 1; ++i)
        {
            PolygonPrintf("point is: %lf %lf -- %lf %lf.\n", firstrightchain_ybase[i]->x, firstrightchain_ybase[i]->y, firstrightchain_ybase[i + 1]->x, firstrightchain_ybase[i + 1]->y);
            cv::line(img_ybase, cv::Point2d(50 + 10 * firstrightchain_ybase[i]->x, 550 - 10 * firstrightchain_ybase[i]->y), cv::Point2d(50 + 10 * firstrightchain_ybase[i + 1]->x, 550 - 10 * firstrightchain_ybase[i + 1]->y), 100, 2);
        }
        // PolygonPrintf("secondleft chain ybase size is %d.\n", (int)secondleftchain_ybase.size());
        for (int i = 0; i < secondleftchain_ybase.size() - 1; ++i)
        {
            // PolygonPrintf("point is: %lf %lf -- %lf %lf.\n", secondleftchain_ybase[i]->x, secondleftchain_ybase[i]->y, secondleftchain_ybase[i+1]->x, secondleftchain_ybase[i+1]->y);
            cv::line(img_ybase, cv::Point2d(50 + 10 * secondleftchain_ybase[i]->x, 550 - 10 * secondleftchain_ybase[i]->y), cv::Point2d(50 + 10 * secondleftchain_ybase[i + 1]->x, 550 - 10 * secondleftchain_ybase[i + 1]->y), 255, 3);
        }
        // PolygonPrintf("secondright chain ybase size is %d.\n", (int)secondrightchain_ybase.size());
        for (int i = 0; i < secondrightchain_ybase.size() - 1; ++i)
        {
            // PolygonPrintf("point is: %lf %lf -- %lf %lf.\n", secondrightchain_ybase[i]->x, secondrightchain_ybase[i]->y, secondrightchain_ybase[i+1]->x, secondrightchain_ybase[i+1]->y);
            cv::line(img_ybase, cv::Point2d(50 + 10 * secondrightchain_ybase[i]->x, 550 - 10 * secondrightchain_ybase[i]->y), cv::Point2d(50 + 10 * secondrightchain_ybase[i + 1]->x, 550 - 10 * secondrightchain_ybase[i + 1]->y), 100, 2);
        }
        // cv::imshow("xbase split", img_xbase);
        // cv::imshow("ybase split", img_ybase);
        // cv::waitKey(0);
        #endif 

        for (auto ip : interpoints)
        {
            PolygonPrintf("x y is %lf, %lf.\n", ip->x, ip->y);
            cv::circle(img_xbase, cv::Point2d(50 + 10 * ip->x, 550 - 10 * ip->y), 5, cv::Scalar(150), 5, 2);
        }
        
        // cv::destroyAllWindows();
        // 将相交的部分重建成polygon
        cv::imshow("xbase split", img_xbase);
        cv::waitKey(1);
        if(interpoints.size() != 2){
            PolygonPrintf("interpoints.size() %d != 2.\n", (int)interpoints.size());
            for(int i=0; i<created_points.size(); ++i){
                PolygonPrintf("created point %lf %lf\n", created_points[i].x, created_points[i].y);
            }
            for(int i=0; i<copyshift_points.size(); ++i){
                PolygonPrintf("copyshift point %lf %lf\n", copyshift_points[i].x, copyshift_points[i].y);
            }
            cv::waitKey(30);
        }
        // polygonIntersecting
        for(int i=0; i<interexedgepairs.size(); ++i){
            printf("%d edge is %lf %lf -- %lf %lf.\n", i, interexedgepairs[i].first_edge.p_start->x, interexedgepairs[i].first_edge.p_start->y, interexedgepairs[i].first_edge.p_end->x, interexedgepairs[i].first_edge.p_end->y);
            printf("%d edge is %lf %lf -- %lf %lf.\n", i, interexedgepairs[i].second_edge.p_start->x, interexedgepairs[i].second_edge.p_start->y, interexedgepairs[i].second_edge.p_end->x, interexedgepairs[i].second_edge.p_end->y);
            
            cv::line(img_xbase, cv::Point2d(50+10*interexedgepairs[i].first_edge.p_start->x, 550-10*interexedgepairs[i].first_edge.p_start->y), cv::Point2d(50+10*interexedgepairs[i].first_edge.p_end->x, 550-10*interexedgepairs[i].first_edge.p_end->y), 50, 5, 3);
            
            cv::line(img_xbase, cv::Point2d(50+10*interexedgepairs[i].second_edge.p_start->x, 550-10*interexedgepairs[i].second_edge.p_start->y), cv::Point2d(50+10*interexedgepairs[i].second_edge.p_end->x, 550-10*interexedgepairs[i].second_edge.p_end->y), 50, 5, 3);
        }

        created_points.clear();
        copyshift_points.clear();
        cv::imshow("img_xbase", img_xbase);
        cv::waitKey(30);
        printf("test counter is %d.\n", testcounter);
    }
    return 0;
}

template <class T>
void DrawPolygon(polygon::Polygon<T>& polygon, cv::Mat& img){
    std::vector<polygon::extreme_edge_t<double>> exedges;
    polygon.GetExtremeEdges(exedges);
    for (int i = 0; i < exedges.size(); ++i)
    {
        cv::line(img, cv::Point2d(50 + 10 * exedges[i].p_start->x, 550 - 10 * exedges[i].p_start->y), cv::Point2d(50 + 10 * exedges[i].p_end->x, 550 - 10 * exedges[i].p_end->y), 255, 1);
        printf("%lf %lf - %lf %lf\n", exedges[i].p_start->x, exedges[i].p_start->y, exedges[i].p_end->x, exedges[i].p_end->y);
    }
}