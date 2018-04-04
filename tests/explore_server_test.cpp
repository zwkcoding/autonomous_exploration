#include <iostream>
#include <opencv2/opencv.hpp>
#include <gtest/gtest.h>
#include <chrono>
using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(ExploreServerTest, counterUnkonwCells) {

    //    vector<vector<unsigned char>> matrix(5, vector<unsigned char>(3));

//    int CC, RR;
//    cin>>CC; cin>>RR;
//    vector<vector<signed char> > matrix;
//    for(int i = 0; i<RR; i++)
//    {
//        vector<signed char> myvector;
//        for(int j = 0; j<CC; j++)
//        {
//            int tempVal = 0;
//            cout<<"Enter the number for Matrix 1";
//            cin>>tempVal;
//            myvector.push_back(tempVal);
//        }
//        matrix.push_back(myvector);
//    }
    vector<signed char> vect{ -1, -1, 0, 100, 100,
                              0, -1, -1, 0, 0,
                              0, 0, -1, -1, 0,
                              0, 100, 100, -1, -1,
                              -1, -1, -1, -1, -1};
    cv::Mat m = cv::Mat(5, 5,
                        CV_8SC1); // initialize matrix of signed char of 1-channel where you will store vec data
    cv::Mat m0 = cv::Mat(5, 5,
                         CV_8UC1);
    //copy vector to mat
    memcpy(m.data, vect.data(), vect.size() * sizeof(signed char));
    m.setTo(cv::Scalar(120), m == -1);
    m.convertTo(m0, CV_8UC1);

    cv::Rect rect1;
    int num_cells_radius = 10;
    // x,y is top left corner point
    rect1.x = 0 ;
    rect1.y = 0;
    rect1.height = rect1.width = 2 * num_cells_radius;
    rect1 = rect1 & cv::Rect(0, 0, m0.cols, m0.rows);  // inside mat
    cv::Mat roiMat = m0(rect1).clone();
//        std::cerr << roiMat.type() << " " << roiMat.channels() << " " << roiMat.size() << std::endl;

    cv::threshold( roiMat, roiMat, 110, 255, cv::THRESH_BINARY);
    int count_white = cv::countNonZero(roiMat); // white cell is unknown cell
    std::cout << count_white << '\n';

    EXPECT_EQ(13, count_white);

}