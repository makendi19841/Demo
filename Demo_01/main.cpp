//============================================================================
// Name        : main.cpp
// Author      : Amos Makendi
// Version     : 1.0
// Copyright   : -
// Description : only calls processing and test routines
//============================================================================

#include "Demo.h"
#include "Helper.h"

#include <cstdlib>
#include<iostream>
#include <opencv2/core/matx.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgcodecs.hpp>
#include<string>
#include<opencv2/opencv.hpp>


void run(const std::string &fnameBase, const std::string &fnameLeft){

    
    // load image first two images
    cv::Mat initialImage = cv::imread(fnameBase, cv::IMREAD_GRAYSCALE);
    cv::Mat initialImage1 (initialImage.rows, initialImage.cols, CV_32F);
    initialImage.convertTo(initialImage1, CV_32F, 1/255.0, 0.0);


    cv::Mat currentImage = cv::imread(fnameLeft, cv::IMREAD_GRAYSCALE);
    cv::Mat currentImage1 (currentImage.rows, currentImage.cols, CV_32F);
    currentImage.convertTo(currentImage1, CV_32F, 1/255.0, 0.0);

    // check content of initialImage
    if (!initialImage.data){
        std::cerr << "ERROR: Cannot read image ( " <<fnameBase << std::endl;  // standard error stream
        std::cin.get(); // accessing character array
        std::exit(-1);
    }

    // check content of currentImage
    if (!currentImage.data) {
        std::cerr << "ERROR: Canot read image ( "<<fnameLeft << std::endl;
        std::cin.get();
        std::exit(-1);

    }

    // get pair of corresponding points
    std::vector<cv::Vec3f> p_initial, p_current;
    int numberOfPointsPairs = demo::getPoints(initialImage, currentImage, p_initial, p_current);

    //
    std::cout<< "Number of defined point pairs: " << numberOfPointsPairs << std::endl;
    std::cout <<std::endl <<"Points in base image:" <<std::endl;

    for (const auto &p : p_initial) {
        std::cout << p << std::endl;
    }

    std::cout << std::endl <<" Points in second image:"<<std::endl;
    for (const auto &p: p_current){
        std::cout << p << std::endl;
    }

    // calculate homography
    cv::Matx33f H = demo::homography2D(p_initial, p_current);


}

int main(){


    std::string fnameBase = "/home/amos/Opencv_projects/Demo/Initial_Image.bmp";
    std::string fnameLeft = "/home/amos/Opencv_projects/Demo/current_Image.bmp";

    // start processing
    run (fnameBase, fnameLeft);

    std::cout <<"Press enter to continue..."<<std::endl;
    std::cin.get();

    return 0;
}