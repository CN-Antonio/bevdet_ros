#ifndef __TEST_H__
#define __TEST_H__

#include "bevdet_node.h"

int cvToArr(cv::Mat img, std::vector<char> &raw_data);
int cvImgToArr(std::vector<cv::Mat> &imgs, std::vector<std::vector<char>> &imgs_data);

#endif