//
// Created by sean on 16/12/15.
//

#include "util/SquareSorter.hpp"
#include "util/CameraId.hpp"

#include <opencv2/opencv.hpp>

using namespace cv;

bool SquareSorter::sortSquares(vector<vector<Point>> &squares, string camera_id) {
    bool valid = false;
    if (camera_id.compare(CameraId::left) == 0)
        valid = SquareSorter::sortSquaresLeft(squares);
    else if (camera_id.compare(CameraId::center) == 0)
        valid = SquareSorter::sortSquaresCenter(squares);
    else if (camera_id.compare(CameraId::right) == 0)
        valid = SquareSorter::sortSquaresRight(squares);

    return valid;
}

bool SquareSorter::sortSquaresLeft(vector<vector<Point>> &squares) {

}

bool SquareSorter::sortSquaresCenter(vector<vector<Point>> &squares) {

}

bool SquareSorter::sortSquaresRight(vector<vector<Point>> &squares) {

}

