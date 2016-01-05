//
// Created by sean on 16/12/15.
//

#include "util/SquareSorter.hpp"
#include "util/CameraId.hpp"

#include <opencv2/opencv.hpp>

using namespace cv;

bool SquareSorter::areSquaresValid(const vector<vector<Point>> &square_contours) {
    return square_contours.size() == NUM_SQUARES;
}

bool SquareSorter::sortSquares(vector<vector<Point>> &square_contours, string camera_id) {
    if(!areSquaresValid(square_contours))
        return false;

    vector<Square> squares;
    for(const auto& contour : square_contours) {
        Moments mu = moments(contour, true);
        Point2d center = Point2d( mu.m10/mu.m00 , mu.m01/mu.m00 );
        Square s;
        s.points = contour;
        s.centroid = center;
        squares.push_back(s);
    }

    if (camera_id.compare(CameraId::left) == 0)
        SquareSorter::sortSquaresLeft(squares);
    else if (camera_id.compare(CameraId::center) == 0)
        SquareSorter::sortSquaresCenter(squares);
    else if (camera_id.compare(CameraId::right) == 0)
        SquareSorter::sortSquaresRight(squares);

    for(size_t i = 0; i < squares.size(); ++i)
        square_contours.at(i) = squares.at(i).points;

    return true;
}

void SquareSorter::sortSquaresLeft(vector<Square> &squares) {
    sort(squares.begin(), squares.end(), compare_horizontal());
    sort(squares.begin(), squares.begin() + (squares.size()/2), compare_vertical_inverse());
    sort(squares.begin() + (squares.size()/2), squares.end(), compare_vertical_inverse());
}

void SquareSorter::sortSquaresCenter(vector<Square> &squares) {
    sort(squares.begin(), squares.end(), compare_vertical_inverse());
    sort(squares.begin(), squares.begin() + (squares.size()/2), compare_horizontal_inverse());
    sort(squares.begin() + (squares.size()/2), squares.end(), compare_horizontal_inverse());
}

void SquareSorter::sortSquaresRight(vector<Square> &squares) {
    sort(squares.begin(), squares.end(), compare_horizontal_inverse());
    sort(squares.begin(), squares.begin() + (squares.size()/2), compare_vertical());
    sort(squares.begin() + (squares.size()/2), squares.end(), compare_vertical());
}

