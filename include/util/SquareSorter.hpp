//
// Created by sean on 16/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_SQUARESORTER_H
#define PCL_CLOUD_REGISTRATION_SQUARESORTER_H

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class SquareSorter {

    friend class SquareDetector;

    static constexpr NUM_SQUARES = 4;

private:
    struct Square {
        vector<Point> points;
        Point2d centroid;
    };

    static bool sortSquares(vector<vector<Point>> &squares, std::string camera_id);
    static bool sortSquaresLeft(vector<vector<Point>> &squares);
    static bool sortSquaresCenter(vector<vector<Point>> &squares);
    static bool sortSquaresRight(vector<vector<Point>> &squares);

    struct compare_vertical {
        inline bool operator()(const Square& square1, const Square& square2) {
            return square1.centroid.x < square2.centroid.x;
        }
    };

    struct compare_vertical_inverse {
        inline bool operator()(const Square& square1, const Square& square2) {
            return square1.centroid.x > square2.centroid.x;
        }
    };

    struct compare_horizontal {
        inline bool operator()(const Square& square1, const Square& square2) {
            return square1.centroid.x < square2.centroid.x;
        }
    };

    struct compare_horizontal_inverse {
        inline bool operator()(const Square& square1, const Square& square2) {
            return square1.centroid.x > square2.centroid.x;
        }
    };
};


#endif //PCL_CLOUD_REGISTRATION_SQUARESORTER_H
