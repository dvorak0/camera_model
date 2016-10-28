#ifndef CHESSBOARDQUAD_H
#define CHESSBOARDQUAD_H

#include <boost/shared_ptr.hpp>

#include "camodocal/chessboard/ChessboardCorner.h"

namespace camodocal
{

class ChessboardQuad;
typedef boost::shared_ptr<ChessboardQuad> ChessboardQuadPtr;

class ChessboardQuad
{
  public:
    ChessboardQuad() : count(0), group_idx(-1), edge_len(FLT_MAX), labeled(false) {}

    int count;                      // Number of quad neighbors
    int group_idx;                  // Quad group ID
    float edge_len;                 // Smallest side length^2
    ChessboardCornerPtr corners[4]; // Coordinates of quad corners
    ChessboardQuadPtr neighbors[4]; // Pointers of quad neighbors
    bool labeled;                   // Has this corner been labeled?
    cv::Point2f center()
    {
        float sx = 0.0, sy = 0.0;
        for (int i = 0; i < 4; i++)
        {
            sx += corners[i]->pt.x;
            sy += corners[i]->pt.y;
        }
        return cv::Point2f{sx / 4, sy / 4};
    }
};
}

#endif
