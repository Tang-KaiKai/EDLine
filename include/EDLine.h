#pragma once

#ifndef EDLINE_H
#define EDLINE_H

#include <string.h>

#include <vector>
#include <array>

#include "Gradient.h"
#include <assert.h>

namespace Feature
{

typedef unsigned char uchar;
typedef std::array<double, 8> Line; //sx,sy,ex,ey,a,b,c,o

struct Parameters
{
    int gradient_threshold;
    int anchor_threshold;

    int scan_intervals;
    int min_length;

    float line_fit_err_threshold;
};

struct Point2
{
    short x;
    short y;

    Point2(): x( 0 ), y( 0 ) {}
    Point2( short _x, short _y ): x( _x ), y( _y ) {}
};

struct PixelChains
{
    std::vector<Point2> vPoints;
    std::vector<int> vStartIds;

    int ChainsNum = 0;
};


class EDLine
{
public:

    EDLine() = delete;

    EDLine( int width, int height );

    EDLine( const Parameters &param, int width, int height );

    ~EDLine();

public:

    bool Detect( const uchar *pImg );

    bool DetectEdges( const uchar *pImg );

    bool DetectLines();

    void GetEdges( PixelChains &edgeChains );

    void GetLines( PixelChains &lineChains );

public:

    short *GetPtrGrad() const
    {
        return pOperator_->GetPtrGrad();
    }

    void ShowAnchor( bool store = false, int showTime = 0 );

private:

    void Initialize();

    void ProcessImage( const uchar *pImg );

    void DetectAnchors();

    bool DetectEdge( int anchorIndex );

    void LinesFit( int edgeIndex );

    double LeastSquareFit( const bool bHorizontal, const Point2 *pPoints,
                           int offsetS, std::array<double, 2> &slopeIntercept );

    void LeastSquareFit( const bool bHorizontal, const Point2 *pPoints,
                         int newOffsetS, int offsetE, std::array<double, 2> &slopeIntercept );

    bool StoreLine( const bool bHorizontal, const std::array<double, 2> &slopeIntercept );

private:

    int thGradient_ = 15;

    int thAnchor_ = 3;

    int ScanIntervals_ = 2;

    int thEdgeLength_ = 25; //25

public:

    const int width_ = 1280;
    const int height_ = 720;

    GradientOperator *pOperator_ = nullptr;

    std::vector<Line> vLines_;

private:

    uchar *pImgEdge_ = nullptr;

    Point2 *pAnchorPoints_ = nullptr;

    int nExpectEdgePixelSize_ = 0;
    int nExpectAnchorSize_ = 0;
    int nEdgeMaxNum_ = 0;
    int AnchorSize_ = 0;

    Point2 *pPartEdgePoints_ = nullptr;
    int nExpectPartSizeEdge_ = 0;
    int IndexEdgeStart_ = 0, IndexEdgeEnd_ = 0;

    Point2 *pEdgePoints_ = nullptr;
    int *pEdgeS_ = nullptr;
    int OffsetEdge_ = 0, CountEdge_ = 0;

private:

    int InitLineLength_ = 20;
    int thLineLength_ = 40;

    float thLineFitErr_ = 1.414;

    int SkipEdgePoint_ = 2;

    int TryTime_ = 6;

    int MaxOutlierNum_ = 5;

private:

    Point2 *pPartLinePoints_ = nullptr;
    int nExpectPartSizeLine_ = 0;
    int IndexLineStart_ = 0, IndexLineEnd_ = 0;

    Point2 *pLinePoints_ = nullptr;
    int *pLineS_ = nullptr;
    int OffsetLine_ = 0, CountLine_ = 0;

private:
    /// (f1,s1), (f2,s2) ... (fn,sn) --->> s = a*f + b
    struct LineFit
    {
        long sum = 0;
        long sum_f = 0;
        long sum_s = 0;
        long sum_fs = 0;
        long sum_ff = 0;

        LineFit() : sum( 0 ), sum_f( 0 ), sum_s( 0 ), sum_fs( 0 ), sum_ff( 0 )  {}
        ~LineFit() {}

        void Reset()
        {
            sum = 0;
            sum_f = 0;
            sum_s = 0;
            sum_fs = 0;
            sum_ff = 0;
        }

        void AddPoint( short first, short second )
        {
            ++sum;

            sum_f += static_cast<long>( first );
            sum_s += static_cast<long>( second );

            sum_fs += static_cast<long>( first * second );
            sum_ff += static_cast<long>( first * first );
        }

        void Solve( std::array<double, 2> &slopeIntercept )
        {
            const double temp = 1.0 / static_cast<double>( sum_ff * sum - sum_f * sum_f );
            slopeIntercept[0] = temp * static_cast<double>( sum_fs * sum - sum_f * sum_s );
            slopeIntercept[1] = temp * static_cast<double>( sum_ff * sum_s - sum_fs * sum_f );
        }
    };

    LineFit FitParams_;

private:

    int Index( const Point2 &p )
    {
        return p.y * width_ + p.x;
    }

    bool IsInBoundary( const Point2 &p )
    {
        return p.x > 0 && p.x < width_ - 1 && p.y > 0 && p.y < height_ - 1;
    }

    int up( int index )
    {
        assert( index >= width_ );
        return index - width_;
    }

    int down( int index )
    {
        return index + width_;
    }

    int left( int index )
    {
        assert( index > 0 );
        return index - 1;
    }

    int right( int index )
    {
        return index + 1;
    }

    int left_up( int index )
    {
        assert( index >= width_ + 1 );
        return index - width_ - 1 ;
    }

    int right_up( int index )
    {
        assert( index >= width_ - 1 );
        return index - width_ + 1;
    }

    int left_down( int index )
    {
        return index + width_ - 1 ;
    }

    int right_down( int index )
    {
        return index + width_ + 1;
    }

    short Max( short x1, short x2, short x3 )
    {
        if ( x1 > x2 && x1 > x3 )
        {
            return -1;
        }
        else if ( x3 > x1 && x3 > x2 )
        {
            return 1;
        }

        return 0;
    }

    enum Direction
    {
        NON = -1,
        UP = 0,
        DOWN = 1,
        LEFT = 2,
        RIGHT = 3
    };

    void SetNextPixel( const short *pImgGra, const Direction direction, const int index,
                       Point2 &p, Direction &lastDirection );
};


inline void EDLine::SetNextPixel( const short *pImgGra, const Direction direction, const int index,
                                  Point2 &p, Direction &lastDirection )
{
    assert( p.x >= 1 && p.y >= 1 );

    lastDirection = direction;

    short offset = 0;

    switch ( direction )
    {

    case UP:
    {
        offset = Max( pImgGra[left_up( index )], pImgGra[up( index )], pImgGra[right_up( index )] );

        p.x += offset;
        --p.y;
    };
    break;

    case DOWN:
    {
        offset = Max( pImgGra[left_down( index )], pImgGra[down( index )], pImgGra[right_down( index )] );

        p.x += offset;
        ++p.y;
    };
    break;

    case LEFT:
    {
        offset = Max( pImgGra[left_up( index )], pImgGra[left( index )], pImgGra[left_down( index )] );

        --p.x;
        p.y += offset;
    };
    break;

    case RIGHT:
    {
        offset = Max( pImgGra[right_up( index )], pImgGra[right( index )], pImgGra[right_down( index )] );

        ++p.x;
        p.y += offset;
    };
    break;

    default :
        break;
    }
}

}

#endif
// kate: indent-mode cstyle; replace-tabs on;
