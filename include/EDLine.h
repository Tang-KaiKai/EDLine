#pragma once

#ifndef EDLINE_H
#define EDLINE_H

#include <string.h>

#include <vector>
#include <array>

#include "Gradient.hpp"
#include <assert.h>

namespace Feature
{

typedef unsigned char uchar;

struct Parameters
{
    int gradient_threshold;
    int anchor_threshold;

    int scan_intervals;
    int min_length;

    float line_fit_err_threshold;
};


struct PixelChains
{
    std::vector<short> vXcoords;
    std::vector<short> vYcoords;

    std::vector<int> vStartIds;

    int ChainsNum = 0;
};


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
        sum_fs += static_cast<long>( first * second );

        sum_f += static_cast<long>( first );
        sum_ff += static_cast<long>( first * first );

        sum_s += static_cast<long>( second );
    }

    void Solve( std::array<double, 2> &slopeIntercept )
    {
        const double temp = 1.0 / static_cast<double>( sum_ff * sum - sum_f * sum_f );
        slopeIntercept[0] = temp * static_cast<double>( sum_fs * sum - sum_f * sum_s );
        slopeIntercept[1] = temp * static_cast<double>( sum_ff * sum_s - sum_fs * sum_f );
    }
};



class EDLine
{
    typedef std::array<double, 3> LineEquation;
    typedef std::array<float, 4> LineEndpoint;

public:

    EDLine() = delete;

    EDLine( int imgWidth, int imgHeight );

    EDLine( const Parameters &param, int imgWidth, int imgHeight );

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

    void DetectLines( int edgeIndex );

    double LeastSquareFit( const short *pFirstCoord, const short *pSecondCoord,
                           int offsetS, std::array<double, 2> &slopeIntercept );

    void LeastSquareFit( const short *pFirstCoord, const short *pSecondCoord,
                         int newOffsetS, int offsetE, std::array<double, 2> &slopeIntercept );

    bool StoreLine( const bool bHorizontal, const std::array<double, 2> &slopeIntercept );

private:

    int GradientThreshold_ = 15;

    int AnchorThreshold_ = 3;

    int ScanIntervals_ = 2;

    int MinEdgeLength_ = 25; //25

public:

    int ImageWidth_ = 1280, ImageHeight_ = 720;

    GradientOperator *pOperator_ = nullptr;

private:

    uchar *pImgEdge_ = nullptr;

    short *pAnchorX_ = nullptr;
    short *pAnchorY_ = nullptr;

    int nExpectEdgePixelSize_ = 0;
    int nExpectAnchorSize_ = 0;
    int nEdgeMaxNum_ = 0;
    int AnchorSize_ = 0;

    short *pPartEdgeX_ = nullptr;
    short *pPartEdgeY_ = nullptr;
    short nExpectPartSizeEdge_ = 0;
    short IndexEdgeStart_ = 0, IndexEdgeEnd_ = 0;

    short *pEdgeX_ = nullptr;
    short *pEdgeY_ = nullptr;
    int *pEdgeS_ = nullptr;
    int OffsetEdge_ = 0, CountEdge_ = 0;

private:

    int InitLineLength_ = 20;
    int MinLineLength_ = 40;

    float LineFitErrThreshold_ = 1.414;

    int SkipEdgePoint_ = 2;

    int TryTime_ = 6;

    int MaxOutlierNum_ = 5;

    LineFit FitParams_;

private:

    short *pPartLineX_ = nullptr;
    short *pPartLineY_ = nullptr;
    short nExpectPartSizeLine_ = 0;
    short IndexLineStart_ = 0, IndexLineEnd_ = 0;

    short *pLineX_ = nullptr;
    short *pLineY_ = nullptr;
    int *pLineS_ = nullptr;
    int OffsetLine_ = 0, CountLine_ = 0;

public:

    std::vector<LineEquation> vLineEquations_;
    std::vector<LineEndpoint> vLineEndpoints_;
    std::vector<float> vLineDirections_;

private:

    int up( int index )
    {
        assert( index >= ImageWidth_ );
        return index - ImageWidth_;
    }

    int down( int index )
    {
        return index + ImageWidth_;
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
        assert( index >= ImageWidth_ + 1 );
        return index - ImageWidth_ - 1 ;
    }

    int right_up( int index )
    {
        assert( index >= ImageWidth_ - 1 );
        return index - ImageWidth_ + 1;
    }

    int left_down( int index )
    {
        return index + ImageWidth_ - 1 ;
    }

    int right_down( int index )
    {
        return index + ImageWidth_ + 1;
    }

    short max( short x1, short x2, short x3 )
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
                       short &x, short &y, Direction &lastDirection );

};


inline void EDLine::SetNextPixel( const short *pImgGra, const Direction direction, const int index,
                                  short &x, short &y, Direction &lastDirection )
{
    assert( x >= 1 && y >= 1 );

    lastDirection = direction;

    short offset = 0;

    switch ( direction )
    {

    case UP:
    {
        offset = max( pImgGra[left_up( index )], pImgGra[up( index )], pImgGra[right_up( index )] );

        x += offset;
        --y;
    };
    break;

    case DOWN:
    {
        offset = max( pImgGra[left_down( index )], pImgGra[down( index )], pImgGra[right_down( index )] );

        x += offset;
        ++y;
    };
    break;

    case LEFT:
    {
        offset = max( pImgGra[left_up( index )], pImgGra[left( index )], pImgGra[left_down( index )] );

        --x;
        y += offset;
    };
    break;

    case RIGHT:
    {
        offset = max( pImgGra[right_up( index )], pImgGra[right( index )], pImgGra[right_down( index )] );

        ++x;
        y += offset;
    };
    break;

    default :
        break;

    }
}

}

#endif
// kate: indent-mode cstyle; replace-tabs on;
