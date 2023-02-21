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
    float gradient_threshold;
    float anchor_threshold;

    uchar scan_intervals;
    uchar min_length;

    float line_fit_err_threshold;
};


struct PixelChains
{
    std::vector<ushort> vXcoords;
    std::vector<ushort> vYcoords;

    std::vector<uint> vStartIds;

    uint ChainsNum = 0;
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

    void AddPoint( ushort first, ushort second )
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

    EDLine( uint imgWidth, uint imgHeight );

    EDLine( const Parameters &param, uint imgWidth, uint imgHeight );

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

    bool DetectEdge( uint anchorIndex );

    void DetectLines( uint edgeIndex );

    double LeastSquareFit( const ushort *pFirstCoord, const ushort *pSecondCoord,
                           uint offsetS, std::array<double, 2> &slopeIntercept );

    void LeastSquareFit( const ushort *pFirstCoord, const ushort *pSecondCoord,
                         uint newOffsetS, uint offsetE, std::array<double, 2> &slopeIntercept );

    bool StoreLine( const bool bHorizontal, const std::array<double, 2> &slopeIntercept );

private:

    uchar GradientThreshold_ = 15;

    uchar AnchorThreshold_ = 3;

    uchar ScanIntervals_ = 2;

    uchar MinEdgeLength_ = 25; //25

public:

    uint ImageWidth_ = 1280, ImageHeight_ = 720;

    GradientOperator *pOperator_ = nullptr;

private:

    uchar *pImgEdge_ = nullptr;

    ushort *pAnchorX_ = nullptr;
    ushort *pAnchorY_ = nullptr;

    uint nExpectEdgePixelSize_ = 0;
    uint nExpectAnchorSize_ = 0;
    uint nEdgeMaxNum_ = 0;
    uint AnchorSize_ = 0;

    ushort *pPartEdgeX_ = nullptr;
    ushort *pPartEdgeY_ = nullptr;
    short nExpectPartSizeEdge_ = 0;
    short IndexEdgeStart_ = 0, IndexEdgeEnd_ = 0;

    ushort *pEdgeX_ = nullptr;
    ushort *pEdgeY_ = nullptr;
    uint *pEdgeS_ = nullptr;
    uint OffsetEdge_ = 0, CountEdge_ = 0;

private:

    uchar InitLineLength_ = 20;
    uchar MinLineLength_ = 40;

    float LineFitErrThreshold_ = 1.414;

    uchar SkipEdgePoint_ = 2;

    uchar TryTime_ = 6;

    uchar MaxOutlierNum_ = 5;

    LineFit FitParams_;

private:

    ushort *pPartLineX_ = nullptr;
    ushort *pPartLineY_ = nullptr;
    short nExpectPartSizeLine_ = 0;
    short IndexLineStart_ = 0, IndexLineEnd_ = 0;

    ushort *pLineX_ = nullptr;
    ushort *pLineY_ = nullptr;
    uint *pLineS_ = nullptr;
    uint OffsetLine_ = 0, CountLine_ = 0;

public:

    std::vector<LineEquation> vLineEquations_;
    std::vector<LineEndpoint> vLineEndpoints_;
    std::vector<float> vLineDirections_;

private:

    uint up( uint index )
    {
        assert( index >= ImageWidth_ );
        return index - ImageWidth_;
    }

    uint down( uint index )
    {
        return index + ImageWidth_;
    }

    uint left( uint index )
    {
        assert( index > 0 );
        return index - 1;
    }

    uint right( uint index )
    {
        return index + 1;
    }

    uint left_up( uint index )
    {
        assert( index >= ImageWidth_ + 1 );
        return index - ImageWidth_ - 1 ;
    }

    uint right_up( uint index )
    {
        assert( index >= ImageWidth_ - 1 );
        return index - ImageWidth_ + 1;
    }

    uint left_down( uint index )
    {
        return index + ImageWidth_ - 1 ;
    }

    uint right_down( uint index )
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

    void SetNextPixel( const short *pImgGra, const Direction direction, const uint index,
                       ushort &x, ushort &y, Direction &lastDirection );

};


inline void EDLine::SetNextPixel( const short *pImgGra, const Direction direction, const uint index,
                                  ushort &x, ushort &y, Direction &lastDirection )
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
