#include "EDLine.h"

#include <cmath>

#include <float.h>
#include <iostream>
#include <chrono>
#include <assert.h>

using namespace std;

using std::vector;
using std::array;

namespace Feature
{


EDLine::EDLine( int width, int height ) : width_( width ), height_( height )
{
    Initialize();
}

EDLine::EDLine( const Parameters &param, int width, int height ) : width_( width ), height_( height )
{
    thGradient_ = param.gradient_threshold;

    thAnchor_ = param.anchor_threshold;

    ScanIntervals_ = param.scan_intervals;

    thEdgeLength_ = param.min_length;

    thLineFitErr_ = param.line_fit_err_threshold;

    Initialize();
}

void EDLine::Initialize()
{
    pOperator_ = new GradientOperator( width_, height_ );

    pImgEdge_ = new uchar[width_ * height_];


    nExpectEdgePixelSize_ = width_ * height_ / 10;
    nExpectAnchorSize_ = nExpectEdgePixelSize_ / 5;
    nEdgeMaxNum_ = max( height_, width_ );

    pAnchorPoints_ = new Point2[nExpectAnchorSize_];

    nExpectPartSizeEdge_ = ( height_ > width_ ? height_ : width_ ) * 6;
    pPartEdgePoints_ = new Point2[nExpectPartSizeEdge_];

    pEdgePoints_ = new Point2[nExpectEdgePixelSize_];
    pEdgeS_ = new int[nEdgeMaxNum_];


    nExpectPartSizeLine_ = ( height_ > width_ ? height_ : width_ ) * 3;
    pPartLinePoints_ = new Point2[nExpectPartSizeLine_];

    pLinePoints_ = new Point2[nExpectEdgePixelSize_];
    pLineS_ = new int[nEdgeMaxNum_];
}

EDLine::~EDLine()
{
    if ( pOperator_ )
    {
        delete pOperator_;
    }

    if ( pImgEdge_ )
    {
        delete [] pImgEdge_;
    }

    if ( pAnchorPoints_ )
    {
        delete [] pAnchorPoints_;
    }

    if ( pPartEdgePoints_ )
    {
        delete [] pPartEdgePoints_;
    }

    if ( pEdgePoints_ )
    {
        delete [] pEdgePoints_;
        delete [] pEdgeS_;

    }

    if ( pPartLinePoints_ )
    {
        delete [] pPartLinePoints_;
    }

    if ( pLinePoints_ )
    {
        delete [] pLinePoints_;
        delete [] pLineS_;
    }
}

bool EDLine::Detect( const uchar *pImg )
{
//     chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    if ( !DetectEdges( pImg ) )
        return false;

    if ( !DetectLines() )
        return false;

//     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//     chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2 - t1 );
//     cout<<"Detect cost time: "<<1000*time_used.count() <<"ms"<<endl;
    return true;
}

bool EDLine::DetectEdges( const uchar *pImg )
{
    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    ProcessImage( pImg );

    DetectAnchors();

    OffsetEdge_ = 0, CountEdge_ = 0;

    for ( int i = 0; i < AnchorSize_; ++i )
    {
        if ( !DetectEdge( i ) )
            continue;

        short length = IndexEdgeEnd_ - IndexEdgeStart_ - 1;

        pEdgeS_[CountEdge_] = OffsetEdge_;

        memcpy( pEdgePoints_ + OffsetEdge_, pPartEdgePoints_ + IndexEdgeStart_ + 1, length * sizeof( Point2 ) );

        OffsetEdge_ += length;
        ++CountEdge_;
    }

    if ( OffsetEdge_ > nExpectEdgePixelSize_ || CountEdge_ > nEdgeMaxNum_ )
    {
        cout << "\033[31m" << "pEdgePoints_ size is larger than its maximal size. OffsetEdge_ = " << OffsetEdge_
             << ", maximal size = " << nExpectEdgePixelSize_ << endl;

        cout << "\033[31m" << "pEdgeS_ size is larger than its maximal size. CountEdge_ = " << CountEdge_
             << ", maximal size = " << nEdgeMaxNum_ << endl;

        exit( 0 );
    }

    if ( CountEdge_ == 0 )
        return false;

    pEdgeS_[CountEdge_] = OffsetEdge_;

    // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2 - t1 );
    // cout<<"DetectEdges cost time: "<<1000*time_used.count() <<"ms"<<endl;

    return true;
}

bool EDLine::DetectLines()
{
    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    OffsetLine_ = 0;
    CountLine_ = 0;

    for ( int i = 0; i < CountEdge_; ++i )
    {
        LinesFit( i );
    }

    if ( OffsetLine_ > nExpectEdgePixelSize_ || CountLine_ > nEdgeMaxNum_ )
    {
        cout << "\033[31m" << "pLinePoints_ size is larger than its maximal size. OffsetLine_ = " << OffsetLine_
             << ", maximal size = " << nExpectEdgePixelSize_ << endl;

        cout << "\033[31m" << "pLineS_ size is larger than its maximal size. CountLine_ = " << CountLine_
             << ", maximal size = " << nEdgeMaxNum_ << endl;

        exit( 0 );
    }

    if ( CountLine_ == 0 )
        return false;

    pLineS_[CountLine_] = OffsetLine_;

    // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2 - t1 );
    // cout<<"DetectLines cost time: "<<1000*time_used.count() <<"ms"<<endl;

    return true;
}

void EDLine::GetEdges( PixelChains &edgeChains )
{
    if ( CountEdge_ ==  0 )
    {
        cout << "No edges be detected!\n";
        return;
    }

    edgeChains.vPoints.resize( OffsetEdge_ );
    edgeChains.vStartIds.resize( CountEdge_ + 1 );

    memcpy( edgeChains.vPoints.data(), pEdgePoints_, OffsetEdge_ * sizeof( Point2 ) );
    memcpy( edgeChains.vStartIds.data(), pEdgeS_, ( CountEdge_ + 1 ) *sizeof( int ) );

    edgeChains.ChainsNum = CountEdge_;
}

void EDLine::GetLines( PixelChains &lineChains )
{
    if ( CountLine_ ==  0 )
    {
        cout << "No lines be detected!\n";
        return;
    }

    lineChains.vPoints.resize( OffsetLine_ );
    lineChains.vStartIds.resize( CountLine_ + 1 );

    memcpy( lineChains.vPoints.data(), pLinePoints_, OffsetLine_ * sizeof( Point2 ) );
    memcpy( lineChains.vStartIds.data(), pLineS_, ( CountLine_ + 1 ) *sizeof( int ) );

    lineChains.ChainsNum = CountLine_;
}

void EDLine::ProcessImage( const uchar *pImg )
{
//     chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    memset( pImgEdge_, 0, width_ * height_ * sizeof( uchar ) );

    vLines_.clear();

    pOperator_->Compute( pImg );

//     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//     chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2 - t1 );
//     cout<<"ProcessImage cost time: "<<1000*time_used.count() <<"ms"<<endl;
}

void EDLine::DetectAnchors()
{
//     chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    const short *pImgGrad = pOperator_->GetPtrGrad();

    int index = 0;
    AnchorSize_ = 0;

    for ( int y = 1; y < height_ - 1; y += ScanIntervals_ )
    {
        for ( int x = 1; x < width_ - 1; x += ScanIntervals_ )
        {
            index = y * width_ + x;

            if ( pImgGrad[index] < thGradient_ )
                continue;

            const short grad = pImgGrad[index] - thAnchor_;

            if ( pOperator_->IsHorizontal( index ) )
            {
                if ( grad < pImgGrad[up( index )] )
                    continue;
                if ( grad < pImgGrad[down( index )] )
                    continue;
            }
            else
            {
                if ( grad < pImgGrad[left( index )] )
                    continue;
                if ( grad < pImgGrad[right( index )] )
                    continue;
            }

            pAnchorPoints_[AnchorSize_] = Point2( x, y );

            ++AnchorSize_;
        }
    }

    if ( AnchorSize_ >= nExpectAnchorSize_ )
    {
        cout << "\033[31m" << "anchor size is larger than its maximal size. anchorsSize = " << AnchorSize_
             << ", maximal size = " << nExpectAnchorSize_ << endl;
        exit( 0 );
    }

//     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//     chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2 - t1 );
//     cout<<"DetectAnchors cost time: "<<1000*time_used.count() <<"ms"<<endl;
}

bool EDLine::DetectEdge( int anchorIndex )
{
    IndexEdgeStart_ = nExpectPartSizeEdge_ / 2;
    IndexEdgeEnd_ = IndexEdgeStart_;

    const short *pImgGrad = pOperator_->GetPtrGrad();

    Point2 lastP;
    Direction LastDirection = NON, ShouldGoDirection = NON;

    Point2 p = pAnchorPoints_[anchorIndex];
    int index = Index( p );

    if ( pImgEdge_[index] == 1 )
        return false;

    if ( pOperator_->IsHorizontal( index ) )  //case1 锚点是水平方向
    {
        LastDirection = RIGHT;
        while ( pImgGrad[index] > thGradient_ && pImgEdge_[index] == 0 && IsInBoundary( p ) ) //case1.1 向右延伸
        {
            pImgEdge_[index] = 1;

            pPartEdgePoints_[IndexEdgeEnd_] = p;
            ++IndexEdgeEnd_;

            ShouldGoDirection = NON;
            if ( pOperator_->IsHorizontal( index ) )  //case1.1.1 水平方向延伸
            {
                if ( LastDirection == UP || LastDirection == DOWN )
                {
                    ShouldGoDirection = p.x > lastP.x ? RIGHT : LEFT;
                }

                lastP = p;

                if ( LastDirection == RIGHT || ShouldGoDirection == RIGHT )
                {
                    SetNextPixel( pImgGrad, RIGHT, index, p, LastDirection );
                }
                else if ( LastDirection == LEFT || ShouldGoDirection == LEFT )
                {
                    SetNextPixel( pImgGrad, LEFT, index, p, LastDirection );
                }
            }  //end case1.1.1
            else  //case1.1.2 竖直方向延伸
            {
                if ( LastDirection == RIGHT || LastDirection == LEFT )
                {
                    ShouldGoDirection = p.y > lastP.y ? DOWN : UP;
                }

                lastP = p;

                if ( LastDirection == DOWN || ShouldGoDirection == DOWN )
                {
                    SetNextPixel( pImgGrad, DOWN, index, p, LastDirection );
                }
                else if ( LastDirection == UP || ShouldGoDirection == UP )
                {
                    SetNextPixel( pImgGrad, UP, index, p, LastDirection );
                }
            }  //end case1.1.2
            index = Index( p );
        } //end while


        p = pAnchorPoints_[anchorIndex];
        index = Index( p );

        LastDirection = LEFT;

        pImgEdge_[index] = 0;

        while ( pImgGrad[index] > thGradient_ && pImgEdge_[index] == 0 && IsInBoundary( p ) ) //case1.2 向左延伸
        {
            pImgEdge_[index] = 1;

            pPartEdgePoints_[IndexEdgeStart_] = p;
            --IndexEdgeStart_;

            ShouldGoDirection = NON;
            if ( pOperator_->IsHorizontal( index ) )  //case1.2.1 水平方向延伸
            {
                if ( LastDirection == UP || LastDirection == DOWN )
                {
                    ShouldGoDirection = p.x > lastP.x ? RIGHT : LEFT;
                }

                lastP = p;

                if ( LastDirection == RIGHT || ShouldGoDirection == RIGHT )
                {
                    SetNextPixel( pImgGrad, RIGHT, index, p, LastDirection );
                }
                else if ( LastDirection == LEFT || ShouldGoDirection == LEFT )
                {
                    SetNextPixel( pImgGrad, LEFT, index, p, LastDirection );
                }
            }  //end case1.2.1
            else  //case1.2.2 竖直方向延伸
            {
                if ( LastDirection == RIGHT || LastDirection == LEFT )
                {
                    ShouldGoDirection = p.y > lastP.y ? DOWN : UP;
                }

                lastP = p;

                if ( LastDirection == DOWN || ShouldGoDirection == DOWN )
                {
                    SetNextPixel( pImgGrad, DOWN, index, p, LastDirection );
                }
                else if ( LastDirection == UP || ShouldGoDirection == UP )
                {
                    SetNextPixel( pImgGrad, UP, index, p, LastDirection );
                }
            }  //end case1.2.2

            index = Index( p );
        }  //end while
    }
    else  //case2 锚点是竖直方向
    {
        LastDirection = DOWN;
        while ( pImgGrad[index] > thGradient_ && pImgEdge_[index] == 0 && IsInBoundary( p ) ) //case2.1 向下延伸
        {
            pImgEdge_[index] = 1;

            pPartEdgePoints_[IndexEdgeEnd_] = p;
            ++IndexEdgeEnd_;

            ShouldGoDirection = NON;
            if ( pOperator_->IsHorizontal( index ) )  //case2.1.1 水平方向延伸
            {
                if ( LastDirection == UP || LastDirection == DOWN )
                {
                    ShouldGoDirection = p.x > lastP.x ? RIGHT : LEFT;
                }

                lastP = p;

                if ( LastDirection == RIGHT || ShouldGoDirection == RIGHT )
                {
                    SetNextPixel( pImgGrad, RIGHT, index, p, LastDirection );
                }
                else if ( LastDirection == LEFT || ShouldGoDirection == LEFT )
                {
                    SetNextPixel( pImgGrad, LEFT, index, p, LastDirection );
                }
            }  //end case2.1.1
            else  //case2.1.2 竖直方向延伸
            {
                if ( LastDirection == RIGHT || LastDirection == LEFT )
                {
                    ShouldGoDirection = p.y > lastP.y ? DOWN : UP;
                }

                lastP = p;

                if ( LastDirection == DOWN || ShouldGoDirection == DOWN )
                {
                    SetNextPixel( pImgGrad, DOWN, index, p, LastDirection );
                }
                else if ( LastDirection == UP || ShouldGoDirection == UP )
                {
                    SetNextPixel( pImgGrad, UP, index, p, LastDirection );
                }
            }  //end case2.1.2
            index = Index( p );
        } //end while


        p = pAnchorPoints_[anchorIndex];
        index = Index( p );

        pImgEdge_[index] = 0;
        LastDirection = UP;

        while ( pImgGrad[index] > thGradient_ && pImgEdge_[index] == 0 && IsInBoundary( p ) ) //case2.2 向上延伸
        {
            pImgEdge_[index] = 1;

            pPartEdgePoints_[IndexEdgeStart_] = p;
            --IndexEdgeStart_;

            ShouldGoDirection = NON;
            if ( pOperator_->IsHorizontal( index ) )  //case2.2.1 水平方向延伸
            {
                if ( LastDirection == UP || LastDirection == DOWN )
                {
                    ShouldGoDirection = p.x > lastP.x ? RIGHT : LEFT;
                }

                lastP = p;

                if ( LastDirection == RIGHT || ShouldGoDirection == RIGHT )
                {
                    SetNextPixel( pImgGrad, RIGHT, index, p, LastDirection );
                }
                else if ( LastDirection == LEFT || ShouldGoDirection == LEFT )
                {
                    SetNextPixel( pImgGrad, LEFT, index, p, LastDirection );
                }
            }  //end case2.2.1
            else  //case2.2.2 竖直方向延伸
            {
                if ( LastDirection == RIGHT || LastDirection == LEFT )
                {
                    ShouldGoDirection = p.y > lastP.y ? DOWN : UP;
                }

                lastP = p;

                if ( LastDirection == DOWN || ShouldGoDirection == DOWN )
                {
                    SetNextPixel( pImgGrad, DOWN, index, p, LastDirection );
                }
                else if ( LastDirection == UP || ShouldGoDirection == UP )
                {
                    SetNextPixel( pImgGrad, UP, index, p, LastDirection );
                }
            }  //end case2.2.2

            index = Index( p );
        }  //end while
    }  //end case2


    if ( IndexEdgeEnd_ >= nExpectPartSizeEdge_ || IndexEdgeStart_ <= 0 )
    {
        cout << "\033[31m" << "\nError, wrong value of array index! IndexEdgeStart_: " << IndexEdgeStart_ << " ,IndexEdgeEnd_: " << IndexEdgeEnd_ << endl;
        cout << "Expected ranges of array index: " << "0 <= IndexEdgeStart_ <= IndexEdgeEnd_ <= " << nExpectPartSizeEdge_ << endl;
        cout << "Please increase pPartEdgePoints_'s length!\n" << endl;
        exit( 0 );
    }

    if ( IndexEdgeEnd_ - IndexEdgeStart_ < thEdgeLength_ + 1 )
        return false;

    return true;
}

inline double Dist( const Point2 &p, const array<double, 2> &slopeIntercept, bool bHorizontal )
{
    const auto temp = bHorizontal ? Point2( p.x, p.y ) : Point2( p.y, p.x );
    return fabs( temp.x * slopeIntercept[0] + slopeIntercept[1] - temp.y );
}
void EDLine::LinesFit( int edgeIndex )
{
    const int edgeLength = pEdgeS_[edgeIndex + 1] - pEdgeS_[edgeIndex];
    if ( edgeLength < InitLineLength_ )
        return;

    IndexEdgeStart_ = 0;
    IndexEdgeEnd_ = edgeLength;
    memcpy( pPartEdgePoints_, pEdgePoints_ + pEdgeS_[edgeIndex], sizeof( Point2 )*edgeLength );

    while ( IndexEdgeStart_ + InitLineLength_ < IndexEdgeEnd_ )
    {
        IndexLineStart_ = nExpectPartSizeLine_ / 3;
        IndexLineEnd_ = IndexLineStart_;

        bool bHorizontal = true;
        double lineFitErr = 0.0;
        array<double, 2> slopeIntercept;

        while ( IndexEdgeStart_ + InitLineLength_ < IndexEdgeEnd_ )
        {
            bHorizontal = pOperator_->IsHorizontal( Index( pPartEdgePoints_[IndexEdgeStart_] ) );

            lineFitErr = LeastSquareFit( bHorizontal, pPartEdgePoints_, IndexEdgeStart_, slopeIntercept );

            if ( lineFitErr <= thLineFitErr_ )
                break;

            IndexEdgeStart_ += SkipEdgePoint_;
        }

        if ( lineFitErr > thLineFitErr_ )
            break;


        bool bExtended = true;
        bool bFirstTry = true;

        int newOffsetS = 0;
        int tryTimes = 0;

        while ( bExtended )
        {
            ++tryTimes;

            if ( bFirstTry )
            {
                bFirstTry = false;

                for ( int i = 0; i < InitLineLength_; ++i )
                {
                    pPartLinePoints_[IndexLineEnd_] = pPartEdgePoints_[IndexEdgeStart_];

                    ++IndexLineEnd_;
                    ++IndexEdgeStart_;
                }
            }
            else
            {
                LeastSquareFit( bHorizontal, pPartLinePoints_, newOffsetS, IndexLineEnd_, slopeIntercept );
            }

            newOffsetS = IndexLineEnd_;

            const double thDist = thLineFitErr_ * sqrt( slopeIntercept[0] * slopeIntercept[0] + 1.0 );
            int outlierCount = 0;

            while ( IndexEdgeStart_ < IndexEdgeEnd_ )
            {
                double dist = Dist( pPartEdgePoints_[IndexEdgeStart_], slopeIntercept, bHorizontal );

                pPartLinePoints_[IndexLineEnd_] = pPartEdgePoints_[IndexEdgeStart_];

                ++IndexLineEnd_;
                ++IndexEdgeStart_;

                if ( dist > thDist )
                {
                    ++outlierCount;
                    if ( outlierCount > MaxOutlierNum_ )
                        break;
                }
                else
                {
                    outlierCount = 0;
                }
            }

            IndexLineEnd_ -= outlierCount;
            IndexEdgeStart_ -= outlierCount;

            if ( IndexLineEnd_ <= newOffsetS || tryTimes >= TryTime_ )
            {
                bExtended = false;
            }
        }

        bool bStored = IndexLineEnd_ - IndexLineStart_ < thLineLength_ ? false : true;
        if ( bStored )
            bStored = StoreLine( bHorizontal, slopeIntercept );
    }
}

double EDLine::LeastSquareFit( const bool bHorizontal, const Point2 *pPoints,
                               int offsetS, array<double, 2> &slopeIntercept )
{
    FitParams_.Reset();

    int offset = offsetS;
    for ( int i = 0; i < InitLineLength_; ++i )
    {
        const auto &p0 = pPoints[offset];
        const auto p = bHorizontal ? Point2( p0.x, p0.y ) : Point2( p0.y, p0.x );
        FitParams_.AddPoint( p.x, p.y );

        ++offset;
    }

    FitParams_.Solve( slopeIntercept );

    offset = offsetS;
    double fitError = 0.0;
    for ( int i = 0; i < InitLineLength_; ++i )
    {
        const auto &p0 = pPoints[offset];
        const auto p = bHorizontal ? Point2( p0.x, p0.y ) : Point2( p0.y, p0.x );

        const double dist = double ( p.y ) - double ( p.x ) * slopeIntercept[0] - slopeIntercept[1];
        fitError += dist * dist;

        ++offset;
    }

    return sqrt( fitError );
}

void EDLine::LeastSquareFit( const bool bHorizontal, const Point2 *pPoints,
                             int newOffsetS, int offsetE, array<double, 2> &slopeIntercept )
{
    int newLength = offsetE - newOffsetS;
    assert( newLength > 0 );

    for ( int i = 0; i < newLength; ++i )
    {
        const auto &p0 = pPoints[newOffsetS];
        const auto p = bHorizontal ? Point2( p0.x, p0.y ) : Point2( p0.y, p0.x );
        FitParams_.AddPoint( p.x, p.y );

        ++newOffsetS;
    }

    FitParams_.Solve( slopeIntercept );
}

bool EDLine::StoreLine( const bool bHorizontal, const std::array<double, 2> &slopeIntercept )
{
    const double rate = 1.0 / sqrt( slopeIntercept[0] * slopeIntercept[0] + 1.0 );
    const double a = rate * ( bHorizontal ? slopeIntercept[0] : -1.0 );
    const double b = rate * ( bHorizontal ? -1.0 : slopeIntercept[0] );
    const double c = rate * slopeIntercept[1];
    const double orientation = atan2( a, b );

    const double k0 = b * b;
    const double k1 = a * a;
    const double k2 = a * b;
    const double k3 = c * a;
    const double k4 = c * b;

    const auto &sp = pPartLinePoints_[IndexLineStart_];
    const auto &ep = pPartLinePoints_[IndexLineEnd_ - 1];

    const double sx = k0 * sp.x - k2 * sp.y - k3;
    const double sy = k1 * sp.y - k2 * sp.x - k4;
    const double ex = k0 * ep.x - k2 * ep.y - k3;
    const double ey = k1 * ep.y - k2 * ep.x - k4;

    Line line{ sx, sy, ex, ey, a, b, c, orientation};
    vLines_.emplace_back( line );

    pLineS_[CountLine_] = OffsetLine_;
    short length = IndexLineEnd_ -  IndexLineStart_;
    memcpy( pLinePoints_ + OffsetLine_, pPartLinePoints_ + IndexLineStart_, length * sizeof( Point2 ) );

    OffsetLine_ += length;
    ++CountLine_;

    return true;
}

}
