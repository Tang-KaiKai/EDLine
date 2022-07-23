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


EDLine::EDLine ( uint imgWidth, uint imgHeight ) :ImageWidth_ ( imgWidth ),ImageHeight_ ( imgHeight )
{
    Initialize();
}

EDLine::EDLine ( const Parameters& param, uint imgWidth, uint imgHeight ) :ImageWidth_ ( imgWidth ),ImageHeight_ ( imgHeight )
{
    GradientThreshold_ = param.gradient_threshold;

    AnchorThreshold_ = param.anchor_threshold;

    ScanIntervals_ = param.scan_intervals;

    MinEdgeLength_ = param.min_length;

    Initialize();
}

void EDLine::Initialize()
{
    pSobel_ = new SobelOperator ( ImageWidth_, ImageHeight_ );

    pImgEdge_ = new uchar[ImageWidth_*ImageHeight_];


    nExpectEdgePixelSize_ = ImageWidth_*ImageHeight_/10;
    nExpectAnchorSize_ = nExpectEdgePixelSize_/5;
    nEdgeMaxNum_ = ImageHeight_>ImageWidth_ ? ImageHeight_:ImageWidth_;

    pAnchorX_ = new ushort[nExpectAnchorSize_];
    pAnchorY_ = new ushort[nExpectAnchorSize_];

    nExpectPartSizeEdge_ = ( ImageHeight_>ImageWidth_ ? ImageHeight_:ImageWidth_ ) *6;
    pPartEdgeX_ = new ushort[nExpectPartSizeEdge_];
    pPartEdgeY_ = new ushort[nExpectPartSizeEdge_];

    pEdgeX_ = new ushort[nExpectEdgePixelSize_];
    pEdgeY_ = new ushort[nExpectEdgePixelSize_];
    pEdgeS_ = new uint[nEdgeMaxNum_];

    
    nExpectPartSizeLine_ = ( ImageHeight_>ImageWidth_ ? ImageHeight_:ImageWidth_ ) *3;
    pPartLineX_ = new ushort[nExpectPartSizeLine_];
    pPartLineY_ = new ushort[nExpectPartSizeLine_];

    pLineX_ = new ushort[nExpectEdgePixelSize_];
    pLineY_ = new ushort[nExpectEdgePixelSize_];
    pLineS_ = new uint[nEdgeMaxNum_];
}

EDLine::~EDLine()
{
    if ( pSobel_ )
    {
        delete pSobel_;
    }

    if ( pImgEdge_ )
    {
        delete [] pImgEdge_;
    }

    if ( pAnchorX_ )
    {
        delete [] pAnchorX_;
        delete [] pAnchorY_;
    }

    if ( pEdgeX_ )
    {
        delete [] pEdgeX_;
        delete [] pEdgeY_;
        delete [] pEdgeS_;

    }

    if ( pPartEdgeX_ )
    {
        delete [] pPartEdgeX_;
        delete [] pPartEdgeY_;
    }
    
    
    if( pPartLineX_ )
    {
        delete [] pPartLineX_;
        delete [] pPartLineY_;
    }
    
    if( pLineX_ )
    {
        delete [] pLineX_;
        delete [] pLineY_;
        delete [] pLineS_;
    }
}


bool EDLine::Detect( const uchar* pImg )
{
//     chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    
    if( !DetectEdges( pImg ) )
        return false;

    if( !DetectLines() )
        return false;
    
//     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//     chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2 - t1 );
//     cout<<"Detect cost time: "<<1000*time_used.count() <<"ms"<<endl;
    return true;
}


bool EDLine::DetectEdges( const uchar* pImg )
{
//     chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    
    ProcessImage ( pImg );

    DetectAnchors();
    

    OffsetEdge_=0, CountEdge_=0;

    for ( uint i=0; i<AnchorSize_; ++i )
    {
        if ( !DetectEdge ( i ) )
            continue;

        short length = IndexEdgeEnd_ - IndexEdgeStart_ - 1;

        pEdgeS_[CountEdge_] = OffsetEdge_;

        memcpy ( pEdgeX_ + OffsetEdge_, pPartEdgeX_ + IndexEdgeStart_ +1, length*sizeof ( ushort ) );
        memcpy ( pEdgeY_ + OffsetEdge_, pPartEdgeY_ + IndexEdgeStart_ +1, length*sizeof ( ushort ) );

        OffsetEdge_ += length;
        ++CountEdge_;
    }
    
    if( OffsetEdge_ > nExpectEdgePixelSize_ || CountEdge_ > nEdgeMaxNum_ )
    {
        cout<<"\033[31m"<<"pEdgeX_ size is larger than its maximal size. OffsetEdge_ = "<<OffsetEdge_
            <<", maximal size = "<<nExpectEdgePixelSize_ <<endl;
        
        cout<<"\033[31m"<<"pEdgeS_ size is larger than its maximal size. CountEdge_ = "<<CountEdge_
            <<", maximal size = "<<nEdgeMaxNum_ <<endl;
            
        exit ( 0 );
    }

    if( CountEdge_ == 0 )
        return false;

    pEdgeS_[CountEdge_] = OffsetEdge_;
    
//     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//     chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2 - t1 );
//     cout<<"DetectEdges cost time: "<<1000*time_used.count() <<"ms"<<endl;
    
    return true;
}


bool EDLine::DetectLines()
{
//     chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    
    OffsetLine_ = 0;
    CountLine_ = 0;

    for( uint i = 0; i < CountEdge_; ++i )
    {
        DetectLines( i );
    }
    
    if( OffsetLine_ > nExpectEdgePixelSize_ || CountLine_ > nEdgeMaxNum_ )
    {
        cout<<"\033[31m"<<"pLineX_ size is larger than its maximal size. OffsetLine_ = "<<OffsetLine_
            <<", maximal size = "<<nExpectEdgePixelSize_ <<endl;
        
        cout<<"\033[31m"<<"pLineS_ size is larger than its maximal size. CountLine_ = "<<CountLine_
            <<", maximal size = "<<nEdgeMaxNum_ <<endl;
            
        exit ( 0 );
    }

    if( CountLine_ == 0 )
        return false;

    pLineS_[CountLine_] = OffsetLine_;
    
//     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//     chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2 - t1 );
//     cout<<"DetectLines cost time: "<<1000*time_used.count() <<"ms"<<endl;
    
    return true;
}



void EDLine::GetEdges( PixelChains& edgeChains )
{
    if( CountEdge_ ==  0 )
    {
        cout<<"No edges be detected!\n";
        return;
    }
    
    edgeChains.vXcoords.resize ( OffsetEdge_ );
    edgeChains.vYcoords.resize ( OffsetEdge_ );
    edgeChains.vStartIds.resize ( CountEdge_+1 );

    ushort* pXcoord = edgeChains.vXcoords.data();
    ushort* pYcoord = edgeChains.vYcoords.data();
    uint* pStartId = edgeChains.vStartIds.data();

    memcpy ( pXcoord, pEdgeX_, OffsetEdge_*sizeof ( ushort ) );
    memcpy ( pYcoord, pEdgeY_, OffsetEdge_*sizeof ( ushort ) );
    memcpy ( pStartId, pEdgeS_, ( CountEdge_+1 ) *sizeof ( uint ) );

    edgeChains.ChainsNum = CountEdge_;
}

void EDLine::GetLines( PixelChains& lineChains )
{
    if( CountLine_ ==  0 )
    {
        cout<<"No lines be detected!\n";
        return;
    }
    
    lineChains.vXcoords.resize ( OffsetLine_ );
    lineChains.vYcoords.resize ( OffsetLine_ );
    lineChains.vStartIds.resize ( CountLine_+1 );

    ushort* pXcoord = lineChains.vXcoords.data();
    ushort* pYcoord = lineChains.vYcoords.data();
    uint* pStartId = lineChains.vStartIds.data();

    memcpy ( pXcoord, pLineX_, OffsetLine_*sizeof ( ushort ) );
    memcpy ( pYcoord, pLineY_, OffsetLine_*sizeof ( ushort ) );
    memcpy ( pStartId, pLineS_, ( CountLine_+1 ) *sizeof ( uint ) );

    lineChains.ChainsNum = CountLine_;
}


void EDLine::ProcessImage ( const uchar* pImg )
{
//     chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    memset ( pImgEdge_,0, ImageWidth_*ImageHeight_*sizeof ( uchar ) );

    vLineDirections_.clear();
    vLineEndpoints_.clear();
    vLineEquations_.clear();

    pSobel_->Compute ( pImg );

//     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//     chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2 - t1 );
//     cout<<"ProcessImage cost time: "<<1000*time_used.count() <<"ms"<<endl;
}


void EDLine::DetectAnchors()
{
//     chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    const short* pImgGrad = pSobel_->GetPtrGrad();

    uint index = 0;
    AnchorSize_ = 0;

    for ( uint h=1,hend=ImageHeight_-1; h<hend; h+=ScanIntervals_ )
    {
        for ( uint w=1,wend=ImageWidth_-1; w<wend; w+=ScanIntervals_ )
        {
            index = h*ImageWidth_+w;

            if ( pImgGrad[index] < GradientThreshold_ )
                continue;

            const short grad = pImgGrad[index] - AnchorThreshold_;

            if ( pSobel_->IsHorizontal ( index ) )
            {
                if ( grad < pImgGrad[up ( index )] )
                    continue;
                if ( grad < pImgGrad[down ( index )] )
                    continue;
            }
            else
            {
                if ( grad < pImgGrad[left ( index )] )
                    continue;
                if ( grad < pImgGrad[right ( index )] )
                    continue;
            }

            pAnchorX_[AnchorSize_] = w;
            pAnchorY_[AnchorSize_] = h;

            ++AnchorSize_;
        }
    }

    if ( AnchorSize_ >= nExpectAnchorSize_ )
    {
        cout<<"\033[31m"<<"anchor size is larger than its maximal size. anchorsSize = "<<AnchorSize_
            <<", maximal size = "<<nExpectAnchorSize_ <<endl;
        exit ( 0 );
    }

//     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//     chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2 - t1 );
//     cout<<"DetectAnchors cost time: "<<1000*time_used.count() <<"ms"<<endl;
}


bool EDLine::DetectEdge ( uint anchorIndex )
{
    ushort x = pAnchorX_[anchorIndex];
    ushort y = pAnchorY_[anchorIndex];
    uint index = y*ImageWidth_+x;

    if ( pImgEdge_[index] == 1 )
        return false;


    IndexEdgeStart_ = nExpectPartSizeEdge_/2;
    IndexEdgeEnd_ = IndexEdgeStart_;

    const short* pImgGrad = pSobel_->GetPtrGrad();

    ushort LastX = 0, LastY = 0;
    Direction LastDirection = NON, ShouldGoDirection = NON;

    if ( pSobel_->IsHorizontal ( index ) ) //case1 Horizontal
    {
        LastDirection = RIGHT;
        while ( pImgGrad[index] > GradientThreshold_ && pImgEdge_[index] == 0 ) //case1.1
        {
            pImgEdge_[index] = 1;

            pPartEdgeX_[IndexEdgeEnd_] = x;
            pPartEdgeY_[IndexEdgeEnd_] = y;
            ++IndexEdgeEnd_;

            ShouldGoDirection = NON;
            if ( pSobel_->IsHorizontal ( index ) ) //case1.1.1
            {
                if ( LastDirection==UP || LastDirection==DOWN )
                {
                    ShouldGoDirection = x>LastX ? RIGHT : LEFT;
                }

                LastX = x;
                LastY = y;

                if ( LastDirection==RIGHT || ShouldGoDirection==RIGHT )
                {
                    if ( x==ImageWidth_-1 || y==0 || y==ImageHeight_-1 )
                        break;

                    SetNextPixel ( pImgGrad, RIGHT, index, x, y, LastDirection );
                }
                else if ( LastDirection==LEFT || ShouldGoDirection==LEFT )
                {
                    if ( x==0 || y==0 || y==ImageHeight_-1 )
                        break;

                    SetNextPixel ( pImgGrad, LEFT, index, x, y, LastDirection );
                }
            }  //end case1.1.1
            else  //case1.1.2
            {
                if ( LastDirection==RIGHT || LastDirection==LEFT )
                {
                    ShouldGoDirection = y>LastY ? DOWN : UP;
                }

                LastX = x;
                LastY = y;

                if ( LastDirection==DOWN || ShouldGoDirection==DOWN )
                {
                    if ( x==ImageWidth_-1 || x==0 || y==ImageHeight_-1 )
                        break;

                    SetNextPixel ( pImgGrad, DOWN, index, x, y, LastDirection );
                }
                else if ( LastDirection==UP || ShouldGoDirection==UP )
                {
                    if ( x==ImageWidth_-1 || x==0 || y==0 )
                        break;

                    SetNextPixel ( pImgGrad, UP, index, x, y, LastDirection );
                }
            }  //end case1.1.2
            index = y*ImageWidth_+x;
        } //end while


        x = pAnchorX_[anchorIndex];
        y = pAnchorY_[anchorIndex];
        index = y*ImageWidth_+x;

        LastDirection = LEFT;

        pImgEdge_[index] = 0;

        while ( pImgGrad[index] > GradientThreshold_ && pImgEdge_[index] == 0 ) //case1.2
        {
            pImgEdge_[index] = 1;

            pPartEdgeX_[IndexEdgeStart_] = x;
            pPartEdgeY_[IndexEdgeStart_] = y;
            --IndexEdgeStart_;

            ShouldGoDirection = NON;
            if ( pSobel_->IsHorizontal ( index ) ) //case1.2.1
            {
                if ( LastDirection==UP || LastDirection==DOWN )
                {
                    ShouldGoDirection = x>LastX ? RIGHT : LEFT;
                }

                LastX = x;
                LastY = y;

                if ( LastDirection==RIGHT || ShouldGoDirection==RIGHT )
                {
                    if ( x==ImageWidth_-1 || y==0 || y==ImageHeight_-1 )
                        break;

                    SetNextPixel ( pImgGrad, RIGHT, index, x, y, LastDirection );
                }
                else if ( LastDirection==LEFT || ShouldGoDirection==LEFT )
                {
                    if ( x==0 || y==0 || y==ImageHeight_-1 )
                        break;

                    SetNextPixel ( pImgGrad, LEFT, index, x, y, LastDirection );
                }
            }  //end case1.2.1
            else  //case1.2.2
            {
                if ( LastDirection==RIGHT || LastDirection==LEFT )
                {
                    ShouldGoDirection = y>LastY ? DOWN : UP;
                }

                LastX = x;
                LastY = y;

                if ( LastDirection==DOWN || ShouldGoDirection==DOWN )
                {
                    if ( x==ImageWidth_-1 || x==0 || y==ImageHeight_-1 )
                        break;

                    SetNextPixel ( pImgGrad, DOWN, index, x, y, LastDirection );
                }
                else if ( LastDirection==UP || ShouldGoDirection==UP )
                {
                    if ( x==ImageWidth_-1 || x==0 || y==0 )
                        break;

                    SetNextPixel ( pImgGrad, UP, index, x, y, LastDirection );
                }
            }  //end case1.2.2

            index = y*ImageWidth_+x;
        }  //end while
    }
    else  //case2 Vertical
    {
        LastDirection = DOWN;
        while ( pImgGrad[index] > GradientThreshold_ && pImgEdge_[index] == 0 ) //case2.1
        {
            pImgEdge_[index] = 1;

            pPartEdgeX_[IndexEdgeEnd_] = x;
            pPartEdgeY_[IndexEdgeEnd_] = y;
            ++IndexEdgeEnd_;

            ShouldGoDirection = NON;
            if ( pSobel_->IsHorizontal ( index ) ) //case2.1.1
            {
                if ( LastDirection==UP || LastDirection==DOWN )
                {
                    ShouldGoDirection = x>LastX ? RIGHT : LEFT;
                }

                LastX = x;
                LastY = y;

                if ( LastDirection==RIGHT || ShouldGoDirection==RIGHT )
                {
                    if ( x==ImageWidth_-1 || y==0 || y==ImageHeight_-1 )
                        break;

                    SetNextPixel ( pImgGrad, RIGHT, index, x, y, LastDirection );
                }
                else if ( LastDirection==LEFT || ShouldGoDirection==LEFT )
                {
                    if ( x==0 || y==0 || y==ImageHeight_-1 )
                        break;

                    SetNextPixel ( pImgGrad, LEFT, index, x, y, LastDirection );
                }
            }  //end case2.1.1
            else  //case2.1.2
            {
                if ( LastDirection==RIGHT || LastDirection==LEFT )
                {
                    ShouldGoDirection = y>LastY ? DOWN : UP;
                }

                LastX = x;
                LastY = y;

                if ( LastDirection==DOWN || ShouldGoDirection==DOWN )
                {
                    if ( x==ImageWidth_-1 || x==0 || y==ImageHeight_-1 )
                        break;

                    SetNextPixel ( pImgGrad, DOWN, index, x, y, LastDirection );
                }
                else if ( LastDirection==UP || ShouldGoDirection==UP )
                {
                    if ( x==ImageWidth_-1 || x==0 || y==0 )
                        break;

                    SetNextPixel ( pImgGrad, UP, index, x, y, LastDirection );
                }
            }  //end case2.1.2
            index = y*ImageWidth_+x;
        } //end while


        x = pAnchorX_[anchorIndex];
        y = pAnchorY_[anchorIndex];
        index = y*ImageWidth_+x;

        pImgEdge_[index] = 0;
        LastDirection = UP;

        while ( pImgGrad[index] > GradientThreshold_ && pImgEdge_[index] == 0 ) //case2.2
        {
            pImgEdge_[index] = 1;

            pPartEdgeX_[IndexEdgeStart_] = x;
            pPartEdgeY_[IndexEdgeStart_] = y;
            --IndexEdgeStart_;

            ShouldGoDirection = NON;
            if ( pSobel_->IsHorizontal ( index ) ) //case2.2.1
            {
                if ( LastDirection==UP|| LastDirection==DOWN )
                {
                    ShouldGoDirection = x>LastX ? RIGHT : LEFT;
                }

                LastX = x;
                LastY = y;

                if ( LastDirection==RIGHT || ShouldGoDirection==RIGHT )
                {
                    if ( x==ImageWidth_-1 || y==0 || y==ImageHeight_-1 )
                        break;

                    SetNextPixel ( pImgGrad, RIGHT, index, x, y, LastDirection );
                }
                else if ( LastDirection==LEFT || ShouldGoDirection==LEFT )
                {
                    if ( x==0 || y==0 || y==ImageHeight_-1 )
                        break;

                    SetNextPixel ( pImgGrad, LEFT, index, x, y, LastDirection );
                }
            }  //end case2.2.1
            else  //case2.2.2
            {
                if ( LastDirection==RIGHT || LastDirection==LEFT )
                {
                    ShouldGoDirection = y>LastY ? DOWN : UP;
                }

                LastX = x;
                LastY = y;

                if ( LastDirection==DOWN || ShouldGoDirection==DOWN )
                {
                    if ( x==ImageWidth_-1 || x==0 || y==ImageHeight_-1 )
                        break;

                    SetNextPixel ( pImgGrad, DOWN, index, x, y, LastDirection );
                }
                else if ( LastDirection==UP || ShouldGoDirection==UP )
                {
                    if ( x==ImageWidth_-1 || x==0 || y==0 )
                        break;

                    SetNextPixel ( pImgGrad, UP, index, x, y, LastDirection );
                }
            }  //end case2.2.2

            index = y*ImageWidth_+x;
        }  //end while
    }  //end case2


    if ( IndexEdgeEnd_>=nExpectPartSizeEdge_ || IndexEdgeStart_<=0 )
    {
        cout<<"\033[31m"<<"\nError, wrong value of array index! IndexEdgeStart_: "<<IndexEdgeStart_<<" ,IndexEdgeEnd_: "<<IndexEdgeEnd_<<endl;
        cout<<"Expected ranges of array index: "<<"0 <= IndexEdgeStart_ <= IndexEdgeEnd_ <= "<<nExpectPartSizeEdge_<<endl;
        cout<<"Please increase pPartEdgeX_'s length and pPartEdgeX_'s length!\n"<<endl;
        exit ( 0 );
    }

    if ( IndexEdgeEnd_ - IndexEdgeStart_ < MinEdgeLength_ + 1 )
        return false;

    return true;
}



inline double Dist ( uint x, uint y, const array<double, 2>& slopeIntercept, bool bHorizontal )
{
    if ( bHorizontal )
    {
        return fabs ( x*slopeIntercept[0] + slopeIntercept[1] - y );
    }
    else
    {
        return fabs ( y*slopeIntercept[0] + slopeIntercept[1] - x );
    }
}
void EDLine::DetectLines( uint edgeIndex )
{
    const short edgeLength = pEdgeS_[edgeIndex+1] - pEdgeS_[edgeIndex];
    if( edgeLength < InitLineLength_ )
        return;
    
    IndexEdgeStart_ = 0;
    IndexEdgeEnd_ = edgeLength;
    memcpy( pPartEdgeX_, pEdgeX_+pEdgeS_[edgeIndex], sizeof(ushort)*edgeLength );
    memcpy( pPartEdgeY_, pEdgeY_+pEdgeS_[edgeIndex], sizeof(ushort)*edgeLength );

    while( IndexEdgeStart_ + InitLineLength_ < IndexEdgeEnd_ )
    {
        IndexLineStart_ = nExpectPartSizeLine_/3;
        IndexLineEnd_ = IndexLineStart_;

        bool bHorizontal = true;
        double lineFitErr = 0.0;
        array<double,2> slopeIntercept;

        while( IndexEdgeStart_ + InitLineLength_ < IndexEdgeEnd_ )
        {
            bHorizontal = pSobel_->IsHorizontal ( pPartEdgeY_[IndexEdgeStart_]*ImageWidth_ + pPartEdgeX_[IndexEdgeStart_] );

            if ( bHorizontal )
            {
                lineFitErr = LeastSquareFit ( pPartEdgeX_,pPartEdgeY_, IndexEdgeStart_,slopeIntercept );
            }
            else
            {
                lineFitErr = LeastSquareFit ( pPartEdgeY_, pPartEdgeX_, IndexEdgeStart_,slopeIntercept );
            }

            if ( lineFitErr <= LineFitErrThreshold_ )
                break;

            IndexEdgeStart_ += SkipEdgePoint_;
        }

        if ( lineFitErr > LineFitErrThreshold_ )
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

                for ( uint i=0; i<InitLineLength_; ++i )
                {
                    pPartLineX_[IndexLineEnd_] = pPartEdgeX_[IndexEdgeStart_];
                    pPartLineY_[IndexLineEnd_] = pPartEdgeY_[IndexEdgeStart_];

                    ++IndexLineEnd_;
                    ++IndexEdgeStart_;
                }
            }
            else
            {
                if ( bHorizontal )
                {
                    LeastSquareFit ( pPartLineX_, pPartLineY_, newOffsetS, IndexLineEnd_, slopeIntercept );
                }
                else
                {
                    LeastSquareFit ( pPartLineY_, pPartLineX_, newOffsetS, IndexLineEnd_, slopeIntercept );
                }
            }

            newOffsetS = IndexLineEnd_;

            const double temp = LineFitErrThreshold_ * sqrt ( slopeIntercept[0]*slopeIntercept[0] + 1.0 );
            int outlierCount = 0;

            while ( IndexEdgeStart_ < IndexEdgeEnd_ )
            {
                double pointToLineDis = Dist ( pPartEdgeX_[IndexEdgeStart_],pPartEdgeY_[IndexEdgeStart_],
                                               slopeIntercept, bHorizontal );

                pPartLineX_[IndexLineEnd_] = pPartEdgeX_[IndexEdgeStart_];
                pPartLineY_[IndexLineEnd_] = pPartEdgeY_[IndexEdgeStart_];

                ++IndexLineEnd_;
                ++IndexEdgeStart_;

                if ( pointToLineDis > temp )
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

        bool bStored = IndexLineEnd_ - IndexLineStart_ < MinLineLength_ ? false : true;
        if ( bStored )
            bStored = StoreLine ( bHorizontal, slopeIntercept );
    }
}

double EDLine::LeastSquareFit ( const ushort* pFirstCoord, const ushort* pSecondCoord,
                                uint offsetS, array<double,2>& slopeIntercept )
{
    FitParams_.Reset();

    uint offset = offsetS;

    for ( uchar i=0; i<InitLineLength_; ++i )
    {
        FitParams_.AddPoint ( pFirstCoord[offsetS], pSecondCoord[offsetS] );

        ++offsetS;
    }

    FitParams_.Solve ( slopeIntercept );

    double fitError = 0.0;
    for ( uchar i=0; i<InitLineLength_; ++i )
    {
        const double dist = double ( pSecondCoord[offset] ) - double ( pFirstCoord[offset] ) * slopeIntercept[0] - slopeIntercept[1];
        fitError += dist*dist;

        ++offset;
    }

    return sqrt ( fitError );
}

void EDLine::LeastSquareFit ( const ushort* pFirstCoord, const ushort* pSecondCoord,
                              uint newOffsetS, uint offsetE, array<double,2>& slopeIntercept )
{
    int newLength = offsetE - newOffsetS;
    assert ( newLength > 0 );

    for ( int i=0; i<newLength; ++i )
    {
        FitParams_.AddPoint ( pFirstCoord[newOffsetS], pSecondCoord[newOffsetS] );

        ++newOffsetS;
    }

    FitParams_.Solve ( slopeIntercept );
}

bool EDLine::StoreLine ( const bool bHorizontal, const std::array<double,2>& slopeIntercept )
{
    const double temp = 1.0/sqrt ( slopeIntercept[0]*slopeIntercept[0] + 1.0 );
    array<double,3> lineEquation;
    if ( bHorizontal )
    {
        lineEquation[0] = temp*slopeIntercept[0];
        lineEquation[1] = -temp;
        lineEquation[2] = temp*slopeIntercept[1];
    }
    else
    {
        lineEquation[0] = -temp;
        lineEquation[1] = temp*slopeIntercept[0];
        lineEquation[2] = temp*slopeIntercept[1];
    }

    float direction = atan2 ( lineEquation[0], lineEquation[1] );

    const double a = lineEquation[1]*lineEquation[1];
    const double b = lineEquation[0]*lineEquation[0];
    const double c = lineEquation[0]*lineEquation[1];
    const double d = lineEquation[2]*lineEquation[0];
    const double e = lineEquation[2]*lineEquation[1];

    const uint Px1 = pPartLineX_[IndexLineStart_];
    const uint Py1 = pPartLineY_[IndexLineStart_];
    const uint Px2 = pPartLineX_[IndexLineEnd_-1];
    const uint Py2 = pPartLineY_[IndexLineEnd_-1];

    array<float,4> lineEndPoints;
    lineEndPoints[0] = a*Px1 - c*Py1 -d;
    lineEndPoints[1] = b*Py1 - c*Px1 -e;
    lineEndPoints[2] = a*Px2 - c*Py2 -d;
    lineEndPoints[3] = b*Py2 - c*Px2 -e;

    vLineEquations_.emplace_back ( lineEquation );
    vLineEndpoints_.emplace_back ( lineEndPoints );
    vLineDirections_.emplace_back ( direction );


    pLineS_[CountLine_] = OffsetLine_;
    short length = IndexLineEnd_ -  IndexLineStart_;

    memcpy ( pLineX_ + OffsetLine_, pPartLineX_ + IndexLineStart_, length*sizeof ( ushort ) );
    memcpy ( pLineY_ + OffsetLine_, pPartLineY_ + IndexLineStart_, length*sizeof ( ushort ) );

    OffsetLine_ += length;
    ++CountLine_;

    return true;
}

}
// kate: indent-mode cstyle; replace-tabs on; 
