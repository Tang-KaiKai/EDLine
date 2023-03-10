#pragma once

namespace Feature
{

class GradientOperator
{
    typedef unsigned char uchar;

public:

    GradientOperator() = delete;

    GradientOperator( int width, int height ) :
        width_( width ), height_( height )
    {
        Initialize();
    }

    ~GradientOperator();

public:

    void Compute( const uchar *pImg );

    short *GetPtrDx() const
    {
        return pImgDx_;
    }

    short *GetPtrDy() const
    {
        return pImgDy_;
    }

    short *GetPtrGrad() const
    {
        return pImgGrad_;
    }

    bool IsHorizontal( int index ) const
    {
        return Abs( pImgDx_[index] ) < Abs( pImgDy_[index] );
    }

private:

    void Initialize();

    short Abs( short a ) const
    {
        return ( a ^ ( a >> 15 ) ) - ( a >> 15 );
    }

    void Compute_AVX( const uchar *pImg );

    void Compute_SSE( const uchar *pImg );

    void Compute_OpenCV( const uchar *pImg );

    void Compute_Common( const uchar *pImg );

private:

    const int width_ = 1280;
    const int height_ = 720;

private:

    uchar *raw_ = nullptr;
    short *dx_dy_grad_ = nullptr;

    short *pImgDx_ = nullptr;
    short *pImgDy_ = nullptr;
    short *pImgGrad_ = nullptr;
};

}
