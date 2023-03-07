#pragma once

#include <string.h>

#define AVX2 // AVX2 or SSE2 or OpenCV or Common

#if defined(AVX2) || defined(SSE2)
    #include <emmintrin.h>
    #include <immintrin.h>
#endif

#ifdef OpenCV
    #include <opencv2/core/core.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
    #include <opencv2/features2d/features2d.hpp>
    #include <opencv2/highgui/highgui.hpp>
    #include <opencv2/core/utility.hpp>

    using cv::Mat;
#endif

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

    bool IsHorizontal( uint index ) const
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


void GradientOperator::Initialize()
{
    raw_ = new uchar[width_ * 3];

    dx_dy_grad_ = new short[width_ * 3];
    memset( dx_dy_grad_, 0, width_ * sizeof( short ) * 3 );


    const int pixelsNum = width_ * height_;
    pImgDx_ = new short[pixelsNum];
    pImgDy_ = new short[pixelsNum];
    pImgGrad_ = new short[pixelsNum];

    memset( pImgDx_, 0, pixelsNum * sizeof( short ) );
    memset( pImgDy_, 0, pixelsNum * sizeof( short ) );
    memset( pImgGrad_, 0, pixelsNum * sizeof( short ) );
}


GradientOperator::~GradientOperator()
{
    delete []raw_;
    delete []dx_dy_grad_;

    delete []pImgDx_;
    delete []pImgDy_;
    delete []pImgGrad_;
}


void GradientOperator::Compute( const uchar *pImg )
{
#if defined AVX2
    Compute_AVX( pImg );
    return;

#elif defined SSE2
    Compute_SSE( pImg );
    return;

#elif defined OpenCV
    Compute_OpenCV( pImg );
    return;

#elif defined Common
    Compute_Common( pImg );
    return;
#else
    cout << " \n************************************************************************************\n"
         << "Error: You should choose conditional compilation options: AVX2 or SSE2 or OpenCV or Common "
         << " \nPlease set it in Sobel.hpp"
         << "\n************************************************************************************\n" << endl;
    exit( 0 );
#endif
}


#ifdef AVX2
void GradientOperator::Compute_AVX( const uchar *pImg )
{
    const int blockSize_AVX = 16;
    const int area_AVX = width_ - ( width_ - 1 ) % blockSize_AVX;
    const int widthBytesShort = width_ * sizeof( short );

    uchar *u = raw_;                  //up
    uchar *m = raw_ + width_;         //mid
    uchar *d = raw_ + width_ * 2;     //down

    short *dx = dx_dy_grad_;
    short *dy = dx_dy_grad_ + width_;
    short *g = dx_dy_grad_ + width_ * 2;

    for ( int y = 1; y < height_ - 1; ++y )
    {
        if ( y == 1 )
        {
            memcpy( raw_, pImg, width_ * 3 );
        }
        else
        {
            uchar *temp = u;
            u = m;
            m = d;
            d = temp;

            memcpy( d, pImg + width_ * ( y + 1 ), width_ );
        }

        for ( int x = 1; x < area_AVX; x += blockSize_AVX )
        {
            __m256i u0 = _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( u + x - 1 ) ) );
            __m256i u1 = _mm256_slli_epi16( _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( u + x ) ) ), 1 );
            __m256i u2 = _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( u + x + 1 ) ) );

            __m256i m0 = _mm256_slli_epi16( _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( m + x - 1 ) ) ), 1 );
            __m256i m2 = _mm256_slli_epi16( _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( m + x + 1 ) ) ), 1 );

            __m256i d0 = _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( d + x - 1 ) ) );
            __m256i d1 = _mm256_slli_epi16( _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( d + x ) ) ), 1 );
            __m256i d2 = _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( d + x + 1 ) ) );


            __m256i dx16 = _mm256_subs_epi16( _mm256_adds_epi16( _mm256_adds_epi16( u2, m2 ), d2 ),
                                              _mm256_adds_epi16( _mm256_adds_epi16( u0, m0 ), d0 ) );

            __m256i dy16 = _mm256_subs_epi16( _mm256_adds_epi16( _mm256_adds_epi16( d0, d1 ), d2 ),
                                              _mm256_adds_epi16( _mm256_adds_epi16( u0, u1 ), u2 ) );

            __m256i g16 = _mm256_srai_epi16( _mm256_adds_epi16( _mm256_abs_epi16( dx16 ), _mm256_abs_epi16( dy16 ) ), 2 );


            _mm256_storeu_si256( ( __m256i * )( dx + x ), dx16 );
            _mm256_storeu_si256( ( __m256i * )( dy + x ), dy16 );
            _mm256_storeu_si256( ( __m256i * )( g + x ), g16 );
        }


        for ( int x = area_AVX; x < width_ - 1; ++x )
        {
            dx[x] = u[x + 1] - u[x - 1] + ( m[x + 1] - m[x - 1] ) * 2 + d[x + 1] - d[x - 1];
            dy[x] = d[x - 1] + d[x + 1] + ( d[x] - u[x] ) * 2 - u[x - 1] - u[x + 1];
            g[x] = ( Abs( dx[x] ) + Abs( dy[x] ) ) >> 2;
        }

        memcpy( pImgDx_ + y * width_, dx, widthBytesShort );
        memcpy( pImgDy_ + y * width_, dy, widthBytesShort );
        memcpy( pImgGrad_ + y * width_, g, widthBytesShort );
    }
}
#endif


#ifdef SSE2
void GradientOperator::Compute_SSE( const uchar *pImg )
{
    const int blockSize_SSE = 8;
    const int area_SSE = width_ - ( width_ - 1 ) % blockSize_SSE;
    const int widthBytesShort = width_ * sizeof( short );

    uchar *u = raw_;
    uchar *m = raw_ + width_;
    uchar *d = raw_ + width_ * 2;

    short *dx = dx_dy_grad_;
    short *dy = dx_dy_grad_ + width_;
    short *g = dx_dy_grad_ + width_ * 2;

    __m128i zero = _mm_setzero_si128();

    for ( int y = 1; y < height_ - 1; ++y )
    {
        if ( y == 1 )
        {
            memcpy( raw_, pImg, width_ * 3 );
        }
        else
        {
            uchar *temp = u;
            u = m;
            m = d;
            d = temp;

            memcpy( d, pImg + width_ * ( y + 1 ), width_ );
        }

        for ( int x = 1; x < area_SSE; x += blockSize_SSE )
        {
            __m128i u0 = _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( u + x - 1 ) ), zero );
            __m128i u1 = _mm_slli_epi16( _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( u + x ) ), zero ), 1 );
            __m128i u2 = _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( u + x + 1 ) ), zero );

            __m128i m0 = _mm_slli_epi16( _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( m + x - 1 ) ), zero ), 1 );
            __m128i m2 = _mm_slli_epi16( _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( m + x + 1 ) ), zero ), 1 );

            __m128i d0 = _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( d + x - 1 ) ), zero );
            __m128i d1 = _mm_slli_epi16( _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( d + x ) ), zero ), 1 );
            __m128i d2 = _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( d + x + 1 ) ), zero );


            __m128i dx8 = _mm_subs_epi16( _mm_adds_epi16( _mm_adds_epi16( u2, m2 ), d2 ),
                                          _mm_adds_epi16( _mm_adds_epi16( u0, m0 ), d0 ) );

            __m128i dy8 = _mm_subs_epi16( _mm_adds_epi16( _mm_adds_epi16( d0, d1 ), d2 ),
                                          _mm_adds_epi16( _mm_adds_epi16( u0, u1 ), u2 ) );

            __m128i g8 = _mm_srai_epi16( _mm_adds_epi16( _mm_abs_epi16( dx8 ), _mm_abs_epi16( dy8 ) ), 2 );

            _mm_storeu_si128( ( __m128i * )( dx + x ), dx8 );
            _mm_storeu_si128( ( __m128i * )( dy + x ), dy8 );
            _mm_storeu_si128( ( __m128i * )( g + x ), g8 );
        }

        for ( int x = area_SSE; x < width_ - 1; ++x )
        {
            dx[x] = u[x + 1] - u[x - 1] + ( m[x + 1] - m[x - 1] ) * 2 + d[x + 1] - d[x - 1];
            dy[x] = d[x - 1] + d[x + 1] + ( d[x] - u[x] ) * 2 - u[x - 1] - u[x + 1];
            g[x] = ( Abs( dx[x] ) + Abs( dy[x] ) ) >> 2;
        }

        memcpy( pImgDx_ + y * width_, dx, widthBytesShort );
        memcpy( pImgDy_ + y * width_, dy, widthBytesShort );
        memcpy( pImgGrad_ + y * width_, g, widthBytesShort );
    }
}
#endif


#ifdef OpenCV
void GradientOperator::Compute_OpenCV( const uchar *pImg )
{
    Mat Img( height_, width_, CV_8UC1 );
    memcpy( Img.data, pImg, height_ * width_ );

    Mat ImgDx( height_, width_, CV_16SC1, pImgDx_ );
    Mat ImgDy( height_, width_, CV_16SC1, pImgDy_ );
    Mat ImgGrad( height_, width_, CV_16SC1, pImgGrad_ );

    cv::Sobel( Img, ImgDx, CV_16SC1, 1, 0 );
    cv::Sobel( Img, ImgDy, CV_16SC1, 0, 1 );

    Mat ImgDxAbs = cv::abs( ImgDx );
    Mat ImgDyAbs = cv::abs( ImgDy );

    cv::add( ImgDxAbs, ImgDyAbs, ImgGrad );
    ImgGrad /= 4;
}
#endif


#ifdef Common
void GradientOperator::Compute_Common( const uchar *pImg )
{
    const int widthBytesShort = width_ * sizeof( short );

    uchar *u = raw_;
    uchar *m = raw_ + width_;
    uchar *d = raw_ + width_ * 2;

    short *dx = dx_dy_grad_;
    short *dy = dx_dy_grad_ + width_;
    short *g = dx_dy_grad_ + width_ * 2;

    for ( int y = 1; y < height_ - 1; ++y )
    {
        if ( y == 1 )
        {
            memcpy( raw_, pImg, width_ * 3 );
        }
        else
        {
            uchar *temp = u;
            u = m;
            m = d;
            d = temp;

            memcpy( d, pImg + width_ * ( y + 1 ), width_ );
        }

        for ( int x = 1; x < width_ - 1; ++x )
        {
            dx[x] = u[x + 1] - u[x - 1] + ( m[x + 1] - m[x - 1] ) * 2 + d[x + 1] - d[x - 1];
            dy[x] = d[x - 1] + d[x + 1] + ( d[x] - u[x] ) * 2 - u[x - 1] - u[x + 1];
            g[x] = ( Abs( dx[x] ) + Abs( dy[x] ) ) >> 2;
        }

        memcpy( pImgDx_ + y * width_, dx, widthBytesShort );
        memcpy( pImgDy_ + y * width_, dy, widthBytesShort );
        memcpy( pImgGrad_ + y * width_, g, widthBytesShort );
    }
}
#endif

}
// kate: indent-mode cstyle; replace-tabs on;
