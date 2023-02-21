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
        ImgWidth( width ), ImgHeight( height )
    {
        Initialize();
    }

    ~GradientOperator();

public:

    void Compute( const uchar *pImg );

    short *GetPtrDx() const
    {
        return pImgDx;
    }

    short *GetPtrDy() const
    {
        return pImgDy;
    }

    short *GetPtrGrad() const
    {
        return pImgGrad;
    }

    bool IsHorizontal( uint index ) const
    {
        return Abs( pImgDx[index] ) < Abs( pImgDy[index] );
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

    const int ImgWidth = 1280;
    const int ImgHeight = 720;

private:

    uchar *raw = nullptr;
    short *dx_dy_grad = nullptr;

    short *pImgDx = nullptr;
    short *pImgDy = nullptr;
    short *pImgGrad = nullptr;
};


void GradientOperator::Initialize()
{
    raw = new uchar[ImgWidth * 3];

    dx_dy_grad = new short[ImgWidth * 3];
    memset( dx_dy_grad, 0, ImgWidth * sizeof( short ) * 3 );


    const int pixelsNum = ImgWidth * ImgHeight;
    pImgDx = new short[pixelsNum];
    pImgDy = new short[pixelsNum];
    pImgGrad = new short[pixelsNum];

    memset( pImgDx, 0, pixelsNum * sizeof( short ) );
    memset( pImgDy, 0, pixelsNum * sizeof( short ) );
    memset( pImgGrad, 0, pixelsNum * sizeof( short ) );
}


GradientOperator::~GradientOperator()
{
    delete []raw;
    delete []dx_dy_grad;

    delete []pImgDx;
    delete []pImgDy;
    delete []pImgGrad;
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
    const int area_AVX = ImgWidth - ( ImgWidth - 1 ) % blockSize_AVX;
    const int widthBytesShort = ImgWidth * sizeof( short );

    uchar *up = raw;
    uchar *mid = raw + ImgWidth;
    uchar *down = raw + ImgWidth * 2;

    short *dx = dx_dy_grad;
    short *dy = dx_dy_grad + ImgWidth;
    short *grad = dx_dy_grad + ImgWidth * 2;

    for ( int y = 1; y < ImgHeight - 1; ++y )
    {
        if ( y == 1 )
        {
            memcpy( raw, pImg, ImgWidth * 3 );
        }
        else
        {
            uchar *temp = up;
            up = mid;
            mid = down;
            down = temp;

            memcpy( down, pImg + ImgWidth * ( y + 1 ), ImgWidth );
        }

        for ( int x = 1; x < area_AVX; x += blockSize_AVX )
        {
            __m256i up0 = _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( up + x - 1 ) ) );
            __m256i up1 = _mm256_slli_epi16( _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( up + x ) ) ), 1 );
            __m256i up2 = _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( up + x + 1 ) ) );

            __m256i mid0 = _mm256_slli_epi16( _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( mid + x - 1 ) ) ), 1 );
            __m256i mid2 = _mm256_slli_epi16( _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( mid + x + 1 ) ) ), 1 );

            __m256i down0 = _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( down + x - 1 ) ) );
            __m256i down1 = _mm256_slli_epi16( _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( down + x ) ) ), 1 );
            __m256i down2 = _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )( down + x + 1 ) ) );


            __m256i dx16 = _mm256_subs_epi16( _mm256_adds_epi16( _mm256_adds_epi16( up2, mid2 ), down2 ),
                                              _mm256_adds_epi16( _mm256_adds_epi16( up0, mid0 ), down0 ) );

            __m256i dy16 = _mm256_subs_epi16( _mm256_adds_epi16( _mm256_adds_epi16( down0, down1 ), down2 ),
                                              _mm256_adds_epi16( _mm256_adds_epi16( up0, up1 ), up2 ) );

            __m256i g16 = _mm256_srai_epi16( _mm256_adds_epi16( _mm256_abs_epi16( dx16 ), _mm256_abs_epi16( dy16 ) ), 2 );


            _mm256_storeu_si256( ( __m256i * )( dx + x ), dx16 );
            _mm256_storeu_si256( ( __m256i * )( dy + x ), dy16 );
            _mm256_storeu_si256( ( __m256i * )( grad + x ), g16 );
        }


        for ( int x = area_AVX; x < ImgWidth - 1; ++x )
        {
            dx[x] = up[x + 1] - up[x - 1] + ( mid[x + 1] - mid[x - 1] ) * 2 + down[x + 1] - down[x - 1];
            dy[x] = down[x - 1] + down[x + 1] + ( down[x] - up[x] ) * 2 - up[x - 1] - up[x + 1];
            grad[x] = ( Abs( dx[x] ) + Abs( dy[x] ) ) >> 2;
        }

        memcpy( pImgDx + y * ImgWidth, dx, widthBytesShort );
        memcpy( pImgDy + y * ImgWidth, dy, widthBytesShort );
        memcpy( pImgGrad + y * ImgWidth, grad, widthBytesShort );
    }
}
#endif


#ifdef SSE2
void GradientOperator::Compute_SSE( const uchar *pImg )
{
    const int blockSize_SSE = 8;
    const int area_SSE = ImgWidth - ( ImgWidth - 1 ) % blockSize_SSE;
    const int widthBytesShort = ImgWidth * sizeof( short );

    uchar *up = raw;
    uchar *mid = raw + ImgWidth;
    uchar *down = raw + ImgWidth * 2;

    short *dx = dx_dy_grad;
    short *dy = dx_dy_grad + ImgWidth;
    short *grad = dx_dy_grad + ImgWidth * 2;

    __m128i zero = _mm_setzero_si128();

    for ( int y = 1; y < ImgHeight - 1; ++y )
    {
        if ( y == 1 )
        {
            memcpy( raw, pImg, ImgWidth * 3 );
        }
        else
        {
            uchar *temp = up;
            up = mid;
            mid = down;
            down = temp;

            memcpy( down, pImg + ImgWidth * ( y + 1 ), ImgWidth );
        }

        for ( int x = 1; x < area_SSE; x += blockSize_SSE )
        {
            __m128i up0 = _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( up + x - 1 ) ), zero );
            __m128i up1 = _mm_slli_epi16( _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( up + x ) ), zero ), 1 );
            __m128i up2 = _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( up + x + 1 ) ), zero );

            __m128i mid0 = _mm_slli_epi16( _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( mid + x - 1 ) ), zero ), 1 );
            __m128i mid2 = _mm_slli_epi16( _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( mid + x + 1 ) ), zero ), 1 );

            __m128i down0 = _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( down + x - 1 ) ), zero );
            __m128i down1 = _mm_slli_epi16( _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( down + x ) ), zero ), 1 );
            __m128i down2 = _mm_unpacklo_epi8( _mm_loadl_epi64( ( __m128i * )( down + x + 1 ) ), zero );


            __m128i dx8 = _mm_subs_epi16( _mm_adds_epi16( _mm_adds_epi16( up2, mid2 ), down2 ),
                                          _mm_adds_epi16( _mm_adds_epi16( up0, mid0 ), down0 ) );

            __m128i dy8 = _mm_subs_epi16( _mm_adds_epi16( _mm_adds_epi16( down0, down1 ), down2 ),
                                          _mm_adds_epi16( _mm_adds_epi16( up0, up1 ), up2 ) );

            __m128i g8 = _mm_srai_epi16( _mm_adds_epi16( _mm_abs_epi16( dx8 ), _mm_abs_epi16( dy8 ) ), 2 );

            _mm_storeu_si128( ( __m128i * )( dx + x ), dx8 );
            _mm_storeu_si128( ( __m128i * )( dy + x ), dy8 );
            _mm_storeu_si128( ( __m128i * )( grad + x ), g8 );
        }

        for ( int x = area_SSE; x < ImgWidth - 1; ++x )
        {
            dx[x] = up[x + 1] - up[x - 1] + ( mid[x + 1] - mid[x - 1] ) * 2 + down[x + 1] - down[x - 1];
            dy[x] = down[x - 1] + down[x + 1] + ( down[x] - up[x] ) * 2 - up[x - 1] - up[x + 1];
            grad[x] = ( Abs( dx[x] ) + Abs( dy[x] ) ) >> 2;
        }

        memcpy( pImgDx + y * ImgWidth, dx, widthBytesShort );
        memcpy( pImgDy + y * ImgWidth, dy, widthBytesShort );
        memcpy( pImgGrad + y * ImgWidth, grad, widthBytesShort );
    }
}
#endif


#ifdef OpenCV
void GradientOperator::Compute_OpenCV( const uchar *pImg )
{
    Mat Img( ImgHeight, ImgWidth, CV_8UC1 );
    memcpy( Img.data, pImg, ImgHeight * ImgWidth );

    Mat ImgDx( ImgHeight, ImgWidth, CV_16SC1, pImgDx );
    Mat ImgDy( ImgHeight, ImgWidth, CV_16SC1, pImgDy );
    Mat ImgGrad( ImgHeight, ImgWidth, CV_16SC1, pImgGrad );

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
    const int widthBytesShort = ImgWidth * sizeof( short );

    uchar *up = raw;
    uchar *mid = raw + ImgWidth;
    uchar *down = raw + ImgWidth * 2;

    short *dx = dx_dy_grad;
    short *dy = dx_dy_grad + ImgWidth;
    short *grad = dx_dy_grad + ImgWidth * 2;

    for ( int y = 1; y < ImgHeight - 1; ++y )
    {
        if ( y == 1 )
        {
            memcpy( raw, pImg, ImgWidth * 3 );
        }
        else
        {
            uchar *temp = up;
            up = mid;
            mid = down;
            down = temp;

            memcpy( down, pImg + ImgWidth * ( y + 1 ), ImgWidth );
        }

        for ( int x = 1; x < ImgWidth - 1; ++x )
        {
            dx[x] = up[x + 1] - up[x - 1] + ( mid[x + 1] - mid[x - 1] ) * 2 + down[x + 1] - down[x - 1];
            dy[x] = down[x - 1] + down[x + 1] + ( down[x] - up[x] ) * 2 - up[x - 1] - up[x + 1];
            grad[x] = ( Abs( dx[x] ) + Abs( dy[x] ) ) >> 2;
        }

        memcpy( pImgDx + y * ImgWidth, dx, widthBytesShort );
        memcpy( pImgDy + y * ImgWidth, dy, widthBytesShort );
        memcpy( pImgGrad + y * ImgWidth, grad, widthBytesShort );
    }
}
#endif

}
// kate: indent-mode cstyle; replace-tabs on;
