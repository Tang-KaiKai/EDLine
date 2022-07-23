#pragma once

#include "EDLine.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>

#include <iostream>
#include <opencv2/imgproc.hpp>

#include <string>


namespace Feature
{

void EDLine::ShowAnchor ( bool store, int showTime )
{
    cv::Mat ImgAnchor = cv::Mat::zeros ( ImageHeight_, ImageWidth_, CV_8UC3 );

    for ( uint i = 0; i<AnchorSize_; ++i )
    {
        uint x= pAnchorX_[i];
        uint y= pAnchorY_[i];

        cv::circle ( ImgAnchor, cv::Point ( x,y ), 2, cv::Scalar ( 255,0,0 ),-1 );
    }

    cv::imshow ( "ImgAnchor", ImgAnchor );
    
    if ( showTime == -1 )
    {
        cv::waitKey (500);
        getchar();
    }
    else if( showTime>=0 )
    {
        cv::waitKey ( showTime );
    }

    if ( store )
        cv::imwrite ( "anchor.png",ImgAnchor );
}


void ShowImageGrad ( const short* pGrad, uint width,uint height, bool store = false, int showTime = 0 )
{
    cv::Mat image = cv::Mat::zeros ( height, width, CV_8UC1 );

    uchar* imgG = image.data;

    int index = 0;
    for ( uint i=0; i<height; ++i )
    {
        for ( uint j=0; j<width; ++j )
        {
            imgG[index] = pGrad[index];
            ++index;
        }
    }

    imshow ( "grad",image );

    if ( showTime == -1 )
    {
        cv::waitKey (500);
        getchar();
    }
    else if( showTime>=0 )
    {
        cv::waitKey ( showTime );
    }

    if ( store )
        cv::imwrite ( "grad.png",image );
}


void ShowPixelChain ( const PixelChains& chains, uint width,uint height,
                      const cv::Mat& src = cv::Mat(), bool store = false, const std::string& name = "edge", int showTime = 0 )
{
    std::cout<<"\nChain's num: "<<chains.ChainsNum<<std::endl;

    cv::Mat image;
    if ( src.empty() )
    {
        image = cv::Mat::zeros ( height, width, CV_8UC3 );
    }
    else
    {
        cvtColor ( src,image,cv::COLOR_GRAY2BGR );
    }

    srand ( ( unsigned ) time ( 0 ) );

    int lowest=100, highest=255;
    int range= ( highest-lowest )+1;
    int r, g, b; //the color of edges

    for ( unsigned int i=0; i<chains.ChainsNum; i++ )
    {
        r = lowest+int ( rand() %range );
        g = lowest+int ( rand() %range );
        b = lowest+int ( rand() %range );

        for ( unsigned int indexInCors = chains.vStartIds[i]; indexInCors<chains.vStartIds[i+1]; indexInCors++ )
        {
            if ( indexInCors == chains.vStartIds[i] )
                cv::circle ( image, cv::Point ( chains.vXcoords[indexInCors],chains.vYcoords[indexInCors] ),
                             2, cv::Scalar ( 0,255,0 ),-1 );

            image.at<cv::Vec3b> ( chains.vYcoords[indexInCors],chains.vXcoords[indexInCors] ) = cv::Vec3b ( g,b,r );
        }
        
//         std::cout<<"index:"<<i<<",size: "<<chains.vStartIds[i+1]-chains.vStartIds[i]<<std::endl;
    }

    imshow ( name,image );

    if ( showTime == -1 )
    {
        cv::waitKey (500);
        getchar();
    }
    else if( showTime>=0 )
    {
        cv::waitKey ( showTime );
    }

    if ( store )
    {
        std::string imageName = name + ".png";
        cv::imwrite ( imageName,image );
    }
}

}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
