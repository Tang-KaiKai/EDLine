#include "EDLine.h"
#include "Utility.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utility.hpp>

#include <iostream>

using namespace std;
using namespace cv;

using namespace Feature;



int main ( int argc, char** argv )
{
    if( argc != 3 )
    {
        cout<<"Usage: ./time_cost path_to_image1 path_to_image2"<<endl;
        return 0;
    }
    
    const string name1 = argv[1];
    const string name2 = argv[2];
    
    Mat image10 = imread ( name1, IMREAD_GRAYSCALE );
    if( image10.empty() )
    {
        cout<<"Can't read image in path: "<<name1<<endl;
        return 0;
    }
    
    Mat image20 = imread ( name2, IMREAD_GRAYSCALE );
    if( image20.empty() )
    {
        cout<<"Can't read image in path: "<<name2<<endl;
        return 0;
    }
    
    Mat image1;
    cv::GaussianBlur ( image10,image1,cv::Size ( 5,5 ),1.0 );
    
    Mat image2;
    cv::GaussianBlur ( image20,image2,cv::Size ( 5,5 ),1.0 );

    const int width = image1.cols;
    const int height = image1.rows;


    PixelChains edgeChains;
    PixelChains lineChains;

    EDLine edline ( width, height );

    int n=1000;
    double t = ( double ) cv::getTickCount();
    for ( int i=0; i<n; ++i )
    {
        if ( i%2==0 )
        {
            edline.Detect ( image1.data );
        }
        else
        {
            edline.Detect ( image2.data );
        }
    }
    t = ( ( double ) cv::getTickCount() - t ) /cv::getTickFrequency();
    cout<<"Detect cost time: "<<1000*t/n<<"ms"<<endl;

    
    edline.GetEdges ( edgeChains );
    edline.GetLines( lineChains );
    
//     edline.ShowAnchor();
    
//     ShowImageGrad ( edline.GetPtrGrad(),width,height, false,100 );
    ShowPixelChain ( edgeChains,width,height, Mat(),false,"edge",100 );
    ShowPixelChain ( lineChains,width,height, Mat(),false, "line",100 );
    
    waitKey();
}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
