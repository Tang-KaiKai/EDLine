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
    if( argc != 2 )
    {
        cout<<"Usage: ./show path_to_image"<<endl;
        return 0;
    }
    
    const string name = argv[1];
    Mat image0 = imread ( name, IMREAD_GRAYSCALE );
    if( image0.empty() )
    {
        cout<<"Can't read image in path: "<<name<<endl;
        return 0;
    }
    
    Mat image;
    cv::GaussianBlur ( image0,image,cv::Size ( 5,5 ),1.0 );
    
    const int width = image.cols;
    const int height = image.rows;

    
    EDLine edline ( width, height );
    edline.Detect ( image.data );
    
    
    PixelChains edgeChains, lineChains;
    edline.GetEdges ( edgeChains );
    edline.GetLines( lineChains );
    
    
//     edline.ShowAnchor( false,-10 );
//     ShowImageGrad ( edline.GetPtrGrad(),width,height, false,-10 );
    ShowPixelChain ( edgeChains,width,height, Mat(),false,"edge",-10 );
    ShowPixelChain ( lineChains,width,height, Mat(),false, "line" ,-10);
    
    waitKey();
}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
