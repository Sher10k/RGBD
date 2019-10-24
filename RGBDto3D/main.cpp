#include <iostream>
#include <vector>
#include <set>
#include <iterator>
#include <math.h>

#include <Eigen/Eigen>
#include <Open3D/Open3D.h>
#include <Open3D/Geometry/Geometry.h>
#include <Open3D/Geometry/Geometry3D.h>
#include <Open3D/Geometry/PointCloud.h>
#include <Open3D/Geometry/Octree.h>
#include <Open3D/Visualization/Visualizer/Visualizer.h>
#include <Open3D/Visualization/Visualizer/ViewControlWithEditing.h>
#include <Open3D/Geometry/BoundingVolume.h>

#include <opencv2/core.hpp>
#include <opencv2//highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace open3d;

void PaintGrid( visualization::Visualizer &vis, double Gsize, double Gstep )
{
    double R = 0.005, H = Gsize;
    int resolution = 3, split = 2;
    for ( double i = 0; i <= Gsize; i += Gstep )
    {
            // XoZ || oX
        auto plane1 = geometry::TriangleMesh::CreateCylinder( R, H, resolution, split );
        plane1->PaintUniformColor( Vector3d( 0.0, 0.0, 0.0 ) );
        Matrix4d Rt;
        Rt << 0.0, 0.0, 1.0, 0.0,
              0.0, 1.0, 0.0, 0.0,
              1.0, 0.0, 0.0,  i,
              0.0, 0.0, 0.0, 1.0;
        plane1->Transform( Rt );
        plane1->ComputeVertexNormals();
        vis.AddGeometry( plane1 );
        
            // XoZ || oZ
        auto plane2 = geometry::TriangleMesh::CreateCylinder( R, H, resolution, split );
        plane2->PaintUniformColor( Vector3d( 0.0, 0.0, 0.0 ) );
        Rt << 1.0, 0.0, 0.0, i-Gsize/2,
              0.0, 1.0, 0.0,    0.0,
              0.0, 0.0, 1.0,    H/2,
              0.0, 0.0, 0.0,    1.0;
        plane2->Transform( Rt );
        plane2->ComputeVertexNormals();
        vis.AddGeometry( plane2 );
        
            // YoZ || oZ
        auto plane3 = geometry::TriangleMesh::CreateCylinder( R, H, resolution, split );
        plane3->PaintUniformColor( Vector3d( 0.0, 0.0, 0.0 ) );
        Rt << 1.0, 0.0, 0.0,    0.0,
              0.0, 1.0, 0.0, i-Gsize/2,
              0.0, 0.0, 1.0,    H/2,
              0.0, 0.0, 0.0,    1.0;
        plane3->Transform( Rt );
        plane3->ComputeVertexNormals();
        vis.AddGeometry( plane3 );
        
            // YoZ || oY
        auto plane4 = geometry::TriangleMesh::CreateCylinder( R, H, resolution, split );
        plane4->PaintUniformColor( Vector3d( 0.0, 0.0, 0.0 ) );
        Rt << 1.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 1.0, 0.0,
              0.0, 1.0, 0.0,  i,
              0.0, 0.0, 0.0, 1.0;
        plane4->Transform( Rt );
        plane4->ComputeVertexNormals();
        vis.AddGeometry( plane4 );
    }
}

int main()  // int argc, char *argv[]
{
    
//    unsigned long long N = 600851475143;
//    double max = 0;
    
//    for ( int i = 1; i < 10; i++ )
//    {
//        max = N % static_cast< unsigned long long >(i);
//        cout << "Max: " << max << endl;
        
//    }
    
//    Mat img = imread( "../RGBD1.jpg" );    
//    Mat img = imread( "../RGBD2.jpg" );
//    Mat img = imread( "../RGBD3.jpg" );
//    Mat img = imread( "../RGBD4.jpg" );
//    Mat img = imread( "../RGBD5.jpg" );
//    Mat img = imread( "../RGBD6.jpg" );
//    Mat img = imread( "../RGBD7.jpg" );
    Mat img = imread( "../RGBD8.jpg" );
    imshow( "IMG", img );
    waitKey(50);
    
    Mat img_HSV;
    cvtColor( img, img_HSV, COLOR_BGR2HSV );
    
    Mat channels_HSV[3];
    split( img_HSV, channels_HSV );
//    imshow( "V-chanel", channels_HSV[2] );
    
    bitwise_not( channels_HSV[2], channels_HSV[2] );
    auto cloud = make_shared< geometry::PointCloud >();
    for ( int x = 0; x < channels_HSV[2].cols; x++ )
    {
        for ( int y = 0; y < channels_HSV[2].rows; y++ )
        {
            Vector3d temPoint, tempColor;
            temPoint.x() = double( x );
            temPoint.y() = double( y );
            temPoint.z() = double( channels_HSV[2].at< uchar >(y, x) );
            tempColor.x() = double( img.at< Vec3b >(y, x).val[2] / 255.0 );
            tempColor.y() = double( img.at< Vec3b >(y, x).val[1] / 255.0 );
            tempColor.z() = double( img.at< Vec3b >(y, x).val[0] / 255.0 );
            cloud->points_.push_back( temPoint );
            cloud->colors_.push_back( tempColor );
        }
    }
    
        // Visualization
    visualization::Visualizer vis;
    vis.CreateVisualizerWindow( "Open3D", 1600, 900, 50, 50 );
    
        // Add Coordinate
    auto coord = geometry::TriangleMesh::CreateCoordinateFrame( 1.0, Vector3d( 0.0, 0.0, 0.0 ) );
    coord->ComputeVertexNormals();
    vis.AddGeometry( coord );
    
        // Add Point cloud
    vis.AddGeometry( cloud );
    
        // Start visualization
    vis.Run();
    
    waitKey(0);
    return 0;
}
