#pragma once

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



  using cv::Mat;
  using cv::Rect;
  using cv::Size;

  struct CompositeCanvas
  {
    CompositeCanvas( void ) {;}

    CompositeCanvas( const Size &sz, int type )
      : canvas( sz, type )
    {
      rect[0] = Rect( 0,0, sz.width/2, sz.height );
      rect[1] = Rect( rect[0].width, 0, rect[0].width, rect[0].height );

      roi[0] = Mat( canvas, rect[0] );
      roi[1] = Mat( canvas, rect[1] );
    }

    CompositeCanvas( const Mat &mat, const Rect &roi1 = Rect(), const Rect &roi2 = Rect() )
      : canvas( mat )
    {
      if( roi1.area() > 0 ) {
        rect[0] = roi1;
      } else {
        rect[0] = Rect( 0,0, mat.size().width / 2, mat.size().height );
      }

      if( roi2.area() > 0 ) {
        rect[1] = roi2;
      } else {
        rect[1] = Rect( rect[0].width, 0, mat.size().width - rect[0].width, mat.size().height );
      }

      roi[0] = Mat( canvas, rect[0] );
      roi[1] = Mat( canvas, rect[1] );
    }

    CompositeCanvas( const Mat &mat0, const Mat &mat1, bool doCopy = true )
   : canvas()
 {
   Size canvasSize( mat0.size().width + mat1.size().width,
       std::max(mat0.size().height, mat1.size().height) );

   canvas.create( canvasSize, mat0.type() );

   rect[0] = Rect( 0,0, mat0.size().width, mat0.size().height );
   rect[1] = Rect( mat0.size().width ,0, mat1.size().width, mat1.size().height );

   roi[0] = Mat( canvas, rect[0] );
   roi[1] = Mat( canvas, rect[1] );

   if( doCopy ) {
     mat0.copyTo( roi[0] );
     mat1.copyTo( roi[1] );
   }
 }




 operator Mat &() { return canvas; }
 operator cv::InputArray() { return cv::InputArray(canvas); }
 operator cv::InputOutputArray() { return cv::InputOutputArray(canvas); }

 Mat &operator[]( int i ){ return roi[i]; }
 const Mat &operator[]( int i ) const { return roi[i]; }

 Size size( void ) const { return canvas.size(); }
 int type( void ) const { return canvas.type(); }

 void copyConvert( int i, const Mat &mat, float alpha = 1 )
 {
   if( mat.type() == canvas.type() )
     mat.copyTo( roi[i] );
   else
     mat.convertTo( roi[i], canvas.type(), alpha );
 }

 Mat scaled( float scale ) const {
   Mat out;
   resize( canvas, out, Size(), scale, scale, cv::INTER_LINEAR );
   return out;
 }

 cv::Point origin( int i )
 { return cv::Point( rect[i].x, rect[i].y ); }

 Mat canvas, roi[2];
 Rect rect[2];
};
