

#include <gtest/gtest.h>

#include "testimages.h"

TEST( TestImages, decodeToBytes )
{
  for( int i = 0; i <  NUM_TEST_IMAGES; ++i ) {
    std::vector<BYTE> out = TestImage(i);
    ASSERT_EQ( out.size(), TestImageWidth*TestImageHeight );

    // Check the snippet
    for( int y = 0; y < SNIPPET_SIZE; ++y ) {
      for( int x = 0; x < SNIPPET_SIZE; ++x ) {
        int idx = x + y*TestImageWidth;

        ASSERT_EQ( out[idx], ImageSnippets[i][y][x] ) << "image " << i << " at " << x << "," << y;

      }
    }
  }

}
