
#include <iostream>
#include <gtest/gtest.h>

#include "libg3logger/g3logger.h"

using namespace std;
using namespace libg3logger;

TEST(test_case_name, test_name) {
  ASSERT_TRUE( true );

  G3Logger logWorker( "test_name" );
  logWorker.logBanner();
}
