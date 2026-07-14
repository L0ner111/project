#include <gtest/gtest.h>
// #define PI 3.1415926535    // 定义π

extern "C" {
    #include "unit_conversion.h"  
    #include "config.h"  
}

TEST(UnitConversion, DegreesToRadians) {
    EXPECT_NEAR(degreesToRadians(0.0), 0.0, 1e-6);
    EXPECT_NEAR(degreesToRadians(90.0), PI / 2.0, 1e-6);
    EXPECT_NEAR(degreesToRadians(180.0), PI, 1e-6);
    EXPECT_NEAR(degreesToRadians(360.0), 2 * PI, 1e-6);
}