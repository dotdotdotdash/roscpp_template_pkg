// Copyright 2018 Venkisagunner. All rights reserved.
/*! 
 *  \brief     A brief class description
 *  \details   A detailed explanation
 *  \author    First author name
 *  \author    Second author name(optional)
 *  \version   v1.0.0
 *  \date      MM/DD/YYYY
 *  \pre       Anything that should be presumed or carried out before ?
 *  \bug       Bugs present in the code.
 *  \warning   Warning that should be mentioned.
 *  \copyright © Venkisagunner
 */
#include <ros/ros.h>
#include <gtest/gtest.h>
TEST(TestSuite, template_pkg)
{
    ASSERT_TRUE(true);
}
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
