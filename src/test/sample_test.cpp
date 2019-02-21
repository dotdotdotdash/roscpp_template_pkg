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
