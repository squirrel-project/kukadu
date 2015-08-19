#include <gtest/gtest.h>

#define TEST_EXPRESSION(a) EXPECT_EQ((a), meval::EvaluateMathExpression(#a))

TEST(MathExpressions, operatorRecognition){
  EXPECT_TRUE(true);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

