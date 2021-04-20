// Copyright 2021 OROCA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>

#include "gtest/gtest.h"

#include "calculator/calculator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "msg_srv_action_interface_example/srv/arithmetic_operator.hpp"


class TestCalculator : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int argc = 0;
    char ** argv = nullptr;

    rclcpp::init(argc, argv);

    calculator_ = std::make_shared<Calculator>();

    rclcpp::spin_some(calculator_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<Calculator> calculator_;
};

TEST_F(TestCalculator, sum)
{
  msg_srv_action_interface_example::srv::ArithmeticOperator::Request arithmetic_operator;
  float result = 0.0;

  result = this->calculator_->calculate_given_formula(4.0f, 2.0f, arithmetic_operator.PLUS);
  ASSERT_TRUE(result == 6.0f);
  ASSERT_FALSE(result == 2.0f);
}

TEST_F(TestCalculator, subtract)
{
  msg_srv_action_interface_example::srv::ArithmeticOperator::Request arithmetic_operator;
  float result = 0.0;

  result = this->calculator_->calculate_given_formula(-4.0f, -4.0f, arithmetic_operator.MINUS);
  ASSERT_EQ(result, 0.0f);
  ASSERT_LE(result, 0.0f);
  ASSERT_LT(result, 0.1f);
}

TEST_F(TestCalculator, multiply)
{
  msg_srv_action_interface_example::srv::ArithmeticOperator::Request arithmetic_operator;
  float result = 0.0;

  result = this->calculator_->calculate_given_formula(-2.0f, 5.0f, arithmetic_operator.MULTIPLY);
  EXPECT_EQ(result, -10.0f);
  EXPECT_GE(result, -10.0f);
  EXPECT_GT(result, -11.0f);
}

TEST_F(TestCalculator, divide)
{
  msg_srv_action_interface_example::srv::ArithmeticOperator::Request arithmetic_operator;
  float result = 0.0;

  result = this->calculator_->calculate_given_formula(10.0f, 5.0f, arithmetic_operator.DIVISION);
  ASSERT_EQ(result, 2.0f);

  result = this->calculator_->calculate_given_formula(10.0f, 0.0f, arithmetic_operator.DIVISION);
  ASSERT_EQ(result, 0.0f) << "Failed to divide";
}

TEST_F(TestCalculator, not_an_operator)
{
  float result = 0.0;

  result = this->calculator_->calculate_given_formula(10.0f, 5.0f, 0);
  EXPECT_EQ(result, 0.0f) << "Not an operator";
}

TEST(TestString, c_string)
{
  std::string abc = "abc";
  std::string ABC = "ABC";

  const char * cba = "cba";
  const char * CBA = "CBA";

  EXPECT_NE(abc.c_str(), "abc");

  EXPECT_STREQ(abc.c_str(), "abc");
  EXPECT_STRCASEEQ(abc.c_str(), ABC.c_str());

  EXPECT_STRNE(abc.c_str(), cba);
  EXPECT_STRCASENE(abc.c_str(), CBA);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
