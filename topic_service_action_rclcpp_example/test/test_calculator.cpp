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

#include <memory>
#include <string>

#include "calculator/calculator.hpp"
#include "gtest/gtest.h"
#include "msg_srv_action_interface_example/srv/arithmetic_operator.hpp"


class TestCalculator : public ::testing::Test
{
protected:
  void SetUp() override
  {
    calculator_ = std::make_unique<Calculator>();
  }

  void TearDown() override
  {
  }

  std::unique_ptr<Calculator> calculator_;
};

TEST_F(TestCalculator, calculate_given_formula)
{
  msg_srv_action_interface_example::srv::ArithmeticOperator::Request arithmetic_operator;
  float result = 0.0;

  result = this->calculator_->calculate_given_formula(4.0f, 2.0f, arithmetic_operator.PLUS);
  ASSERT_EQ(result, 6.0f);


}

TEST(TestString, string_comp)
{

}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
