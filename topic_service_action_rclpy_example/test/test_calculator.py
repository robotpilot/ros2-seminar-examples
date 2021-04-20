# Copyright 2021 OROCA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from msg_srv_action_interface_example.srv import ArithmeticOperator
import pytest
import rclpy
from topic_service_action_rclpy_example.calculator.calculator import Calculator


@pytest.fixture(scope='module')
def setup_calculator():
    rclpy.init()
    calculator = Calculator()
    try:
        yield calculator
    finally:
        rclpy.shutdown()


def test_sum(setup_calculator):
    result = setup_calculator.calculate_given_formula(
            4.0, 2.0, ArithmeticOperator.Request.PLUS)

    assert result == 6.0
    assert result != 2.0


def test_subtract(setup_calculator):
    result = setup_calculator.calculate_given_formula(
            -4.0, -4.0, ArithmeticOperator.Request.MINUS)

    assert result == 0.0
    assert result <= 0.0
    assert result < 0.1


def test_multiply(setup_calculator):
    result = setup_calculator.calculate_given_formula(
            -2.0, 5.0, ArithmeticOperator.Request.MULTIPLY)

    assert result == -10.0
    assert result >= -10.0
    assert result > -11.0


def test_divide(setup_calculator):
    result = setup_calculator.calculate_given_formula(
            10.0, 5.0, ArithmeticOperator.Request.DIVISION)

    assert result == 2.0

    result = setup_calculator.calculate_given_formula(
            10.0, 0.0, ArithmeticOperator.Request.DIVISION)
    assert result == 0.0


def test_no_an_operator(setup_calculator):
    result = setup_calculator.calculate_given_formula(
            10.0, 5.0, 0)
    assert result == 0.0


def test_string():
    abc = 'abc'
    ABC = 'ABC'

    assert abc == abc
    assert abc == ABC.lower()
