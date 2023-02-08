"""
ECE487 square root

Write a Python script that computes square roots using the Newton's method of successive approximation,
which says that whenever we have a guess y for the value of the square root of a number x,
we can perform a simple manipulation to get a better guess by averaging $y$ with x/y.
For example we can compute the square root of 2 as shown below

Guess       Quotient            Average
1           2/1 = 1             (2+1)/2 = 1.5
1.5         2/1.5 = 1.333       (1.333+1.5)/2 = 1.4176
1.4176      2/1.4176 = 1.4188   (1.4188+1.4176)/2 = 1.4142
1.4142      :                       :
:           :                       :
:           :                       :

You need to iterate until  |y**2-x| < epsilon, where epsilon is the tolerance.
You are not allowed to use any functions in the math module except math.fabs for the absolute values.
"""


from math import fabs
# NOTE: DO NOT USE ANY OTHER FUNCTIONS IN THE MATH MODULE


def sqrt(x, guess, tolerance):
    while fabs(guess * guess - x) > tolerance:
        quotient = x/guess
        average = (quotient + guess) / 2.0
        guess = average
    return guess


# Test your function with the following code
numbers = [2, 3, 4, 343.23, 1, 1/2, 0]
guess = 1
tolerance = 1e-9

[print(sqrt(n, guess, tolerance)) for n in numbers]

