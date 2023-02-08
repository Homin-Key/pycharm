"""
In the Fibonacci sequence, each number is the sum of the two
numbers that precede it. That is,
f0 = 0; f1 = 1; fn = fn

So, the beginning of the sequence is:
0, 1, 1, 2, 3, 5, 8, 13, 21, 34, ...

Write a Python function that takes an argument, n, and returns a list of n Fibonacci
numbers. For example, if n is 9, your function returns [0, 1, 1, 2, 3, 5, 8, 13, 21]. Submit the
Python function with your test scripts and outputs. You must try at least three different
numbers.
"""


def fib(num):
    pass


def fib2(a, b, count):
    pass


while True:

    s = input("Enter a positive integer: ")
    if s.isnumeric():
        n = int(s)
        print(fib(n))
        #break
    else:
        print("Not a non-negative integer")
        continue

print(fib(n))

# print(fib2(0, 1, n))



