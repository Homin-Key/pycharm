"""
ECE487 num_vowels
"""

"""
Takes a string argument and returns the number
of lowercase vowels in the string.
"""


def num_vowels(str):
    num = 0
    for s in str:
        if s == 'A' or s == 'a' or s == 'E' or s == 'e' or s == 'I' or s == 'i' or s == 'O' or s == 'o' or s == 'U' or s == 'u':
            num = num + 1
    return num


quotes = list()

quotes.append("Don’t worry if it doesn’t work right. If everything did, you’d be out of a job.")   # 21 vowels
quotes.append("The best thing about a boolean is even if you are wrong, you are only off by a bit.")  # 28 vowels
quotes.append("There are two ways to write error-free programs; only the third one works. (Alan J. Perlis)")  # 25 vowels
quotes.append("One man’s crappy software is another man’s full-time job. (Jessica Gaston)")  # 21 vowels
quotes.append("It’s not a bug – it’s an undocumented feature.")  # 15 vowels
quotes.append("Always code as if the guy who ends up maintaining your code will be a violent psychopath who knows where you live.")  # 36 vowels
quotes.append("A good programmer is someone who always looks both ways before crossing a one-way street. (Doug Linder)")  # 33 vowels
quotes.append("If debugging is the process of removing software bugs, then programming must be the process of putting them in. (Edsger Dijkstra)")  # 34 vowels
quotes.append("In order to understand recursion, one must first understand recursion.")  # 22 vowels
quotes.append("There are only two kinds of programming languages: those people always bitch about and those nobody uses. (Bjarne Stroustrup)")   # 38 vowels
quotes.append("Carefully read comments, but do not always trust them. (Stan Baek)")   # 18 vowels

for quote in quotes:
    print(quote + " ==> " + str(num_vowels(quote)) + " vowels.")

