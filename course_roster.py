
"""
ECE487: Course Roster
"""


class Cadet:
    def __init__(self, name):
        """
        Constructor
        :param name: name of the cadet in string
        """
        self.name = name        # cadet name
        self.courses = list()   # initially an empty list

    def add_course(self, course):
        """
        Add a course to the list of courses the cadet takes
        :param course: an instance of Course
        :return: None
        """
        self.courses.append(course)     # add the course

    def list_courses(self):
        """
        return a string representation of courses.
        If the cadet is taking ECE487 and ECE387, it will return
        "ECE487, ECE387"
        :return: string representation of the list of courses enrolled
        """
        return ", ".join(map(str, self.courses))

    def get_gpa(self):
        """
        Calculate the GPA, which is the sum of the weighted grades divided by the total credit hours.
        GPA = sum(course*credit)/total_credit.
        :return: GPA
        """
        # Write this part for ICE3
        sum = 0 # initialized outside of for loop
        total_credit = 0
        for course in self.courses: # dict initialized in Cadet Class
            sum = sum + (course.get_grade(self) * course.credit) # continously iterating and adding to sum
            total_credit = total_credit + course.credit # totaled credits
        return sum/total_credit

class Course:

    def __init__(self, name, credit):
        """
        Constructor
        :param name: name of the course in string
        :param credit: credit hours
        """
        self.name = name        # course name
        self.credit = credit    # credit hours
        self.grades = dict()    # initially empty dictionary

    def __str__(self):
        """
        Return a string representation of this class. For example,
        ece487 = Course("ECE387", 3)
        print(ece487) will print "ECE387".
        If this method is not implemented, print(ece487) will throw an error.
        :return: course name
        """
        return self.name

    def enroll(self, cadets):
        """
        Add cadets to this course.
        :param cadets: list of cadets to enroll
        :return: Nothing
        """
        # Write this part for ICE3
        for cadet in cadets:
            cadet.add_course(self)
            self.grades[cadet] = 0

        # Add this course to each cadet in the cadets list.
        # The initial grade is 0.
        # Use self.grades[each_cadet] = 0. Note self.grades is a dictionary.
        # Use the add_course method in the Cadet class to add this course to each cadet.

    def give_grade(self, grades):
        """
        Assign grade to each cadet in the grades dictionary.
        :param grades: a list of two-element lists, [[cadet1, grade1], [cadet2, grade2], ...]
        :return: Nothing
        """
        for grade in grades:
            self.grades[grade[0]] = grade[1]

    def get_course_average(self):
        """
        Calculate the course average
        :return: course average
        """
        # Write this part for ICE3
        return sum(self.grades.values())/len(self.grades.values())
        # .value = the actual grade value, sum = total grades added for the course its in, len = the total number of grades in the course

    def get_roster(self):
        """
        Return a string of all cadets enrolled in this course. For example, it will return
        "Peter Parker, Clark Kent, Brue Wayne"
        :return: a string of cadets' names enrolled in this course.
        """
        # Write this part for ICE3
        names_roster = list() # make empty list
        cadets = self.grades.keys()
        for cadet in cadets:
            names_roster.append(cadet.name) # cadet.name = self.name; initialized above
        return ", ".join(names_roster)

    def get_grade(self, cadet):
        """
        Return the grade of the cadet
        :param cadet: an instance of cadet.
        :return: course grade of the cadet.
        """
        # Write this part for ICE3
        return self.grades[cadet] # brackets because dict

""" ============================================================================
    DO NOT CHANGE ANYTHING BELOW THIS LINE
    =========================================================================== """


def main():


    """ Create courses """
    ece245 = Course("ECE245", 3)
    ece382 = Course("ECE382", 3)
    ece487 = Course("ECE487", 3)
    ece499 = Course("ECE499", 3)

    """ Create cadets """
    spiderman = Cadet("Peter Parker")
    superman = Cadet("Clark Kent")
    batman = Cadet("Brue Wayne")
    blackwidow = Cadet("Natasha Romanoff")

    """ Enroll cadets """
    ece245.enroll([spiderman, superman, batman, blackwidow])
    ece382.enroll([superman, batman, blackwidow])
    ece487.enroll([superman, blackwidow])
    ece499.enroll([batman])

    """ Create a list of courses and a list of cadets"""
    courses = [ece245, ece382, ece487, ece499]
    cadets = [spiderman, superman, batman, blackwidow]

    """ Print the course roster for each course"""
    for course in courses:
        print(f"Cadets enrolled in {course.name}: {course.get_roster()}")

    """ Print the list of courses each cadet is taking"""
    for cadet in cadets:
        print(f"{cadet.name} is taking {cadet.list_courses()}")

    """ Assign grades """
    ece245.give_grade([[spiderman, 58], [superman, 95], [batman, 73], [blackwidow, 87]])
    ece382.give_grade([[superman, 83], [batman, 82], [blackwidow, 92]])
    ece487.give_grade([[superman, 91], [blackwidow, 98]])
    ece499.give_grade([[batman, 91]])

    """ Print the course average for each course"""
    for course in courses:
        print(f"{course} average is {course.get_course_average()}")

    """ Print GPA for each cadet """
    for cadet in cadets:
        print(f"{cadet.name}'s GPA is {cadet.get_gpa()}")


if __name__ == '__main__':
    main()
