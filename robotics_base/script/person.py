class Person():
    def __init__(self, name, surname, birthday_year):
        self.name_ = name
        self.surname_ = surname
        self.birthday_year_ = birthday_year

    def print_person(self):
        print("NAME ", self.name_)
        print("SURNAME ", self.surname_)

    def get_age(self, current_year):
        age = current_year - self.birthday_year_
        return age

###########################

persona = Person('Andrea','Roberti',1990)
persona.print_person()
age = persona.get_age(2024)
print(age)

# seconda_persona = Person('Matteo','Caobelli')
# seconda_persona.print_person()