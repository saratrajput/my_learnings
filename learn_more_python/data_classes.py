from dataclasses import dataclass


@dataclass
class Person:
    name: str
    job: str
    age: int
    # Setting a default value which is optional for initialization
    strength: int = 100


person1 = Person("Geralt", "Witcher", 30, 99)
person2 = Person("Yennefer", "Sorceress", 30)
person3 = Person("Yennefer", "Sorceress", 30)

print("ID of Person2: {}".format(id(person2)))
print(id("ID of Person3: {}".format(person3)))
print("Person1: {}".format(person1))

print("Is person3 == person2: {}".format(person3 == person2))

