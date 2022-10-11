class Person:
    name: str
    job: str
    age: int
    
    def __init__(self, name, job, age):
        self.name = name
        self.job = job
        self.age = age


person1 = Person("Geralt", "Witcher", 30)
person2 = Person("Yennefer", "Sorceress", 30)
person3 = Person("Yennefer", "Sorceress", 30)

print("ID of Person2: {}".format(id(person2)))
print(id("ID of Person3: {}".format(person3)))
print("Person1: {}".format(person1))

print("Is person3 == person2: {}".format(person3 == person2))

