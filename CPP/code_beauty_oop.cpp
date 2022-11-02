// https://www.youtube.com/watch?v=wN0x9eZLix4


#include <iostream>
using std::string;


class AbstractEmployee
{
    virtual void AskForPromotion()=0;
};


class Employee : AbstractEmployee
{
    private:
        string Company;
        int Age;

    protected:
        string Name;

    public:

        void SetName(string name)
        {
            Name = name;
        }

        string GetName()
        {
            return Name;
        }

        void SetCompany(string company)
        {
            Company = company;
        }

        string GetCompany()
        {
            return Company;
        }

        void SetAge(int age)
        {
            if (age >= 18)
            {
                Age = age;
            }
        }

        int GetAge()
        {
            return Age;
        }

        void IntroduceYourself()
        {
            std::cout << "Name: " << Name << std::endl;
            std::cout << "Company: " << Company << std::endl;
            std::cout << "Age: " << Age << std::endl;
        }

        Employee(string name, string company, int age)
        {
            Name = name;
            Company = company;
            Age = age;
        }

        void AskForPromotion()
        {
            if (Age>30)
            {
                std::cout << Name << " got promoted!" << std::endl;
            }
            else
            {
                std::cout << Name << ", sorry NO promotion for you!" << std::endl;
            }
        }

        virtual void Work()
        {
            std::cout << Name << " is checking email, task backlog, performing tasks..." << std::endl;
        }
};


class Developer : public Employee
{
    public:
        string fav_programming_language;

        Developer(string name, string company, int age, string language) : Employee(name, company, age)
        {
            fav_programming_language = language;
        }

        void FixBug()
        {
            std::cout << Name << " fixed bug using " << fav_programming_language << std::endl;
        }

        void Work()
        {
            std::cout << Name << " is writing " << fav_programming_language << std::endl;
        }

};


class Teacher : public Employee
{
    public:
        string Subject;

        void PreparedLesson()
        {
            std::cout << Name << " is preparing " << Subject << " lesson." << std::endl;

        }

        Teacher(string name, string company, int age, string subject) : Employee(name, company, age)
        {
            Subject = subject;
        }

        void Work()
        {
            std::cout << Name << " is teaching " << Subject << std::endl;
        }

};

int main()
{
    // Employee employee1;
    Employee employee1 = Employee("Saldina", "Apple", 25);
    employee1.IntroduceYourself();

    Employee employee2 = Employee("John", "Amazon", 45);
    employee2.IntroduceYourself();

    employee1.SetAge(15);
    std::cout << employee1.GetName() << " is " << employee1.GetAge() << " years old." << std::endl;

    employee1.AskForPromotion();
    employee2.AskForPromotion();

    Developer developer1("Mac", "Rocker", 33, "Python");
    developer1.FixBug();
    developer1.AskForPromotion();

    Teacher teacher1 = Teacher("Jack", "Stanford", 35, "History");
    teacher1.PreparedLesson();
    teacher1.AskForPromotion();

    developer1.Work();
    teacher1.Work();

    Employee* e1 = &developer1;
    Employee* e2 = &teacher1;

    e1->Work();
    e2->Work();
}
