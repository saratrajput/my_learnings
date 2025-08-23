#include <iostream>

using namespace std;

int main()
{
    char op;
    double num1, num2;

    cout << "Enter the operator ( +, -, *, /) : ";
    cin >> op;

    cout << "Enter two numbers one by one : ";
    cin >> num1 >> num2;

    switch(op)
    {
        case '+':
            cout << num1 << " + " << num2 << " = " << (num1 + num2);
            break;
        case '-':
            cout << num1 << " - " << num2 << " = " << (num1 - num2);
            break;
        case '*':
            cout << num1 << " * " << num2 << " = " << (num1 * num2);
            break;
        case '/':
            if (num2 != 0.0)
                cout << num1 << " / " << num2 << " = " << (num1 / num2);
            else
                cout << "Division not possible. Division by zero.";
            break;
        
        default:
            cout << op << " is an invalid operator";
    }

    return 0;
}