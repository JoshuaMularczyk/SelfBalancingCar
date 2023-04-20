#include <iostream>

int main()
{
    int x;
    char name;
    std::cout << "what is my name?" << std::endl;
    std::cin >> name;
    std::cout << name << std::endl;
    std::cout << "Give me a Number: ";
    std::cin >> x;
    for (int i = 0; i < x; i++) {
        std::cout << name << std::endl;
    }
    return 0;
}
