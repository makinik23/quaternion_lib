#include "quaternionlib/Quaternion.hpp"
#include <iostream>
#include <chrono>

int main()
{
    using namespace std::chrono;
    using quaternionlib::Quaternion;

    auto start = high_resolution_clock::now();

    Quaternion<double> q{1.0, 2.0, 3.0, 4.0};
    // std::cout << "Norm:       " << q.Norm() << '\n';
    // std::cout << "Normalized: " << q.Normalize() << '\n';
    // std::cout << "Conjugate:  " << q.Conjugate() << '\n';

    auto end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - start);

    std::cout << "Czas wykonania: " << duration.count() << " microsec\n";
}
