#include <iostream>
#include <chrono>
#include "Quaternion.hpp"

int main()
{
    using namespace std::chrono;
    using quaternionlib::Quaternion;

    auto start = high_resolution_clock::now();

    Quaternion<double> q = {1.0, 2.0, 3.0, 4.0};
    Quaternion<float> q2 = {1, 2, 3, 4};
    q = std::move(q2);

    std::cout << "Quaternion: " << q << '\n';
    std::cout << "Norm:       " << q.Norm() << '\n';
    std::cout << "Normalized: " << q.Normalize() << '\n';
    std::cout << "Conjugate:  " << q.Conjugate() << '\n';

    auto end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - start);

    std::cout << "Czas wykonania: " << duration.count() << " microsec\n";
}
