#include <iostream>
#include <chrono>
#include "Quaternion.hpp"

int main()
{
    using namespace std::chrono;
    using quaternionlib::Quaternion;

    auto start = high_resolution_clock::now();

    Quaternion<double> q;
    q = {1, 2, 3, 4};
    Quaternion<float> q2 = {1, 2, 4};
    // Quaternion<float> q1 = static_cast<Quaternion<double>>(q);
    // q = std::move(q2);

    std::cout << "Quaternion: " << q << '\n';
    std::cout << "Norm:       " << q.Norm() << '\n';
    std::cout << "Normalized: " << q.Normalize() << '\n';
    std::cout << "Conjugate:  " << q.Conjugate() << '\n';

    auto end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - start);

    std::cout << "Czas wykonania: " << duration.count() << " microsec\n";
}
