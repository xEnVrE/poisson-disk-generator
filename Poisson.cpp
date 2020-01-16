/**
 * \file Poisson.cpp
 * \brief
 *
 * Poisson Disk Points Generator example
 *
 * \version 1.2.0
 * \date 28/12/2019
 * \author Sergey Kosarevsky, 2014-2019
 * \author support@linderdaum.com   http://www.linderdaum.com   http://blog.linderdaum.com
 */

#include "PoissonGenerator.h"

#include <iostream>

int main(int argc, char** argv)
{
    std::size_t number_points = 100;

    PoissonGenerator::DefaultPRNG PRNG;

    const auto points = PoissonGenerator::generate_poisson_points(number_points, PRNG, 0.005);

    std::cout << "Generated " << points.size() << " points:" << std::endl;
    for (const auto& point : points)
        std::cout << point << std::endl;

    return EXIT_SUCCESS;
}
