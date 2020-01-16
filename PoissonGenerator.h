/**
 * \file PoissonGenerator.h
 * \brief
 *
 * Poisson Disk Points Generator
 *
 * \version 1.2.0
 * \date 28/12/2019
 * \author Nicola Piga, 2020
 * \author Sergey Kosarevsky, 2014-2019
 * \author support@linderdaum.com   http://www.linderdaum.com   http://blog.linderdaum.com
 */

/*
  Usage example:

  #include "PoissonGenerator.h"
  ...
  PoissonGenerator::DefaultPRNG PRNG;
  const auto Points = PoissonGenerator::GeneratePoissonPoints( NumPoints, PRNG );
*/

// Fast Poisson Disk Sampling in Arbitrary Dimensions
// http://people.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf

// Implementation based on http://devmag.org.za/2009/05/03/poisson-disk-sampling/

/* Versions history:
 *              1.2.1   Gen 16, 2020            Change the library to be a 1D Poisson disk generator
 *              1.2     Dec 28, 2019            Bugfixes; more consistent progress indicator; new command line options in demo app
 *              1.1.6   Dec  7, 2019            Removed duplicate seed initialization; fixed warnings
 *              1.1.5   Jun 16, 2019            In-class initializers; default ctors; naming, shorter code
 *              1.1.4   Oct 19, 2016            POISSON_PROGRESS_INDICATOR can be defined outside of the header file, disabled by default
 *              1.1.3a  Jun  9, 2016            Update constructor for DefaultPRNG
 *              1.1.3   Mar 10, 2016            Header-only library, no global mutable state
 *              1.1.2   Apr  9, 2015            Output a text file with XY coordinates
 *              1.1.1   May 23, 2014            Initialize PRNG seed, fixed uninitialized fields
 *              1.1     May  7, 2014            Support of density maps
 *              1.0     May  6, 2014
 */

#include <algorithm>
#include <random>
#include <vector>


namespace PoissonGenerator
{
    const char* Version = "1.2.1 (16/01/2020)";


    class DefaultPRNG
    {
    public:
        DefaultPRNG() = default;
        explicit DefaultPRNG(uint32_t seed)
            : gen_(seed)
        {}


        float random_float()
        {
            return static_cast<float>(dis_(gen_));
        }


        int random_int(int maxValue)
        {
            std::uniform_int_distribution<> disInt(0, maxValue);
            return disInt(gen_);
        }

    private:
        std::mt19937 gen_ = std::mt19937(std::random_device()());

        std::uniform_real_distribution<float> dis_ = std::uniform_real_distribution<float>(0.0f, 1.0f);
    };

    struct Point
    {
    public:
        Point() = default;
        Point(float value) :
            x_(value)
        {}


        inline bool is_in_domain() const
        {
            return x_ >= 0.0 && x_ <= 1.0;
        }


        inline float get_distance(const Point& p) const
        {
            return std::abs(p.x_ - x_);
        }


        inline float x() const
        {
            return x_;
        }

    private:
        float x_ = 0.0f;
    };

    struct Line
    {
        inline void insert(const Point& p)
        {
            line_.push_back(p);
        }


        bool is_in_neighbourhood(const Point& point, const float& minimum_distance)
        {
            std::vector<float> differences;
            for (const auto& item : line_)
                differences.push_back(point.get_distance(item));

            auto min_difference = std::min_element(differences.begin(), differences.end());
            if (*min_difference < minimum_distance)
                return true;

            return false;
        }

    private:
        std::vector<Point> line_;
    };


    template <typename PRNG>
    Point pop_random(std::vector<Point>& points, PRNG& generator)
    {
        const int idx = generator.random_int(static_cast<int>(points.size()) - 1);
        const Point p = points[idx];
        points.erase(points.begin() + idx);
        return p;
    }


    template <typename PRNG>
    Point generate_random_point_around(const Point& p, float minimum_distance, PRNG& generator)
    {
        // start with non-uniform distribution
        const float R1 = generator.random_float();
        const float R2 = generator.random_float();

        // radius should be between MinDist and 2 * MinDist
        const float radius = minimum_distance * ( R1 + 1.0f );

        // random angle
        const float angle = 2 * 3.141592653589f * R2;

        auto sign = [](const float& value) {return (value > 0) ? 1.0 : -1.0;};

        // the new point is generated around the point (x)
        const float x = p.x() + radius * sign(cos(angle));

        return Point(x);
    }


    template <typename PRNG = DefaultPRNG>
    std::vector<float> generate_poisson_points(const std::size_t& number_points, PRNG& generator, const float& minimum_distance = -1.0f, const std::size_t& new_points_count = 30)
    {
        float desired_minimum_distance = minimum_distance;

        if (desired_minimum_distance < 0.0f)
            desired_minimum_distance = sqrt(float(number_points)) / float(number_points);

        std::vector<Point> sample_points;
        std::vector<Point> process_list;

        Line line;

        Point first_point;
        do
        {
            first_point = Point(generator.random_float());
        }
        while (!first_point.is_in_domain());

        // update containers
        process_list.push_back(first_point);
        sample_points.push_back(first_point);
        line.insert(first_point);

        // generate new points for each point in the queue
        while ((!process_list.empty()) && (sample_points.size() < number_points))
        {
            const Point point = pop_random<PRNG>(process_list, generator);

            for (std::size_t i = 0; i < new_points_count; i++)
            {
                const Point new_point = generate_random_point_around(point, desired_minimum_distance, generator);
                const bool can_fit_point = new_point.is_in_domain();

                if (can_fit_point && !line.is_in_neighbourhood(new_point, desired_minimum_distance))
                {
                    process_list.push_back(new_point);
                    sample_points.push_back(new_point);
                    line.insert(new_point);
                    continue;
                }
            }
        }

        std::vector<float> output;
        for (const auto& point : sample_points)
            output.push_back(point.x());

        return output;
    }

} // namespace PoissonGenerator
