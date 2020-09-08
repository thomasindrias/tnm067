#pragma once

#include <modules/tnm067lab1/tnm067lab1moduledefine.h>
#include <inviwo/core/util/glm.h>


namespace inviwo {

template <typename T>
struct float_type {
    using type = double;
};

template <>
struct float_type<float> {
    using type = float;
};
template <>
struct float_type<vec3> {
    using type = float;
};
template <>
struct float_type<vec2> {
    using type = float;
};
template <>
struct float_type<vec4> {
    using type = float;
};

namespace TNM067 {
namespace Interpolation {

#define ENABLE_LINEAR_UNITTEST 0
template <typename T, typename F = double>
T linear(const T& a, const T& b, F x) {
    if (x <= 0) return a;
    if (x >= 1) return b;

    //Task 7
    T f = (1.0 - x) * a + x * b;

    return f;
}

// clang-format off
    /*
     2------3
     |      |
    y|  •   |
     |      |
     0------1
        x
    */
    // clang format on
#define ENABLE_BILINEAR_UNITTEST 0
template<typename T, typename F = double> 
T bilinear(const std::array<T, 4> &v, F x, F y) {
    //Task 7
    T fx1 = linear(v[0], v[1], x);
    T fx2 = linear(v[2], v[3], x);
    T fy = linear(fx1, fx2, y);

    return fy;
}


    // clang-format off
    /* 
    a--•----b------c
    0  x    1      2
    */
// clang-format on
#define ENABLE_QUADRATIC_UNITTEST 0
template <typename T, typename F = double>
T quadratic(const T& a, const T& b, const T& c, F x) {

    // Alternative description (Task 8)
    T f = (1.0 - x) * (1.0 - 2.0 * x) * a + 4.0 * x * (1.0 - x) * b + x * (2.0 * x - 1.0) * c;
    
    return f;
}

// clang-format off
    /* 
    6-------7-------8
    |       |       |
    |       |       |
    |       |       |
    3-------4-------5
    |       |       |
   y|  •    |       |
    |       |       |
    0-------1-------2
    0  x    1       2
    */
// clang-format on
#define ENABLE_BIQUADRATIC_UNITTEST 0
template <typename T, typename F = double>
T biQuadratic(const std::array<T, 9>& v, F x, F y) {

    T fx1 = quadratic(v[0], v[1], v[2], x);
    T fx2 = quadratic(v[3], v[4], v[5], x);
    T fx3 = quadratic(v[6], v[7], v[8], x);

    return quadratic(fx1, fx2, fx3, y);
}

// clang-format off
    /*
     2---------3
     |'-.      |
     |   -,    |
   y |  •  -,  |
     |       -,|
     0---------1
        x
    */
// clang-format on
#define ENABLE_BARYCENTRIC_UNITTEST 1
template <typename T, typename F = double>
T barycentric(const std::array<T, 4>& v, F x, F y) {

    double full_area = glm::determinant(glm::dmat3(
        dvec3(v[1], 1.0), dvec3(v[2], 1.0), dvec3(v[3], 1.0))) * 0.5

    double alpha = glm::determinant(glm::dmat3(
        dvec3(v[0], 1.0), dvec3(v[1], 1.0), dvec3(v[2], 1.0)) * 0.5 / full_area;

    double beta = glm::determinant(glm::dmat3(
        dvec3(v[0], 1.0), dvec3(v[1], 1.0), dvec3(v[3], 1.0))) * 0.5 / full_area;

    double beta = glm::determinant(glm::dmat3(
        dvec3(v[0], 1.0), dvec3(v[2], 1.0), dvec3(v[3], 1.0))) * 0.5 / full_area;



    return v[0];
}

}  // namespace Interpolation
}  // namespace TNM067
}  // namespace inviwo
