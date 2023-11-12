
/*******************************************************************************

MIT License

Copyright (c) dynamic-static

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*******************************************************************************/

#include "shape-shooter/utilities.hpp"

namespace shape_shooter {

float get_orientation(const glm::vec3& v)
{
    return glm::atan(-v.z, v.x);
}

glm::vec3 from_polar(float angle, float magnitude)
{
    return glm::vec3{ glm::cos(angle), 0, glm::sin(angle) } * magnitude;
}

float from_1920x1080(float base, float value)
{
    return value / 1920 * base;
}

glm::vec3 ray_plane_intersection(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const glm::vec3& planePoint, const glm::vec3& planeNormal)
{
    // FROM : https://stackoverflow.com/questions/69257700/finding-the-intersection-of-a-ray-and-a-plane-programmatically
    auto difference = planePoint - rayOrigin;
    auto dot0 = glm::dot(difference, planeNormal);
    auto dot1 = glm::dot(rayDirection, planeNormal);
    auto distance = dot0 / dot1;
    return rayOrigin + rayDirection * distance;
}

} // namespace shape_shooter