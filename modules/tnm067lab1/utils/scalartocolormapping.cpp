#include <modules/tnm067lab1/utils/scalartocolormapping.h>

namespace inviwo {

void ScalarToColorMapping::clearColors() { baseColors_.clear(); }
void ScalarToColorMapping::addBaseColors(vec4 color) { baseColors_.push_back(color); }

vec4 ScalarToColorMapping::sample(float t) const {
    if (baseColors_.size() == 0) return vec4(t);
    if (baseColors_.size() == 1) return vec4(baseColors_[0]);

    // Implement here:
    // Interpolate colors in baseColors_
    // return the right values

    if (t <= 0) return vec4(baseColors_.front());
    if (t >= 1) return vec4(baseColors_.back());

    float interpolatedPoint = t * (baseColors_.size() - 1.f);
    int pointIndex = interpolatedPoint;

    vec4 colorLeft = baseColors_[pointIndex];
    vec4 colorRight = baseColors_[pointIndex + 1];

    const vec4 finalColor = (colorRight - colorLeft) * (interpolatedPoint - pointIndex) + colorLeft;

    return finalColor;
}

}  // namespace inviwo
