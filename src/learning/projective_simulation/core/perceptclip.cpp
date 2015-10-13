#include "perceptclip.h"

using namespace std;

PerceptClip::PerceptClip(int perceptId, std::string label, std::shared_ptr<std::mt19937> generator, std::string clipDimensionValues, int immunity) : Clip(0, generator, clipDimensionValues, immunity) {
    construct(perceptId, label);
}

PerceptClip::PerceptClip(int perceptId, std::string label, std::shared_ptr<std::mt19937> generator, std::shared_ptr<std::vector<int>> clipDimensionValues, int immunity) : Clip(0, generator, clipDimensionValues, immunity) {
    construct(perceptId, label);
}

void PerceptClip::construct(int perceptId, std::string label) {
    this->perceptId = perceptId;
    this->label = label;
}

int PerceptClip::getPerceptId() {
    return perceptId;
}

std::string PerceptClip::getLabel() {
    return label;
}

std::string PerceptClip::toString() const {
    return label;
}
