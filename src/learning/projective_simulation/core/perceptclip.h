#ifndef PERCEPTCLIP_H
#define PERCEPTCLIP_H

#include <string>
#include <random>
#include "clip.h"

class PerceptClip : public Clip {

private:

    int perceptId;
    std::string label;

    void construct(int perceptId, std::string label);

public:

    PerceptClip(int perceptId, std::string label, std::shared_ptr<std::mt19937> generator, std::string clipDimensionValues, int immunity);
    PerceptClip(int perceptId, std::string label, std::shared_ptr<std::mt19937> generator, std::shared_ptr<std::vector<int>> clipDimensionValues, int immunity);

    int getPerceptId();
    std::string getLabel();
    std::string toString() const;

};

#endif // PERCEPTCLIP_H
