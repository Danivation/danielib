#pragma once
#include <cmath>
#include <random>
#include <vector>
#include <span>
#include "pros/distance.hpp"
#include "danielib/pose.hpp"
#include "danielib/utils.hpp"

namespace danielib {
inline namespace MCL {
// tunable parameters

class Beam {
    public:
        Beam(float angleOffset, float xOffset, float yOffset, pros::Distance& sensor);
        float angleOffset;
        float xOffset;
        float yOffset;
        pros::Distance& sensor;
        int distance = 0;

        void update();
};

class Particle {
    public:
        Particle(const Pose& pose, float weight = -1);
        Particle(float x, float y, float theta, float weight = -1);

        float x;
        float y;
        float theta;
        float weight;

        // update noise on the particle randomly
        void updateDeltaNoise(const Pose& delta);
        // what do we expect the distance of the given beam to be?
        float expectedDistance(const Beam& beam);
        // gaussian distribution
        float gaussian(float x);
        // updates the weight of a particle
        void updateWeight(std::span<const Beam> beams);
};

class Localization {
    public:
        Localization(std::vector<Beam> sensors);

        std::vector<Beam> beams;
        std::vector<Particle> particles;
        Pose averagePose = {0, 0, 0};

        void setPose(Pose pose);

        // runs the localization loop
        Pose run(const Pose& delta, std::span<const Beam> beams, float currentTheta);
        // updates particles (applies delta noise)
        void update(const Pose& delta);
        // resamples particles (resamples using beams)
        void resample(std::span<const Beam> beams);
};
}
}
