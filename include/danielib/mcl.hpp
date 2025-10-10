#pragma once
#include <cmath>
#include <random>
#include <vector>
#include <span>
#include "pros/distance.hpp"
#include "danielib/pose.hpp"
#include "danielib/utils.hpp"

namespace danielib::MCL {
// tunable parameters
const float numParticles = 100;
const float gaussianStDev = 1;
const float gaussianFactor = 1;
const float thetaNoise = toRadians(2);
const float xyNoise = 0.1;

class BeamSensor {
    public:
        BeamSensor(float angleOffset, pros::Distance* sensor);
        float angleOffset;
        pros::Distance* sensor;
};

class Beam {
    public:
        Beam(float angleOffset, float distance);
        float angleOffset;
        float distance;
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
        Localization(std::initializer_list<const BeamSensor> sensors);

        std::vector<const BeamSensor> sensors;
        std::vector<Particle> particles;
        Pose averagePose = {0, 0, 0};

        // runs the localization loop
        Pose run(const Pose& delta, std::span<const Beam> beams);
        // updates particles (applies delta noise)
        void update(const Pose& delta);
        // resamples particles (resamples using beams)
        void resample(std::span<const Beam> beams);
};
}
