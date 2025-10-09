#pragma once
#include <cmath>
#include <random>
#include <vector>
#include <span>
#include "danielib/pose.hpp"
#include "danielib/utils.hpp"

namespace danielib::MCL {
// tunable parameters
const float numParticles = 100;
const float gaussianStDev = 1;
const float gaussianFactor = 1;
const float thetaNoise = toRadians(2);
const float xyNoise = 0.1;

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
        // what 

        // if this particle is correct, given the measured beam, where would the object be that the beam hit?
        Pose expectedPoint(Beam beam);
        // what should the distance sensor measurement be? (distance from the given pose to its first obstacle)
        float distanceToWall(Pose point);
        // gaussian distribution
        float gaussian(float x);
        // updates the weight of a point
        void updateWeight(std::span<const Beam> beams);
};

class Localization {
    public:
        Localization();

        std::vector<Particle> particles;
        Pose averagePose = {0, 0, 0};

        // runs the localization loop
        Pose run(Pose delta, std::span<Beam> beams);
        // updates particles (applies delta noise)
        void update(Pose delta);
        // resamples particles (resamples using beams)
        void resample(std::span<Beam> beams);
};
}
