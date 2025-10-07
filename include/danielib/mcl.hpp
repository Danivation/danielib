#pragma once
#include <cmath>
#include <random>
#include <vector>
#include "danielib/pose.hpp"
#include "danielib/utils.hpp"

namespace danielib::MCL {
// tunable parameters
const float numParticles = 100;
const float gaussianStdev = 1;
const float gaussianFactor = 1;
const float thetaNoise = toRadians(2);
const float xyNoise = 0.1;

class Beam {
    public:
        Beam(float angle, float distance);
        float angle;
        float distance;
};

class Particle {
    public:
        Particle(Pose pose, float weight = -1);
        Particle(float x, float y, float theta, float weight = -1);

        float x;
        float y;
        float theta;
        float weight;

        // update noise on the particle randomly
        void updateDeltaNoise(Pose delta);
        // if the particle is correct, where should the beam have landed?
        Pose expectedPoint(Beam beam);
        // approximate distance from point to wall
        float distanceToWall(Pose point);
        // gaussian distribution
        float gaussian(float x);
        // updates the weight of a point
        void updateWeight(std::vector<Beam> beams);
};

class Localization {
    public:
        Localization();

        std::vector<Particle> particles;
        Pose averagePose = {0, 0, 0};

        // runs the localization loop
        Pose run(Pose delta, std::vector<Beam> beams);
        // updates particles (applies delta noise)
        void update(Pose delta);
        // resamples particles (resamples using beams)
        void resample(std::vector<Beam> beams);
};
}
