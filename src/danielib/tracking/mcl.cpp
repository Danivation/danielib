#include "danielib/danielib.hpp"
#include <random>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <vector>
#include <span>

namespace danielib::MCL {

// create random number generator
std::random_device rd;
std::mt19937 rng(rd());
std::uniform_real_distribution<float> dist(0.0f, 1.0f);

const float numParticles = 500;
const float gaussianStDev = 0.8;
const float gaussianFactor = 0.6;
const float thetaNoise = toRadians(0.3);
const float xyNoise = 2;

BeamSensor::BeamSensor(pros::Distance& sensor, float xOffset, float yOffset, float angleOffset) :
    xOffset(xOffset),
    yOffset(yOffset),
    angleOffset(angleOffset),
    sensor(sensor)
{}

Beam::Beam(float angleOffset, float distance, float xOffset, float yOffset) :
    angleOffset(angleOffset),
    distance(distance),
    xOffset(xOffset),
    yOffset(yOffset)
{}

Particle::Particle(const Pose& pose, float weight) :
    x(pose.x),
    y(pose.y),
    theta(pose.theta),
    weight(weight)
{}

Particle::Particle(float x, float y, float theta, float weight) :
    x(x),
    y(y),
    theta(theta),
    weight(weight)
{}

// this has to be fast
void Particle::updateDeltaNoise(const Pose& delta) {
    this->x += xyNoise * 2 * (dist(rng) - 0.5) + delta.x;
    this->y += xyNoise * 2 * (dist(rng) - 0.5) + delta.y;
    this->theta += thetaNoise * 2 * (dist(rng) - 0.5) + delta.theta;
}

float Particle::expectedDistance(const Beam& beam) {
    // fix radians so sin and cos work right
    // this is really particle not robot but oh well
    float robotAngle = fixRadians(toRadians(this->theta));
    float sinRobotAngle = sinf(robotAngle);
    float cosRobotAngle = cosf(robotAngle);

    // calculate beam angle and position using offset
    float beamAngle = fixRadians(toRadians(this->theta + beam.angleOffset));
    float beamX = this->x + beam.yOffset * cosRobotAngle + beam.xOffset * sinRobotAngle;
    float beamY = this->y + beam.yOffset * sinRobotAngle - beam.xOffset * cosRobotAngle;
    float sinBeamAngle = sinf(beamAngle);
    float cosBeamAngle = cosf(beamAngle);

    // use inches for field units, the field is about 140 inches across
    float x1 = (beamX - 70) / cosBeamAngle;
    float x2 = (beamX + 70) / cosBeamAngle;
    float y1 = (beamY - 70) / sinBeamAngle;
    float y2 = (beamY + 70) / sinBeamAngle;
    return fabs(std::max(std::min(x1, x2), std::min(y1, y2)));
}

float Particle::gaussian(float x) {
    return (gaussianFactor * expf(-0.5 * powf((x / gaussianStDev), 2)) / (gaussianStDev * sqrtf(2 * M_PI)));
}

// add some check to make sure that beams of distances that are too long dont get included
// and maybe average the weights instead of summing
// and maybe make sure theyre positive
void Particle::updateWeight(std::span<const Beam> beams) {
    float sum = 1.0f;
    for (const Beam& beam : beams) {
        if (beam.distance >= 2200) continue;       // skip if beam records nothing or is invalid
        sum *= fabs(gaussian(expectedDistance(beam) - toInches(beam.distance)));
    }
    this->weight = sum;
}

Localization::Localization(std::vector<BeamSensor> sensors) :
    sensors(sensors),
    particles(numParticles, {0, 0, 0}),
    averagePose(0, 0, 0)
{}

void Localization::setPose(Pose pose) {
    particles = std::vector<Particle>(numParticles, pose);
    averagePose = pose;
}

Pose Localization::run(const Pose& delta, std::span<const Beam> beams) {
    this->update(delta);
    this->resample(beams);

    printf("{\"particles\": [");

    for (auto& particle : particles) {
        printf("[%.3f, %.3f, %.3f, %.5f]", particle.x, particle.y, particle.theta, particle.weight);

        if (&particle != &particles.back()) {
            printf(",");
        }
    }

    printf("], \"beams\": [");

    for (auto& beam : beams) {
        float beamDistance = beam.distance;
        if (beamDistance >= 2200) beamDistance = 0;
        printf("[%.1f, %.1f, %.1f, %.3f]", beam.xOffset, beam.yOffset, beam.angleOffset, /* toInches */(Particle(averagePose).expectedDistance(beam)));

        if (&beam != &beams.back()) {
            printf(",");
        }
    }

    printf("], \"pose\": [%f, %f, %f]}\n", averagePose.x, averagePose.y, toDegrees(averagePose.theta));

    return this->averagePose;
}

void Localization::update(const Pose& delta) {
    // delta pose = odom deltas, so (delta x, delta y, delta theta)
    for (Particle& particle : this->particles) {
        particle.updateDeltaNoise(delta);
    }
}

void Localization::resample(std::span<const Beam> beams) {
    // update and sum particle weights
    float sumWeights = 0;
    std::vector<float> cumulativeWeights;
    cumulativeWeights.reserve(numParticles);
    for (Particle& particle : this->particles) {
        particle.updateWeight(beams);
        sumWeights += particle.weight;
        cumulativeWeights.push_back(sumWeights);
    }

    if (sumWeights <= 0) return;

    // actual resampling step
    float randomOffset = dist(rng) / static_cast<float>(numParticles);      // [0,1) / numParticles could be 0.5/100

    std::vector<float> offsets;
    offsets.reserve(numParticles);

    for (size_t i = 0; i < numParticles; ++i) {
        float offset = randomOffset + static_cast<float>(i) / numParticles;     // 1.5/100, 2.5/100, etc
        offsets.push_back(offset * sumWeights);                                 // scale by total weight
    }

    std::vector<Particle> newParticles;
    newParticles.reserve(numParticles);

    /**
     * for each offset, go through the cumulative weights until the weight at the current index exceeds the current offset
     * this means that it will keep the offsets aligned with the particles they fall into
     * if the first particle has a high weight, it will add it to the new list and then move onto the next offset
     * the index (and the particle its checking) wont change until the offset becomes larger than the current particle's weight
     * meaning that the offset (where we are looking at) has moved to the next particle
     */
    size_t index = 0;
    for (float offset: offsets) {
        while (index < cumulativeWeights.size() && cumulativeWeights[index] < offset) {
            ++index;
        }

        // catch edge cases
        if (index >= this->particles.size()) index = this->particles.size() - 1;

        // when looping stops, add the particle at the current index to the 
        newParticles.push_back(this->particles[index]);
    }

    // replace the old particles with the new ones
    this->particles = std::move(newParticles);

    // find new average pose
    float avgX = 0.0f;
    float avgY = 0.0f;
    float avgThetaSin = 0.0f;
    float avgThetaCos = 0.0f;

    for (const Particle& p : this->particles) {
        avgX += p.x;
        avgY += p.y;
        avgThetaSin += std::sin(p.theta);
        avgThetaCos += std::cos(p.theta);
    }

    avgX /= numParticles;
    avgY /= numParticles;
    avgThetaSin /= numParticles;
    avgThetaCos /= numParticles;

    float avgTheta = std::atan2(avgThetaSin, avgThetaCos);

    this->averagePose = {avgX, avgY, avgTheta};
}

} // namespace danielib::MCl