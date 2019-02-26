#include "particle.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"
#include "Map.h"

using std::normal_distribution;
using std::string;
using std::vector;

#define EPS 0.00001

double Particle::calculateDistance(const Projection& landmark) const {
	double x1 = landmark.readX();
	double y1 = landmark.readY();

	double x2 = this->readY();    // TODO: (shouldn't it be getX() here?)
	double y2 = this->readX();    // ??

	return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

const Landmarks Particle::getLandmarksWithinRange(Landmarks& landmarks) const {
    throw;
}

void Particle::move(double delta_t, double velocity, double yaw_rate) {
    if (fabs(yaw_rate) < EPS) {
        // We treat this as a zero yaw-rate:
        this->getX() += velocity * delta_t * cos(this->getTheta());
        this->getY() += velocity * delta_t * sin(this->getTheta());
        this->getTheta() = this->getTheta();  // Since yaw-rate is zero, this remains the same
    } else {
        // We treat this as a non-zero yaw-rate
        this->getX() += velocity / yaw_rate * ( sin( this->getTheta() + yaw_rate * delta_t ) - sin( this->getTheta() ) );
        this->getY() += velocity / yaw_rate * ( cos( this->getTheta() ) - cos( this->getTheta() + yaw_rate * delta_t ) );
        this->getTheta() += yaw_rate * delta_t;
    }
}