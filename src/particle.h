
#ifndef PARTICLE_H_
#define PARTICLE_H_

#include <string>
#include <vector>
#include "helper_functions.h"
#include "map.h"
#include <random>

using namespace Map;
// using namespace std;
using std::default_random_engine;
using std::make_tuple;
using std::tuple;

class Particle : public Absolute {
  public:
    Particle() {}
    double& getTheta() { return this->theta; }
    double readTheta() const { return this->theta; }
    double& getWeight() { return this->weight; }
    double readWeight() const { return this->weight; }
    const tuple<Projections, Landmarks, Distances> getAlignments() const {
      return make_tuple(this->projections, this->aligned_landmarks, this->manhattan_distances);
    }
    void move(double delta_t, double velocity, double yaw_rate);
    Projection getHomogenousTransformation(const Observation& observation) const;
    Landmarks getLandmarksWithinRange(double sensor_range, const Landmarks& reference_landmarks) const;
    Projections transformToGlobalPerspective(const Observations& observations) const;
    tuple<Projections, Landmarks, Distances> alignObservationsWithClosestLandmarks(const Projections &observations, const Landmarks& landmarks);

    ~Particle(){};

  private:
    double calculateDistance(const Projection& landmark) const;
    double theta;
    double weight;
    Projections projections;
    Landmarks aligned_landmarks;
    Distances manhattan_distances;
};

#endif // PARTICLE_H