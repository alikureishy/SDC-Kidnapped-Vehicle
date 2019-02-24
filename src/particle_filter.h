/**
 * particle_filter.h
 * 2D particle filter class.
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <string>
#include <vector>
#include "helper_functions.h"

using Map::single_landmark_s;
using std::tuple;

struct Particle {
  public:
    Particle() {}
    int& getId() { return this->id; }
    double& getX() { return this->x; }
    double& getY() { return this->y; }
    double& getTheta() { return this->theta; }
    double& getWeight() { return this->weight; }
    void move(double delta_t, double& std_pos[], double velocity, double yaw_rate);
    const Map createMapOfInRangeLandmarks(double sensor_range, const Map &reference_landmarks) const;
    const Map createMapOfObservedLandmarks(const std::vector<LandmarkObs>& observations) const;
    const tuple<Map&, Map&> alignObservationsWithClosestLandmarks(const Map &observations, const Map& landmarks);
    const tuple<Map, Map> getAlignedMaps() const { return make_tuple(this->observed_landmarks, this->aligned_landmarks); }
    ~Particle(){};

  private:
    int id;
    double x;
    double y;
    double theta;
    double weight;
    Map aligned_landmarks;
    Map observed_landmarks;
};


class ParticleFilter {
 public:
  // Constructor
  // @param num_particles Number of particles
  ParticleFilter(int num_particles = 1000, int seed = 0) : gen(seed), particles(num_particles), num_particles(num_particles), is_initialized(false) {}

  // Destructor
  ~ParticleFilter() {}

  /**
   * init Initializes particle filter by initializing particles to Gaussian
   *   distribution around first position and all the weights to 1.
   * @param x Initial x position [m] (simulated estimate from GPS)
   * @param y Initial y position [m]
   * @param theta Initial orientation [rad]
   * @param std[] Array of dimension 3 [standard deviation of x [m],
   *   standard deviation of y [m], standard deviation of yaw [rad]]
   */
  void init(double x, double y, double theta, double std[]);

  /**
   * prediction Predicts the state for the next time step
   *   using the process model.
   * @param delta_t Time between time step t and t+1 in measurements [s]
   * @param std_pos[] Array of dimension 3 [standard deviation of x [m],
   *   standard deviation of y [m], standard deviation of yaw [rad]]
   * @param velocity Velocity of car from t to t+1 [m/s]
   * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
   */
  void moveParticles(double delta_t, double std_pos[], double velocity,
                     double yaw_rate);

  /**
   * updateWeights Updates the weights for each particle based on the likelihood
   *   of the observed measurements.
   * @param sensor_range Range [m] of sensor
   * @param std_landmark[] Array of dimension 2
   *   [Landmark measurement uncertainty [x [m], y [m]]]
   * @param observations Vector of landmark observations
   * @param map Map class containing map landmarks
   */
  void updateParticles(double sensor_range,
                     double sensor_stds[],
                     const std::vector<LandmarkObs> &vehicle_observed_landmarks, // vehicle-based
                     const Map &reference_landmarks);                            // map-based

  /**
   * resample Resamples from the updated set of particles to form
   *   the new set of particles.
   */
  void resampleParticles();

  /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const bool initialized() const {
    return is_initialized;
  }

  /**
   * Used for obtaining debugging information related to particles.
   */
  const std::vector<Particle>& getParticles() const { return this->particles; }

private:
  double calculateAlignmentProbability(const single_landmark_s &observation, const single_landmark_s &landmark) const;
  double calculateDistance(const single_landmark_s &observation, const single_landmark_s &landmark);

  // Random number generator
  std::default_random_engine gen;

  // Set of current particles
  std::vector<Particle> particles;

  // Number of particles to draw
  int num_particles;

  // Flag, if filter is initialized
  bool is_initialized;

  // Vector of weights of all particles
  std::vector<double> weights;
};

#endif  // PARTICLE_FILTER_H_

/**
 *
 /**
   * dataAssociation Finds which observations correspond to which landmarks
   *   (likely by using a nearest-neighbors data association).
   * @param predicted Vector of predicted landmark observations
   * @param observations Vector of landmark observations
   *
  void dataAssociation(std::vector<LandmarkObs> predicted,
                       std::vector<LandmarkObs>& observations);


 *
 *
 */