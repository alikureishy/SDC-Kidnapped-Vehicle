#include "particle_filter.h"
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
#include "map.h"

using std::normal_distribution;
using std::string;
using std::vector;

#define EPS 0.00001

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  // These particles' location & bearing are both Global coordinates
  for (int i = 0; i < num_particles; i++) {
    this->particles.at(i).getId() = i;
    this->particles.at(i).getX() = dist_x(this->gen);
    this->particles.at(i).getY() = dist_y(this->gen);
    this->particles.at(i).getTheta() = dist_theta(this->gen);
    this->particles.at(i).getWeight() = 1.0; // No need to divide by # of particles because the std::discreet_distribution does it anyway
    this->weights.at(i) = 1.0; // Same as above
  }
  this->is_initialized = true;
}

void ParticleFilter::moveParticles(double delta_t, double std_pos[],
                                   double velocity, double yaw_rate) {
	std::normal_distribution<double> noise_x(0, std_pos[0]);
	std::normal_distribution<double> noise_y(0, std_pos[1]);
	std::normal_distribution<double> noise_theta(0, std_pos[2]);

  for (Particle& p : this->particles) {
    p.move(delta_t, velocity, yaw_rate);

    // Add random noise to the movement of each particle:
    p.getX() += noise_x(this->gen);
    p.getY() += noise_y(this->gen);
    p.getTheta() += noise_theta(this->gen);
  }
}

void ParticleFilter::updateParticles(double sensor_range,
                                     double sensor_stds[],
                                     const Observations &vehicle_observed_landmarks, // Vehicle-based
                                     const Landmarks &reference_landmarks) {                      // Map-based
  /**
   *  - For each particle:
   *    - Determine reference landmarks that are within range to this particle --> "in_range_landmarks"
   *    - Convert all the vehicle_observed_landmarks to map-based coordinates -> "observed_landmarks",
   *          RELATIVE to this PARTICLE (ie, we will determine what the map-based readings would be
   *          if each observation was made by that particular particle).
   *    - Get an ordered list of the within_range reference-landmarks that match (1-1) the observed_landmarks (map-based) --> "associated_reference_landmarks"
   *      - DEBUG: Save this to the particle
   *    - Set the particle's weight to EPS
   *    - If there are associated observations:
   *      - Keep a running variable "weight" = 1
   *      - For each zipped-(observation-landmark) pair -- Big-O(#observations):
   *        - Determine x_gap and y_gap
   *        - Determine the partial_weight of the particle based on this gap --> observation_probability
   *        - partial_weight = max(EPS, observation_probability)
   *        - Multiply observation_probability into the running 'weight' value
   *      - Set particle's weight = weight (otherwise, rightfully, it remains set to EPS)
   *        - Update the filter's "weights" parameter for that particle (needed for resampling)
   */

  // Determine weight for each particle:
  for (int i = 0; i < this->num_particles; i++) {
    Particle& p = this->particles.at(i);

    p.getWeight() = EPS;  // This is in case there are no observations
    double weight = 1;
    if (vehicle_observed_landmarks.size()>0) {
      Landmarks in_range_landmarks_map = p.getLandmarksWithinRange(sensor_range, reference_landmarks);
      // std::cout << "Nearest landmarks (returned):" << std::endl;
      // for (int i = 0; i < in_range_landmarks_map.size(); i++) {
      //     Landmark& l = in_range_landmarks_map.at(i);
      //     std::cout << "\t" << l.getX() << ", " << l.getY() << std::endl;
      // }

      Projections observed_landmarks = p.transformToGlobalPerspective(vehicle_observed_landmarks);
      tuple<Projections, Landmarks, Distances> associations = p.alignObservationsWithClosestLandmarks(observed_landmarks, in_range_landmarks_map);

      Projections &projections = std::get<0>(associations);
      Landmarks &landmarks = std::get<1>(associations);
      Distances &distances = std::get<2>(associations);
      assert(projections.size() == landmarks.size());
      for (int i = 0; i<projections.size(); i++) {
        Projection &projection = projections[i];
        Landmark &landmark = landmarks[i];
        double probability = this->calculateAlignmentProbability(projection, landmark, sensor_stds, distances.at(i));
        weight *= max(EPS, probability);
      }
    }
    p.getWeight() = weight;
    this->weights[i] = weight;
  }
}

double ParticleFilter::calculateAlignmentProbability(const Projection& projection, const Landmark& landmark, double measurement_uncertainties[], double manhattan_distance) const {
  const double d_x = projection.readX() - landmark.readX();
  const double d_y = projection.readY() - landmark.readY();

  const double sigma_x = measurement_uncertainties[0];
	const double sigma_y = measurement_uncertainties[1];

  const double dx_squared = pow(d_x, 2);
  const double dy_squared = pow(d_y, 2);
  const double manhattan = sqrt(dx_squared + dy_squared);
  assert(manhattan == manhattan_distance);

  const double power_denominator_x = (2 * sigma_x * sigma_y);
  const double power_denominator_y = (2 * sigma_y * sigma_y);
  const double power_term = -((dx_squared / power_denominator_x) + (dy_squared / power_denominator_y));
  const double numerator = pow(M_E, power_term);
  const double denominator = 2. * M_PI * sigma_x * sigma_y;
  const double density = numerator / denominator;

  // double a = (1 / (2 * M_PI * sigma_x * sigma_y));
  // double prob = a * pow(M_E, -(pow(x - mew_x, 2) / (2 * pow(sigma_x, 2)) + pow(y - mew_y, 2) / (2 * pow(sigma_y, 2))));
  return density;
}

void ParticleFilter::resampleParticles() {
	 std::random_device rd;
	 std::mt19937 gen(rd());
	 std::discrete_distribution<> distribution(this->weights.begin(), this->weights.end());

	 Particles new_particles(this->num_particles);
   std::vector<double> new_weights(this->num_particles);
   for (int i = 0; i < particles.size(); ++i) {
     Particle& choice = particles.at(distribution(gen));
     new_particles[i] = choice;
     new_weights[i] = choice.getWeight();
   }
   this->particles = new_particles;
   this->weights = new_weights;
}