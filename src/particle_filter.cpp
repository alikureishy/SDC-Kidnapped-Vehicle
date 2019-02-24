/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::normal_distribution;
using std::string;
using std::vector;

#define EPS 0.00001

/**
 * Particle constructor
 */
// Particle::Particle(int id, double x, double y, double theta, double weight) : id(id), x(x), y(y), theta(theta), weight(weight) {
//   this->associations.clear();
//   this->sense_x.clear();
//   this->sense_y.clear();
// }

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Initialize all particles to first position (based on estimates of x, y,
   *  theta and their uncertainties from GPS) and all weights to 1.
   */
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  // These particles' location & bearing are both MAP-BASED coordinates
  for (int i = 0; i < num_particles; i++) {
    this->particles.at(i).getId() = i;
    this->particles.at(i).getX() = dist_x(this->gen);
    this->particles.at(i).getY() = dist_y(this->gen);
    this->particles.at(i).getTheta() = dist_theta(this->gen);
    this->particles.at(i).getWeight() = 1.0 /* /num_particles */;
  }
  this->is_initialized = true;
}

void ParticleFilter::moveParticles(double delta_t, double std_pos[],
                                   double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
	std::normal_distribution<double> noise_x(0, std_pos[0]);
	std::normal_distribution<double> noise_y(0, std_pos[1]);
	std::normal_distribution<double> noise_theta(0, std_pos[2]);

  for (int i = 0; i < this->num_particles; i++) {
    Particle &p = this->particles.at(i);
    if (fabs(yaw_rate) < EPS)
    {
      // We treat this as a zero yaw-rate:
      p.getX() += velocity * delta_t * cos(p.getTheta());
      p.getY() += velocity * delta_t * sin(p.getTheta());
      p.getTheta() = p.getTheta();  // Since yaw-rate is zero, this remains the same
    }
    else
    {
      // We treat this as a non-zero yaw-rate
      p.getX() += velocity / yaw_rate * ( sin( p.getTheta() + yaw_rate * delta_t ) - sin( p.getTheta() ) );
      p.getY() += velocity / yaw_rate * ( cos( p.getTheta() ) - cos( p.getTheta() + yaw_rate * delta_t ) );
      p.getTheta() += yaw_rate * delta_t;
    }

    // Add random noise:
    p.getX() += noise_x(this->gen);
    p.getY() += noise_y(this->gen);
    p.getTheta() += noise_theta(this->gen);

    // Post-variant:
    // All particle locations & bearings are MAP-BASED
  }
}

void ParticleFilter::updateParticles(double sensor_range,
                                   double sensor_stds[],
                                   const vector<LandmarkObs> &vehicle_observed_landmarks, // Vehicle-based
                                   const Map &reference_landmarks) {                      // Map-based
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

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
    if (vehicle_observed_landmarks.size()>0) {
      Map in_range_landmarks_map = p.createMapOfInRangeLandmarks(sensor_range, reference_landmarks);
      Map observed_landmarks = p.createMapOfObservedLandmarks(vehicle_observed_landmarks);
      tuple<Map &, Map &> associations = p.alignObservationsWithClosestLandmarks(observed_landmarks, in_range_landmarks_map);

      double weight = 1;


    }

  }
}

/**
 * Find the distance between a map landmark and a transformed particle
 */
double ParticleFilter::calculateDistance(single_landmark_s observation, single_landmark_s land_mark) {
	double x1 = land_mark.x_f;
	double y1 = land_mark.y_f;

	double x2 = map_coordinates.x;
	double y2 = map_coordinates.y;

	return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

void ParticleFilter::resampleParticles() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

}

// -----------------------------------------

// string ParticleFilter::getAssociatedLandmarks(Particle best) {
//   vector<int> v = best.associations;
//   std::stringstream ss;
//   copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
//   string s = ss.str();
//   s = s.substr(0, s.length()-1);  // get rid of the trailing space
//   return s;
// }

// string ParticleFilter::getSenseCoord(Particle best, string coord) {
//   vector<double> v;

//   if (coord == "X") {
//     v = best.sense_x;
//   } else {
//     v = best.sense_y;
//   }

//   std::stringstream ss;
//   copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
//   string s = ss.str();
//   s = s.substr(0, s.length()-1);  // get rid of the trailing space
//   return s;
// }

// void ParticleFilter::SetAssociations(Particle& particle,
//                                      const vector<int>& associations,
//                                      const vector<double>& sense_x,
//                                      const vector<double>& sense_y) {
//   // particle: the particle to which assign each listed association,
//   //   and association's (x,y) world coordinates mapping
//   // associations: The landmark id that goes along with each listed association
//   // sense_x: the associations x mapping already converted to world coordinates
//   // sense_y: the associations y mapping already converted to world coordinates
//   particle.associations= associations;
//   particle.sense_x = sense_x;
//   particle.sense_y = sense_y;
// }

// void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
//                                      vector<LandmarkObs>& observations) {
//   /**
//    * TODO: Find the predicted measurement that is closest to each
//    *   observed measurement and assign the observed measurement to this
//    *   particular landmark.
//    * NOTE: this method will NOT be called by the grading code. But you will
//    *   probably find it useful to implement this method and use it as a helper
//    *   during the updateWeights phase.
//    */

// }
