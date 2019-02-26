#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <string>
#include <vector>
#include "helper_functions.h"
#include "map.h"
#include "particle.h"

using namespace Map;
using namespace std;
using std::tuple;

class ParticleFilter {
 public:
  ParticleFilter(int num_particles = 1000, int seed = 0) : is_initialized(false), num_particles(num_particles), particles(num_particles), weights(num_particles), gen(seed) {}
  void init(double x, double y, double theta, double std[]);
  const bool initialized() const { return is_initialized; }
  const Particles& getParticles() const { return this->particles; }

  void moveParticles(double delta_t, double std_pos[], double velocity,
                     double yaw_rate);
  void updateParticles(double sensor_range,
                     double sensor_stds[],
                     const Observations &vehicle_observed_landmarks, // vehicle-based
                     const Landmarks &reference_landmarks);                            // map-based
  void resampleParticles();

  ~ParticleFilter() {}

private:
  double calculateAlignmentProbability(const Projection &projection, const Landmark &landmark, double measurement_uncertainties[]) const;

  bool is_initialized;
  int num_particles;
  Particles particles;
  std::vector<double> weights;
  std::default_random_engine gen;
};

#endif // PARTICLE_FILTER_H