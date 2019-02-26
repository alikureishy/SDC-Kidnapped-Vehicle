/**
 * map.h
 *
 * Created on: Dec 12, 2016
 * Author: mufferm
 */

#ifndef MAP_H_
#define MAP_H_

#include <vector>

/**
Location (id, x, y)
  - Global
    - Particle (theta)
      - getHomogenousTransformation(Observation)::Projection
    - Projection (source)
    - Landmark
  - Local
    - Observation ()
*/

#define Landmarks std::vector<Landmark>
#define Projections std::vector<Projection>
#define Particles std::vector<Particle>
#define Observations std::vector<Observation>

namespace Map {
  class Location {
    private:
      double x;
      double y;

    public:
      Location() {};
      double& getX() { return this->x; }
      double readX() const { return this->x; }
      double& getY() { return this->y; }
      double readY() const { return this->y; }
  };
      class Relative : public Location {
        public:
          Relative() : Location() {}
      };

          class Observation : public Relative {
            public:
              Observation() : Relative() {}
          };

      class Absolute : public Location {
        private:
          int id = -1;

        public:
          Absolute() : Location() { }
          int &getId() { return this->id; }
          bool isValid() { return this->id >= 0; }
      };

          class GPS : public Absolute {
          private:
            double bearing;

          public:
            GPS() : Absolute() {}
            double &getBearing() { return this->bearing; }
            double readBearing() const { return this->bearing; }
          };

          class Projection : public Absolute {
            public:
              Projection() : Absolute() {}
              Observation& getSource() { return this->source; }
              Observation readSource() const { return this->source; }

            private:
              Observation source;
          };

          class Landmark : public Absolute {
            public:
              Landmark() : Absolute() {}
          };
}

#endif  // MAP_H_
