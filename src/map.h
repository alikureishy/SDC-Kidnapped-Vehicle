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

#define Distances std::vector<double>
#define Landmarks std::vector<Landmark>
#define Projections std::vector<Projection>
#define Particles std::vector<Particle>
#define Observations std::vector<Observation>

namespace Map {
  class Location {
    private:
      int id;
      double x;
      double y;

    public:
      Location() : id(-1), x(0), y(0) {};
      Location (int id, double x, double y) {
        this->id = id;
        this->x = x;
        this->y = y;
      }
      double& getX() { return this-> x; }
      double readX() const { return this->x; }
      double& getY() { return this->y; }
      double readY() const { return this->y; }
      int &getId() { return this->id; }
      int readId() const { return this -> id; }
      bool isValid() { return this->id >= 0; }
  };

      class Relative : public Location {
        public:
          Relative() : Location() {}
          Relative(int id, double x, double y) : Location(id, x, y) { }
      };

          class Observation : public Relative {
            public:
              Observation() : Relative() {}
              Observation(int id, double x, double y) : Relative(id, x, y) { }
          };

      class Absolute : public Location {
        private:

        public:
          Absolute() : Location() { }
          Absolute(int id, double x, double y) : Location(id, x, y) { }
      };

          class GPS : public Absolute {
          private:
            double bearing;

          public:
            GPS() : Absolute() {}
            GPS(int id, double x, double y, double bearing) : Absolute(id, x, y), bearing(bearing) { }
            double &getBearing() { return this->bearing; }
            double readBearing() const { return this->bearing; }
          };

          class Landmark : public Absolute {
            public:
              Landmark() : Absolute() {}
              Landmark(int id, double x, double y) : Absolute(id, x, y) { }
          };

          class Projection : public Absolute {
            public:
              Projection() : Absolute() {}
              Projection(int id, double x, double y, const Observation& source) : Absolute(id, x, y), source(source) { }
              Observation& getSource() { return this->source; }
              Observation readSource() const { return this->source; }

            private:
              Observation source;
          };
}

#endif  // MAP_H_
