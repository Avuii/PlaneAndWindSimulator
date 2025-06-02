#ifndef HEADER_H
#define HEADER_H

#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <list>
#include <iomanip>
using namespace std;

struct Vector {
    double vx = 0.0, vy = 0.0; // składowe prędkości wiatru (wschód, północ)

    Vector(double vx = 0.0, double vy = 0.0) : vx(vx), vy(vy) {}

    // Dodawanie wektorów z zachowaniem kierunku (przyda się w Airplane)
    double magnitude() const {
        return sqrt(vx*vx + vy*vy);
    }

    Vector operator+(const Vector& other) const {
        return Vector(vx + other.vx, vy + other.vy);
    }
};

class WindSimulator {
private:
    double x1, x2, y1, y2;
    int Nx, Ny;
    double A = 200.0; // max prędkość wiatru
    double B = 30.0;
    bool high = false; // wyż
    bool low = false;  // niż
    double xc = 0, yc = 0;
    vector<vector<Vector>> windGrid;

public:
    WindSimulator(double x1, double x2, int Nx, double y1, double y2, int Ny);

    void setRandomCenter();
    void generatorWindGrind();
    void writeWindGrind(ostream& out) const;
    void writeWindGridCSV(std::ostream& out) const;
    Vector getWind(double x, double y) const;
};

// Punkt na trasie
struct Waypoint {
    string name;
    double lat = 0.0;
    double lon = 0.0;
};

// Odcinek między punktami
struct Segment {
    Waypoint from;
    Waypoint to;
    double time_no_wind = 0.0;
    double time_with_wind = 0.0;
};

// Prototypy funkcji pomocniczych
double degToRad(double deg);
double distanceBetweenPoints(double lat1, double lon1, double lat2, double lon2);
double distanceOfSegment(const Segment& segment);

class Airplane {
public:
    double planeSpeed = 800.0; // km/h
    double totalTimeNoWind = 0.0;
    double totalTimeWithWind = 0.0;

    void computeTimeWithoutWind(list<Segment>& segments);
    void computeTimeWithWind(list<Segment>& segments, const WindSimulator& windSim);
};

class writeData {
public:
    void writePlan(ostream& out, const list<Segment>& segments) const;
    void writeTrajectory(ostream& out, const list<Segment>& segments) const;
    void writeWindGrind(ostream& out, const WindSimulator& windSim) const;
};

#endif
