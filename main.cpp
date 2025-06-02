// Wzór:
// V(r) = A * (e/B) * r * e^(-r/B)
// A - max wartosc wiatru
// B -r, dla którego mamy A
// e - liczba Eulera
// r - odległość od 0°

//      >  Trzeba zrobić siatke dla wiatru, gdzie bedziemy co 0.5° obliczać dla każdego oczka wektor wiatru.           +
//         Wymiary siatki Ny=|y(d2) - y(d1)|, Nx=|x(d2) - x(d1)|
//      >  Bedzie tez funckcja losujaca gdzies na siatce Niż albo Wyż, dzięki czemu będziemy znać kierunek wektorów.   +
//      >  Używając f. Eulera obliczyć wartosci wektorów. Dane wektorów przechowywujemy w tablicy dwuwymiarowej.       +
//      >  Jak dodawać wektory: Vwiatru + Vsamolotu
/*
Konwersja stopni na radiany
double deg2rad(double degrees) {return degrees * M_PI / 180.0;}

Struktura wektora z długością i kątem
struct Vector {
    double magnitude;
    double angle_deg; // w stopniach
};

Dodawanie wektorów z zachowaniem kierunku większego
Vector addPreservingDirection(const Vector& a, const Vector& b)
{
Ustal który wektor jest większy
    const Vector& main = (a.magnitude >= b.magnitude) ? a : b;
    const Vector& other = (a.magnitude >= b.magnitude) ? b : a;

Różnica kątów (w radianach)
    double delta_angle = deg2rad(other.angle_deg - main.angle_deg);

Oblicz rzut wektora "other" na kierunek "main"
    double projection = other.magnitude * std::cos(delta_angle);

Dodaj rzut do długości wektora "main"
    double result_magnitude = main.magnitude + projection;

    return {result_magnitude, main.angle_deg};
}
 */
//      >  Dane bedą wczytywane z pliku tekstowego.                                 `                                  +
//              track.txt -input,  odczytac segmentami po dwa punkty
//              plan.txt - output z obliczeń bez uwzglednienia wektorów wiatrów,
//              trajectory - output z uwzglednieniem wektorów wiatrów i zkorygowanymi  wartosciami

#include "header.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <random>
#include <iomanip>
#include <algorithm>

using namespace std;

// Funkcje pomocnicze
double degToRad(double deg) {return deg * M_PI / 180.0;}

double distanceBetweenPoints(double lat1, double lon1, double lat2, double lon2)
{
    /*użyjemy wzór Haversine'a, który uwzględnia kulisty kształt Ziemi.
    a - pośrednia wartość zależna od różnic współrzędnych geograficznych.
    c -kąt centralny pomiędzy dwoma punktami na sferze.
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = sin²(dlat / 2) + cos(lat1) * cos(lat2) * sin²(dlon / 2)
        c = 2 * atan2(√a, √(1 - a))
        distance = R * c                                                */

    const double R = 6371.0; // km
    double dlat = degToRad(lat2 - lat1);
    double dlon = degToRad(lon2 - lon1);
    lat1 = degToRad(lat1);
    lat2 = degToRad(lat2);

    double a = sin(dlat/2)*sin(dlat/2) + cos(lat1)*cos(lat2)*sin(dlon/2)*sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;  //długość łuku na sferze o promieniu R
}

double distanceOfSegment(const Segment& segment) {
    return distanceBetweenPoints(segment.from.lat, segment.from.lon, segment.to.lat, segment.to.lon);
}

//-WindSimulator--------------------------------------------------------------------------------------------------------
WindSimulator::WindSimulator(double x1_, double x2_, int Nx_, double y1_, double y2_, int Ny_)
    : x1(x1_), x2(x2_), Nx(Nx_), y1(y1_), y2(y2_), Ny(Ny_) { }

void WindSimulator::setRandomCenter() {
    random_device rd;                               //losowy seed
    mt19937 gen(rd());                           //Mersenne Twister – wydajny generator liczb pseudolosowych
    uniform_real_distribution<> distX(x1, x2);  //rozkład jednostajny dla współrzędnej x z przedziału [x1, x2].
    uniform_real_distribution<> distY(y1, y2);  //rozkład jednostajny dla współrzędnej y z przedziału [y1, y2].
    uniform_int_distribution<> distType(0, 1);  //losuje typ centrum - wyż czy niż

    //xc i yc są współrzędnymi losowo wybranego punktu wewnątrz prostokąta określonego przez rogi (x1, y1) i (x2, y2).
    xc = distX(gen);
    yc = distY(gen);

    if (distType(gen) == 1) {
        high = true;
        low = false;
    } else {
        high = false;
        low = true;
    }
}

void WindSimulator::generatorWindGrind() {
    setRandomCenter();

    windGrid.resize(Nx, vector<Vector>(Ny));

    for (int i = 0; i < Nx; ++i) {
        for (int j = 0; j < Ny; ++j) {
            double lon = x1 + (x2 - x1) * i / (Nx - 1);
            double lat = y1 + (y2 - y1) * j / (Ny - 1);

            // Tw. Pitagorasa
            double dx = lon - xc;
            double dy = lat - yc;
            double r = sqrt(dx * dx + dy * dy);

            double V = A * (exp(1.0) / B) * r * exp(-r / B);

            // Arcus tangens – obliczanie kąta
            double angle_rad = atan2(dy, dx);
            //Obrót kąta o ±90°
            //  jesli niż to wiatr kręci się przeciwnie do ruchu wskazówek zegara
            //  jesli wyż to zgodnie z ruchem wskazówek zegara
            if (low) angle_rad += M_PI / 2;
            else angle_rad -= M_PI / 2;

            //Rzutowanie prędkości na osie:
            double vx = V * cos(angle_rad);
            double vy = V * sin(angle_rad);

            windGrid[i][j] = Vector(vx, vy);
        }
    }
}

void WindSimulator::writeWindGrind(ostream &out) const {
    double deltaX = (x2 - x1) / (Nx - 1);
    double deltaY = (y2 - y1) / (Ny - 1);
//Grid(lon,lat): Vx([-zachód, +wschód ]), Vy ([-południe, +północ])
    out << "Wind field grid (" << Nx << " x " << Ny << ") centered at ("<< fixed << setprecision(2) << xc << ", " << yc << ")\n";
    out << "Grid resolution: Δlon = " << deltaX << "°, Δlat = " << deltaY << "°\n\n\n";

    for (int j = Ny-1; j >= 0; --j) {
        for (int i = 0; i < Nx; ++i) {
            double x = x1 + (x2 - x1) * i / (Nx - 1);
            double y = y1 + (y2 - y1) * j / (Ny - 1);
            const Vector& wind = windGrid[i][j];

            double speed = sqrt(wind.vx * wind.vx + wind.vy * wind.vy);
            double angle_deg = atan2(wind.vx, wind.vy) * 180.0 / M_PI;
            if (angle_deg < 0) angle_deg += 360.0;

            // Określenie kierunku tekstowego (8 kierunków)
            string direction;
            if (angle_deg >= 337.5 || angle_deg < 22.5) direction = "N";
            else if (angle_deg < 67.5) direction = "NE";
            else if (angle_deg < 112.5) direction = "E";
            else if (angle_deg < 157.5) direction = "SE";
            else if (angle_deg < 202.5) direction = "S";
            else if (angle_deg < 247.5) direction = "SW";
            else if (angle_deg < 292.5) direction = "W";
            else direction = "NW";

            out << "Grid (" << setw(6) << x << ", " << setw(6) << y << "): "
                << "Vx = " << setw(8) << setprecision(3) << wind.vx
                << "  Vy = " << setw(8) << setprecision(3) << wind.vy
                << "  | Speed = " << setw(8) << setprecision(3) << speed
                << "  Dir = " << direction << "\n";
        }
        out << "\n\n";// oddziela wiersze siatki (Nx punktów na tej samej szerokości geograficznej (lat))
    }
}

void WindSimulator::writeWindGridCSV(std::ostream &out) const {
    double deltaX = (x2 - x1) / (Nx - 1);
    double deltaY = (y2 - y1) / (Ny - 1);

    out << std::fixed << std::setprecision(2);

    // Nagłówek: pusta komórka, potem wartości y (szerokość geograficzna) w kolumnach
    out << "x/y";
    for (int j = Ny - 1; j >= 0; --j) {
        double y = y1 + deltaY * j;
        out << ";" << y << " speed" << ";" << y << " dir";
    }
    out << "\n";

    // Teraz wiersze: każdy x i odpowiednie wartości dla różnych y
    for (int i = 0; i < Nx; ++i) {
        double x = x1 + deltaX * i;
        out << x;

        for (int j = Ny - 1; j >= 0; --j) {
            const Vector& wind = windGrid[i][j];
            double speed = std::sqrt(wind.vx * wind.vx + wind.vy * wind.vy);
            double angle_deg = std::atan2(wind.vx, wind.vy) * 180.0 / M_PI;
            if (angle_deg < 0) angle_deg += 360.0;

            std::string direction;
            if (angle_deg >= 337.5 || angle_deg < 22.5)       direction = "N";
            else if (angle_deg < 67.5)                        direction = "NE";
            else if (angle_deg < 112.5)                       direction = "E";
            else if (angle_deg < 157.5)                       direction = "SE";
            else if (angle_deg < 202.5)                       direction = "S";
            else if (angle_deg < 247.5)                       direction = "SW";
            else if (angle_deg < 292.5)                       direction = "W";
            else                                              direction = "NW";

            out << ";" << speed << ";" << direction;
        }
        out << "\n";
    }
}

Vector WindSimulator::getWind(double x, double y) const {
    int i = round((x - x1) * (Nx - 1) / (x2 - x1));
    int j = round((y - y1) * (Ny - 1) / (y2 - y1));
    i = max(0, min(i, Nx - 1));
    j = max(0, min(j, Ny - 1));
    return windGrid[i][j];
}

//-Airplane-------------------------------------------------------------------------------------------------------------

// funkcja do konwersji kierunku i prędkości w wektora
Vector directionSpeedToVector(double speed, double angle_deg) {
    double angle_rad = angle_deg * M_PI / 180.0;
    return Vector(speed * cos(angle_rad), speed * sin(angle_rad));
}

// funkcja do sumowania prędkości samolotu i wiatru z zachowaniem kierunku lotu
Vector addPreservingDirection(const Vector& planeVec, const Vector& windVec) {
    return Vector(planeVec.vx + windVec.vx, planeVec.vy + windVec.vy);
}

void Airplane::computeTimeWithoutWind(list<Segment>& segments) {
    totalTimeNoWind = 0.0;
    cout<<"----------------------DATA FOR SEGMENTS----------------------\n\n";
    cout<<"                       NO     WIND                           \n\n";

 for (auto& seg : segments) {
        double dist = distanceOfSegment(seg);
        seg.time_no_wind = dist / planeSpeed;
        totalTimeNoWind += seg.time_no_wind;

        cout << seg.from.name << " -> " << seg.to.name << " | Time: " << fixed << setprecision(2) << seg.time_no_wind << " h\n";
        cout << "Plane Speed : " << planeSpeed << endl;
        cout<<endl;
    }
cout << "Total flight time: " << totalTimeNoWind << " h\n\n";
cout<< "----------------------------------------------------------------\n\n";

}

void Airplane::computeTimeWithWind(list<Segment>& segments, const WindSimulator& windSim) {
    totalTimeWithWind = 0.0;

cout << "                           WIND                        \n\n";

    for (auto& seg : segments) {
        // Środek segmentu (lat, lon)
        double midLat = (seg.from.lat + seg.to.lat) / 2.0;
        double midLon = (seg.from.lon + seg.to.lon) / 2.0;

        Vector wind = windSim.getWind(midLon, midLat);

        // Kierunek lotu w radianach
        double dx = seg.to.lon - seg.from.lon;
        double dy = seg.to.lat - seg.from.lat;
        double angle_rad = atan2(dy, dx);

        // Wektor prędkości samolotu (vx, vy)
        Vector planeVec{
            planeSpeed * cos(angle_rad),
            planeSpeed * sin(angle_rad)
        };

        // Suma wektorów (prędkość samolotu + wiatr)
        Vector corrected{
            planeVec.vx + wind.vx,
            planeVec.vy + wind.vy
        };

        // Projeksja skorygowanej prędkości na kierunek lotu (skalarny iloczyn)
        double correctedSpeedAlongRoute = (corrected.vx * cos(angle_rad)) + (corrected.vy * sin(angle_rad));

        if (correctedSpeedAlongRoute <= 0) {
            // Wiatr powoduje, że samolot "stoi w miejscu" lub cofa się,
            // ustaw minimalną prędkość by uniknąć dzielenia przez zero lub negatywnego czasu
            correctedSpeedAlongRoute = 1.0;
        }

        double dist = distanceOfSegment(seg);
        seg.time_with_wind = dist / correctedSpeedAlongRoute;
        totalTimeWithWind += seg.time_with_wind;

        cout << seg.from.name << " -> " << seg.to.name
             << " | Time : " << fixed << setprecision(2)
             << seg.time_with_wind << " h" << endl;
        cout << "Corrected speed vector: (" << corrected.vx << ", " << corrected.vy << ")" << endl;
        cout << "Corrected Airplane Speed : " << correctedSpeedAlongRoute << endl;
        cout<<endl;

    }

    cout << "Total flight time : " << totalTimeWithWind << " h" << endl;
    cout<<endl;
    cout<< "----------------------------------------------------------------\n\n";
}


// Wczytywanie track.txt
vector<Waypoint> readTrackFile(const string& filename) {
    vector<Waypoint> points;
    ifstream infile(filename);
    if (!infile) {
        throw runtime_error("Error file " + filename);
    }

    string line;
    Waypoint wp;
    bool waitingForCoords = false;

    while (getline(infile, line)) {
        if (line.find("Departure") != string::npos) {
            wp.name = "Departure";
            waitingForCoords = true;
        }
        else if (line.find("Tower") != string::npos) {
            wp.name = line; // np. "Tower 1"
            waitingForCoords = true;
        }
        else if (line.find("Arrive") != string::npos) {
            wp.name = "Arrive";
            waitingForCoords = true;
        }
        else if (waitingForCoords && line.find("lat:") != string::npos && line.find("lon:") != string::npos) {
            double lat = 0, lon = 0;
            sscanf(line.c_str(), "lat: %lf lon: %lf", &lat, &lon);
            wp.lat = lat;
            wp.lon = lon;
            points.push_back(wp);
            waitingForCoords = false;
        }
    }
    return points;
}


list<Segment> generateSegments(const vector<Waypoint>& points) {
    list<Segment> segments;
    for (size_t i = 1; i < points.size(); ++i) {
        segments.push_back({points[i-1], points[i], 0.0, 0.0});
    }
    return segments;
}

//-writeData------------------------------------------------------------------------------------------------------------

void writeData::writePlan(ostream& out, const list<Segment>& segments) const {
    out << "===================== Flight plan (no wind) =====================\n";
    out << fixed << setprecision(1);

    for (auto it = segments.begin(); it != segments.end(); ++it) {
        const Waypoint& wp = it->from;
        out << wp.name << "\n";
        out << "lat:\t" << setw(6) << wp.lat << "\t\tlon:\t" << setw(6) << wp.lon
            << "\t\ttime: " << setprecision(2) << it->time_no_wind << " h\n\n";
    }

    // Dodaj ostatni punkt (Arrive)
    const Waypoint& wp = segments.back().to;
    out << wp.name << "\n";
    out << "lat:\t" << setw(6) << wp.lat << "\t\tlon:\t" << setw(6) << wp.lon
        << "\t\ttime: " << setprecision(2) << segments.back().time_no_wind << " h\n";
}


void writeData::writeTrajectory(ostream& out, const list<Segment>& segments) const {
    out << "===================== Trajectory (with wind) =====================\n";
    out << fixed << setprecision(1);

    for (auto it = segments.begin(); it != segments.end(); ++it) {
        const Waypoint& wp = it->from;
        out << wp.name << "\n";
        out << "lat:\t" << setw(6) << wp.lat << "\t\tlon:\t" << setw(6) << wp.lon
            << "\t\tcor. time: " << setprecision(2) << it->time_with_wind << " h\n\n";
    }

    // Dodaj ostatni punkt (Arrive)
    const Waypoint& wp = segments.back().to;
    out << wp.name << "\n";
    out << "lat:\t" << setw(6) << wp.lat << "\t\tlon:\t" << setw(6) << wp.lon
        << "\t\tcor. time: " << setprecision(2) << segments.back().time_with_wind << " h\n";
}

void writeData::writeWindGrind(ostream& out, const WindSimulator& windSim) const {
    windSim.writeWindGrind(out);
}

// --- MAIN ------------------------------------------------------------------------------------------------------------
int main() {
    string planFile = "plan.txt";
    string trajectoryFile = "trajectory.txt";
    string windFile = "wind.txt";
    string trackFile = "track.txt";
    string csvWindFile = "wind.csv";
    try {

        // Ustawienia gridu
        double x1 = 0.0, x2 = 90.0;
        int Nx = 90;
        double y1 = 0.0, y2 = 90.0;
        int Ny = 90;

        WindSimulator windSim(x1, x2, Nx, y1, y2, Ny);
        windSim.generatorWindGrind();

        vector<Waypoint> points = readTrackFile(trackFile);

        list<Segment> segments = generateSegments(points);

        Airplane airplane;
        airplane.computeTimeWithoutWind(segments);
        airplane.computeTimeWithWind(segments, windSim);

        writeData writer;
        ofstream foutPlan(planFile);
        ofstream foutTrajectory(trajectoryFile);
        ofstream foutWind(windFile);
        ofstream csvFile(csvWindFile);


        if (!foutPlan || !foutTrajectory || !foutWind)
            throw runtime_error("Error output files");

        writer.writePlan(foutPlan, segments);
        writer.writeTrajectory(foutTrajectory, segments);
        writer.writeWindGrind(foutWind, windSim);
        windSim.writeWindGridCSV(csvFile);

        cout << "Data saved in: " << planFile << " , "<<trajectoryFile<<" , "<< windFile<<" , " <<csvWindFile<< endl;

    }
    catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
    }
    return 0;
}
