# PlaneAndWindSimulator

A simple flight simulation tool that calculates aircraft travel time along a defined route, both with and without wind influence.

## Description

This program generates a random 2D wind vector field and simulates the flight of an aircraft across a series of waypoints. It compares the travel time in two scenarios:

- Without wind (constant aircraft velocity),
- With wind influence (aircraft velocity adjusted by wind vectors).

All input and output data are saved to `.txt` and `.csv` files.

## Features

- Random 2D wind field generation,
- Multi-point route support,
- Travel time calculation with/without wind,
- Data export to text and CSV formats.

---

## Input Example (`track.txt`)
Departure
lat: 20.0 lon: 20.0

Tower 1
lat: 30.0 lon: 25.0

Tower 2
lat: 42.0 lon: 37.0

Arrive
lat: 60.0 lon: 30.0

## Console Output Example

----------------------DATA FOR SEGMENTS----------------------

                       NO     WIND

Departure -> Tower 1 | Time: 1.53 h
Plane Speed : 800.00

Tower 1 -> Tower 2 | Time: 2.14 h
Plane Speed : 800.00

Tower 2 -> Arrive | Time: 2.57 h
Plane Speed : 800.00

Total flight time: 6.24 h

----------------------------------------------------------------

                           WIND

Departure -> Tower 1 | Time : 1.41 h
Corrected speed vector: (554.01, 692.77)
Corrected Airplane Speed : 867.39

Tower 1 -> Tower 2 | Time : 1.78 h
Corrected speed vector: (756.95, 606.80)
Corrected Airplane Speed : 964.32

Tower 2 -> Arrive | Time : 2.45 h
Corrected speed vector: (-174.58, 832.34)
Corrected Airplane Speed : 839.02

Total flight time : 5.64 h

----------------------------------------------------------------

Data saved in: plan.txt , trajectory.txt , wind.txt , wind.csv


## Output Files 

### `plan.txt`

===================== Flight plan (no wind) =====================
Departure
lat:	  20.0		lon:	  20.0		time: 1.53 h

Tower 1
lat:	 30.00		lon:	 25.00		time: 2.14 h

Tower 2
lat:	 42.00		lon:	 37.00		time: 2.57 h

Arrive
lat:	 60.00		lon:	 30.00		time: 2.57 h

### `trajectory.txt`

===================== Trajectory (with wind) =====================
Departure
lat:	  20.0		lon:	  20.0		cor. time: 1.41 h

Tower 1
lat:	 30.00		lon:	 25.00		cor. time: 1.78 h

Tower 2
lat:	 42.00		lon:	 37.00		cor. time: 2.45 h

Arrive
lat:	 60.00		lon:	 30.00		cor. time: 2.45 h

### `wind.txt`

Wind field grid (90 x 90) centered at (26.28, 60.00)
Grid resolution: Δlon = 1.01°, Δlat = 1.01°


Grid (  0.00,  90.00): Vx = -143.880  Vy = -126.017  | Speed =  191.264  Dir = SW
Grid ( 1.011, 90.000): Vx = -147.075  Vy = -123.858  | Speed =  192.281  Dir = SW
Grid ( 2.022, 90.000): Vx = -150.265  Vy = -121.479  | Speed =  193.227  Dir = SW
Grid ( 3.034, 90.000): Vx = -153.442  Vy = -118.875  | Speed =  194.102  Dir = SW
(...)

### `wind.csv`

Structured CSV file containing the wind vector components and metadata for the grid, suitable for further data analysis and visualization.

---

## Usage

1. Prepare the input file `track.txt` with the list of waypoints (latitude and longitude).
2. Run the simulator executable.
3. View the console output for flight times.
4. Check generated output files: `plan.txt`, `trajectory.txt`, `wind.txt`, and `wind.csv`.
## Author

Created for educational purposes by Avui.
