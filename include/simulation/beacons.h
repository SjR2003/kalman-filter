#pragma once

#include <vector>

class Display;

struct BeaconData
{
    double x, y;
    int id;
    BeaconData() : x(0.0), y(0.0), id(-1) {}
    BeaconData(double xPos, double yPos) : x(xPos), y(yPos), id(-1) {}
    BeaconData(double xPos, double yPos, int beaconId) : x(xPos), y(yPos), id(beaconId) {}
};

class BeaconMap
{
public:
    BeaconMap();

    void add_beacon(double x, double y);

    BeaconData get_beacon_with_id(int id) const;
    std::vector<BeaconData> get_beacons_within_range(double x, double y, double range) const;
    std::vector<BeaconData> get_beacons() const;

    void render(Display &disp) const;

private:
    std::vector<BeaconData> m_beacon_map;
};