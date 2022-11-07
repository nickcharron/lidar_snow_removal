#include "DROR.h"

void DROR::SetRadiusMultiplier(double radius_multiplier) {
    radius_multiplier_ = radius_multiplier;
}

double DROR::GetRadiusMultiplier() {
    return radius_multiplier_;
}

void DROR::SetAzimuthAngle(double azimuth_angle_deg) {
    azimuth_angle__rad_ = azimuth_angle_deg * M_PI / 180;
}

double DROR::GetAzimuthAngle() {
    return azimuth_angle__rad_ * 180 / M_PI;
}

void DROR::SetMinNeighbors(double min_neighbors) {
    min_neighbors_ = min_neighbors;
}

double DROR::GetMinNeighbors() {
    return min_neighbors_;
}

void DROR::SetMinSearchRadius(double min_search_radius) {
    min_search_radius_ = min_search_radius;
}

double DROR::GetMinSearchRadius() {
    return min_search_radius_;
}
