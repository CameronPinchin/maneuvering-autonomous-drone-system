#ifndef COLLISION_H
#define COLLISION_H

#include <cmath>

struct DroneData {
    float curX, curY;
    float dirX, dirY;
    float destX, destY;
};

struct GhostDroneData {
    float currentX, currentY;
    float currentDirectionX, currentDirectionY;
};

struct Vec2 {
    double x, y;
    Vec2(double x = 0, double y = 0) : x(x), y(y) {}
};

struct LineSegment {
    double x1, y1, x2, y2;
    LineSegment(double x1 = 0, double y1 = 0, double x2 = 0, double y2 = 0)
        : x1(x1), y1(y1), x2(x2), y2(y2) {}
};

// Function declarations
double calculateAngle(double curX, double curY, double destX, double destY);
bool checkCollision(const DroneData& realDrone, const GhostDroneData& ghostDrone);
bool checkLineSegmentCollision(const LineSegment& seg1, const LineSegment& seg2);
Vec2 getIntersectionPoint(const LineSegment& seg1, const LineSegment& seg2);
Vec2 avoidCollision(const Vec2& cur, const Vec2& dir);

#endif // COLLISION_H