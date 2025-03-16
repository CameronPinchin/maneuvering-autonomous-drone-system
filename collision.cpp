#include "collision.h"
#include <iostream>
#include <cmath>

using namespace std;

#define SPEED 1.5
#define EPSILON 1e-6

// Calculate the angle between two points
double calculateAngle(double curX, double curY, double destX, double destY) {
    return atan2(destY - curY, destX - curX);
}

// Calculate the orientation of three points
double orientation(double p1x, double p1y, double p2x, double p2y, double p3x, double p3y) {
    return (p3x - p1x) * (p2y - p1y) - (p2x - p1x) * (p3y - p1y);
}

// Check if a point lies on a line segment
bool pointOnSegment(double px, double py, const LineSegment& seg) {
    return (px >= min(seg.x1, seg.x2) && px <= max(seg.x1, seg.x2) &&
           (py >= min(seg.y1, seg.y2) && py <= max(seg.y1, seg.y2)));
}

// Check if two intervals overlap
bool intervalsOverlap(double a1, double a2, double b1, double b2) {
    return max(a1, a2) >= min(b1, b2) && min(a1, a2) <= max(b1, b2);
}

// Get the flight path of a drone
LineSegment getFlightPath(const DroneData& drone) {
    double dx = drone.destX - drone.curX;
    double dy = drone.destY - drone.curY;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance < SPEED) {
        // If the destination is within one step, snap to the destination
        return LineSegment{drone.curX, drone.curY, drone.destX, drone.destY};
    }

    // Normalize the direction vector and scale by SPEED
    double nextX = drone.curX + (dx / distance) * SPEED;
    double nextY = drone.curY + (dy / distance) * SPEED;

    return LineSegment{drone.curX, drone.curY, nextX, nextY};
}

// Check collision between a real drone and a ghost drone
bool checkCollision(const DroneData& realDrone, const GhostDroneData& ghostDrone) {
    // Get flight paths
    LineSegment realPath = getFlightPath(realDrone);

    // Calculate ghost drone's next position based on its direction vector
    double ghostNextX = ghostDrone.currentX + ghostDrone.currentDirectionX * SPEED;
    double ghostNextY = ghostDrone.currentY + ghostDrone.currentDirectionY * SPEED;
    LineSegment ghostPath = {ghostDrone.currentX, ghostDrone.currentY, ghostNextX, ghostNextY};

    // Check for collision between the two paths
    return checkLineSegmentCollision(realPath, ghostPath);
}

// Check collision between two line segments
bool checkLineSegmentCollision(const LineSegment& seg1, const LineSegment& seg2) {
    // Calculate orientations
    double o1 = orientation(seg1.x1, seg1.y1, seg1.x2, seg1.y2, seg2.x1, seg2.y1);
    double o2 = orientation(seg1.x1, seg1.y1, seg1.x2, seg1.y2, seg2.x2, seg2.y2);
    double o3 = orientation(seg2.x1, seg2.y1, seg2.x2, seg2.y2, seg1.x1, seg1.y1);
    double o4 = orientation(seg2.x1, seg2.y1, seg2.x2, seg2.y2, seg1.x2, seg1.y2);

    // General case: Segments intersect in the interior
    if ((o1 * o2 < 0) && (o3 * o4 < 0)) return true;

    // Special cases: Check endpoints lying on segments
    if (std::abs(o1) < EPSILON && pointOnSegment(seg2.x1, seg2.y1, seg1)) return true;
    if (std::abs(o2) < EPSILON && pointOnSegment(seg2.x2, seg2.y2, seg1)) return true;
    if (std::abs(o3) < EPSILON && pointOnSegment(seg1.x1, seg1.y1, seg2)) return true;
    if (std::abs(o4) < EPSILON && pointOnSegment(seg1.x2, seg1.y2, seg2)) return true;

    // Special case: Collinear segments overlapping
    if (std::abs(o1) < EPSILON && std::abs(o2) < EPSILON && 
        std::abs(o3) < EPSILON && std::abs(o4) < EPSILON) {
        bool xOverlap = intervalsOverlap(seg1.x1, seg1.x2, seg2.x1, seg2.x2);
        bool yOverlap = intervalsOverlap(seg1.y1, seg1.y2, seg2.y1, seg2.y2);
        if (xOverlap && yOverlap) return true;
    }

    return false;
}

// Get the intersection point of two line segments
Vec2 getIntersectionPoint(const LineSegment& seg1, const LineSegment& seg2) {
    double a1 = seg1.y2 - seg1.y1;
    double b1 = seg1.x1 - seg1.x2;
    double c1 = a1 * seg1.x1 + b1 * seg1.y1;

    double a2 = seg2.y2 - seg2.y1;
    double b2 = seg2.x1 - seg2.x2;
    double c2 = a2 * seg2.x1 + b2 * seg2.y1;

    double determinant = a1 * b2 - a2 * b1;

    if (std::abs(determinant) < EPSILON) {
        // Lines are parallel or collinear
        return Vec2{NAN, NAN}; // No intersection
    }

    Vec2 intersection_point;
    intersection_point.x = (b2 * c1 - b1 * c2) / determinant;
    intersection_point.y = (a1 * c2 - a2 * c1) / determinant;

    // Check if the intersection point lies within both segments
    if (pointOnSegment(intersection_point.x, intersection_point.y, seg1) &&
        pointOnSegment(intersection_point.x, intersection_point.y, seg2)) {
        return intersection_point;
    }

    return Vec2{NAN, NAN}; // No valid intersection
}

Vec2 avoidCollision(const Vec2& cur, const Vec2& dir) {
    // Calculate difference between points
    double dx = dir.x - cur.x;
    double dy = dir.y - cur.y;

    // Find angle between points and shift by a small value
    double angle = calculateAngle(cur.x, cur.y, dir.x, dir.y) + M_PI/16;
    float distance = sqrt(dx * dx + dy * dy);

    // Assign new direction
    //Vec2 newDir;
    //newDir.x = distance*cos(angle);
    //newDir.y = distance*sin(angle);
    Vec2 newPos;
    newPos.x = cur.x + distance * cos(angle); 
    newPos.y = cur.y + distance * sin(angle);
    
    return newPos;
}