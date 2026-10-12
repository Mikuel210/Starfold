#pragma once
#ifndef STARFOLD_VECTOR3_H
#define STARFOLD_VECTOR3_H

struct Vector3 {
    double x, y, z;
    Vector3(double x_ = 0, double y_ = 0, double z_ = 0) : x(x_), y(y_), z(z_) {}
};

#endif
