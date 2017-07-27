/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Vehicle.h
 * Author: merbar
 *
 * Created on July 23, 2017, 1:34 PM
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

class Vehicle {
public:
    Vehicle();
    Vehicle(const Vehicle& orig);
    virtual ~Vehicle();
    void set_frenet_pos(double pos_s, double pos_d);
    void set_frenet_motion(double vel_s, double acc_s, double vel_d, double acc_d);
    vector<double> get_s() const;
    vector<double> get_d() const;
    vector<double> state_at(double t) const;
private:
    //double _pos_x;
    //double _pos_y;
    double _pos_s;
    double _pos_d;
    //double _vel_x;
    //double _vel_y;
    double _vel_s;
    double _vel_d;
    //double _acc_x;
    //double _acc_y;
    double _acc_s;
    double _acc_d;
};

#endif /* VEHICLE_H */

