/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Polynomial.h
 * Author: merbar
 *
 * Created on July 22, 2017, 8:03 PM
 */

#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

class Polynomial {
public:
    Polynomial();
    Polynomial(const Polynomial& orig);
    Polynomial(vector<double> const &coefficients);
    virtual ~Polynomial();
    
    void set(vector<double> const &coefficients);
    double eval(double x);
    double eval_d(double x);
    double eval_double_d(double x);
    void print();
    
    
private:
    vector<double> _coeff;
    vector<double> _coeff_d;
    vector<double> _coeff_double_d;
};

#endif /* POLYNOMIAL_H */

