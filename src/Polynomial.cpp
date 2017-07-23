/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Polynomial.cpp
 * Author: merbar
 * 
 * Created on July 22, 2017, 8:03 PM
 */

#include "Polynomial.h"

Polynomial::Polynomial() {
}

Polynomial::Polynomial(const Polynomial& orig) {
    _coeff = orig._coeff;
    _coeff_d = orig._coeff_d;
    _coeff_double_d = orig._coeff_double_d;
}

Polynomial::~Polynomial() {
}

Polynomial::Polynomial(vector<double> const &coefficients) {
    for (int i = 0; i < coefficients.size(); i++) {
      _coeff.push_back(coefficients[i]);
      if (i > 0) {
        double d = i * coefficients[i];
        _coeff_d.push_back(d);
        if (i > 1) {
          _coeff_double_d.push_back((i - 1) * d);
        }
      }
    }
    
}

double Polynomial::eval(double x) {
    double result = 0;
    for (int i = 0; i < _coeff.size(); i++) {
       result += _coeff[i] * pow(x, i);
    }
    return result;
}

double Polynomial::eval_d(double x) {
    double result = 0;
    for (int i = 0; i < _coeff_d.size(); i++) {
       result += _coeff_d[i] * pow(x, i);
    }
    return result;
}

double Polynomial::eval_double_d(double x) {
    double result = 0;
    for (int i = 0; i < _coeff_double_d.size(); i++) {
       result += _coeff_double_d[i] * pow(x, i);
    }
    return result;
}

void Polynomial::print() {
    cout << "Polynomial Coefficients: "<< endl;
    for (double x : _coeff)
        cout << x << " :: ";
    cout << endl;
    cout << "Polynomial Derivative Coefficients:" << endl;
    for (double x : _coeff_d)
        cout << x << " :: ";
    cout << endl;
    cout << "Polynomial Double-D Coefficients:" << endl;
    for (double x : _coeff_double_d)
        cout << x << " :: ";
    cout << endl;
}
