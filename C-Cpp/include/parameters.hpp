#ifndef _PARAMS__
#define _PARAMS__
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <exception>
#include <iostream>
#include <vector>

struct Edge
{
    double x1;
    double y1;
    double x2;
    double y2;
};

struct Point
{
    double x;
    double y;

    Point operator-(const Point &other) const;
    Point operator+(const Point &other) const;
    Point operator*(const double &scallar) const;
    Point operator/(const double &scallar) const;
    friend std::ostream &operator<<(std::ostream &, const Point &);
};

struct parameters
{
    int data[13];

    int &operator[](int);
};

struct _PCA
{
    Point ANC = {0, 0};
    Point COM;
    Point v0;
    double L_MAX;
    double L_MIN;
    bool Err = false;
    friend std::ostream &operator<<(std::ostream &, const _PCA &);
};

struct EigenResult
{
    double w[2];
    double v[2][2];
};

struct features
{
    double f1;
    double f2;
    double f3;
    double f4;
    double f5;
    int f6;
    friend std::ostream &operator<<(std::ostream &, const features &);
};

double EdgeLen(Edge);

double norm(Point);

double norm(features);

double norm(int, int, int, int);

double dot(features, features);

double cross2D(Point, Point);

Point vsDivide(Point, double);

int do_intersect(Edge, Edge);

int intersect(std::vector<Point> &, int);

void resample(std::vector<Point> &, int, std::vector<Point> &);

double **interparc(int, double *, double *, int);

void FVC(std::vector<Point> &, double, double, Point, Point, features &);

_PCA pca(std::vector<Point> &);

EigenResult eig(Point[]);

void normalizeVector(double &, double &);

void normalizeEigenvectors(EigenResult &);

double Sim(features, features);

#endif
