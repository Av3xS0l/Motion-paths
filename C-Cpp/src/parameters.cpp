#include <stdio.h>
#include <math.h>
#include <cmath>
#include <stdlib.h>
#include <exception>
#include <iostream>
#include <vector>
#include <iomanip>

#include "../include/parameters.hpp"

using namespace std;

const double EPSILON = 1e-10;

Point Point::operator-(const Point &other) const
{
    Point result;
    result.x = this->x - other.x;
    if (result.x < EPSILON)
    {
        result.x = 0;
    }
    result.y = this->y - other.y;
    if (result.y < EPSILON)
    {
        result.y = 0;
    }
    return result;
}

Point Point::operator+(const Point &other) const
{
    Point result;
    result.x = this->x + other.x;
    if (result.x < EPSILON)
    {
        result.x = 0;
    }
    result.y = this->y + other.y;
    if (result.y < EPSILON)
    {
        result.y = 0;
    }
    return result;
}

Point Point::operator*(const double &scallar) const
{
    Point result;
    result.x = this->x * scallar;
    if (result.x < EPSILON)
    {
        result.x = 0;
    }
    result.y = this->y * scallar;
    if (result.y < EPSILON)
    {
        result.y = 0;
    }

    return result;
}

Point Point::operator/(const double &scallar) const
{
    Point result;
    if (scallar == 0)
    {
        throw exception();
    }
    result.x = this->x / scallar;
    if (result.x < EPSILON)
    {
        result.x = 0;
    }
    result.y = this->y / scallar;
    if (result.y < EPSILON)
    {
        result.y = 0;
    }
    return result;
}

ostream &operator<<(ostream &os, const Point &obj)
{
    os << "x: " << obj.x << " y: " << obj.y;
    return os;
}

ostream &operator<<(ostream &os, const _PCA &obj)
{
    os << "PCA:" << endl
       << "ANC: " << obj.ANC.x << " " << obj.ANC.y << endl
       << "COM: " << obj.COM.x << " " << obj.COM.y << endl
       << "V0: " << obj.v0.x << " " << obj.v0.y << endl
       << "L_MAX: " << obj.L_MAX << endl
       << "L_MIN: " << obj.L_MIN;
    return os;
}

ostream &operator<<(ostream &os, const features &fi)
{
    os << "f1: " << fi.f1 << endl
       << "f2: " << fi.f2 << endl
       << "f3: " << fi.f3 << endl
       << "f4: " << fi.f4 << endl
       << "f5: " << fi.f5 << endl
       << "f6: " << fi.f6;
    return os;
}

int &parameters::operator[](int index)
{
    if (index >= 0 && index < 13)
    {
        return data[index];
    }
    else
    {
        throw exception();
    }
}

double EdgeLen(Edge edge)
{
    double dx, dy;
    dx = edge.x2 - edge.x1;
    dy = edge.y2 - edge.y1;
    return sqrt(dx * dx + dy * dy);
}

double norm(Point v)
{
    return sqrt(v.x * v.x + v.y * v.y);
}

double norm(int vx, int vy, int ux, int uy)
{
    int vX = ux - vx;
    int vY = uy - vy;
    return sqrt(vX * vX + vY * vY);
}

double norm(features a)
{
    return sqrt(a.f1 * a.f1 + a.f2 * a.f2 + a.f3 * a.f3 + a.f4 * a.f4 + a.f5 * a.f5 + a.f6 * a.f6);
}

double dot(features a, features b)
{
    return (a.f1 * b.f1 + a.f2 * b.f2 + a.f3 * b.f3 + a.f4 * b.f4 + a.f5 * b.f5 + a.f6 * b.f6);
}

double cross2D(Point v, Point u)
{
    return (v.x * u.y - v.y * u.x);
}

Point vsDivide(Point u, double scalar)
{
    u.x = u.x / scalar;
    u.y = u.y / scalar;
    return u;
};

int do_intersect(Edge e1, struct Edge e2)
{
    double t, s, a, b, c, d;
    a = (e1.x1 - e2.x1) * (e2.y1 - e2.y2) - (e1.y1 - e2.y1) * (e2.x1 - e2.x2);
    b = (e1.x1 - e1.x2) * (e2.y1 - e2.y2) - (e1.y1 - e1.y2) * (e2.x1 - e2.x2);
    c = (e1.x1 - e2.x1) * (e1.y1 - e1.y2) - (e1.y1 - e2.y1) * (e1.x1 - e1.x2);
    d = (e1.x1 - e1.x2) * (e2.y1 - e2.y2) - (e1.y1 - e1.y2) * (e2.x1 - e2.x2);

    if (a == 0 || b == 0 || c == 0 || d == 0)
    {
        return 0;
    }

    t = a / b;
    s = c / d;

    if (t < 1 && t > 0 && s < 1 && s > 0)
    {
        return 1;
    }
    return 0;
}

int intersect(vector<Point> &curve, int size)
{
    Point line[size + 1];
    Edge edges[size];
    for (int i = 0; i < size; i++)
    {
        line[i] = curve[i];
    }
    line[size] = curve[0];

    for (int c = 0; c < size; c++)
    {
        edges[c].x1 = line[c].x;
        edges[c].y1 = line[c].y;
        edges[c].x2 = line[c + 1].x;
        edges[c].y2 = line[c + 1].y;
    }
    int ints = 0;
    for (int i = 0; i < size; i++)
    {
        for (int z = i + 1; z < size; z++)
        {
            if (do_intersect(edges[i], edges[z]))
            {
                ints++;
            }
        }
    }
    return ints / 2;
}

void resample(vector<Point> &curve, int resolution, vector<Point> &out)
{
    const int n = (curve.size() / 2) + 1;
    double px[n], py[n];
    for (int i = 0; i < n; i++)
    {
        px[i] = curve[i].x;
        py[i] = curve[i].y;
    }
    auto newcurve = interparc(360, px, py, n);
    for (int i = 0; i<360; i++)
    {
        out.push_back({newcurve[i][0], newcurve[i][1]});
    }
    return;
}

double **interparc(int t, double *px, double *py, int n)
{
    // n - size of px/py
    // t - number of target points

    // Distance calculations
    double *dists = (double *)malloc(n * sizeof(double));
    double *Dx = (double *)malloc(n * sizeof(double));
    double *Dy = (double *)malloc(n * sizeof(double));
    double dx, dy, cumDist = 0;
    for (int i = 0; i < n - 1; i++)
    {
        dx = px[i + 1] - px[i];
        dy = py[i + 1] - py[i];
        Dx[i] = dx;
        Dy[i] = dy;
        dists[i] = sqrt(dx * dx + dy * dy);
        cumDist = cumDist + dists[i];
    }
    dx = px[0] - px[n - 1];
    dy = py[0] - py[n - 1];
    Dx[n - 1] = dx;
    Dy[n - 1] = dy;
    dists[n - 1] = sqrt(dx * dx + dy * dy);
    cumDist = cumDist + dists[n - 1];

    // Relative distance calculations
    double *relDist = (double *)malloc(n * sizeof(double));
    double *cumRelDist = (double *)malloc(n * sizeof(double));
    relDist[0] = dists[0] / cumDist;
    cumRelDist[0] = relDist[0];
    for (int i = 1; i < n; i++)
    {
        relDist[i] = dists[i] / cumDist;
        cumRelDist[i] = cumRelDist[i - 1] + relDist[i];
    }
    free(dists);

    // Distribution calculations
    int *edgeDistr = (int *)malloc(t * sizeof(int));
    int *edgePoses = (int *)malloc(t * sizeof(int));
    const double spacing = 1.0 / t;
    double curRelPos = 0;
    int index = 0, posOnEdge = 0;
    for (int i = 0; i < t; i++)
    {
        curRelPos = i * spacing;
        if (curRelPos < cumRelDist[index])
        {
            edgeDistr[i] = index;
            edgePoses[i] = posOnEdge;
            posOnEdge++;
        }
        else
        {
            index++;
            posOnEdge = 0;
            edgeDistr[i] = index;
            edgePoses[i] = posOnEdge;
            posOnEdge++;
        }
    }

    // positioning the points
    int orgin;
    double offset = 0, orginX, orginY, offX, offY, ux, uy;
    double *finalx = (double *)malloc(t * sizeof(double));
    double *finaly = (double *)malloc(t * sizeof(double));

    for (int i = 0; i < t; i++)
    {
        if (edgePoses[i] == 0)
        {
            orgin = edgeDistr[i];

            orginX = px[orgin];
            orginY = py[orgin];
            ux = Dx[orgin] / relDist[orgin];
            uy = Dy[orgin] / relDist[orgin];
        }
        offX = ux * offset * spacing;
        offY = uy * offset * spacing;
        finalx[i] = offX + orginX + ux * spacing * edgePoses[i];
        finaly[i] = offY + orginY + uy * spacing * edgePoses[i];

        if (offset + (i + 1) * spacing > cumRelDist[orgin] && (i + 1) * spacing < 1.0)
        {
            offset = offset + (i + 1) * spacing - cumRelDist[orgin];
        }
    }
    free(Dx);
    free(Dy);
    free(relDist);
    free(cumRelDist);
    free(edgeDistr);
    free(edgePoses);
    // Concentrate cords into double**
    double **FinalXY = (double **)malloc(t * sizeof(double *));
    for (int i = 0; i < t; i++)
    {
        FinalXY[i] = (double *)malloc(2 * sizeof(double));
        FinalXY[i][0] = finalx[i];
        FinalXY[i][1] = finaly[i];
    }
    free(finalx);
    free(finaly);

    return FinalXY;
}

void FVC(vector<Point> &curve, double Lmax, double Lmin, Point d, Point Vmax, features &features)
{
    int size = curve.size();
    Point edges[size];

    edges[0] = curve[0] - curve.back();
    for (int i = 1; i < size - 1; i++)
    {
        edges[i] = curve[i] - curve[i - 1];
        features.f2 += abs(cross2D(curve[i], curve[i - 1]));
    }
    features.f2 += abs(cross2D(curve[0], curve.back()));

    for (int i = 0; i < size; i++)
    {
        features.f1 += norm(edges[i]);
    }

    features.f2 = features.f2 / 2.0;
    features.f3 = Lmin / Lmax;
    features.f4 = norm(d);
    features.f5 = asin(abs(cross2D((d / norm(d)), Vmax)));
    features.f6 = intersect(curve, curve.size()); // TOTALY FUCKED
    return;
}

_PCA pca(vector<Point> &curve)
{
    _PCA result;
    result.COM.x = 0;
    result.COM.y = 0;
    const int size = curve.size();
    // COM and mean calculations
    for (int i = 0; i < size; i++)
    {
        result.COM.x += curve[i].x;
        result.COM.y += curve[i].y;
    }
    result.COM.x = result.COM.x / size;
    result.COM.y = result.COM.y / size;

    double sum_xx = 0;
    double sum_yy = 0;
    double sum_xy = 0;
    double diff_x;
    double diff_y;

    for (int i = 0; i < size; i++)
    {
        diff_x = curve[i].x - result.COM.x;
        diff_y = curve[i].y - result.COM.y;
        sum_xx += diff_x * diff_x;
        sum_yy += diff_y * diff_y;
        sum_xy += diff_x * diff_y;
    }

    sum_xx /= (size - 1); // Corrected divisor
    sum_yy /= (size - 1); // Corrected divisor
    sum_xy /= (size - 1); // Corrected divisor

    Point COVARI[2] = {{sum_xx, sum_xy}, {sum_xy, sum_yy}};

    result.L_MAX = COVARI[0].x;
    result.L_MIN = COVARI[1].y;

    // Eigenvalues
    EigenResult wv;
    try
    {
        wv = eig(COVARI);
        normalizeEigenvectors(wv);
        result.v0.x = wv.v[0][0];
        result.v0.y = wv.v[0][1];
    }
    catch (exception)
    {
        result.Err = true;
        return result;
    }

    return result;
}

EigenResult eig(Point A[2])
{
    EigenResult result;

    double a = A[0].x;
    double b = A[0].y;
    double c = A[1].x;
    double d = A[1].y;

    double trace = a + d;
    double det = a * d - b * c;

    // Compute eigenvalues
    double discriminant = sqrt(trace * trace - 4 * det);
    result.w[0] = (trace + discriminant) / 2.0;
    result.w[1] = (trace - discriminant) / 2.0;

    // Compute eigenvectors
    double lambda1 = result.w[0];
    double lambda2 = result.w[1];

    if (b != 0.0)
    {
        result.v[0][0] = 1.0;
        result.v[0][1] = (lambda1 - a) / b;

        result.v[1][0] = 1.0;
        result.v[1][1] = (lambda2 - a) / b;
    }
    else if (c != 0.0)
    {
        result.v[0][0] = (lambda1 - d) / c;
        result.v[0][1] = 1.0;

        result.v[1][0] = (lambda2 - d) / c;
        result.v[1][1] = 1.0;
    }
    else
    {
        // Handle special cases when b and c are both 0
        result.v[0][0] = 1.0;
        result.v[0][1] = 0.0;

        result.v[1][0] = 0.0;
        result.v[1][1] = 1.0;
    }

    return result;
}

void normalizeVector(double &x, double &y)
{
    double magnitude = sqrt(x * x + y * y);
    if (magnitude != 0.0)
    {
        x = x / magnitude * -1;
        y = y / magnitude * -1;
    }
}

void normalizeEigenvectors(EigenResult &result)
{
    normalizeVector(result.v[0][0], result.v[0][1]);
    normalizeVector(result.v[1][0], result.v[1][1]);
}

double Sim(features fi, features fj)
{
    return dot(fi, fj) / (norm(fi) * norm(fj));
}
