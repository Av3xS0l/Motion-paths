#include <stdio.h>
#include <math.h>
#include <stdlib.h>

struct Edge
{
    double x1;
    double y1;
    double x2;
    double y2;
};

struct Point
{
    double X;
    double Y;
};

double EdgeLen(struct Edge edge)
{
    double dx, dy;
    dx = edge.x2 - edge.x1;
    dy = edge.y2 - edge.y1;
    return sqrt(dx*dx + dy*dy);
}

double norm(struct Point v)
{
    return sqrt(v.X*v.X + v.Y*v.Y);
}

double cross2D(struct Point v, struct Point u)
{
    return (v.X*u.Y-v.Y*u.X);
}

struct Point vsDivide(struct Point u, double scalar)
{
    u.X = u.X/scalar;
    u.Y = u.Y/scalar;
    return u;
};

int do_intersect(struct Edge e1, struct Edge e2)
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

int intersect(struct Point curve[], int size)
{
    struct Point line[size+1];
    struct Edge edges[size];
    for (int i = 0; i < size; i++)
    {
        line[i] = curve[i];
    }
    line[size] = curve[0];

    for (int c = 0; c < size; c++)
    {
        edges[c].x1 = line[c].X;
        edges[c].y1 = line[c].Y;
        edges[c].x2 = line[c + 1].X;
        edges[c].y2 = line[c + 1].Y;
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
    return ints/2;
}



double** interparc(int t, double* px, double* py, int n)
{
    // n - size of px/py
    // t - number of target points


    // Distance calculations
    double* dists = (double*)malloc(n*sizeof(double));
    double* Dx = (double*)malloc(n*sizeof(double));
    double* Dy = (double*)malloc(n*sizeof(double));
    double dx, dy, cumDist = 0;
    for (int i = 0; i < n-1; i++)
    {
        dx = px[i+1] - px[i];
        dy = py[i+1] - py[i];
        Dx[i] = dx;
        Dy[i] = dy;
        dists[i] = sqrt(dx*dx + dy*dy);
        cumDist = cumDist + dists[i];
    }
    dx = px[0] - px[n-1];
    dy = py[0]- py[n-1];
    Dx[n-1] = dx;
    Dy[n-1] = dy;
    dists[n-1] = sqrt(dx*dx + dy*dy);
    cumDist = cumDist + dists[n-1];

    // Relative distance calculations
    double* relDist = (double*)malloc(n*sizeof(double));
    double* cumRelDist = (double*)malloc(n*sizeof(double));
    relDist[0] = dists[0]/cumDist;
    cumRelDist[0] = relDist[0];
    for (int i = 1; i < n; i++)
    {
        relDist[i] = dists[i]/cumDist;
        cumRelDist[i] = cumRelDist[i-1] + relDist[i];
    }
    free(dists);

    // Distribution calculations
    int* edgeDistr = (int*)malloc(t*sizeof(int));
    int* edgePoses = (int*)malloc(t*sizeof(int));
    const double spacing = 1.0/t;
    double curRelPos = 0;
    int index = 0, posOnEdge = 0;
    for (int i = 0; i < t; i++)
    {
        curRelPos = i*spacing;
        if(curRelPos < cumRelDist[index])
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

    //positioning the points
    int orgin;
    double offset = 0, orginX, orginY, offX, offY, ux, uy;
    double* finalx = (double*)malloc(t*sizeof(double));
    double* finaly = (double*)malloc(t*sizeof(double));

    for (int i = 0; i < t; i++)
    {
        if(edgePoses[i]==0)
        {
            orgin = edgeDistr[i];

            orginX = px[orgin];
            orginY = py[orgin];
            ux = Dx[orgin]/relDist[orgin];
            uy = Dy[orgin]/relDist[orgin];
        }
        offX = ux*offset*spacing;
        offY = uy*offset*spacing;
        finalx[i] = offX + orginX + ux * spacing * edgePoses[i];
        finaly[i] = offY + orginY + uy * spacing * edgePoses[i];


        if (offset + (i+1)*spacing > cumRelDist[orgin] && (i+1)*spacing<1.0)
        {
            offset = offset + (i+1)*spacing - cumRelDist[orgin];
        }
    }
    free(Dx);
    free(Dy);
    free(relDist);
    free(cumRelDist);
    free(edgeDistr);
    free(edgePoses);
    // Concentrate cords into double**
    double** FinalXY = (double**)malloc(t*sizeof(double*));
    for (int i = 0; i < t; i++)
    {
        FinalXY[i] = (double*)malloc(2*sizeof(double));
        FinalXY[i][0] = finalx[i];
        FinalXY[i][1] = finaly[i];
    }
    free(finalx);
    free(finaly);


    return FinalXY;
}

double* FVC(struct Point* curve, double Lmax, double Lmin, struct Point d, struct Point Vmax, int size, double features[])
{
    struct Point line[size+1];
    struct Edge edges[size];
    for (int i = 0; i < size; i++)
    {
        line[i] = curve[i];
    }
    line[size] = curve[0];

    for (int c = 0; c < size; c++)
    {
        edges[c].x1 = line[c].X;
        edges[c].y1 = line[c].Y;
        edges[c].x2 = line[c + 1].X;
        edges[c].y2 = line[c + 1].Y;
    }
    for (int i = 0; i < size-1; i++)
    {
        features[0] = features[0] + EdgeLen(edges[i]);
        features[1] = features[1] + fabs(cross2D(curve[i], curve[i+1]));
    }
    features[0] = features[0] + EdgeLen(edges[size-1]);
    features[1] = features[1] / 2;
    features[2] = Lmin / Lmax;
    features[3] = norm(d);
    features[4] = asin(cross2D(vsDivide(d, norm(d)), Vmax));
    //features[5] = (double)intersect(edges, size);
    return features;
}


/*
int main()
{
    // Example usage
    int t = 100;
    int n = 7;
    struct Point punkti[] = {{1.0, 1.0},{2.0, 6.0}, {5.0, 3.0}, {4.0,-2.0}, {1.6, -1.5}, {2.5, 0.4}};
    double* fvc = FVC(punkti, );
    for (int i = 0; i < t; i++)
    {
        printf("%f\n", fvc[i]);
    }

    return 0;
}
*/
