#include <iostream>
#include <cstdlib>
#include <time.h>
#include <cmath>
#include <exception>
#include <string>
#include <numeric>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <thread>
#include <mutex>
#include <functional>

#include "include/parameters.hpp"

#pragma GCC optimize("O3,unroll-loops")

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;


class Except : public exception
{
public:
    Except(const char *message) : errorMessage(message) {}

    // Override the what() method to provide a custom error message
    virtual const char *what() const throw()
    {
        return errorMessage.c_str();
    }

private:
    string errorMessage;
};

static int randint(int l_bound, int u_bound)
{
    return (l_bound + rand() % (u_bound - l_bound + 1));
}

void save_coords(vector<Point> &curve, string id = "")
{
    string exit_file = "outputs/base_" + id + ".cor";
    ofstream file(exit_file);

    if (!file.is_open())
    {
        cerr << "Failed to open the file." << endl;
    }
    else
    {
        for (Point x : curve)
        {
            file << std::fixed << std::setprecision(30) << x.x << ", " << x.y << "\n";
        }
    }

    file.close();
    return;
}

void save_params(parameters par, string id = "")
{
    string exit_file = "outputs/base_" + id + ".par";
    ofstream file(exit_file);

    if (!file.is_open())
    {
        cerr << "Failed to open the file." << endl;
    }
    else
    {
        for (int x = 0; x < 13; x++)
        {
            file << par[x] << " ";
        }
    }

    file.close();
    return;
}

class Bar
{
public:
    int length;
    int joint = 0;

    Bar(int length, int joint = 0)
    {
        this->length = length;
        this->joint = joint;
    }
};

class Anchor
{
public:
    double x;
    double y;
    int r;
    int speed;
    int initial;

    Anchor(int x, int y, int r, int speed, int initial)
    {
        this->x = (double)x;
        this->y = (double)y;
        this->r = r;
        this->speed = speed;
        this->initial = initial;
    }

    Point base_point(int angle)
    {
        if (speed < 0)
        {
            angle = initial - angle;
        }
        else
        {
            angle = initial + angle;
        }
        Point center;
        center.x = (r * deg_to_x(angle)) + x;
        center.y = (r * deg_to_y(angle)) + y;
        return center;
    }

    Point base_point_distance(double angle1, double angle2, Point base1, Point base2)
    {
        double x = base1.x - base2.x;
        double y = base1.y - base2.y;

        Point _packed;
        _packed.x = xy_to_angle(x, y);
        _packed.y = xy_to_hyp(x, y);
        return _packed;
    }

    Point distance_angle_from(double x, double y)
    {
        Point _packed;
        _packed.x = xy_to_angle((this->x - x), (this->y - y));
        _packed.y = xy_to_hyp((x - this->x), (y - this->y));
        return _packed;
    }

private:
    double deg_to_x(int angle)
    {
        return cos(angle * (M_PI / 180.0));
    };

    double deg_to_y(int angle)
    {
        return sin(angle * (M_PI / 180.0));
    }

    double xy_to_angle(double x, double y)
    {
        if (x == 0.0)
        {
            throw Except("Angle is invalid");
        }
        return atan(y / x);
    }

    double xy_to_hyp(double x, double y)
    {
        return sqrt(x * x + y * y);
    }
};

class TwoBar
{
public:
    double resolution = 360.0;
    int totalFrames;
    double stepSize;
    double d1Distance;

    TwoBar(Anchor &drive1, Anchor &drive2, Bar &bar1, Bar &bar2)
    {
        set_speeds(drive1, drive2);

        validate_physics(drive1, drive2, bar1, bar2);
    }

    Point end_path(int i, Anchor &drive1, Anchor &drive2, Bar &bar1, Bar &bar2)
    {
        Point barEnd;

        int angle1 = i * drive1.speed;

        Point d1XY = drive1.base_point(angle1);
        int angle2 = i * drive2.speed;

        Point AngLen = drive1.base_point_distance(angle1, angle2, drive1.base_point(angle1), drive2.base_point(angle2));
        double angle = sides_to_angle(bar1.joint, AngLen.y, bar2.length);

        barEnd = line_end(d1XY.x, d1XY.y, bar1.length, (angle + AngLen.x));

        return barEnd;
    }

private:
    void validate_physics(Anchor &drive1, Anchor &drive2, Bar &bar1, Bar &bar2)
    {
        d1Distance = drive1.distance_angle_from(drive2.x, drive2.y).y;
        if ((d1Distance + drive1.r + drive2.r) >= (bar1.joint + bar2.length))
        {
            throw Except("Bars too short");
        }
        if (((d1Distance - drive1.r) + bar1.joint) < (bar2.length))
        {
            throw Except("Bars too long");
        }
        return;
    }

    void set_speeds(Anchor &drive1, Anchor &drive2)
    {
        if (drive1.speed == drive2.speed)
        {
            totalFrames = resolution;
        }
        else
        {
            int _gcd = std::gcd(drive1.speed, drive2.speed);
            if (_gcd > 1)
            {
                if (_gcd == drive1.speed)
                {
                    totalFrames = (int)((drive2.speed / _gcd) * resolution);
                }
                else if (_gcd == drive2.speed)
                {
                    totalFrames = (int)((drive1.speed / _gcd) * resolution);
                }
                else
                {
                    totalFrames = (int)(_gcd * resolution);
                }
            }
            else
            {
                totalFrames = (int)(drive1.speed * drive2.speed * resolution);
            }
        }
        stepSize = totalFrames / resolution;
        return;
    }

    double sides_to_angle(double a, double b, double c)
    {
        if (a + b <= c)
        {
            throw Except("Cosine lawbreaker");
        }
        else
        {
            if (((a * a + b * b - c * c) / (2.0 * a * b) > 1 || (a * a + b * b - c * c) / (2.0 * a * b) < -1))
            {
                throw Except("Arccos lawbreaker");
            }
            double aco = acos((a * a + b * b - c * c) / (2.0 * a * b));
            return aco;
        }
    }

    Point line_end(double x, double y, int r, double angle)
    {
        Point _packed;
        _packed.x = x + cos(angle) * r;
        _packed.y = y + sin(angle) * r;
        return _packed;
    }
};

class attempt
{
public:
    string id;
    features o_params;
    parameters params;
    double fit = -2;
    bool suc = true;
    bool ini = false;

    vector<Point> low_sys;
    vector<Point> after_sys;
    features aprox;

    attempt(string id, features o_params, bool ini)
    {
        this->id = id;
        this->o_params = o_params;
        this->ini = ini;

        do
        {
            try
            {

                create_system(low_sys, ini);

                suc = false;
            }
            catch (exception)
            {
                if (this->ini == false)
                {
                    suc = true;
                    fit = -1;
                }
            };
        } while (suc);
        if (fit == -2)
        {
            try
            {
                make_aprox();
                if (o_params.f6 != aprox.f6)
                {
                    fit = -1;
                }
                else
                {
                    fit = Sim(o_params, aprox);
                }
            }
            catch (exception)
            {
                fit = -1;
            }
        }
    };
    attempt(string id, features o_params, parameters params, bool ini)
    {
        this->id = id;
        this->o_params = o_params;
        this->ini = ini;
        this->params = params;

        do
        {
            try
            {
                create_system(low_sys, ini);
                suc = false;
            }
            catch (exception)
            {
                if (this->ini == false)
                {
                    suc = false;
                    fit = -1;
                }
            };
        } while (suc);
        if (fit == -2)
        {
            try
            {
                make_aprox();
                if (o_params.f6 != aprox.f6)
                {
                    fit = -1;
                }
                else
                {
                    fit = Sim(o_params, aprox);
                }
            }
            catch (exception)
            {
                fit = -1;
            }
        }
    };

    void mutate(int prob)
    {
        int hb, lb, amount;
        for (int i = 0; i < 13; i++)
        {
            int chance = randint(1, 100);
            if (chance < prob)
            {
                switch (i)
                {
                case 0:
                    hb = 15;
                    lb = 80;
                    break;
                case 1:
                    hb = 15;
                    lb = this->params[0];
                    break;
                case 2:
                    hb = 5;
                    lb = 60;
                    break;
                case 3:
                    hb = 0;
                    lb = 0;
                    break;
                case 4:
                    hb = 0;
                    lb = 0;
                    break;
                case 5:
                    hb = 1;
                    lb = 1;
                    break;
                case 6:
                    hb = 1;
                    lb = 10;
                    break;
                case 7:
                    hb = 0;
                    lb = 359;
                    break;
                case 8:
                    hb = 0;
                    lb = 60;
                    break;
                case 9:
                    hb = 0;
                    lb = 60;
                    break;
                case 10:
                    hb = 2;
                    lb = 2;
                    break;
                case 11:
                    hb = 1;
                    lb = 10;
                    break;
                case 12:
                    hb = 0;
                    lb = 359;
                    break;
                }
                amount = randint(-10, 10);
                if ((params[i] + amount) > hb && (params[i] + amount < lb))
                {
                    params[i] += amount;
                }
            }
        }
    }

private:
    void make_aprox()
    {
        resample(this->low_sys, 360, this->after_sys);
        _PCA pca_result = pca(this->after_sys);
        if (pca_result.Err)
        {
            throw Except("Value error");
        }
        Point d = pca_result.ANC - pca_result.COM;
        FVC(after_sys, pca_result.L_MAX, pca_result.L_MIN, d, pca_result.v0, aprox);
    };

    void create_system(vector<Point> &output, bool random = true)

    {
        if (random)
        {
            generate_params();
        }
        Bar bar1(params[0], params[1]);
        Bar bar2(params[2]);

        Anchor drive1(params[3], params[4], params[5], params[6], params[7]);
        Anchor drive2(params[8], params[9], params[10], params[11], params[12]);

        TwoBar motionSystem(drive1, drive2, bar1, bar2);
        double inputRange[360];
        for (int i = 0; i < 360; i++)
        {
            inputRange[i] = i * motionSystem.stepSize;
        }

        for (int i = 0; i < 360; i++)
        {
            output.push_back(motionSystem.end_path(inputRange[i], drive1, drive2, bar1, bar2));
        }
        return;
    };

    void generate_params()
    {
        params.data[3] = 0; // x_1
        params.data[4] = 0;
        params[5] = randint(1, 10);
        params[10] = randint(1, 10);
        params[6] = 1;
        params[7] = randint(0, 359);
        params[11] = 2;
        params[12] = randint(0, 359);
        bool neder = true;
        do
        {
            params[8] = randint(0, 60);
            params[9] = randint(0, 60);
            if (norm(params[3], params[4], params[8], params[9]) >= (params[11] + params[6]))
            {
                neder = false;
            };
        } while (neder);

        params[0] = randint(15, 80);
        params[1] = randint(15, params[10]);
        params[2] = randint(5, 60);
    }
};

class generations
{
private:
    const int gen_size = 10000;
    const int good = 128;
    const int mutate_prob = 60;
    const int cross_prob = 80;
    double best_fit = -1;
    vector<double> prev_best_fits;

    vector<Point> o_cords;
    vector<Point> curve;
    features fi;

    vector<attempt> Population;
    vector<attempt> prev_population;
    bool converge = false;
    int counter = 1;

    const int pool = 10;
    vector<thread> threads;
    const int chunk_size = gen_size / pool;
    const int next_chunk_size = gen_size / 2 / pool;
    mutex mtx;

    void get_coords(vector<Point> &curve)
    {
        ifstream file("outputs/base.cor");

        if (!file.is_open())
        {
            cerr << "Failed to open the file." << endl;
        }

        string point;
        bool xy = true;
        int cnt = 0;
        Point val;
        while (getline(file, point))
        {
            istringstream iss(point);
            string value;

            while (std::getline(iss, value, ','))
            {
                if (xy)
                {
                    val.x = stod(value);
                    xy = !xy;
                }
                else
                {
                    val.y = stod(value);
                    xy = !xy;
                }
            }
            curve.push_back(val);
            cnt++;
        }
        file.close();
        return;
    }

    void initial_pop()
    {

        cout << "\x1b[32m"
             << "Starting"
             << "\x1b[0m" << endl;
        for (int i = 0; i < pool; i++)
        {
            int start = i * chunk_size;
            int end = (i == pool - 1) ? gen_size : (i + 1) * chunk_size;
            threads.push_back(thread(bind(&generations::initial_pop_chunk, this, start, end)));
        }
        for (auto &thread : threads)
        {
            thread.join();
        }
    }

    void initial_pop_chunk(int start, int end)
    {
        string id;
        for (int i = start; i < end; i++)
        {
            id = "_" + to_string(i);
            std::lock_guard<std::mutex> lock(mtx);
            Population.push_back(attempt(id, fi, true));
        }
    }

    void next_gen()
    {
        this->Population.clear();
        int corssover;
        parameters f1;
        parameters f2;
        for (int i = 0; i < gen_size / 2; i++)
        {
            int a = randint(0, good);
            int b;
            do
            {
                b = randint(0, good);
            } while (a == b);
            parameters o1 = prev_population[a].params;
            parameters o2 = prev_population[b].params;
            f1 = o1;
            f2 = o2;
            if (randint(1, 100) < cross_prob)
            {
                corssover = randint(0, 12);

                for (int c = 0; c < corssover; c++)
                {
                    f1[c] = o2[c];
                    f2[c] = o1[c];
                }
            }
            attempt child1((to_string(this->counter) + "_" + to_string(i)), fi, f1, false);
            attempt child2((to_string(this->counter) + "_" + to_string(i + gen_size / 2)), fi, f2, false);
            int mut1 = randint(0, 99);
            int mut2 = randint(0, 99);

            if (mut1 > mutate_prob)
            {
                child1.mutate(mutate_prob);
            }

            if (mut2 > mutate_prob)
            {
                child2.mutate(mutate_prob);
            }
            Population.push_back(child1);
            Population.push_back(child2);
            if (i % 100  == 0)
            {
                cout << '\r' << i * 2 << "\t|  " << gen_size;
            }
                }
        cout << endl;
        return;
    }

    void selection()
    {
        this->prev_population.clear();
        std::sort(Population.begin(), Population.end(), generations::attempt_compare);

        best_fit = Population[0].fit;
        // save_coords(Population[0].low_sys, Population[0].id);
        prev_best_fits.push_back(best_fit);
        for (int i = 0; i < good; i++)
        {
            prev_population.push_back(Population[i]);
        }
        return;
    }

    void test_convergence()
    {
        if (prev_best_fits.back() > 0.99999)
        {
            converge = true;
            return;
        }
        return;
    }

public:
    static bool attempt_compare(const attempt &a, const attempt &b)
    {
        return (a.fit > b.fit);
    }

    generations()
    {

        get_coords(o_cords);
        resample(o_cords, 360, curve);
        _PCA pca_data = pca(curve);
        Point d = pca_data.ANC - pca_data.COM;
        FVC(curve, pca_data.L_MAX, pca_data.L_MIN, d, pca_data.v0, fi);

        initial_pop();
        bool converge = false;
        threads.clear();
        while (!converge)
        {

            selection();
            cout << best_fit << endl;
            save_params(Population[0].params, Population[0].id);
            next_gen();
            this->counter++;

            if (this->counter > 10)
            {
                test_convergence();
            }
        }
        std::sort(Population.begin(), Population.end(), generations::attempt_compare);
        cout << Population[0].fit << endl;
        cout << "Convergence in " << counter << " generations" << endl;
        save_params(Population[0].params, Population[0].id);
    };
};

int main()
{
    srand(time(0));

    cout.precision(20);
    generations process;
    system("pause");
    return 0;
}