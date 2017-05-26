#include <math.h>
#include <ctime>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi()
{
    return M_PI;
}
double deg2rad(double x)
{
    return x * pi() / 180;
}
double rad2deg(double x)
{
    return x * 180 / pi();
}

std::clock_t timenow;
std::clock_t timelast = clock();
double delt;

double cte;
double epsi;

double previous_s = 0;
double previous_a = 0;
double slim = 20; // speed limit

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}


// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

int main()
{
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode)
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);

        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
        {
            string s = hasData(sdata);
            if (s != "")
            {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry")
                {
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];

                    int timenow = clock();
                    delt = timenow-timelast;

                    /*
                    * TODO: Calculate steeering angle and throttle using MPC.
                    *
                    * Both are in between [-1, 1].
                    *
                    */
                    double steer_value = 0;
                    double throttle_value = 0;

                    // x, y, psi, v
                    Eigen::VectorXd initstate(4);
                    initstate << px, py, psi, v;

                    // steer_value, throttle_value
                    Eigen::VectorXd actuators(2);
                    actuators << previous_s, previous_a;

                    int Nsize = 25;

                    auto local_xy = mpc.Transform(ptsx, ptsy, initstate);

                    Eigen::VectorXd next_x(ptsx.size());
                    Eigen::VectorXd next_y(ptsx.size());

                    json msgJson;


                    // waypoints
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    for (int kk = 0; kk < ptsx.size(); kk ++)
                    {
                        next_x(kk) = local_xy(0,kk);
                        next_y(kk) = local_xy(1,kk);
                    }

                    // Fit cubic
                    auto coeffs = Eigen::VectorXd(4);
                    coeffs = polyfit(next_x, next_y, 3);
                    next_x_vals.clear();
                    next_y_vals.clear();

                    for (double x = 0.0; x <= 100.0; x += 10.0)
                    {
                        // plot 0-100 ahead
                        auto ref = polyeval(coeffs, x);
                        next_x_vals.push_back(x);
                        next_y_vals.push_back(ref);
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    double cte = 0.0; // cte and epsi calculated in mpc
                    double epsi = 0.0;

                    Eigen::VectorXd state(6);
                    // px, py, psi, speed, cte, epsi
                    state << 0, 0, 0, v, cte, epsi;

                    // Solve
                    auto opt_sol = mpc.Solve(state, coeffs, slim);

                    std::vector<double> steering;
                    std::vector<double> throttle;
                    std::vector<double> mpc_x_vals;
                    std::vector<double> mpc_y_vals;


                    for (int i = 0; i < Nsize-1 ; i++)
                    {
                        steering.push_back(opt_sol(0,i));
                        throttle.push_back(opt_sol(1,i));
                        mpc_x_vals.push_back(opt_sol(2,i));
                        mpc_y_vals.push_back(opt_sol(3,i));
                    }

                    msgJson["steering_angle"] = -steering[0];
                    msgJson["throttle"] = throttle[0];

                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    previous_s = -steering[0];
                    previous_a = throttle[0];

                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    //std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    timelast = timenow;

                }
            }
            else
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t)
    {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length)
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
