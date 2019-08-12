#include <cmath>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main() {
    uWS::Hub h;

    PID pid;

    //test different parameter settings

    //P-low --> good behaviour, but in first curve CTE > 1, in 2. curve > 2.5, in 3. curve < -2.6 and out of lane marks
//    pid.Init(0.1, 0.00001, 2.0);

    //P-high --> after a few seconds big problems to stay in the middle of the road
//    pid.Init(5, 0.00001, 2.0);

    //I-low --> first curve CTE = .7, 2.curve CTE = 1.1, 2.curve CTE = -1.3
    pid.Init(0.3, 0.00001, 3.5);

    //I-high makes steering very agressive, right after start out of lane marks
//    pid.Init(0.1, 5, 2.0);

    //D-low --> looks good, first curve CTE > 1.6, 2.curve > 2 and out of lane marks, 3.curve CTE<-3
//    pid.Init(0.1, 0.00001, 0.5);

    //D-high --> first curve CTE < 0.5, 2.curve CTE < 1., 3.curve CTE > -1.1  --> looks like a nice setting
//    pid.Init(0.1, 0.00001, 5.0);


    h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(string(data).substr(0, length));

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<string>());
                    double speed = std::stod(j[1]["speed"].get<string>());
                    double angle = std::stod(j[1]["steering_angle"].get<string>());
                    double steer_value = 0;
                    /**
                     * Calculate steering value here, the steering value is [-1, 1].
                     */

                    pid.UpdateError(cte);
                    double totalError = pid.TotalError();

                    if (totalError > 1) {
                        steer_value = 1;
                    } else if (totalError < -1) {
                        steer_value = -1;
                    } else {
                        steer_value = totalError;
                    }

                    //TODO use another PID controller to control the speed!
                    double accel = 0.2;
                    if (fabs(cte) > 0.5 && fabs(angle) > 4.0 && speed > 20.0) {
                        accel = -1.0;
                    }

                    // DEBUG
                    std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = accel;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    std::cout << "------------" << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket message if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}