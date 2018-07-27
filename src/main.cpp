#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <typeinfo>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

//start lane
int lane = 1;
int intended_lane = 1;

double desired_speed = 5.0; //mile
bool changing_lane = false;
bool safe_to_change = false;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
    uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          //*************************************************************//
          //**********Take the Control of the Car***********************//

          vector<double> ptsx;
          vector<double> ptsy;           

          //cout << "car_x: " << car_x << " car_y: " << car_y << " car_s: " << car_s << " car_d: " << car_d << endl << " car_yaw: " << car_yaw << " car_speed: " << car_speed << endl;

          int prev_size = previous_path_x.size();

          //previous_path_x returns around 35-48 at 49.5 mile/hour (probably due to cout slowing down computer?)
          //cout << "prev_size: " << prev_size << endl;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw); //TODO: Learn why we are converting to radian

          
          //If it is increased over around 55++ mile/h previous_path will increase so much that spline will crash
          //due to no space for new next x,y values to be added. 

          //# of Time steps to go back to calculate prev. steps
          int prev_limit = 2;

          //If prev. time steps are less than limits, create prev. points according to cars yaw angle
          if(prev_size < prev_limit){

            for(int i = prev_limit; i > prev_size; --i){
              ptsx.push_back(car_x-cos(car_yaw)*i);
              ptsy.push_back(car_y-sin(car_yaw)*i);
            } 
          }    
          else{
            ref_x = previous_path_x[prev_size-1]; // Last x element
            ref_y = previous_path_y[prev_size-1]; // Last y element

            double prev_ref_x =previous_path_x[prev_size-2]; //Prev. x element
            double prev_ref_y =previous_path_y[prev_size-2]; //Prev. y element
            ref_yaw = atan2(ref_y-prev_ref_y,ref_x-prev_ref_x); //Yaw angle according to last 2 points

            //TODO: Learn what does the below sentence mean? 
            //Use two points that make path tangent to the previous path's end point. 

            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);

            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);
          }

          //cout << "previous_path_x size: " << previous_path_x.size() << endl;

          //int lane = 1; //0 left, 1 mid, 2 right

          int forward_steps = 3;
          double step_dist = 30; //meter

          for (int i = 1; i <= forward_steps; i++)
          {
            vector<double> next_waypoint = getXY(car_s+step_dist*i, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            ptsx.push_back(next_waypoint[0]);
            ptsy.push_back(next_waypoint[1]);
          }   

          //ptsx: 5 
          //cout << "ptsx size: " << ptsx.size() << endl;

          //Shifting car reference angle to 0!
          for (int i = 0; i < ptsx.size(); i++)
          {
            //TODO: Learn what below sentence means
            //Shift car reference angle to 0 degrees
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          //namespace tk, class spline
          tk::spline s;

          try{s.set_points(ptsx,ptsy,true);}
          catch(int e){
            cout << "An exception occurred. Exception Nr. " << e << '\n';
          }

          vector<double> next_x_vals;
          vector<double> next_y_vals;   

          //Feed into previous path points from last time 
          for (int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
            //cout << "previous_path_x: " << previous_path_x[i] << endl;
          }

          //Calculate how many spline points needed to travel at desired speed
          //s(target_x) will give y, according to polynomial. We will take x,y points and
          // calculate a d-distance from hypotenus. Since distance = 0.02 sec * desired_speed (m/s^2)* # N points
          double target_x = 30.0; //or step_dist
          double target_y = s(target_x);
          //double target_dist = sqrt(pow(target_x,2)+pow(target_y,2));
          double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

          double  x_add_on = 0;

          //cout << "For will turn: " << ceil(desired_speed)-previous_path_x.size() << endl;
          
          for (int i = 1; i <= 50-previous_path_x.size(); i++)
          {
            double N = (target_dist/(0.02*desired_speed/2.24));
            double x_point = x_add_on+(target_x/N);
            double y_point = s(x_point); 

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            //Shift back to world coordinates
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }    

          bool possible_front_collision_detected = false;
          bool possible_back_collision_detected = false;
          
          vector< vector<double> > other_cars;
        //Check if our waypoints collide with another car!
          for(int i = 0; i < sensor_fusion.size(); i++)
          {              
            other_cars.push_back(sensor_fusion[i]);
            double other_cars_d = sensor_fusion[i][6];

            //Are we in the same lane?
            double dist_d = fabs(car_d - other_cars_d);

            if(dist_d<0.5){        

              double other_cars_s = sensor_fusion[i][5];
              double dist = other_cars_s - car_s; //Make distance with the car infront positive           
              //cout << "dist: " << dist << endl;   

              if(dist>10 & dist<30){
                possible_front_collision_detected = true;
                //cout << "front collision detected to id: " << sensor_fusion[i] << endl;              
              }else if(dist> -30 & dist < 0){
                possible_back_collision_detected = true;
                //cout << "back collision detected from id: " << sensor_fusion[i] << endl;   
              }
            }
          }         

          if (possible_front_collision_detected)
          {
            cout << "possible_front_collision_detected! changing_lane: " << changing_lane << endl;            
            desired_speed -= 0.25;

            if (!changing_lane)
            {
              safe_to_change = true;
              cout << "safe_to_change: " << safe_to_change << endl;
              //intended_lane = (lane+1)%3;

              for(int i=0; i<other_cars.size(); i++){
                //cout << "other cars: " << other_cars[i][0] << endl;    
                double d_dist_ = fabs(other_cars[i][6]-(2+intended_lane*4));
                double s_dist_ = fabs(other_cars[i][5] - car_s);       

                //Do we have enough space to change lane? (All possible lanes)
                if(s_dist_ > 3.0 & s_dist_ < 10.0){                                  

                  //Is there any car at the intended lane?
                  if(d_dist_ < 0.05){
                    cout << "car at intended lane id: " <<other_cars[i][0] << " d_dist_: " << d_dist_ << " s_dist_: " << s_dist_ << endl;
                    safe_to_change = true;
                  }else{
                    safe_to_change = false;
                    //Try other lane!
                    cout << "Intentded lane " << intended_lane <<" is not safe to change, trying lane: " << flush;
                    intended_lane = (intended_lane+1)%3;
                    cout << intended_lane << endl;
                    
                  }
                }
              }              
              if(safe_to_change){
                changing_lane = true;                
              }            
            }
          }
          else if(possible_back_collision_detected){
            if(desired_speed<49.5){
              desired_speed += 0.25;
            }         
          }
          else if(desired_speed<49.5){
            desired_speed += 0.25;
          } 

          //*****************************************//
          //*******Check if changing lane completed***//
          
          if(fabs(car_d-(2+intended_lane*4)) > 0.05 & changing_lane == true){
            lane = intended_lane;
          }else if( (fabs(car_d-(2+intended_lane*4)) < 0.05) & (changing_lane == true) ){
            cout << "Change lane completed" << endl;
            intended_lane = lane;
            changing_lane = false;
          }
          //cout << "desired_speed: " << desired_speed << endl;
          cout << "lane: " << lane << endl;
          cout << "intended lane: " << intended_lane << endl;  
          cout << "changing_lane: " << changing_lane << endl;
          cout << "**********************************" << endl;









          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
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
  size_t, size_t) {
  const std::string s = "<h1>Hello world!</h1>";
  if (req.getUrl().valueLength == 1) {
    res->end(s.data(), s.length());
  } else {
      // i guess this should be done more gracefully?
    res->end(nullptr, 0);
  }
});

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
