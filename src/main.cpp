#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <typeinfo>
#include <algorithm>
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

double desired_speed = 1.0; //mile

bool safe_to_change = false;
bool changing_lane = false;
int counter = 0;
int lane_change_counter = 0;

int next_move_counter = 0;

vector<double> lane_costs(3, 0);

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
          
          vector<double> next_waypoint;
          for (int i = 1; i <= forward_steps; i++)
          {            
            next_waypoint = getXY(car_s+step_dist*i, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            ptsx.push_back(next_waypoint[0]);
            ptsy.push_back(next_waypoint[1]);
          }   
          
          /*
          ptsx.push_back(getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y)[0]);
          ptsx.push_back(getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y)[0]);
          ptsx.push_back(getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y)[0]);

          ptsy.push_back(getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y)[1]);
          ptsy.push_back(getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y)[1]);
          ptsy.push_back(getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y)[1]);
          */

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

          bool collision_detected = false;
          bool slow_down = false;
          //bool possible_back_collision_detected = false;
          
          vector< vector<double> > cars_in_range(1, vector<double>(7, 0.0));
          vector< vector<double> > cars_in_intended_lane;


        //Check if our waypoints collide with another car!

          int forward_control_dist = 100;
          
          //***********Start counting other cars********************//
          //Find the cars in control range, add them to the list
          for(int i = 0; i < sensor_fusion.size(); i++)
          { 
            double other_cars_s = sensor_fusion[i][5];

            double distance_btw_collider = other_cars_s - car_s; //Make distance with the car infront positive     
            
            if(distance_btw_collider < forward_control_dist & distance_btw_collider > -10){
              cars_in_range.push_back(sensor_fusion[i]);             
            }                     
          } 
          cout << "# cars in range: " << cars_in_range.size() << endl;
          //***************End of counting other cars********************//      
          vector<int> cars_in_lanes(3,0.0);
          int cars_in_lane1 = 0;
          int front_car_id = -1;


          for (int i = 0; i < cars_in_range.size(); i++)
          {
            double d_dist = fabs(cars_in_range[i][6]-car_d);
            double s_dist = cars_in_range[i][5]-car_s;
            if (s_dist < 30 & s_dist > 0)
            {
              if (d_dist < 0.5)
              {
                slow_down = true;
                front_car_id = i;

              }              
              double intended_lane_distance = fabs(cars_in_range[i][6]-(2+4*intended_lane));
              //cout << "intended_lane_distance: " << intended_lane_distance << endl;
              //cout << "lane!= intended_lane: " << (lane!=intended_lane) << endl;
              if (intended_lane_distance < 0.5)
              {                
                //cout << "you are looking for this?" << endl;
                cars_in_intended_lane.push_back(cars_in_range[i]);
              }

            }  

            for(int j=0; j < 3; j++){
              if ( fabs(cars_in_range[i][6]-(2+(4*j))) < 0.5 & s_dist < 20 & s_dist > -20)
              {
                  //cout << "cars_in_lanes" << endl;
                cars_in_lanes[j] += 1;
              }              
            }          
            //*******************************//
            //*********Calculate Lane Costs*************//
            //Calculate other cars speed
            double other_car_speed = sqrt(pow(cars_in_range[i][3],2)+pow(cars_in_range[i][4],2));
            double lane_cost_by_car_in_range = log(forward_control_dist-s_dist);

            if (s_dist < 5 & d_dist < 5)
            {
              lane_cost_by_car_in_range *= 2;
            }            
            double total_lane_cost = lane_cost_by_car_in_range / other_car_speed;

            //pow((forward_control_dist - s_dist),2)

            if (cars_in_range[i][6] > 0 & cars_in_range[i][6] < 4)
            {
              lane_costs[0] += total_lane_cost;
            }else if(cars_in_range[i][6] > 4 & cars_in_range[i][6] < 8){
              lane_costs[1] += total_lane_cost;
            }else if(cars_in_range[i][6] > 8 & cars_in_range[i][6] < 12){
              lane_costs[2] += total_lane_cost;
            }        

          //*******End Of Cost Calculation*************//
          }
          cout << "cars_in_intended_lane.size(): " << cars_in_intended_lane.size() << endl;

          counter++;
          int car_speed_int = car_speed;
          if (counter%10 == 0)
          {
            for(int i = 0; i < lane_costs.size(); i++){
              lane_costs[i] /= 2.0;
              if(lane_costs[i] < 0.1) lane_costs[i] = 0;

            }
          }
          //
          cout << "counter: " << counter << endl;
          
          
          //********************************************//
          //************Lane Decision by Cost Begins***********//

          //**********Find lane with min cost*************//
          int min_pos = 0;  
          for(int i=0; i < lane_costs.size(); i++){
            if(lane_costs[i]< lane_costs[min_pos]) min_pos = i;            
          }          

          

          //***********Take Decision according to the costs calculated********************//
          //Avoid lane change first 200 frames, and decide change lane every 50 frames (50*20 milisec = 1 sec)
          if(counter > 200 & counter % 50 == 0){ 
            changing_lane = false;
            //Decide but not move until next_move_counter satisfied
            next_move_counter = counter + 10;

            intended_lane = min_pos;

            if (lane == 2 & intended_lane == 1)
            {
              if (cars_in_lanes[1] == 0)
              {
                intended_lane = 1;
              }else{intended_lane = lane;}
            }
            else if(lane == 1 & intended_lane == 2){
              if (cars_in_lanes[2] == 0)
              {
                intended_lane = 2;
              }else{intended_lane=lane;}
            }
            else if(lane == 1 & intended_lane == 0){
              if (cars_in_lanes[0] == 0)
              {
                intended_lane = 0;
              }else{intended_lane=lane;}
            }
            else if(lane == 0 & intended_lane == 1){
              if (cars_in_lanes[1] == 0)
              {
                intended_lane = 1;
              }else{intended_lane=lane;}
            }          
            else if (lane == 0 & intended_lane==2)
            {
              cout << "*********Two Lane Movement!***********" << endl;
              if (cars_in_lanes[1] == 0)
              {
                intended_lane = 1;
              }else{
                intended_lane = lane;}
              }
            else if (lane == 2 & intended_lane==0)
            {
              cout << "*********Two Lane Movement!***********" << endl;
              if (cars_in_lanes[1] == 0)
              {
                intended_lane = 1;
              }else{
                intended_lane = lane;}
            }

            else{intended_lane = lane;}

              if (intended_lane != lane)
              {
                changing_lane = true;
              }            
            }

            if (next_move_counter>counter)
            {
              cout << "Until Next Move: " << next_move_counter-counter <<endl;
            }
            if (counter == next_move_counter & car_speed < 35.0)
            {
              lane = intended_lane;
            }
            /*
            
            if (counter > 200 & fabs((2+(4*intended_lane))-car_d)> 0.1)
            {
              changing_lane = true;
            }
            
            /*
            for (int i = 0; i < lane_costs.size(); i++)
            {
              cout << "lane cost["<<i<<"] cost: " << lane_costs[i] << endl;
            }
            for (int i = 0; i < cars_in_lanes.size(); i++)
            {
              cout << "cars in lanes["<<i<<"]: " << cars_in_lanes[i] << endl;
            }
            */

          //********************************************//
          //************Lane Decision by Cost Ends***********//

            if (counter%1 == 0)
              {
                cout << "changing_lane: " << changing_lane << endl;
                cout << "****** intended_lane: " << intended_lane << endl;
                cout << "//**********************************//" << endl;
             }

          //************* Acceleration, Deacceleration Start**************//
          if (slow_down)
          {
            double front_car_speed = sqrt(pow(cars_in_range[front_car_id][3],2)+pow(cars_in_range[front_car_id][4],2));
            if (car_speed/2.54 > front_car_speed)
            {
              cout << "front_car_speed: " << front_car_speed << endl;
              double ratio = cars_in_range[front_car_id][5]-car_s;
              desired_speed -= 10/ratio;
            }            
          }   
          else if (changing_lane & car_speed > 35.0)
          {
            desired_speed -= 0.254;
          }   
          else if (desired_speed < 49.5 & !changing_lane)
          {
            desired_speed += .254;
          }

          //************* Acceleration, Deacceleration Ends**************//


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
