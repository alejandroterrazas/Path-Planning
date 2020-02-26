#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


//decrease the forward speed (never slower than zero)
void slow_down(double rate) {
   if ((ref_vel-slow_factor) > target_velocity) {ref_vel -= rate;}
}  

//increase the forward speed (never faster than max_speed)
void speed_up(double rate) {
   //std::cout << "speed_up" << std::endl;
   if ((ref_vel+rate) < target_velocity) {ref_vel += rate;}
}

//return points ready for the spline function to create a trajectory
trajectory_points generate_points(double car_x, double car_y, 
                                  double car_yaw, double car_s, int prev_size, 
                                  vector<double> previous_path_x, 
                                  vector<double> previous_path_y,
                                  const vector<double> &maps_s, 
                                  const vector<double> &maps_x, 
                                  const vector<double> &maps_y) {
  
    //reference y,y, yaw states
    //where the car is or the previous endpoint
    trajectory_points p;
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);
          
    if (prev_size < 2) {             
       double prev_car_x = car_x - cos(car_yaw);
       double prev_car_y = car_y - sin(car_yaw);
             
       p.x.push_back(prev_car_x);
       p.x.push_back(car_x);
            
       p.y.push_back(prev_car_y);
       p.y.push_back(car_y);          
    } else {              
       ref_x = previous_path_x[prev_size-1];
       ref_y = previous_path_y[prev_size-1];
            
       double ref_x_prev = previous_path_x[prev_size-2];
       double ref_y_prev = previous_path_y[prev_size-2];
       ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

       p.x.push_back(ref_x_prev);
       p.x.push_back(ref_x);
            
       p.y.push_back(ref_y_prev);
       p.y.push_back(ref_y);
                
    }
 
    double lane_midpoint = target_lane + (current_lane - target_lane)/2;
    vector<double> way_point_locations;
    way_point_locations.push_back(static_cast<double>(current_lane));
    way_point_locations.push_back(lane_midpoint);
    way_point_locations.push_back(static_cast<double>(target_lane));
   
    //create waypoints for the trajectory; use current lane and target lane
    //if current lane == target lane you stay in lane
    for (int i = 0; i < number_waypoints; i++) {
       double factor = 20 + (i*30) + ref_vel/4;
       auto way_point = getXY(car_s+factor,(2+4*way_point_locations[i]),
                                     maps_s,
                                     maps_x,maps_y);
       p.x.push_back(way_point[0]);
       p.y.push_back(way_point[1]);
    }

 
    for (int i = 0; i < p.x.size(); i++) {
            
        //a transform to the car's reference frame
        //shift and rotation
        double shift_x = p.x[i]-ref_x;
        double shift_y = p.y[i]-ref_y;
            
        p.x[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw)); 
        p.y[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
     } 
  
  
     p.ref_x = ref_x;
     p.ref_y = ref_y;
     p.ref_yaw = ref_yaw;
     
     current_lane = target_lane;
     
     return p;
}

//returns the lane from the sensor d value
//find a way to simplify this calculation
int return_lane(float d) {
    //std::cout << "d: " << d << std::endl;
    if (d >= 0 && d < 4) {return 0;}
    else if (d >= 4 && d < 8) {return 1;}
    else if (d >= 8 && d <= 12) {return 2;}
    else {return -1;}
}


//return information for each car in the vicinity
//the fusion object contains sensor data passed from main
//uses car_information struct from helpers.h
car_information check_car(int prev_size, double car_s, vector<double> fusion) {
     double vx = fusion[3];
     double vy = fusion[4];
     double speed = sqrt(pow(vx,2)+pow(vy,2));
     double check_car_s = fusion[5];
     car_information car_info;
     //if using previous points can project s values out to future             
     check_car_s += ((double)prev_size*.02*speed);
     car_info.distance = check_car_s-car_s;
     car_info.speed = speed;
     return car_info;     
}

//returns infornation on the lane provided
//lane_information is a struct in helpers.h
//contains separate inforation about cars ahead and cars behind
lane_information lane_info(vector<car_information> lane) {
      //lane is a vector of car objects'      
      vector<double> distances;
      vector<double> speeds;
      
      for (unsigned int i = 0; i < lane.size(); i++) {
        distances.push_back(lane[i].distance);
        speeds.push_back(lane[i].speed);
      }
  
      //returns the 
      vector<long unsigned int> speed_indices = sort_indexes(speeds);
      //std::cout << "print lane info: " << std::endl;
   
      //slowest_ahead is the slowest car in the lane ahead
      double slowest_ahead = 100;  //initialize high and find minimum
      for (unsigned int i = 0; i < speed_indices.size(); i++) {
         if (distances[speed_indices[i]] > 0) {  ///these are objects ahead in the lane
            double speed = speeds[speed_indices[i]];
            slowest_ahead = (speed < slowest_ahead) ? speed : slowest_ahead; 
         }
      }
  
      //sort the distances to find the closest car ahead and closest car behind
      //std::sort (distances.begin(), distances.end());
      //partition into separate vectors for ahead and behind
      auto it = std::partition(distances.begin(), distances.end(), 
                               [](double val){ return val > 0; });
      auto ahead = vector<double> (distances.begin(), it);
      auto behind = vector<double> (it, distances.end());
  
      //sort ahead and behind  
      std::sort (ahead.begin(), ahead.end());
      std::sort (behind.begin(), behind.end());
  
      //if lane is empty, make behind speed 0;otherwise, take speed of closest
      
      double behind_speed = -100.;
      if (!behind.empty()) {
         auto indx = std::find(distances.begin(),distances.end(), 
                               behind[behind.size()-1]) - distances.begin();
         behind_speed = speeds[indx];
      }
     
      //if lane is empty, make ahead speed 100; otherwise, take speed of closest
      double ahead_speed = 100;
      if (!ahead.empty()) {
         auto indx = std::find(distances.begin(), 
                               distances.end(), ahead[0]) - distances.begin();
         ahead_speed = speeds[indx];
      }
   
     
      //creat lane_information struct 
      //ahead variables are for objects ahead, behind for objects behind
      lane_information lane_data;
      
      lane_data.ahead = (ahead.size()>0) ? ahead[0] : ignore_distance+1;
      lane_data.behind = (behind.size()>0) ? behind[behind.size()-1] : -ignore_distance-1;
      lane_data.behind_speed = (behind.size()>0) ? behind_speed : -100;
      lane_data.ahead_speed = (ahead.size()>0) ? ahead_speed : 100.;
      lane_data.slowest_ahead = slowest_ahead;
      lane_data.gap_size = lane_data.ahead - lane_data.behind;
      lane_data.gap_center = lane_data.ahead - lane_data.gap_size/2;
      lane_data.number_of_cars_ahead = ahead.size();
  
      return lane_data;
}
//utility for printing lane information
void print_lane_info(string message, lane_information lane_info) {
      std::cout << message << std::endl;
      std::cout << "ahead: " << lane_info.ahead << " ahead_speed: " << lane_info.ahead_speed 
                << "behind: " << lane_info.behind << " behind_speed: " << lane_info.behind_speed  
                << "gap size: " << lane_info.gap_size << "gap center: " << lane_info.gap_center
                << std::endl;
}


//function to evaluate the value of a lane change (like cost)
//take current lane and target lane
double eval_target_lane(vector<car_information> target, vector<car_information> current) {
  
  //bool lan;  //default to open
  double score = 0;
  
  if (!target.empty()) {
      lane_information target_info = lane_info(target);
      lane_information current_info = lane_info(current);
      
      bool room_available = (target_info.ahead > min_room_ahead) && 
                            (target_info.behind < -min_room_behind);
 
      //room available determines if there is room to make a change--overrides everything
      bool room_ahead (target_info.ahead > min_room_ahead);
      bool room_behind (target_info.behind < -min_room_behind);  
     // bool tight_behind (-min_room_behind < target_info.behind < -5);
         
      if (!room_available) {return 0;}    
      
      //create a score that gives the desirability of a proposed lane change
      //add these together and see if they are bigger than a threshold (can be passed)
      
      score += (target_info.ahead_speed-current_info.ahead_speed);  //closest target faster?
      score += (target_info.ahead-current_info.ahead)/5;  //closest target father ahead?
      score += (ref_vel - target_info.behind_speed)/5;  //going faster than closest target behind
      score += (target_info.slowest_ahead-current_info.slowest_ahead)/5; //slow ahead
      score += (target_info.ahead-target_info.behind)/5; //gap size big
      score += same_lane_slow;
      score -= (target_info.number_of_cars_ahead)*10;
      
      
      //std::cout << "score: " << score 
      //          << "same_lane_slow: " << same_lane_slow << std::endl;
    
   }
  
   return score;
}
  


int main() {
  
  uWS::Hub h;
 

  // Vectors to hold waypoints made up of x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
  

  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
                
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
                 
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);
      if (s != "") {
        auto j = json::parse(s);        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
   
          // j[1] is the data JSON object
          
          // Car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
     
          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          
         // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
              
          int prev_size = previous_path_x.size();
         
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          
          //initialize a vector to hold vectors of car_information for each lane
 
          vector<vector<car_information>> lane_data;
          
          for (int i = 0; i < number_lanes; i++) {
             lane_data.push_back(vector<car_information> ());
          }
          
          for (int i = 0; i < sensor_fusion.size(); i++) {

            vector<double> fusion = sensor_fusion[i]; 
            float d = fusion[6];
            if ((d < 0) && (d>12)) {break;}  //don't process data outside of the lane
            int lane_number = return_lane(d);
            if (lane_number == -1) {break;}  //don't process lanes outside of 0,1,2
               
            car_information car_data = check_car(prev_size,car_s,fusion);
            
            //drop cars too far ahead or too far behind--ignore_distnace
            if (abs(car_data.distance)<ignore_distance) {
               lane_data[lane_number].push_back(car_data);
            } 
          }  //end sensor_fusion set
          
          too_close = true;
          too_slow = true;
          target_lane_open = false;
          target_velocity = max_speed;  //if current lane is empty get target_vel
          if (!lane_data[current_lane].empty()) {
              lane_information current_lane_info = lane_info(lane_data[current_lane]);
              
              //check if the current object is too close
              distance_to_target = current_lane_info.ahead;
              safe_distance = (current_lane_info.ahead < 8) ? 8: ref_vel/2;
              
              too_close = (current_lane_info.ahead < safe_distance);
              if (too_close) {          
                 target_velocity = current_lane_info.ahead_speed; 
                 lane_change_desired = true;
                // std::cout << "LOOK OUT!" << std::endl;
                               
                 slow_factor = (ref_vel-target_velocity)/distance_to_target;  
                 slow_factor *= slow_multiplier;
              } 
            
              if (!stuck && !too_close) {lane_change_desired = false;}
                        
              if (lane_change_desired) {
                 double score;
                 change_lane = false;
                 //hard coded logic to determine next lane
                 if (current_lane == 0) {                    
                   score = eval_target_lane(lane_data[1],
                                            lane_data[current_lane]);
                   if (score>score_threshold) {target_lane = 1;change_lane=true;}
                 } else if (current_lane == 2) {
                    score = eval_target_lane(lane_data[1],
                                                        lane_data[current_lane]);
                    if (score>score_threshold) {target_lane = 1;change_lane=true;}
                 } else if (current_lane == 1) {  //0 or 2 possible               
                     double score0 = eval_target_lane(lane_data[0],
                                                      lane_data[current_lane]);
                    
                     double score2 = eval_target_lane(lane_data[2],
                                                      lane_data[current_lane]);
                     int target = (score2 > score0) ? 2 : 0;
                     score = (score2 > score0) ? score2 : score0;
                      if (score>score_threshold) {target_lane = target;change_lane=true;}
                 } 
                
                //use to count cycles of being stuck
                 if ((!change_lane) && (ref_vel < 40)) {
                     same_lane_slow += .1;
                 }
                
                 if (change_lane) {same_lane_slow = 0;look_back=false;}
                /*
                 if (change_lane) {
                     std::cout << "change_lane-"  
                               << "same_lane_slow: " << same_lane_slow << std::endl;
                 } else {
                   std::cout << "NO CHANGE-" 
                             << "same_lane_slow: " << same_lane_slow << std::endl;
                 }
                 */            
     
              } //ahead not < safe distance
            
          }  //end else !empty
          
          //determine if we are stuck; look_back 
          stuck = (same_lane_slow > 1.); 
          if (stuck && look_back) {
             if (follow_distance < 35) {
                 follow_distance += .2;
                 target_velocity -= .2;
                 too_slow = false;
             } else {look_back = false;}  //if greater than 35
          } else if (stuck && !look_back) {  //looking forward
             if (follow_distance > 8) {
                 follow_distance -= .2;
                 target_velocity += .2;
                 too_close = true;
             } else {look_back = true;}
          }
          
          if (stuck) {lane_change_desired = true;}
         
          if (!stuck) {  //reset
            follow_distance = safe_distance;
            lane_change_desired = false;
          }
           
          
          if (too_slow) {speed_up((target_velocity-ref_vel)/50);}
          if (too_close) {slow_down((ref_vel-target_velocity)/distance_to_target);}
                                    
          auto points = generate_points(car_x, car_y, car_yaw, car_s, prev_size,
                                   previous_path_x, previous_path_y,
                                   map_waypoints_s,
                                   map_waypoints_x, map_waypoints_y);
              
          tk::spline s;
                  
          s.set_points(points.x, points.y);
          
          //Define the actual x,y ponits we will use with the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //Start with all the previous ponts from last time
          for (int i = 0; i < previous_path_x.size(); i++) {           
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double target_x = 30;
          double target_y = s(target_x);  
          double target_dist = sqrt(pow(target_x,2)+pow(target_y,2));
          
          double x_add_on = 0;
          
          for (int i = 0; i<=50-previous_path_x.size(); i++) {
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);
            
            //x_point is the number of hash marks on x axis
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
           // std::cout << "xref, yref" << x_ref << y_ref << std::endl;
            //rotate back to global coordinates rotating it earlier
            x_point = (x_ref*cos(points.ref_yaw)-y_ref*sin(points.ref_yaw));
            y_point = (x_ref*sin(points.ref_yaw)+y_ref*cos(points.ref_yaw));
           
            x_point += points.ref_x;
            y_point += points.ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            
          }
          
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          //std::cout << "after msgJson" << std::endl;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
       
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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