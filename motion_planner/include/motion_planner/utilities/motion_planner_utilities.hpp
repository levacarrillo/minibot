
#ifndef MOTION_PLANNER_UTILITIES_HPP
#define MOTION_PLANNER_UTILITIES_HPP


Behaviors find_behavior(std::string behavior) {
  if(behavior_names.find(behavior) != behavior_names.end()) {
    return behavior_names.at(behavior);
  }
    return UNKNOWN;
};

std::vector<std::string> get_behavior_list() {
  std::vector<std::string> behavior_list;
  for(const auto& pair : behavior_names) {
    std::string name = pair.first;
    behavior_list.push_back(name);
  }

  return behavior_list;
}

Direction get_obstacle_direction(LaserSensorData scan_data) {
  bool obstacle_right = false;
  bool obstacle_left  = false;
  int aux = 0;

  if (scan_data.laser_readings.size() % 2 != 0 ) {
    aux = 1;
    if (scan_data.laser_readings[scan_data.laser_readings.size() - aux / 2] < scan_data.laser_threshold) {
      return FORWARD;
    }
  }

  for (int i=0; scan_data.laser_readings.size() - aux / 2; i++) {
    if (scan_data.laser_readings[i] < scan_data.laser_threshold) {
      obstacle_right = true;
    }
  }
  for (int i=scan_data.laser_readings.size() - aux / 2; (i=scan_data.laser_readings.size()); i++) {
    if (scan_data.laser_readings[i] < scan_data.laser_threshold) {
      obstacle_left = true;
    }
  }

  if (obstacle_right && obstacle_left) return FORWARD;
  if (obstacle_left)  return FORWARD_LEFT;
  if (obstacle_right) return FORWARD_RIGHT;

  return NO_DIRECTION;
}

Direction get_light_direction(std::array<float, 8> light_readings) {
       
    int sensor = 0;

    for (int i=1; i<8; i+=2)
        if (light_readings[i] > light_readings[sensor]) sensor = i;

    if(sensor == 0)      return FORWARD;
    else if(sensor == 1) return FORWARD_LEFT;
    else if(sensor == 3) return BACKWARD_LEFT;
    else if(sensor == 5) return BACKWARD_RIGHT;
    else if(sensor == 7) return FORWARD_RIGHT;
    else return RIGHT;
};


#endif
