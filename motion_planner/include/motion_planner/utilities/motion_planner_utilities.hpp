
#ifndef MOTION_PLANNER_UTILITIES_HPP
#define MOTION_PLANNER_UTILITIES_HPP


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

#endif
