#ifndef STRUCTURES_HPP
#define STRUCTURES__HPP

enum Behaviors {
    USER_SM,
    UNKNOWN,
    NO_SELECTED,
    LIGHT_FOLLOWER,
    SM_DESTINATION,
    SM_AVOID_OBSTACLES,
    SM_AVOIDANCE_DESTINATION
};

enum Direction {
    LEFT,
    RIGHT,
    FORWARD,
    BACKWARD,
    FORWARD_RIGHT,
    FORWARD_LEFT,
    BACKWARD_LEFT,
    BACKWARD_RIGHT,
    NONE
};

const std::map<std::string, Behaviors> behavior_names = {
    {"", NO_SELECTED},
    {"USER_SM", USER_SM},
    {"UNKNOWN", UNKNOWN},
    {"LIGHT_FOLLOWER", LIGHT_FOLLOWER},
    {"SM_DESTINATION", SM_DESTINATION},
    {"SM_AVOID_OBSTACLES", SM_AVOID_OBSTACLES},
    {"SM_AVOIDANCE_DESTINATION", SM_AVOIDANCE_DESTINATION}
};

typedef struct Movement_ {
    float twist;
    float advance;
} Movement;

typedef struct Sensors_ {
    float light_threshold;
    float laser_threshold;
    float light_sensor_max;
    Direction light_direction;
    int light_sensor_max_id;
    Direction obstacle_direction;
    std::vector<float>   laser_readings;
    std::array<float, 8> light_readings;
} Sensors;

typedef struct MovementParams_ {
    float max_advance;
    float max_turn_angle;
    int state;
    int step;
    int max_steps;
} MovementParams;

#endif