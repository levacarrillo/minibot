#ifndef STRUCTURES_HPP
#define STRUCTURES__HPP

enum Behaviors {
    NONE,
    UNKNOWN,
    LIGHT_FOLLOWER,
    SM_DESTINATION,
    SM_AVOID_OBSTACLES,
    SM_AVOIDANCE_DESTINATION,
    USER_SM
};

enum Direction {
    FORWARD,
    FORWARD_RIGHT,
    FORWARD_LEFT,
    LEFT,
    RIGHT,
    BACKWARD,
    BACKWARD_LEFT,
    BACKWARD_RIGHT,
    NO_DIRECTION
};

const std::map<std::string, Behaviors> behavior_names = {
    {"", NONE},
    {"UNKNOWN", UNKNOWN},
    {"LIGHT_FOLLOWER", LIGHT_FOLLOWER},
    {"SM_DESTINATION", SM_DESTINATION},
    {"SM_AVOID_OBSTACLES", SM_AVOID_OBSTACLES},
    {"SM_AVOIDANCE_DESTINATION", SM_AVOIDANCE_DESTINATION},
    {"USER_SM", USER_SM},
};

typedef struct Movement_ {
    float twist;
    float advance;
} Movement;

typedef struct LightSensorsData_ {
    std::array<float, 8> light_readings;
    Direction light_direction;
    int light_sensor_max_id;
    float light_sensor_max;
    float light_threshold;
} LightSensorsData;

typedef struct LaserSensorData_ {
    std::array<float, 512> laser_readings;
    Direction obstacle_direction;
    float laser_threshold;
} LaserSensorData;

typedef struct MovementParams_ {
    float max_advance;
    float max_turn_angle;
    int state;
} MovementParams;

#endif