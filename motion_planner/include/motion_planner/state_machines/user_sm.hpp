/***********************************************
*                                              *
*      user_sm.hpp                             *
*                                              *
*      Jesus Savage			                   *
*      Diego Cordero                           *
*	   Luis González                           *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 2019                      *
*                                              *
*                                              *
************************************************/


bool user_sm(Sensors sensors_data, MovementParams *params, Movement *movement) {

    float THRESHOLD = sensors_data.light_threshold;

    int state = params->state;
    
    float intensity = sensors_data.light_sensor_max;
    std::array<float, 8> light_values = sensors_data.light_readings;

    Direction light_direction = sensors_data.light_direction;
    Direction obstacle_direction = sensors_data.obstacle_direction;

    std::cout << "______________________________________________: SM STATE->" << state << std::endl;

    switch(state) {
        
        case 0: // SM STATE: TO DO
            *movement = generate_movement(FORWARD, *params);
            params->state = 1;
            break;

        case 1: // SM STATE: TO DO
            *movement = generate_movement(LEFT, *params);
            params->state = 0;
            break;

        default:
            *movement = generate_movement(NONE, *params);
            params->state = 0;
            break;
    }

    return true; // CONTINUE RUNNING: TRUE
}