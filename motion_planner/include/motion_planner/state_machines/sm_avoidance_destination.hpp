/***********************************************
*                                              *
*      sm_avoidance_destination.hpp            *
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


bool sm_avoidance_destination(Sensors sensors_data, MovementParams *params, Movement *movement) {

    float THRESHOLD = sensors_data.light_threshold;

    int state = params->state;
    
    float intensity = sensors_data.light_sensor_max;
    std::array<float, 8> light_values = sensors_data.light_readings;

    Direction light_direction = sensors_data.light_direction;
    Direction obstacle_direction = sensors_data.obstacle_direction;
    

    std::cout << "______________________________________________: SM STATE->" << state << std::endl;
    
    switch(state) {

        case 0: // SM STATE: CHECK FOR GOAL 
            if (intensity > THRESHOLD) {
                *movement = generate_movement(NONE, *params);
                std::cout << "\n ****************** MOTION PLANNER: sm_avoidance_destination.-> LIGHT SOURCE REACHED ***************\n" << std::endl;
                params->state = 0;                
                return false; // CONTINUE RUNNING: FALSE
            } else {
                *movement = generate_movement(FORWARD, *params);
                params->state = 1;
            }
            break;

        case 1: // SM STATE: CHECK FOR OBSTACLES
            if (obstacle_direction == NONE) {
                *movement = generate_movement(FORWARD, *params);
                params->state = 7;
            } else {
                *movement = generate_movement(NONE, *params);
                if (obstacle_direction == FORWARD_RIGHT) params->state = 4;
                else if (obstacle_direction == FORWARD_LEFT) params->state = 2;
                else if (obstacle_direction == FORWARD) params->state = 6;
            }
            break;

        case 2: // SM STATE: GO BACKWARD, OBSTACLE IN FRONT ON THE LEFT
            *movement = generate_movement(BACKWARD, *params);
            params->state = 3;
            break;

        case 3: // SM STATE: TURN RIGHT
            *movement = generate_movement(RIGHT, *params);
            params->state = 0;
            break;

        case 4: // SM STATE: GO BACKWARD, OBSTACLE IN FRONT ON THE RIGHT
            *movement = generate_movement(BACKWARD, *params);
            params->state = 5;
            break;

        case 5: // SM STATE: TURN LEFT
            *movement = generate_movement(LEFT, *params);
            params->state = 0;
            break;

        case 6: // SM STATE: GO BACKWARD, OBSTACLE IN FRONT
            *movement = generate_movement(BACKWARD, *params);
            params->state = 0;
            break;

        case 7: // SM STATE: CHECK FOR DESTINATION
            if (light_direction == BACKWARD_RIGHT) {
                *movement = generate_movement(RIGHT, *params);
                params->state = 3;
            } else if (light_direction == BACKWARD_LEFT) {
                *movement = generate_movement(LEFT, *params);
                params->state = 5;
            } else if (light_direction == FORWARD) {
                *movement = generate_movement(FORWARD, *params);
                params->state = 1;
            } else if (light_direction == FORWARD_RIGHT) {
                *movement = generate_movement(FORWARD, *params);
                params->state = 3;
            } else if (light_direction == FORWARD_LEFT) {
                *movement = generate_movement(FORWARD, *params);
                params->state = 5;
            }
            break;

        default:
            *movement = generate_movement(NONE, *params);
            params->state = 0;
            break;
    }

    return true; // CONTINUE RUNNING: TRUE
}