/***********************************************
*                                              *
*      sm_destination.hpp                      *
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


bool sm_destination(Sensors light_data, MovementParams *params, Movement *movement) {

    int state = params->state;
    float intensity = light_data.light_sensor_max;
    float THRESHOLD_DEST = light_data.light_threshold;
    Direction light_direction = light_data.light_direction;

    std::cout << "______________________________________________: SM STATE->" << state << std::endl;
    switch (state) {
        case 1: // SM STATE: CHECK FOR THRESHOLD REACHED
            if (intensity > THRESHOLD_DEST) {
                *movement = generate_movement(NONE, *params);
                std::cout << "\n ****************** MOTION PLANNER: sm_destination.-> LIGHT SOURCE REACHED ***************\n" << std::endl;
                params->state = 1;
                return false; // CONTINUE RUNNING: FALSE
            } else {
                *movement = generate_movement(FORWARD, *params);
                params->state = 2;
            }
            break;
        
        case 2: // SM STATE: CHECK FOR DESTINATION
            if (light_direction == BACKWARD_RIGHT) {
                *movement = generate_movement(RIGHT, *params);
                params->state = 3;
            } else if (light_direction == BACKWARD_LEFT) {
                *movement = generate_movement(LEFT, *params);
                params->state = 4;
            } else if (light_direction == FORWARD_RIGHT) {
                *movement = generate_movement(FORWARD, *params);
                params->state = 3;
            } else if (light_direction == FORWARD_LEFT) {
                *movement = generate_movement(FORWARD, *params);
                params->state = 4;
            } else if (light_direction == FORWARD) {
                *movement = generate_movement(FORWARD, *params);
                params->state = 1;
            }
            break;
        
        case 3: // SM STATE: TURN RIGHT
            *movement = generate_movement(RIGHT, *params);
            params->state = 1;
            break;
        
        case 4: // SM STATE: TURN LEFT
            *movement = generate_movement(LEFT, *params);
            params->state = 1;
            break;
        
        default:
            *movement = generate_movement(NONE, *params);
            params->state = 1;
            break;
    }

    return true; // CONTINUE RUNNING: TRUE
}