/***********************************************
*                                              *
*      sm_avoid_obstacles.hpp                  *
*                                              *
*      Diego Cordero                           *
*      Jesus Savage			                   *
*	   Luis González                           *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 2019                      *
*                                              *
*                                              *
************************************************/


void sm_avoid_obstacles(LaserSensorData laser_data, MovementParams *params, Movement *movement) {
    
    int state = params->state;
    Direction obstacle_direction = laser_data.obstacle_direction;

    std::cout << "______________________________________________  SM_AVOID_OBSTACLES: STATE->" << state << std::endl;
    switch(state) {

        case 0: // SM STATE: CHECK FOR DESTINATION
            if (obstacle_direction == NONE) {
                *movement = generate_movement(FORWARD, *params);
                params->state = 0;
            } else {
                *movement = generate_movement(NONE, *params);

                if (obstacle_direction == FORWARD_RIGHT) params->state = 1;
                else if (obstacle_direction == FORWARD_LEFT) params->state = 3;
                else if (obstacle_direction == FORWARD) params->state = 5;
            }
            break;

        case 1: // SM STATE: GO BACKWARD, OBSTACLE IN FRONT ON THE RIGHT
            *movement = generate_movement(BACKWARD, *params);
            params->state = 2;
            break;

        case 2: // SM STATE: TURN LEFT
            *movement = generate_movement(LEFT, *params);
            params->state = 0;
            break;
        
        case 3: // SM STATE: GO BACKWARD, OBSTACLE IN FRONT ON THE LEFT
            *movement = generate_movement(BACKWARD, *params);
            params->state = 4;
            break;

        case 4: // SM STATE: TURN RIGHT
            *movement = generate_movement(RIGHT, *params);
            params->state = 0;
            break;

        case 5: // SM STATE: BACKWARD, OBSTACLE IN FRONT
            *movement = generate_movement(BACKWARD, *params);
            params->state = 0;
            break;

        default:
            *movement = generate_movement(NONE, *params);
            params->state = 0;
            break;
    }
}