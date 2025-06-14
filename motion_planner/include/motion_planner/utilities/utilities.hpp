
#ifndef UTILITIES_HPP
#define UTILITIES_HPP


Movement generate_movement(Direction direction, MovementParams movement) {
    Movement output;

    switch(direction) {
        case NONE:
            output.twist   = 0.0f;
            output.advance = 0.0f;
            break;

        case FORWARD:
            output.twist   = 0.0f;
            output.advance = movement.max_advance;
            break;

        case BACKWARD:
            output.twist   = 0.0f;
            output.advance = -movement.max_advance;
            break;

        case LEFT:
            output.twist   = movement.max_turn_angle;
            output.advance = 0.0f;
            break;

        case RIGHT:
            output.twist   = -movement.max_turn_angle;
            output.advance = 0.0f;
            break;

        default:
            std::cout << "DIRECTION " << direction << " NOT VALID" << std::endl;
            output.twist   = 0.0f;
            output.advance = 0.0f;
            break;
    }

    return(output);
};

#endif
