/***********************************************
*                                              *
*      light_follower.hpp                      *
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


bool light_follower(LightSensorsData light_data, MovementParams params, Movement *movement) {
    
    int sensor_max_value = 0;

    float intensity = light_data.light_sensor_max;
    float THRESHOLD_FOLLOWER = light_data.light_threshold;
    std::array<float, 8> light_values = light_data.light_readings;
    float max_advance = params.max_advance;

    if(intensity > THRESHOLD_FOLLOWER) {
        movement->twist   = 0.0;
        movement->advance = 0.0;
        std::cout << "\n ****************** MOTION PLANNER: light_follower.-> LIGHT SOURCE REACHED ***************\n" << std::endl;
        return false; // CONTINUE RUNNING: FALSE
    }

    for (size_t i=1; i<light_values.size(); i++) {
        if (light_values[i] > light_values[sensor_max_value]) 
            sensor_max_value = i;
    }
    if (sensor_max_value > 4) {
        sensor_max_value = - (8 - sensor_max_value);
    }
    
    movement->twist   = sensor_max_value * M_PI / 16;
    movement->advance = max_advance;
    
    return true; // CONTINUE RUNNING: TRUE
}