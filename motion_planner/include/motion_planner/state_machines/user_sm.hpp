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


bool user_sm(LightSensorsData light_data, LaserSensorData laser_data, MovementParams m, Movement *movement) {
    int sensor_max_value = 0;
    
    bool continue_running;

    float intensity = light_data.light_sensor_max;
    float THRESHOLD_FOLLOWER = light_data.light_threshold;
    std::array<float, 8> light_values = light_data.light_readings;
    float max_advance = m.max_advance;


    if(intensity > THRESHOLD_FOLLOWER) {
        movement->twist   = 0.0;
        movement->advance = 0.0;
        std::cout << "\n ****************** Motion Planner: user_sm.-> Reached light source ***************\n" << std::endl;
        continue_running = false;
    } else {

        for (size_t i=1; i<light_values.size(); i++) {
            if (light_values[i] > light_values[sensor_max_value]) 
                sensor_max_value = i;
        }
        if (sensor_max_value > 4) {
            sensor_max_value = - (8 - sensor_max_value);
        }
    
        movement->twist   = sensor_max_value * M_PI / 16;
        movement->advance = max_advance;
        continue_running = true;
    }

    return continue_running;
}