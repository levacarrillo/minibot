constants = {
    'sigma': 3, # STANDAR DEVIATION OF DISTRIBUTION FOR NOISE
    'angle_increment': 1, # ANGLE FOR TURN IN DEGREES
    'distance_increment': 2, # DISTANCE FOR DISPLACEMENT IN PX
    # POINTS MAGNITUDES RELATIVE TO ROBOT'S RADIUS
    'head_rel_points': [{ 'x': 2/3, 'y': - 1/3 },
                        { 'x': 2/3, 'y':   1/3 },
                        { 'x': 5/6, 'y':  0.0  }],

    'left_wheel_rel_points': [{'x': -1/2, 'y': -5/6 },
                              {'x':  1/2, 'y': -5/6 },
                              {'x':  1/2, 'y': -3/6 },
                              {'x': -1/2, 'y': -3/6 }],

    'right_wheel_rel_points': [{'x': -1/2, 'y':  3/6 },
                               {'x':  1/2, 'y':  3/6 },
                               {'x':  1/2, 'y':  5/6 },
                               {'x': -1/2, 'y':  5/6 }]
}