def head_points():
    # POINTS MAGNITUDES RELATIVE TO ROBOT'S RADIUS
    return [ 
            { 'x': 2/3, 'y': - 1/3 },
            { 'x': 2/3, 'y':   1/3 },
            { 'x': 5/6, 'y':    0  }
        ]

def get_body(canvas, pose, radius, color):
    return canvas.create_oval(
        pose['x'] - radius,
        pose['y'] - radius,
        pose['x'] + radius,
        pose['y'] + radius,
        outline = color['robot'],
        fill    = color['robot'],
        width   = 1
    )

def get_hokuyo(canvas, pose, radius, color):
    return canvas.create_oval(
            pose['x'] - (radius / 5),
            pose['y'] - (radius / 5),
            pose['x'] + (radius / 5),
            pose['y'] + (radius / 5),
            outline = color['hokuyo'],
            fill    = color['hokuyo'],
            width   = 1
        )

def left_wheel_points():
    # POINTS MAGNITUDES RELATIVE TO ROBOT'S RADIUS
    return [
            {'x': -1/2, 'y': -5/6 },
            {'x':  1/2, 'y': -5/6 },
            {'x':  1/2, 'y': -3/6 },
            {'x': -1/2, 'y': -3/6 }
        ]

def right_wheel_points():
    # POINTS MAGNITUDES RELATIVE TO ROBOT'S RADIUS
    return [
            {'x': -1/2, 'y':  3/6 },
            {'x':  1/2, 'y':  3/6 },
            {'x':  1/2, 'y':  5/6 },
            {'x': -1/2, 'y':  5/6 },
        ]


