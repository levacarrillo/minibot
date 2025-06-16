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

