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
