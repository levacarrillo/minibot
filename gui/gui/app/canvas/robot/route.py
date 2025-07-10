class Route:
    def __init__(self, context):
        controller  = context.controller
        self.canvas = context.canvas
        self.color  = context.color
        self.robot  = context.robot

        self.list_poses  = []
        context.set_route(self)

    def init_list_poses(self, initial_pose):
        self.list_poses.append(initial_pose)

    def is_empty(self):
        return True if len(self.list_poses) == 0 else False

    def trace(self, initial_pose, final_pose):
        xi = initial_pose['x']
        yi = initial_pose['y']
        xf = final_pose['x']
        yf = final_pose['y']
        self.list_poses.append(final_pose)
        self.canvas.create_line(xi, yi, xf, yf, dash = (4, 4),
                    fill = self.color['trace'], tag = 'route')
        self.robot.plot()

    def delete(self):
        self.list_poses = []
        self.canvas.delete('route')
