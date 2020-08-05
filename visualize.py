#
# Copyright (c) 2020 FRC Team 3260
#


class Visualize:
    def __init__(self, config):
        self.ball_radius = config.ball_radius

    def run(self, world_state, plan):
        draw = []

        # Draw yellow rectangles around all balls
        num_balls = len(world_state['balls'])
        if num_balls > 50:
            raise ValueError(f'Sending {num_balls} balls will overload comms')

        for ball in world_state['balls']:
            x, y = ball
            radius = self.ball_radius
            draw.append({
                'shape': 'box',
                'text': 'ball',
                'color': 'yellow',
                'x': x,
                'y': y,
                'width': 2*radius,
                'height': 2*radius,
            })

        # Draw red rectangles around all obstacles
        for obstacle in world_state['obstacles']:
            min_x, min_y = obstacle[0]
            max_x, max_y = obstacle[1]
            width = max_x - min_x
            height = max_y - min_y
            draw.append({
                'shape': 'box',
                'text': 'obstacle',
                'color': 'red',
                'x': min_x + width/2,
                'y': min_y + height/2,
                'width': width,
                'height': height
            })

        # Draw green line for nominal trajectory
        if plan['trajectory'] is not None:
            draw.append({
                'shape': 'line',
                'text': 'trajectory',
                'color': 'green',
                'vertices': [[point[0], point[1]] for point in plan['trajectory']]
            })

        # Draw grid
        grid_drawer = {
                'shape': 'grid',
                'text': 'grid1',
                'color': 'darkgray',
                'cols': 100,
                'rows': 160,
                'cellSize': 0.1,
                'occupancy': []
        }
        occupancy = plan['grid'].occupancy
        for col in occupancy:
            for o in col:
                grid_drawer['occupancy'].append(int(100*o))
        draw.append(grid_drawer)
        return draw
