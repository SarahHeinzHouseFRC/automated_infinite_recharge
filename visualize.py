#
# Copyright (c) 2020 FRC Team 3260
#


class Visualize:
    def __init__(self):
        pass

    def run(self, world_state, plan_state):
        draw = []

        # Draw blue rectangles around all balls
        counter = 0
        for ball in world_state['obstacles']['balls']:
            counter += 1
            x, y = ball[0]
            radius = ball[1]
            draw.append({
                'shape': 'box',
                'text': 'ball',
                'color': 'blue',
                'x': x,
                'y': y,
                'width': 2*radius,
                'height': 2*radius,
            })

        # Draw yellow rectangles around all obstacles
        for obstacle in world_state['obstacles']['others']:
            min_x, min_y = obstacle[0]
            max_x, max_y = obstacle[1]
            width = max_x - min_x
            height = max_y - min_y
            draw.append({
                'shape': 'box',
                'text': 'obstacle',
                'color': 'yellow',
                'x': min_x + width/2,
                'y': min_y + height/2,
                'width': width,
                'height': height
            })

        # Draw green line for nominal trajectory
        if plan_state['trajectory'] is not None:
            draw.append({
                'shape': 'line',
                'text': 'path',
                'color': 'green',
                'vertices': [
                    {
                        'x': world_state['pose'][0][0],
                        'y': world_state['pose'][0][1],
                    },
                    {
                        'x': plan_state['trajectory'][0],
                        'y': plan_state['trajectory'][1],
                    }
                ]
            })

        return draw
