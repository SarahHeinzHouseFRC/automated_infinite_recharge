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
                'color': 'yellow',
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
                'color': 'red',
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
                'vertices': [{'x': point[0], 'y': point[1]} for point in plan_state['trajectory']]
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
        for col in plan_state['grid'].grid:
            for cell in col:
                grid_drawer['occupancy'].append(1 if cell.occupied else 0)
        draw.append(grid_drawer)
        return draw
