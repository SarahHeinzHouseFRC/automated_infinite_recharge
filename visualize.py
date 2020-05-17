#
# Copyright (c) 2020 FRC Team 3260
#


class Visualize:
    def __init__(self):
        pass

    def run(self, world_state, plan_state):
        draw = []
        for ball in world_state['obstacles']['balls']:
            ball_x, ball_y = ball[0]
            ball_radius = ball[1]
            box_x = ball_x - ball_radius - 0.05
            box_y = ball_y - ball_radius - 0.05
            box_width = 2 * ball_radius + 0.1
            box_height = 2 * ball_radius + 0.1
            draw.append({
                "shape": "box",
                "text": "ball",
                "color": "blue",
                "x": box_x + box_width / 2,
                "y": box_y + box_height / 2,
                "width": box_width,
                "height": box_height,
            })
        if plan_state['trajectory'] is not None:
            draw.append({
                "shape": "line",
                "text": "path",
                "color": "green",
                "vertices": [
                    {
                        "x": world_state['pose'][0][0],
                        "y": world_state['pose'][0][1],
                    },
                    {
                        "x": plan_state['trajectory'][0],
                        "y": plan_state['trajectory'][1],
                    }
                ]
            })

        return draw
