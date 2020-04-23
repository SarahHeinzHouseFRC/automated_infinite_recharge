#
# Copyright (c) 2020 FRC Team 3260
#

class Cell:
    def __init__(self, bucket_id):
        # Flag for visited/unvisited
        # Field for which cc this cell belongs to
        self.x = bucket_id[0]
        self.y = bucket_id[1]
        self.cluster_id = None

    def get_neighbors(self):
        # Returns IDs of neighboring cells
        return [(self.x-1,self.y),
                (self.x+1,self.y),
                (self.x,self.y-1),
                (self.x,self.y+1)]

def connected_components(buckets):
    """
    Connects neighboring buckets into connected components.
    buckets = {
        (0,0) : list([1,10], [10,3]),
        (0,1) : list([1,10], [10,3]),
        ...
    }
    """

    # Create the grid (list of cells)
    grid = list()
    for bucket in buckets:
        cell = Cell(bucket)
        grid.append(cell)

    # Outer for loop
    cluster_id = 1
    for cell in grid:
        # if we've been to a cell, try the next one
        if cell.cluster_id is not None:
            continue
        cell.cluster_id = cluster_id

        queue = [cell]
        while len(queue) > 0:
            neighbor = queue.pop(0)
            if neighbor.visited:
                continue
            neighbor.visited = True


        # DFS to connect neighbors
