# Geometry #

Hello I am geometry.

## Occupancy Grid ##
The Occupancy Grid class is what A* acts on to create a path around obstacles. It constructs a list of nodes, their
neighbors, and their occupancy so that A* star can find a path of connected cells from the start to the goal.

In the future the Occupancy Grid may benefit from having a memory of where objects are on the grid, so that we have an
idea of where everything is even when the lidar is partially blocked. It may also help in making more efficient paths
from no balls to 5 balls scored.



## Polygon ##
The polygon class holds a collection of vertices as well as associated information: the center, bounding box, and
whether the shape is convex.

### Counterclockwise ###
Counter clockwise is a function used by this class to see the direction of the 'turn' formed by 3 points. It works by
comparing the slopes of both edges.

### Point in Convex Polygon ###
Point in convex polygon works by walking around the edges of the polygon in counterclockwise order, and makes sure the
point is on the left of the edge. If this is true, the point is inside of the polygon. This trick is very fast, but only
works on convex shapes. Currently, it will throw an error whenever the shape is non convex, determining from the value
stored in the polygon. A future option is to have a general point in polygon function that will only call this if the
polygon is convex, otherwise use a slower, general purpose function.



## Algorithms ##

### A* ###
A* (pronounced: a-star) is the pathfinding algorithm we decided to use for our project, A* is based off of two other
similar pathfinding algorithms, one being Dijkstra's algorithm which can be used to find the shortest path between two
objects, because of this, A* is capable of sharing this property. The other algorithm A* is based off of is the Greedy
Best-First-Search pathfinding algorithm, which doesn't necessarily find the shortest path between to objects, but finds
the path faster than Dijkstra's algorithm does. As a result of A* being a combination of these two algorithms, it finds rather quick path, and does so rather quickly, because of these properties, we decided that it would be a good choice for our project as it fit our needs and wants for our pathfinding algorithm.

It returns the points of navigation as "nodes" which are made of two tuples, one for the x,y position of the node, and
the other as its column and row on the field since it is split into a grid. The the robot uses the nodes as waypoints so
it can find its next goal wether it be scoring or grabbing another ball somewhere on the field.


### Trajectory Smoother ###


### Connected Components ###


### RANSAC Circle Fit ###
Ransac Circle Fit attempts to create a best-fit circle from a group of points. It takes a random sample of 3 points from the list and creates a circle. If this circle fits our tolerance of inlier points and radius, it returns that circle. If it is not within our tolerances, it runs again with a new sample of points. If enough iterations fail, it returns that there is no best-fit circle