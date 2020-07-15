# Perception #

The purpose of perception is to take raw data points from the lidar sweep and convert them into a more abstract and
useable format for the decision making code.

## LIDAR Preprocessing ##



## Localization ##



## Coordinate Transforms ##
Every LIDAR point in a given sweep has to be converted across coordinate frames in reference to the vehicle to
coordinates in reference to the entire field.

We chose to perform all our perception algorithms in the world frame, so it was important to convert all our LIDAR
points from vehicle frame to world frame.


## Background Subtraction ##
The purpose of this function is to determine for each LIDAR point whether it hit a static part of the field or a
"dynamic" object, like a robot or a ball. The static parts of the field are the "background," and are not interesting to
us.


## Clustering ##
There are many different kinds of clustering algorithms that exist. We explored DBSCAN and k-means clustering, but
ultimately chose to do a very simple grid-based algorithm because it was easy to implement and very fast. Our algorithm
can also be executed in parallel.

Our approach uses a grid to partition the field into square cells, then performs connected component analysis to connect
neighboring occupied cells. Although this approach is fast and simple, it often fails to associate clusters of points
together on other robots. A more improved algorithm might catch these cases and prevent A* from right through the middle
of a robot.


## Classification ##
Classification is used to identify which clusters represent balls, which don't, annd fit approximate shapes that represent
them. We do this by seeing if we can fit a circle on the cluster, (via the ransac algorithm), which means
the cluster probably represents a ball. Successful circles are added to a list of balls.

Clusters that are not balls get marked as bounding boxes, as they are almost always robots, and added to a separate list.
