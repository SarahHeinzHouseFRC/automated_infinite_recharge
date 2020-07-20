# Perception #

The purpose of our perception stack is to take raw data from the onboard LIDAR and other sensors and convert it into a
more usable form for planning. Specifically, our perception stack detects balls and other robots from the LIDAR data and
outputs their positions and sizes for putting into an occupancy grid. We have to distinguish between these two types of
objects because we want to drive toward balls but avoid driving into obstacles.


## LIDAR Preprocessing ##

The stack begins with some very simple preprocessing of the LIDAR data. The data comes from the sensor in polar
coordinates in the sensor frame, just like we would expect to receive from a real LIDAR.

Firstly, we convert these points from polar \\( (r, \theta) \\) to Cartesian \\( (x, y) \\) using the following formula,
where \\( r \\) is the range (distance) measurement of the LIDAR and \\( \theta \\) is the azimuth angle measured off
the sensor's x-axis according to the right-hand rule.

$$ x = r \cos \theta \\\ y = r \sin \theta $$

![A high-level overview of our autonomy stack](img/polar-to-cartesian.svg)

This gives us our points in sensor frame Cartesian. Next, we convert the points from sensor to vehicle frame. In the
sim, the sensor and vehicle frames happen to be perfectly aligned so no actual calculation is explicitly done here.


## Localization ##

Once we have our LIDAR sweep in vehicle frame, we can compare it to a LIDAR sweep taken from the center of the field to
calculate the vehicle's position on the field. We use an algorithm called Iterative Closest Point (ICP) to achieve this.
The algorithm iteratively calculates the mean-squared loss by comparing both sweeps point-by-point to find the rotation
and translation needed to overlay the two given sweeps on top of one another. This is the same rotation and translation
of the vehicle on the field.

The purpose of localization is to transform our points from vehicle frame to global frame. Note that our "global" frame
is actually just affixed to the field. The center of the global frame is the center the field, with the x- and y-axes
parallel to the shorter and longer widths of the field, respectively, as shown in the figure below. The output of
localization is three constants representing the vehicle's current position and orientation on the field: \\( t_x \\),
\\( t_y \\), and \\( \phi \\).

![The coordinate frames we used](img/coordinate-frames.svg)

Once we have localized ourselves with respect to the field, we can convert all our points from vehicle frame to global
frame. This is done using matrix multiplication by inverting the following affine transformation:

$$ \left[\begin{matrix} x_\text{vehicle} \\\ y_\text{vehicle} \end{matrix}\right] =
\left[\begin{matrix} \cos \phi & - \sin \phi \\\ \sin \phi & \cos \phi \end{matrix}\right] \cdot
\left[\begin{matrix} x_\text{global} \\\ y_\text{global} \end{matrix}\right] +
\left[\begin{matrix} t_x \\\ t_y \end{matrix}\right] $$


## Background Subtraction ##

Now that our sweep is in the global frame, we can finally begin the more complicated logic for finding balls and
obstacles in the field. We begin by removing the points which belong to the static obstacles of the field itself, such
as the various columns and trench walls. Because these objects are static, we simply store the coordinates of each as a
polygon. During runtime, we slightly expand each polygon by a buffer, then remove any points which fall inside these
polygons using a fast point-in-polygon check. For the exterior wall of the field, we shrink its polygon by some buffer
and remove any points that fall outside it. The resulting list contains only points which belong to the "foreground," or
dynamic obstacles such as balls and robots.


## Clustering ##

Next, we cluster together points that are close to each other. By doing this, we hope to collect the points that belong
to a single ball or robot together so that they can be be classified.

We considered many different kinds of clustering algorithms, including DBSCAN, but ultimately chose to go with a simple
grid-based algorithm because it was easy to implement and very fast. Our algorithm can also be executed in parallel,
although it isn't currently.

Our approach partitions the field into square cells, then performs connected component analysis to connect neighboring
occupied cells into connected components. Although this approach is quick and simple, it often fails to associate
clusters of points together on other robots, which results in the same robot being identified as two separate obstacles.
A more improved algorithm might perform better on these cases and help prevent the motion planner from planning from
right through the middle of a robot. Secondly, if two balls are very close together, they are currently clustered
together and fail to be recognized as a ball in the classification step. A better stack might use the watershed
algorithm to separate this into two clusters, or skip clustering altogether and perform accurate classification on
unclustered data to avoid these issues.

![Our clustering algorithm simply partitions the grid and clusters connected components.](img/clustering.svg)


## Classification ##

Classification is used to identify which clusters represent balls vs. robots and fit approximate shapes that represent
them. For our oversimplified case, we assume everything that is not a ball is a robot. We run RANSAC on each cluster of
points to see if a circle with a radius of 3.5 inches can be fit to the points with 99% consensus after 10 random
trials. All clusters that succeed are classified as balls while all clusters that fail are assumed to be other robots
and have a simple axis-aligned bounding box (AABB) fit to them. Potentially this can be improved in the future by
fitting an oriented-bounding box (OBB) fit to them instead since most robots are OBB-shaped.

![We use RANSAC to classify each cluster as a ball or a robot.](img/classification.svg)


## Future Work ##

  - <input type="checkbox"> Perception can be very jumpy. Probabilistic updates in the occupancy grid can smooth this out.

  - <input type="checkbox"> Balls very close to each other fail to be recognized as balls. Maybe implement a
        convolutional neural network (CNN) to replace segmentation and classification to solve this.

  - <input type="checkbox"> Add predictions on whether an opponent is likely to reach a ball before us.
