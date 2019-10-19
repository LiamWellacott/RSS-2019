# RSS-2019
Practical assignment for RSS 2019 Course

## Dependencies

This package depends on [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) and [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)

## Run

Right now only a run
      '''
      $ roslaunch rss_gazebo test.launch
      '''


## TODO

### Paritlce filter

 - [ ] profile the particle filter.
 - [x] it looks like the intersection dosen't work then the robot has negative coordinates: FIX THIS!!
 - [ ] determine where we should place the map orign.
 - [ ] find a simpler and more elegant way to compute intersections between segments.
 - [ ] implement various resampling algorithms and evaluate their performances.
