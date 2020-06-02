
#!/bin/bash
source /opt/ros/melodic/setup.bash
source src/mbf/.travis/util.sh

travis_run catkin init
travis_run rosdep update --as-root apt:false
travis_run rosdep install --from-paths src --ignore-src --rosdistro melodic -y -r

travis_run catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release
travis_run_wait 60 catkin build move_base_flex
