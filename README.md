# Project Name

This package provieds an GPS-like camera-based indoor-localization-solution. The localization is given as a transformation from the stargazer frame to the camera frame.


## Installation

TODO: Describe the installation process

## Usage

### Calibration
1. Get an approximate intrisic camera calibration file
   * e.g. chessboard calibration
   * Outputs
     * cam_guess.yaml
2. Get an approximate stargazer landmark map file
   * Most importantly tells which landmarks can be found at all
   * Outputs
     * map_guess.yaml
2. Record raw rosbag for calibration
   * All landmarks should be seen
   * Outputs
     * stargazer_raw.bag
       * /image_raw
2. Play rosbag and tune detection parameters manually (then save them)
   * rosbag play -l stargazer_raw.bag -r 0.2
     * slow rate for better tuning
   * roslaunch stargazer_ros_tool stargazer_nodes.launch debug_finder:=true undistorted_image_topic:=<topic> map_config:=map_guess.yaml
     * map tells which landmarks exist at all (pose is unimportant here)
   * Outputs
     * landmark_finder.yaml
2. Run Stargazer Localization and record bag
   * roslaunch stargazer_ros_tool stargazer_nodelets.launch record:=true yournondefaultparams:=foobar [...]
   * Inputs
     * cam_guess.yaml
     * map_guess.yaml
     * landmark_finder.yaml
     * stargazer_raw.bag
       * /image_raw
   * Outputs
     * stargazer_localized.bag
       * /landmarks_seen
       * /camera_pose
2. Run Stargazer Calibration
   * roslaunch stargazer_ros_tool offline_landmark_calibrator.launch yournondefaultparams:=foobar [...]
   * Inputs
     * cam_guess.yaml
     * map_guess.yaml
     * stargazer_localized.bag
       * /landmarks_seen
       * /camera_pose
   * Outputs
     * cam_optimized.yaml
     * map_optimized.yaml
2. Repeat the last two steps with better new guess
   * Should converge
2. Use config files for other (non calibration) rosbags

### Localization

Just launch 
    roslaunch stargazer_ros_tool stargazer_nodelets.launch

### Visualization

For visualization of the landmarks and the agent launch
    roslaunch stargazer_ros_tool landmark_visualizer.launch

## Contributing

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D

## History

TODO: Write history

## Credits

TODO: Write credits

## License

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
