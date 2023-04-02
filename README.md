### Packages Required:

 - gazebo11-server
 - ros-noetic-xacro
 - ros-noetic-ira-laser-tools
 - rviz
 - controller-manager
 - joint-state-publisher
 - robot-state-publisher
 - gazebo-ros-control
 - diff-drive-controller
 - joint-state-controller
 - robot-state-controller
 - rosbridge-suite
 
 
 ### Notes to remember
  - Setup a docker network
  - Set gazebo master uri in server and client container by
   > export GAZEBO_MASTER_URI=http://"master-container-name":11345
  - Set the same in gazebo client
  - Create a volume and mount both containers on the same volume. Add the models into the volume from server and the copy to client assets folder
  - Set path of models by:
  > export GAZEBO_MODEL_PATH=/path/to/models
  - The usual path set for above is /usr/share/gazebo-11/models
  - gzweb will try to load models from the assets folder in "/root/gzweb/http/client/assets". make sure to copy all the models there.
  - gzweb tries tp search for models in the directory as assets/"package-name"/"same-hierarchy-as-server"/
   - For example in our package, models are in diff_drive/meshes, so gzweb will search in assets/diff_drive/meshes
