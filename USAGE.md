## Node API

The move3d\_facts package has only one node: move3d\_facts.

### move3d\_facts

This node reads world updates and computes facts and affordances on the current worldstate. It can compute them automatically on each update or on request.

#### Subscribed topics

* ```/pdg/objectList``` (toaster_msgs/ObjectListStamped)     
    * lists of objects, furnitures,...
* ```/pdg/humanList```  (toaster_msgs/HumanListStamped)
    * list of humans
* ```/pdg/robotList``` (toaster_msgs/RobotListStamped)
    * list of robots

#### Published topics
* ```~factList``` (toaster_msgs/FactList)
    * the computed facts. Publish only one list of facts for a worldstate (or every time the service ```computeOnce``` is called)

#### Parameters

* ```/move3d/p3dFile``` or ```~p3dFile``` (```string```,required)
    * the path or package:// url to the .p3d file describing the environment and its objects, robots and humans.
* ```~sceFile``` (```string```)
    * the path or package:// url to the .sce file describing configuration of the environment to load on node startup. Optional.
* ```/move3d/dof_name_corresp```
    * a structure representing the correspondance between the joint and dof names from Toaster (ROS) to Move3d (p3d).
* ```~update_base_only``` (```bool```, default:false)
    * when set to True, robots and human are updated even if the configuration is not complete. In such case, only the base joint is updated, the other remain unmodified.
* ```/human_mgmt``` (```dict```, default:empty)
    * a structure representing how humans are represented and managed. See ```move3d_ros_lib```.


#### Services


* ```~enable``` ([std_srvs/SetBool](http://docs.ros.org/api/std_srvs/html/srv/SetBool.html))
    * enable or disable the automatic computation of facts on each update reception (or as fast as possible, computing only on the most recent world-state)
* ```~computeOnce```  ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))
    * compute facts only once, on the last worldstate received.
* ```~save_scenario``` (move3d_ros_lib/SaveScenario)
    * saves the current world state to a scenario file that can be read by any move3d software.


