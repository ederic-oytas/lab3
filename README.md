# Lab 3: Automatic Emergency Braking

This lab is the first lab that you'll be working in teams. Each team member
will contribute their own implementation of the safety node and put them in
the same package.

## Learning Goals

- Use Time To Collision (TTC) to predict when the car is going to collide with
  an obstacle.
- Apply principles of a safety-critical system by applying safety measures
  (brakes) to prevent the system from going into an unsafe state.
- Learn about the `LaserScan` and `Odometry` messages in ROS 2.

## File Structure

We will build off the local file structure given in the first lab. Keep this
structure in mind while you are working through the instructions!

```
${HOME}
  |
  +-- lab1_ws/              -- Lab 1 Workspace folder
  |
  +-- lab2_ws/              -- Lab 2 Workspace folder
  |
  +-- lab3_ws/              -- Lab 3 Workspace folder (NEW)
  |
  +-- sim_ws/               -- Simulator Workspace folder
```

> **NOTE**: You may choose a different directory to substitute for ${HOME} for
> your setup; these commands will assume you are using ${HOME} for your setup,
> so adjust accordingly.

> **NOTE**: Keep in mind that `~` is always replaced with your home directory,
> so `${HOME}/some_dir` and `~/some_dir` are equivalent.

This lab is the first lab that you'll be working in teams. Each team member
will contribute their own implementation of the safety node and put them in
the same package. For example, if your teammates are `alice`, `bob`, and
`charlie`, your workspace folder would look like this:

```
${HOME}
  |
  +-- lab3_ws/
      |
      +-- src/
          |
          +-- aeb_pkg/
               |
               +-- aeb_pkg/
               |    |
               |    +-- alice_safety_node.py
               |    |
               |    +-- bob_safety_node.py
               |    |
               |    +-- charlie_safety_node.py
               |    |
               |    +-- YOURNAME_safety_node.py   -- Your safety node!
               |
               +-- launch/
               |    |
               |    +-- sim_launch.py
               |    |
               |    +-- veh_launch.py
               |
               +-- resource/
               |    :
               |
               +-- test/
               |    :
               |
               +-- package.xml
               |
               +-- setup.cfg
               |
               +-- setup.py
```

Writing your own node is still an independent task, but future labs will be
entirely team-based.

## Part 1: Safety Node in the Simulator

### 1-1: Specification

Your team will be making a package named `aeb_pkg`, containing a node for each
member of your team, each named `<student>_safety_node` with `<student>`
replaced with each teammate's first name.

#### Launching

The base code located in this repository gives completed launch files and
installs them as data files already (so you don't need to modify the launch
files or add them as data files).

The launch file `sim_launch.py` has the following interface:

```
ros2 launch aeb_pkg sim_launch.py target:=<target_node> ttc_thresh:=<ttc_threshold>
```

For example, if you want to launch the node of a team member named `charlie`
with a TTC threshold of 1.0, the command would be:

```
ros2 launch aeb_pkg sim_launch.py target:=charlie_safety_node ttc_thresh:=1.0
```

The launch file will then run that specific node, passing the TTC threshold to
it.

#### Safety Nodes

Each teammates in your team has their own safety node. When running, this node
will calculate Time-to-Collision (TTC) based on the car's odometry and scan
data. If any TTC falls below the given TTC threshold (`ttc_thresh`), then the
car sends a drive message to `/drive`, stopping the car
(`msg.drive.speed = 0.0`).

#### Deliverable (demonstration)

You must be able to demonstrate that your code operators properly in the
simulator before testing it on the car. You will run your safety node with a
TTC threshold of `1.0`. It will consist of two tests:

* Have the car face a wall. Drive towards it using `teleop_twist_keyboard` at
  a speed of about `1.0`. Your safety node must brake before it collides with
  the wall.
* Have the car facing straight through one of the hallways in the `levine` map.
  Drive forward. Your car should not brake and make it to the end of the
  hallway.

### 1-2: ROS 2 Messages

This section will cover several of the messages to know about for this lab.

#### The `AckermannDriveStamped` Message

This is a message you've used in Lab 1.
[AckermannDriveStamped](http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html)
is the message that is used to send drive commands to the car in the simulator
and to the motors in real life. You can cause the car to brake by sending an
`AckermannDriveStamped` message with the `drive.speed` field set to `0.0`.

#### The `LaserScan` Message

This is a message you've used in Lab 2. The
[LaserScan](https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html)
message contains several fields that will be useful to us. You can see
detailed descriptions of what each field contains in the API. The one
we'll be using the most is the `ranges` field. This is an array that
contains all range measurements from the LiDAR radially
ordered. You'll need to subscribe to the `/scan` topic and calculate
iTTC with the LaserScan messages.

#### The `Odometry` Message

This is a new message that you'll be using in this lab.
Both the simulator node and the car itself publish
[Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)
messages. Within its several fields, the message includes the cars
position, orientation, and velocity. This will be useful for getting the speed
of the car.

Now, the odometry topics vary between the simulator and vehicle:

* Simulator: `/ego_racecar/odom`
* Vehicle: `/odom`

This nuance is already covered by the launch files provided to you in this lab.
But it's important to keep this mind for future labs.

NOTE: The odometry data provided by `/ego_racecar/odom` is near-perfect
(providing almost the exact location where the car in in the simulator).
However, in real life, the odometry data from `/odom` will contain
inaccuracies.

### 1-3: TTC Calculation

**Time-to-Collision** (TTC) is the time it would take for the car to
collide with an obstacle if it maintained its current heading and
velocity. We approximate the time-to-collision using **Instantaneous
Time-to-Collision** (iTTC), which is the ratio of instantaneous range to
range rate calculated from current range measurements and velocity
measurements of the vehicle. Below is the formula for the iTTC:

$$
iTTC
= \frac{r}{\lbrace -\dot{r} \rbrace_{+}}
= \frac{r}{\max(x, 0)}
$$

where:

* $r$ is the instantaneous range measurement
* $\dot{r}$ is the current rate of change for that range measurement.

#### Calculating $r$

To obtain the instantaneous range $r$ to an obstacle, first use the
corresponding range measurement from the `LaserScan` message. Now since the
scan data comes from the laser frame, the range will also need to be converted
to the base frame.

#### Calculating range rate $\dot{r}$

The range rate $\dot{r}$ is the expected rate of change along each scan beam.
A positive range rate means the range measurement is expanding, and a negative
one means the range measurement is shrinking.
Thus, it can be calculated in two different ways.

First, it can be calculated by mapping the vehicle's current
longitudinal velocity onto each scan beam's angle by using $v_x
\cos{\theta_{i}}$. Be careful with assigning the range rate a positive
or a negative value.

The angles could also be determined by the information in `LaserScan`
messages. The range rate could also be interpreted as how much the
range measurement will change if the vehicle keeps the current
velocity and the obstacle remains stationary.

Second, you can take the difference between the previous range
measurement and the current one, divide it by how much time has passed
in between (timestamps are available in message headers), and
calculate the range rate that way.

Note the negation in the calculation this is to correctly interpret
whether the range measurement should be decreasing or increasing. For
a vehicle travelling forward towards an obstacle, the corresponding
range rate for the beam right in front of the vehicle should be
negative since the range measurement should be shrinking.

Vice versa,
the range rate corresponding to the vehicle travelling away from an
obstacle should be positive since the range measurement should be
increasing. The operator is in place so the iTTC calculation will be
meaningful. When the range rate is positive, the operator will make
sure iTTC for that angle goes to infinity.

After your calculations, you should end up with an array of iTTCs that
correspond to each angle. When a time to collision drops below a
certain threshold, it means a collision is imminent.

### 1-4. Automatic Emergency Braking

When your car needs to brake, you'll send a drive message to `/drive` with a
speed of 0. Below is some example Python code which creates such a message:

```python
drive_msg = AckermannDriveStamped()
drive_msg.drive.speed = 0.0
```

### 1-5. Testing your Node

To test your node in the simulator, use `teleop_twist_keyboard` in a separate
terminal and have your node running at the same time. Use

```
ros2 launch aeb_pkg target:=YOUR_NAME_safety_node ttc_thresh:=1.0
```

to launch your node. In the simulator, you can teleport the car using **2D**
**Pose Estimate**. It is recommended that you do the two tests specified in
1-1.

## 5. Package

You can implement this node in either C++ or Python. Clone this repository
and make necessary changes to personalize the "**safety_node**" in the 
directory created in the previous lab.

```bash
# On the localhost
gordon@f1sim:~$ mkdir git
gordon@f1sim:~$ cd git/
gordon@f1sim:~/git$ git clone git@github.com:unlv-f1/lab3.git
```

Copy the skeleton into the same directory containing the gym workspace.
```bash
# On the localhost
gordon@f1sim:~$ cd sim_ws/src/
gordon@f1sim:~sim_ws/src/$ cp -r ~/git/lab3/safety_node .
gordon@f1sim:~sim_ws/src/$ ls
f1tenth_gym_ros safety_node
```
### 5.1 Personalizing your Node by Name 
You cannot have more than one node of the same name built and sourced to run at one time in ROS.
You will need to change the name of the node to fit the following convention: 
**student's_name**_*safety_node*. 

You will need to edit or change the following within the copied package:
1. package.xml
2. CMakeLists.txt 
3. setup.py
4. setup.cfg
5. safety_node/ to **student's_name**_*safety_node*
6. Both the cpp file and python file to **student's_name**_*safety_node*.py/cpp

An example package will be uploaded to this repository that will show changes required 
for an example node **bree_safety_node**.

Develop your solution directly in the simulation container, with your
package in `/sim_ws/src` alongside the simulation package. To do so,
edit `docker-compose.yml` in the `f1tenth_gym_ros` container.

```yaml
# sim_ws/src/f1tenth_gym_ros/docker-compose.yml
# Do for each node you create in this course
<<< SNIP >>>
    build: ./
    volumes:
      - .:/sim_ws/src/f1tenth_gym_ros                             # this should have been modified already
      - <abspath>/sim_ws/src/YOUR_NAME_safety_node:/sim_ws/src/YOUR_NAME_safety_node  #Newly added
    environment:
<<< SNIP >>>
```

If the following error is encountered, the safety_node has not been properly included in the container.
Retry adding it as a mount point and restarting the composition.
```bash
#In Container
root@445b65e71801:/sim_ws# colcon build --packages-select safety_node
[0.362s] WARNING:colcon.colcon_core.package_selection:ignoring unknown package ’safety_node’ in --packages-select
```
Note that if you're using Windows, make sure your files have Unix
style line endings. You can use `dos2unix` or have correct settings in
your text editor.

## 6. Physical Vehicle Changes
Given the maximum speed of 60 miles per hour on the vehicle, the 
maximum permissible TTC value is 1.5 seconds.

Within the simulator, the odometry topic is published to **/ego_racecar/odom**, 
on the vehicle the topic is published to **/odom**. Before deploying to 
the vehicle, subscriptions must be updated.

You must be able to change the parameter on the command line between the two for 
your implementation. You should be able to run the following 
command in your launch file:

```bash
ros2 launch safety_node_launch.py mode:=sim student:=bree ttc:=2.0
```

Note that the mode changes your odometry publisher between **/ego_racecar/odom**, for simulator, 
and **/odom**, for vehicle.

## 7. Deliverables

**Deliverable 1**: After you're finished, update the entire skeleton
package directory with your `safety_node` package. When all group
members have completed their implementations, select one group member
as the leader and create a new branch for the group's submission. 
The submission will contain your group's launch file, a **`SUBMISSION.md`** file 
with instructions to run all individual nodes, and the nodes themselves. 
The group leader will need to make the TA a collaborator on this "group" 
submission.



**Deliverable 2**: Students will present their implementations 
in class to the TA using the designated group leader's computer 
for the simulator portion.

- **Simulator**: Drive the car with keyboard teleop along the hallways
of Levine (the default map), showing it doesn't brake when traveling straight in the
hallway. You need to show that your safety node doesn't generate false
positives. i.e. The car doesn't suddenly stop while traveling down
the hallway. Then show the car driving towards a wall and braking
correctly. After a successful demonstration, students will be able to run 
their nodes on vehicle. 
- **Vehicle**: A successful vehicle demonstration does not 
collide with any wall during manual control. The TTC will be set to 3 seconds.


## 8. Grading Rubric 
### Individual
- Compilation: **20** Points
- Student Simulator Demo: **30** Points
- Student Vehicle Demo: **30** Points

### Group
- Launch File Implemented Correctly: **20** Points

### Total: 100pts
