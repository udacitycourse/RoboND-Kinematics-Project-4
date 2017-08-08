## Project: Kinematics Pick & Place

---

[//]: # (Image References)

[image1]: ./misc_images/misc3.png
[image2]: ./misc_images/general_transform.png
[image3]: ./misc_images/transform_matrices.png
[image4]: ./misc_images/total_transform.png
[image5]: ./misc_images/general_transform_sequence.png
[image6]: ./misc_images/kinematics_diagram.jpg
[image7]: ./misc_images/theta_1.png
[image8]: ./misc_images/theta_2_3.png
[image9]: ./misc_images/theta_4_5_6.png
[image10]: ./misc_images/position_diagram.jpg


### Kinematic Analysis

#### 1. DH Parameters

i | alpha<sub>i-1</sub> | a<sub>i-1</sub> | d<sub>i</sub> | q<sub>i</sub>
--- | --- | --- | --- | ---
1 | 0 | 0 | .75 | theta<sub>1</sub>
2 | -pi/2 | .35 | 0 | theta<sub>2</sub> - pi/2
3 | 0 | 1.25 | 0 | theta<sub>3</sub>
4 |  -pi/2  | -.054 | 1.5 | theta<sub>4</sub>
5 | pi/2 | 0 | 0 | theta<sub>5</sub>
6 | -pi/2 | 0 | 0 | theta<sub>6</sub>
EE | 0 | 0 | .303 | 0

These values are derived in the usual way. Note that <b>d<sub>0</sub></b> and <b>q<sub>0</sub></b> are both undefined since, for the base, there is no previous joint reference frame from which to measure. Similarly, <b>alpha<sub>EE</sub></b> and <b>a<sub>EE</sub></b> lack a subsequent joint reference frame and so are both undefined as well. 

Also note that <b>q<sub>i</sub></b> (a DH parameter) and <b>theta<sub>i</sub></b> (a joint variable) are distinct. In general <b>theta<sub>i</sub></b> corresponds directly to the value of the joint angle of revolute joint <b>i</b>. This is particularly relevant in Row 2 of the DH parameter table because of the angle between <b>X<sub>1</sub></b> and <b>X<sub>2</sub></b>. Please refer to the sketch below for more detail on DH parameter and joint axis assignment.

![alt text][image6]

####2. Transformation Matrices

Transformation matrices between subsequent joint reference frames were derived using the DH parameters listed above and the observation that the homogenous transform from frame <b>i-1</b> to frame <b>i</b> can be described as a sequence of transformations correstponding to row <b>i</b> of the DH parameter table. he general transformation looks like this:

![alt text][image5]

![alt text][image2]

What follows is the collection of individual transformation matrices substituted with values and expressions from the DH parameter table and partially evaluated. In these matrices, instances of <b>theta<sub>i</sub></b> refer to real joint angles rather than the analogous DH parameters.

![alt text][image3]

Finally, I show the general total homogenous transform from the base link to the end effector such that a Real valued tranformation matrix can be derived from an incoming end effector orientation and position. <b>r</b>, <b>p</b>, and <b>gamma</b> correspond to the orientation of the end effector (Roll, Pitch, and Yaw, respectively) while <b>x</b>, <b>y</b>, and <b>z</b> correspond to its position. Following to the <b>ZYX</b> convention, the rotational portion of this matrix is derived by multiplying the three elementary rotation matrices <b>R<sub>Z</sub></b>, <b>R<sub>Y</sub></b>, and <b>R<sub>X</sub></b>

![alt text][image4]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

#####Inverse Position (Wrist Center)

Since the wrist center lies on the <b>Z</b> axis of the end effector and the orientation of the end effector is given (see total transformation matrix above), the position of the wrist center can be derived as a translation of magnitude <b>d<sub>7</sub></b> along the <b>Z</b> axis of the end effector's local coordinate frame.

Once we have the position of the wrist center, derivation of the first three joint angles proceeds naturally. Since <b>theta<sub>1</sub></b> is measured relative to the same coordinate frame as the position of the end effector (the base frame) we can derive its value directly from the position of the wrist center:

![alt text][image7]

<b>theta<sub>2</sub></b> and <b>theta<sub>3</sub></b> are derived by triangulating joints 2 and 3 with the wrist center and solving the joint angles using the law of cosines. Using <b>atan2</b> instead of <b>arccos</b> ensures numerical stability and avoids ambiguity.

![alt text][image10]

![alt text][image8]

#####Inverse Orientation (End Effector)

Finally, using the given end effector orientation (<b>R<sub>rpy</sub></b>), we calculate the rotation matrix from the wrist center to the end effector and decompose that rotation into Euler angles which correspond to the final three joint angles. This final step is done by using the <b>symbolically</b> derived rotation matrix to determine the structure of the decomposition. 

![alt text][image9]


### Project Implementation

My implementation proceeds linearly through the operations outlined in the above description. I used `sympy` to perform symbolic algebraic operations and to evaluate same with real values obtained from the DH parameter table and incoming end effector position and orientation data. 

First we instantiate the DH parameter table as a native dictionary and symbolically define the individual transformation matrices between joint reference frames as sympy matrices. Next we define the total homogeneous transform from the base frame to the end effector frame by postmultiplying the individual transform matrices. For later use in our inverse kinematics calculations, we maintain the intermediate transformation matrices from the base frame to each other frame and extract rotation matrices from each of those transform matrices.

The next step is to extract the end effector orientation and position from incoming simulation data. Orientation data is provided in quaternions from the simulator, so we must use the `euler_from_quaternion` method from the `tf.transformations` package to get the data into a form that is useful for us. Next we create rotation matrices that correspond to <b>yaw</b>, <b>pitch</b>, and <b>roll</b> values so obtained. From this, we can calculate the total rotation from the base frame to the end effector frame, but not before rotating the incoming rotation (which is relative to the base reference frame) into coincidence with the end effector frame. Now that we have a total rotation from base frame to end effector frame, we can calculate the position of the wrist center and proceed to define the algebraic operations outlined above to arrive at the desired joint angles, which are then used to populate the response to the simulator in the form of `JointTrajectoryPoint` objects. Once all the incoming end effector poses (which correspond to the desired trajectory of the Kuka Arm) have been successfully calculated, the list of `JointTrajectoryPoint` objects is returned in the form of a `CalculateIKResponse` object.



			