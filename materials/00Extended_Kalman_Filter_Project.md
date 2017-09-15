
# Extended Kalman Filter Project

## Project Introduction

Now that you have learned how the extended Kalman filter works, you
are going to implement the extended Kalman filter in C++. We are
providing simulated lidar and radar measurements detecting a bicycle
that travels around your vehicle. You will use a Kalman filter, lidar
measurements and radar measurements to track the bicycle's position
and velocity.

The first step is to download the Term 2 simulator, which contains all
the projects for Term 2 Self-Driving Car Nanodegree. More detailed
instruction about setting up the simulator with uWebSocketIO can be
found at the end of this section.

Lidar measurements are red circles, radar measurements are blue
circles with an arrow pointing in the direction of the observed angle,
and estimation markers are green triangles. The video below shows what
the simulator looks like when a c++ script is using its Kalman filter
to track the object. The simulator provides the script the measured
data (either lidar or radar), and the script feeds back the measured
estimation marker, and RMSE values from its Kalman filter.

## Download Links for Term 2 Simulator

https://github.com/udacity/self-driving-car-sim/releases/


## Running the Program

1. Download the simulator and open it. In the main menu screen select
   Project 1: Bicycle tracker with EKF.
2. Once the scene is loaded you can hit the START button to observe
   how the object moves and how measurement markers are positioned in
   the data set. Also for more experimentation, "Data set 2" is
   included which is a reversed version of "Data set 1", also the
   second data set starts with a radar measurement where the first
   data set starts with a lidar measurement. At any time you can press
   the PAUSE button, to pause the scene or hit the RESTART button to
   reset the scene. Also the ARROW KEYS can be used to move the camera
   around, and the top left ZOOM IN/OUT buttons can be used to focus
   the camera. Pressing the ESCAPE KEY returns to the simulator main
   menu.
3. The EKF project Github repository README has more detailed
   instructions for installing and using c++ uWebScoketIO.

## NOTES:

- Currently hitting Restart or switching between Data sets only
  refreshes the simulator state and not the Kalman Filter's saved
  results. The current procedure for refreshing the Kalman Filter is
  to close the connection, ctrl+c and reopen it, ./ExtendedKF. If you
  don't do this when trying to run a different Data set or running the
  same Data set multiple times in a row, the RMSE values will become
  large because of the the previous different filter results still
  being observed in memory.
- Students have reported rapid expansion of log files when using the
  term 2 simulator. This appears to be associated with not being
  connected to uWebSockets. If this does occur, please make sure you
  are connected to uWebSockets. The following workaround may also be
  effective at preventing large log files.

  - create an empty log file
  - remove write permissions so that the simulator can't write to log

## What You'll Need to Do

1. Read the repo's README for more detailed instructions.
2. Complete the Extended Kalman Filter algorithm in C++.
3. Ensure that your project compiles.
4. Test your Kalman Filter in the simulator with Dataset 1. Ensure
   that the px, py, vx, and vy RMSE are below the values specified in
   the rubric.

	Note that the programs that need to be written to accomplish the project are 
	  - src/FusionEKF.cpp
	  - src/FusionEKF.h
	  - kalman_filter.cpp
	  - kalman_filter.h
	  - tools.cpp
	  - tools.h

5. Submit your project!

The project interface recently changed that allows the simulator to be
used to run the Kalman filter and visualize the scene instead of only
getting feedback from text files. The old project directory can still
be accessed from Github in the branch and for the time being students
can submit either version of the project, using either the new
simulator interface or the previous text based interface. Running the
previous project set up requires ./ExtendedKF path/to/input.txt
path/to/output.txt


## Example of Tracking with Lidar

Check out the video below to see a real world example of object
tracking with lidar. In this project, you will only be tracking one
object, but the video will give you a sense for how object tracking
with lidar works:



# File Structure

## Overview of a Kalman Filter: Initialize, Predict, Update

To review what we learned in the extended Kalman filter lectures,
let's discuss the three main steps for programming a Kalman filter:

- initializing Kalman filter variables
- predicting where our object is going to be after a time step Δt
- updating where our object is based on sensor measurements

Then the prediction and update steps repeat themselves in a loop.

To measure how well our Kalman filter performs, we will then calculate
root mean squared error comparing the Kalman filter results with the
provided ground truth.

These three steps (initialize, predict, update) plus calculating RMSE
encapsulate the entire extended Kalman filter project.

## Files in the Github src Folder

The files you need to work with are in the src folder of the github
repository.

- main.cpp - communicates with the Term 2 Simulator receiving data
  measurements, calls a function to run the Kalman filter, calls a
  function to calculate RMSE
- FusionEKF.cpp - initializes the filter, calls the predict function,
  calls the update function
- kalman_filter.cpp- defines the predict function, the update function
  for lidar, and the update function for radar
- tools.cpp- function to calculate RMSE and the Jacobian matrix

The only files you need to modify are FusionEKF.cpp,
kalman_filter.cpp, and tools.cpp.

## How the Files Relate to Each Other

Here is a brief overview of what happens when you run the code files:

1. Main.cpp reads in the data and sends a sensor measurement to
   FusionEKF.cpp
2. FusionEKF.cpp takes the sensor data and initializes variables and
   updates variables. The Kalman filter equations are not in this
   file. FusionEKF.cpp has a variable called ekf_, which is an
   instance of a KalmanFilter class. The ekf_ will hold the matrix and
   vector values. You will also use the ekf_ instance to call the
   predict and update equations.
3. The KalmanFilter class is defined in kalman_filter.cpp and
   kalman_filter.h. You will only need to modify 'kalman_filter.cpp',
   which contains functions for the prediction and update steps.



# Main.cpp

Here we will discuss the main.cpp file. Although you will not need to
modify this file, the project is easier to implement once you
understand what the file is doing. As a suggestion, open the github
repository for the project and look at the code files simultaneously
with this lecture slide.

## Main.cpp

You do not need to modify the main.cpp, but let's discuss what the
file does.

The Term 2 simulator is a client, and the c++ program software is a
web server.

We already discussed how main.cpp reads in the sensor data. Recall
that main.cpp reads in the sensor data line by line from the client
and stores the data into a measurement object that it passes to the
Kalman filter for processing. Also a ground truth list and an
estimation list are used for tracking RMSE.

main.cpp is made up of several functions within main(), these all
handle the uWebsocketIO communication between the simulator and it's
self.

All the main code loops in h.onMessage(), to have access to intial
variables that we created at the beginning of main(), we pass pointers
as arguments into the header of h.onMessage().

For example

h.onMessage([&fusionEKF,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode).

The rest of the arguments in h.onMessage are used to set up the server.

```
 // Create a Fusion EKF instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  //Call the EKF-based fusion
  fusionEKF.ProcessMeasurement(meas_package);
```

The code is:

- creating an instance of the FusionEKF class
- Receiving the measurement data calling the ProcessMeasurement()
  function. ProcessMeasurement() is responsible for the initialization
  of the Kalman filter as well as calling the prediction and update
  steps of the Kalman filter. You will be implementing the
  ProcessMeasurement() function in FusionEKF.cpp:

Finally,
The rest of main.cpp will output the following results to the simulator:

- estimation position
- calculated RMSE

main.cpp will call a function to calculate root mean squared error:

```
  // compute the accuracy (RMSE)
  Tools tools;
  cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;
```

You will implement an RMSE function in the tools.cpp file.



# Project Code

Let's discuss the three files that you will need to modify.

## FusionEKF.cpp

In FusionEKF.cpp, we have given some starter code for implementing
sensor fusion. In this file, you won't need to include the actual
Kalman filter equations; instead, you will be initializing variables,
initializing the Kalman filters, and then calling functions that
implement the prediction step or update step. You will see TODO
comments indicating where to put your code.

You will need to:

1. initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
2. initialize the Kalman filter position vector with the first sensor measurements
3. modify the F and Q matrices prior to the prediction step based on the elapsed time between measurements
4. call the update step for either the lidar or radar sensor measurement. Because the update step for lidar and radar are slightly different, there are different functions for updating lidar and radar.

## Initializing Variables in FusionEKF.cpp

```
  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
```

Every time main.cpp calls
fusionEKF.ProcessMeasurement(measurement_pack_list[k]), the code in
FusionEKF.cpp will run. - If this is the first measurement, the Kalman
filter will try to initialize the object's location with the sensor
measurement.

## Initializing the Kalman Filter in FusionEKF.cpp

```
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
```

## Predict and Update Steps in FusionEKF.cpp

Once the Kalman filter gets initialized, the next iterations of the
for loop will call the ProcessMeasurement() function to do the predict
and update steps.

```
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    // Laser updates
  }
```

In FusionEKF.cpp, you will see references to a variable called
ekf_. The ekf_ variable is an instance of the KalmanFilter class. You
will use ekf_ to store your Kalman filter variables (x, P, F, H, R, Q)
and call the predict and update functions. Let's talk more about the
KalmanFilter class.

## KalmanFilter Class

kalman_filter.h defines the KalmanFilter class containing the x vector
as well as the P, F, Q, H and R matrices. The KalmanFilter class also
contains functions for the prediction step as well as the Kalman
filter update step (lidar) and extended Kalman filter update step
(radar).

You will need to add your code to kalman_filter.cpp to implement the
prediction and update equations. You do not need to modify
'kalman_filter.h'.

Because lidar uses linear equations, the update step will use the
basic Kalman filter equations. On the other hand, radar uses
non-linear equations, so the update step involves linearizing the
equations with the Jacobian matrix. The Update function will use the
standard Kalman filter equations. The UpdateEKF will use the extended
Kalman filter equations:

```
void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
```

## Tools.cpp

This file is relatively straight forward. You will implement functions
to calculate root mean squared error and the Jacobian matrix:

```
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
}
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}
```

HINT: You implemented these already in the coding quizzes.

## Compiling and Running Your Code

Take a look at the github repo README file for instructions on how to
compile and run your code.




# Tips and Tricks

## Summary of What Needs to Be Done
1. In tools.cpp, fill in the functions that calculate root mean squared error (RMSE) and the Jacobian matrix.
2. Fill in the code in FusionEKF.cpp. You'll need to initialize the Kalman Filter, prepare the Q and F matrices for the prediction step, and call the radar and lidar update functions.
3. In kalman_filter.cpp, fill out the Predict(), Update(), and UpdateEKF() functions.

## Tips and Tricks

### Review the Previous Lessons

- Review the previous lessons! Andrei, Dominik and co. have given you
  everything you need. In fact, you've built most of an Extended
  Kalman Filter already! Take a look at the programming assignments
  and apply the techniques you used to this project.

### No Need to Tune Parameters

- The R matrix values and Q noise values are provided for you. There
  is no need to tune these parameters for this project. In the
  unscented Kalman Filter lectures, we'll discuss how to determine
  these parameters.

### Initializing the State Vector
- You'll need to initialize the state vector with the first sensor
  measurement.
- Although radar gives velocity data in the form of the range rate
  ​ρ​˙, a radar measurement does not contain enough information to
  determine the state variable velocities v_x and v_y. You can,
  however, use the radar measurements ρ and ϕ to initialize the state
  variable locations p_x and p_y​ .

### Calculating y = z - H * x'

- For lidar measurements, the error equation is y = z - H * x'. For
  radar measurements, the functions that map the x vector [px, py, vx,
  vy] to polar coordinates are non-linear. Instead of using H to
  calculate y = z - H * x', for radar measurements you'll have to use
  the equations that map from cartesian to polar coordinates: y = z -
  h(x').

### Normalizing Angles

- In C++, atan2() returns values between -pi and pi. When calculating
  phi in y = z - h(x) for radar measurements, the resulting angle phi
  in the y vector should be adjusted so that it is between -pi and
  pi. The Kalman filter is expecting small angle values between the
  range -pi and pi. HINT: when working in radians, you can add 2π or
  subtract 2π until the angle is within the desired range.

### Avoid Divide by Zero throughout the Implementation

- Before and while calculating the Jacobian matrix Hj, make sure your
  code avoids dividing by zero. For example, both the x and y values
  might be zero or px*px + py*py might be close to zero. What should
  be done in those cases?

### Test Your Implementation

- Test! We're giving you the ability to analyze your output data and
  calculate RMSE. As you make changes, keep testing your algorithm! If
  you are getting stuck, add print statements to pinpoint any
  issues. But please remove extra print statements before turning in
  the code.

## Ideas for Standing out!

The Kalman Filter general processing flow that you've learned in the
preceding lessons gives you the basic knowledge needed to track an
object. However, there are ways that you can make your algorithm more
efficient!

- Dealing with the first frame, in particular, offers opportunities
  for improvement.
- Experiment and see how low your RMSE can go!
- Try removing radar or lidar data from the filter. Observe how your
  estimations change when running against a single sensor type! Do the
  results make sense given what you know about the nature of radar and
  lidar data?
- We give you starter code, but you are not required to use it! You
  may want to start from scratch if: You want a bigger challenge! You
  want to redesign the project architecture. There are many valid
  design patterns for approaching the Kalman Filter algorithm. Feel
  free to experiment and try your own! You want to use a different
  coding style, eg. functional programming. While C++ code naturally
  tends towards being object-oriented in nature, it's perfectly
  reasonable to attempt a functional approach. Give it a shot and
  maybe you can improve its efficiency!



# Project Submission

- Clone/fork the project's template files from the project
  repository. (Note: Please do not submit your project as a pull
  request against our repo!)
- Clone the visualization and data generation utilities from the
  utilities repository.
- Build an Extended Kalman Filter by applying the general processing
  flow as described in the previous lessons.
- Test your code!
- Check your project against the project rubric. Make sure your RMSE
  values satisfy the thresholds listed in the rubric!



# Rubic

## Compile

- Your code should compile.

  - Code must compile without errors with cmake and make.
  - Given that we've made CMakeLists.txt as general as possible, it's
    recommended that you do not change it unless you can guarantee
    that your changes will still compile on any platform.

## Accuracy

- px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52]
  when using the file: "obj_pose-laser-radar-synthetic-input.txt which
  is the same data file the simulator uses for Dataset 1"

  - Your algorithm will be run against Dataset 1 in the simulator
    which is the same as
    "data/obj_pose-laser-radar-synthetic-input.txt" in the
    repository. We'll collect the positions that your algorithm
    outputs and compare them to ground truth data. Your px, py, vx,
    and vy RMSE should be less than or equal to the values [.11, .11,
    0.52, 0.52].

## Follows the Correct Algorithm

- Your Sensor Fusion algorithm follows the general processing flow as
  taught in the preceding lessons.

  - While you may be creative with your implementation, there is a
    well-defined set of steps that must take place in order to
    successfully build a Kalman Filter. As such, your project should
    follow the algorithm as described in the preceding lesson.

- Your Kalman Filter algorithm handles the first measurements
  appropriately.

  - Your algorithm should use the first measurements to initialize the
    state vectors and covariance matrices.

- Your Kalman Filter algorithm first predicts then updates.

  - Upon receiving a measurement after the first, the algorithm should
    predict object position to the current timestep and then update
    the prediction using the new measurement.

- Your Kalman Filter can handle radar and lidar measurements.

  - Your algorithm sets up the appropriate matrices given the type of
    measurement and calls the correct measurement function for a given
    sensor type.

## Code Efficiency

- Your algorithm should avoid unnecessary calculations.

  - This is mostly a "code smell" test. Your algorithm does not need
    to sacrifice comprehension, stability, robustness or security for
    speed, however it should maintain good practice with respect to
    calculations.

  - Here are some things to avoid. This is not a complete list, but
    rather a few examples of inefficiencies.

    - Running the exact same calculation repeatedly when you can run
      it once, store the value and then reuse the value later.
    - Loops that run too many times.
    - Creating unnecessarily complex data structures when simpler
      structures work equivalently.
    - Unnecessary control flow checks.


## Suggestions to Make Your Project Stand Out!

- While we're giving this project to you with starter code, you are
  not actually required to use it! If you think you can organize your
  Kalman Filter better than us, go for it! Also, this project was
  templatized in an object-oriented style, however it's reasonable to
  build a Kalman Filter in a functional style. Feel free to start from
  scratch with a functional algorithm!

  - Keep in mind that your code must compile. If your changes
    necessitate modifying CMakeLists.txt, you are responsible for
    ensuring that any reviewer can still compile your code given the
    dependencies listed earlier in the instructions - platform
    specific errors will not be debugged by graders.

- There is some room for improvement with the Kalman Filter
  algorithm. Maybe some aspects of the algorithm could be combined?
  Maybe some could be skipped under certain circumstances? Maybe there
  are other ways to improve performance? Get creative!

- Analyze what happens when you turn off radar or lidar. Which sensor
  type provides more accurate readings? How does fusing the two
  sensors' data improve the tracking results?



EOF
