## Dependencies

- Only dependency is Eigen 3 (3.3+ should work)

- Requires C++ 11 (but not 14).

## Usage

### Overview

The high level overview is as follows:
- Include `UKF.h`
- Define a `UKFModel` class
  - typedefs `StateVec`, `MeasureVec` to specify vector formats
  - functions `dF`, `H`: to specify process, measurement models 
  - function `init`: for initializing the UKF
- Create a `UKF<UKFModel[, InputType]>` object
  - All template arguments possible: `UKF<UKFModel, InputType = Eigen::MatrixXd, UKFParams = DefaultUKFParams<UKFModel>, Integrator = kalman::util::integrator::RK4>`
- Call `ukf.update(time_since_last, measurement, input)`

### Description of Vector

State and measurement vectors are implemented in the `Vector<NUM_SCALARS, NUM_3D_VECS, NUM_QUATERNIONS>` class. The template arguments specify the number of scalars, 3-vectors and quaternions in the state vector. Note that 3D vecs are unnecessary (could add three scalars) and are included for cleanness. `Vector` is actually a subclass of the `Eigen::Matrix` class and thus all computations with `Vector` are internally handled by Eigen.  

Please use typedefs in the `UKFModel` definition to specify state and measurement vector formats. For example:

```cpp
    /** define state vector: <scalars, 3-vectors, quaternions> */
    typedef kalman::Vector<0, 3, 1> StateVec; // the layout is: (pos x 3, vel x 3, angularvel x 3, attitude x 4)

    /** define measurement vector <scalars, 3-vectors, quaternions> */
    typedef kalman::Vector<0, 2, 0> MeasureVec; // the layout is: (gps-pos x 3, gyro-angularvel x 3) 
```

### Description of UKF Fields

In the init function you should define the following:
- `ukf.state`: type `StateVec`. The state vector.
- `ukf.stateRootCov`: type `MatrixXd`. Square root of state covariance matrix. Square with size `StateVec::COV_SIZE`.
- `ukf.processNoiseRootCov`: type `MatrixXd`. Square root of process covariance matrix. Square with size `StateVec::COV_SIZE`.
- `ukf.measureNoiseRootCov`: type `MatrixXd`. Square root of measurement covariance matrix. Square with size `MeasureVec::COV_SIZE`.

You may use `ukf.defaultInitialize(state, proc, measure)` to set the three matrices to diagonal matrices with diagonal coefficients equal to state, proc, and measure respectively.

After updates, you may retrieve the state and root covariance of the filter by accessing `ukf.state` and `ukf.stateRootCov`.

### Description of Process and Measurement Models

- `StateVec dF(state, input)` should specify the derivative (over time) of the process model given a current state and an input. The use of the derivative allows us to have non-uniform time steps.
    - `kalman::util::diffQuaternion(state, omega_start)` helper to compute process model derivatives for all quaternions. `state` is the state vector, while `omega_start` is the index in the state vector where angular velocities begin.
    - `kalman::util::diffPosition(state, p_start, p_end, v_start)` similar to above, but for positions. Positions are between `p_start`, `p_end` and velocities corresponding are between `v_start` and `v_start + (p_end - p_start)`
- `MeasureVec H(state, input)` should compute the measurement vector corresponding to a state and input. Note that the input is not really necessary here but is included for convenience.

### Usage Example

Please see `example.cpp` for usage example.

**To Compile Example:**

- Install CMake
- `mkdir build && cd build`
- `cmake ..`
  - if this doesn't find Eigen automatically, you may need `cmake .. -DEIGEN_INCLUDE_DIRS="path/to/eigen/includes"`
- `cmake --build . --config Release`

## Credits

Significant parts of this implementation are based on [sfwa/ukf](https://github.com/sfwa/ukf). Publications referenced include:

- Kraft, "A Quaternion-based Unscented Kalman Filter for Orientation Tracking", 2003
- Van der Merwe and Wan, "The Square-root Unscented Kalman Filter for state and Parameter-estimation", 2001
- Wan and Van der Merwe, "The Unscented Kalman Filter for Nonlinear Estimation", 2000
- Markley, "Attitude Error Representations for Kalman Filtering", 2002

## License

apache 2.0
