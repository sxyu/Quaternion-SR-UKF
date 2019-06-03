## Dependencies

- Only dependency is Eigen 3 (3.3+ should work)

- Requires C++ 11 (but not 14) support.

## Usage

### Overview

- Include `UKF.h`
- Define a `UKFModel` class
  - typedefs `StateVec`, `MeasureVec` to specify vector formats
  - functions `dF`, `H`: to specify process, measurement models 
  - function `init`: for initializing the UKF
- Create a `UKF<UKFModel[, InputType]>` object
- Call `ukf.update(time_since_last, measurement, input)` on each measurement
- Read off `ukf.state` and `ukf.stateRootCov`
 
### The UKF Class

This provides the core SR-UKF functionality. All template arguments possible: `UKF<UKFModel, InputType = Eigen::MatrixXd, UKFParams = DefaultUKFParams<UKFModel>, Integrator = kalman::util::integrator::RK4>`. For typical uses you would specify the UKFModel and the input type.

The UKF class has the following fields:
- `ukf.state`: type `StateVec`. The state vector.
- `ukf.stateRootCov`: type `MatrixXd`. Square root of state covariance matrix. Square with size `StateVec::COV_SIZE`.
- `ukf.processNoiseRootCov`: type `MatrixXd`. Square root of process covariance matrix. Square with size `StateVec::COV_SIZE`.
- `ukf.measureNoiseRootCov`: type `MatrixXd`. Square root of measurement covariance matrix. Square with size `MeasureVec::COV_SIZE`.

After updates, you may retrieve the state and root covariance of the filter by accessing `ukf.state` and `ukf.stateRootCov`.

### The Vector class

State and measurement vectors are implemented in the `Vector<NUM_SCALARS, NUM_3D_VECS, NUM_QUATERNIONS>` class. The template arguments specify the number of scalars, 3-vectors and quaternions in the state vector. Note that 3D vecs are unnecessary (could add three scalars) and are included for cleanness. `Vector` is actually a subclass of the `Eigen::Matrix` class and thus all computations with `Vector` are internally handled by Eigen.  

Please use typedefs in the `UKFModel` definition to specify state and measurement vector formats. For example:

```cpp
    /** define state vector: <scalars, 3-vectors, quaternions> */
    typedef kalman::Vector<0, 3, 1> StateVec; // the layout is: (pos x 3, vel x 3, angularvel x 3, attitude x 4)

    /** define measurement vector <scalars, 3-vectors, quaternions> */
    typedef kalman::Vector<0, 2, 0> MeasureVec; // the layout is: (gps-pos x 3, gyro-angularvel x 3) 
```

### Model Definition

For maximum efficiency, the UKF model is defined using a class with only static members which allows the functions to be statically linked. You must define this class and pass it as the first template argument for the `UKF` class.

**State and Measurement Vector Definitions**
Simply `typedef Vector<x, y, z> StateVec` and `typedef Vector<x, y, z> MeasureVec` in the model definition, where x,y,z are set appropriately to the numbers of scalars, 3-vectors, and quaternions.

**Description of Process and Measurement Models**
The model definition class should have two static functions specifying the process and measurement models.
- The function `StateVec dF(state, input)` should specify the derivative (over time) of the process model given a current state and an input. The use of the derivative allows us to have non-uniform time steps. Below are some functions that may be helpful:
    - `kalman::util::diffQuaternion(state, omega_start)` helper to compute process model derivatives for all quaternions. `state` is the state vector, while `omega_start` is the index in the state vector where angular velocities begin.
    - `kalman::util::diffPosition(state, p_start, p_end, v_start)` similar to above, but for positions. Positions are between `p_start`, `p_end` and velocities corresponding are between `v_start` and `v_start + (p_end - p_start)`
    - The full signature is `static StateVec dF(const StateVec & state, const TheInputType & u)`
- `MeasureVec H(state, input)` should compute the measurement vector corresponding to a state and input. Note that the input is not really necessary here but is included for convenience.
    - The full signature is `static MeasureVec H(const StateVec & state, const TheInputType & u)`

**UKF Initialization**
The UKF model definition class should also contain a static function called init:
`static void init(kalman::UKF<TheUKFModelClass> & ukf)`

This is ran on each UKF instance upon creation. The init function is expected to initialize the `state`, `stateRootCov`, `processNoiseRootCov`, `measureNoiseRootCov` fields.

You may use `ukf.defaultInitialize(state, proc, measure)` to set the three matrices to diagonal matrices with diagonal coefficients equal to state, proc, and measure respectively.

### Optional: Input

You may pass an input type as the second template parameter to `UKF`. The input is provided by the user in the `ukf.update()` function and passed to the process and measurement model functions. Modifying this allows you to make custom classes available to the measurement and process models.

### Optional: UKF Parameters

Optionally, you may also define a class to specify UKF parameters, and pass it as the third template argument to `UKF`. For example:

```c++
template<UKFModel>
class MyUKFParams {
        // parameters
        static constexpr double alphaSquared = 1.0;
        static constexpr double beta = 0.0;
        static constexpr double lambda = alphaSquared * (UKFModel::StateVec::COV_SIZE + 3.0)
                                         - UKFModel::StateVec::COV_SIZE;

        /*
        Definitions for parameters used to calculated MRP vectors.
        See the Markley paper for further details.

        Note: By default, we use Gibbs vectors, which have a singularity at 180
        degrees. This is to avoid numerical issues calculating the covariance
        matrix when the quaternion covariance vector is large and the SUT scaling
        parameter alpha is set very small.

        The singularity being 180 degrees instead of 360 is not a problem unless
        the rotation is expected to change by more than 180 degrees in a single
        filter iteration; if it is, setting the MRP_A parameter to 1.0 moves the
        singularity to 360 degrees.
        */
        static constexpr double MRP_A = 0.0;
        static constexpr double MRP_F = 2.0 * (MRP_A + 1.0);

        /*
        Definitions for sigma point weights. The naming convention follows that used
        in in Van der Merwe 2001.
        */
        static constexpr double sigma_WM0 = lambda / (UKFModel::StateVec::COV_SIZE + lambda);
        static constexpr double sigma_WC0 = sigma_WM0 + (1.0 - alphaSquared + beta);
        static constexpr double sigma_WMI = 1.0 / (2.0 * (UKFModel::StateVec::COV_SIZE + lambda));
        static constexpr double sigma_WCI = sigma_WMI;
}
```
Then pass `MyUKFParams<MyUKFModel>` as the third template argument. Typically the default parameters should work well.

### Optional: Numerical Integrators

You may also use a different numerical integrator for the process model by passing the integrator class as the fourth argument to the `UKF`.
- `kalman::util::integrator::RK4` 4th order integrator. Default and usually the best choice.
- `kalman::util::integrator::Heun` 2nd order integrator.
- `kalman::util::integrator::Euler` 1st order integrator. Recommended if the process model is very simple, e.g. for parameter estimation.

It is also possible, though unlikely to be useful, to define your own integrator, following the following format:
```c++
 class RK4 {
    public:
        template <class Derivative, class InputType>
        static typename Derivative::StateVec integrate(double delta, const typename Derivative::StateVec & state,
                                                       const InputType & u) {
            typename Derivative::StateVec a = Derivative::dF(state, u);
            typename Derivative::StateVec b = Derivative::dF(typename Derivative::StateVec(state + 0.5 * delta * a), u);
            typename Derivative::StateVec c = Derivative::dF(typename Derivative::StateVec(state + 0.5 * delta * b), u);
            typename Derivative::StateVec d = Derivative::dF(typename Derivative::StateVec(state + delta * c), u);
            return state + (delta / 6.0) * (a + (b * 2.0) + (c * 2.0) + d);
        }
 };
```

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
