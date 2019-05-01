## Usage

The high level overview is as follows:
- Include `UKF.h`
- Define a `UKFModel` class
  - typedefs StateVec, MeasureVec to specify vector formats
  - functions dF, H: to specify process, measurement models 
  - function init: for initializing the UKF
- Create a `UKF<UKFModel[, InputType]>` object
- Call `ukf.update(time_since_last, measurement, input)`

Please see `example.cpp` for usage example.

*Dependency:* Eigen 3

Requires C++ 11 (but not 14).

## Credits

Significant parts of this implementation are based on [sfwa/ukf](https://github.com/sfwa/ukf). Publications referenced include:

- Kraft, "A Quaternion-based Unscented Kalman Filter for Orientation Tracking", 2003
- Van der Merwe and Wan, "The Square-root Unscented Kalman Filter for state and Parameter-estimation", 2001
- Wan and Van der Merwe, "The Unscented Kalman Filter for Nonlinear Estimation", 2000
- Markley, "Attitude Error Representations for Kalman Filtering", 2002

## License

apache 2.0
