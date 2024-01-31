
enum UpdateTarget {
    PidProportional,
    PidIntegral,
    PidDerivative,
    LinearVelocityTarget,
    AngularVelocityTarget,
    MotorsEnabled,
    GyroOffset,
    GyroValue,
    MotorTargetOmega,
    IntegralSum,
    };

/// @brief passed to operation handlers; contains information about request
struct OperationRequest
{
  bool is_valid;
  uint8_t operation_code;
  uint8_t *payload;
  uint16_t payload_length;
  WiFiClient *client;
};

struct PidState
{
  int16_t proportional;
  int16_t derivative;
  int16_t integral;
};

struct KinematicState
{
  int16_t linear_velocity_target;
  int16_t angular_velocity_target;
  bool motors_enabled;
  double gyro_offset;
};

struct MotorTarget
{
  double mot_1_omega;
  double mot_2_omega;
};

struct MotionInfo{
  double gyro_value;
  double motor_target;
  double integral_sum;
};

typedef struct {
  int16_t value;
  UpdateTarget target;
} ConfigQueueItem;

typedef struct {
  MotorTarget motor_target;
} MotorQueueItem;