/*
General Datatypes

January 2024
*/

/// Maps the different operation codes that the ESP server expects
#[derive(Clone)]
pub enum EspOperation {
    Message = 0,
    Update = 1,
    Ping = 2,
    StatusRequest = 3,
}

/// Maps the different variable target codes that the ESP server expects
#[derive(Clone, Debug)]
pub enum VariableUpdateTarget {
    PidProportional = 0,
    PidIntegral = 1,
    PidDerivative = 2,
    // LinearVelocity = 3,
    // AngularVelocity = 4,
    MotorEnabled = 5,
    GyroOffset = 6,
}
