package frc.robot.subsystems.swerve

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import frc.robot.lib.webconstants.LoggedTunableNumber
import kotlin.math.pow
import kotlin.math.sqrt

object SwerveConstants {

    enum class SwerveType {
        WCP,
        NEO,
        SIM
    }

    val SWERVE_TYPE = SwerveType.WCP

    val OFFSETS = arrayOf(0.533935546875, 0.762939453125, 0.023681640625, 0.3232421875)

    const val NEO_CURRENT_LIMIT: Double = 40.0
    const val NEO_550_CURRENT_LIMIT: Double = 20.0
    val TALON_FX_CURRENT_LIMIT_CONFIGS: CurrentLimitsConfigs = CurrentLimitsConfigs()
        .withSupplyCurrentLimit(40.0)
        .withStatorCurrentLimit(80.0)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)

    const val VOLT_COMP_SATURATION: Double = 12.0
    const val NEUTRAL_DEADBAND: Double = 0.0
    const val XBOX_DEADBAND: Double = 0.1
    val STEERING_MULTIPLIER =
        LoggedTunableNumber("Steering multiplier", 0.6)
    val DRIVE_KP: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/PID/driveKP")
    val DRIVE_KI: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/PID/driveKI")
    val DRIVE_KD: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/PID/driveKD")
    val DRIVE_KV: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/PID/driveKV")
    val DRIVE_KS: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/PID/driveKS")
    val DRIVE_KA: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/PID/driveKA")
    val ANGLE_KP: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/PID/angleKP")
    val ANGLE_KI: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/PID/angleKI")
    val ANGLE_KD: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/PID/angleKD")
    val ANGLE_KV: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/PID/angleKV")
    val ANGLE_KS: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/PID/angleKS")
    val ANGLE_KA: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/PID/angleKA")
    val PID_VALUES: Array<LoggedTunableNumber> = arrayOf(
        DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KV, DRIVE_KS, DRIVE_KA, ANGLE_KP, ANGLE_KI,
        ANGLE_KD, ANGLE_KV, ANGLE_KS, ANGLE_KA
    )
    val ROTATION_KP: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/Rotation/rotationKP")
    val ROTATION_KI: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/Rotation/rotationKI")
    val ROTATION_KD: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/Rotation/rotationKD")
    val ROTATION_KDIETER: LoggedTunableNumber =
        LoggedTunableNumber("Swerve Drive/Rotation/rotationKDIETER")

    val VOLTAGE_CONFIGS: VoltageConfigs = VoltageConfigs()
        .withPeakForwardVoltage(VOLT_COMP_SATURATION)
        .withPeakReverseVoltage(VOLT_COMP_SATURATION)
    val MOTOR_OUTPUT_CONFIGS: MotorOutputConfigs = MotorOutputConfigs().withDutyCycleNeutralDeadband(NEUTRAL_DEADBAND)
    val MOTION_MAGIC_CONFIGS: MotionMagicConfigs = MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(3.0)
        .withMotionMagicAcceleration(12.0)
    const val ODOMETRY_FREQUENCY: Double = 250.0
    var ROBOT_WIDTH: Double = 0.0
    var ROBOT_LENGTH: Double = 0.0
    var WHEEL_POSITIONS: Array<Translation2d?>
    var WHEEL_DIAMETER: Double = 0.0
    var DRIVE_REDUCTION: Double = 0.0
    var FEEDBACK_CONFIGS_DRIVE: FeedbackConfigs? = null
    var DRIVE_MOTOR_CONFIGS: TalonFXConfiguration? = null
    var ANGLE_REDUCTION: Double = 0.0
    var FEEDBACK_CONFIGS_ANGLE: FeedbackConfigs? = null
    var ANGLE_MOTOR_CONFIGS: TalonFXConfiguration? = null
    var ENCODER_CONFIGS: CANcoderConfiguration? = null
    val HOLONOMIC_PATH_FOLLOWER_CONFIG: HolonomicPathFollowerConfig

    var DRIVE_MOTOR_MOMENT_OF_INERTIA: Double = 0.025
    var ANGLE_MOTOR_MOMENT_OF_INERTIA: Double = 0.004
    var MAX_X_Y_VELOCITY: Double = 0.0
    var MAX_OMEGA_VELOCITY: Double = 0.0
    var VY_NOTE_DETECTION_CONTROLLER: PIDController = PIDController(5.0, 0.0, 0.3)

    init {
        when (SWERVE_TYPE) {
            SwerveType.SIM -> {
                DRIVE_KP.initDefault(2.0)
                DRIVE_KI.initDefault(0.0)
                DRIVE_KD.initDefault(0.0)
                DRIVE_KV.initDefault(0.0)
                DRIVE_KS.initDefault(0.0)
                DRIVE_KA.initDefault(0.0)

                ANGLE_KP.initDefault(12.0)
                ANGLE_KI.initDefault(0.0)
                ANGLE_KD.initDefault(0.0)
                ANGLE_KS.initDefault(0.0)

                ROTATION_KP.initDefault(0.2)
                ROTATION_KI.initDefault(0.0)
                ROTATION_KD.initDefault(0.0)
                ROTATION_KDIETER.initDefault(0.0)

                ROBOT_WIDTH = 0.584
                ROBOT_LENGTH = 0.584
                WHEEL_DIAMETER = 0.099
                DRIVE_REDUCTION = (1 / 2.0) * (24.0 / 22.0) * (15.0 / 45.0)
                ANGLE_REDUCTION = (14.0 / 72.0) * 0.5

                MAX_X_Y_VELOCITY =
                    ((6000.0
                            / 60.0) *  // [m/s]
                            DRIVE_REDUCTION
                            * WHEEL_DIAMETER
                            * Math.PI)
            }

            SwerveType.WCP -> {
                DRIVE_KP.initDefault(0.3)
                DRIVE_KI.initDefault(0.0)
                DRIVE_KD.initDefault(0.0)
                DRIVE_KV.initDefault(0.675205)
                DRIVE_KS.initDefault(0.24833)
                DRIVE_KA.initDefault(0.05)

                ANGLE_KP.initDefault(100.0)
                ANGLE_KI.initDefault(0.0)
                ANGLE_KD.initDefault(0.0)
                ANGLE_KS.initDefault(0.335905)
                ANGLE_KV.initDefault(1.32755)
                ANGLE_KA.initDefault(0.1976375)

                ROTATION_KP.initDefault(2.3)
                ROTATION_KI.initDefault(0.0)
                ROTATION_KD.initDefault(0.2)
                ROTATION_KDIETER.initDefault(0.002)

                ROBOT_WIDTH = 0.585
                ROBOT_LENGTH = 0.585
                WHEEL_DIAMETER = 0.09854
                DRIVE_REDUCTION = (1 / 2.0) * (24.0 / 22.0) * (15.0 / 45.0)
                ANGLE_REDUCTION = (14.0 / 72.0) * 0.5

                MAX_X_Y_VELOCITY =
                    ((6000.0
                            / 60.0) *  // [m/s]
                            DRIVE_REDUCTION
                            * WHEEL_DIAMETER
                            * Math.PI)
            }

            SwerveType.NEO -> {
                DRIVE_KP.initDefault(0.0006)
                DRIVE_KI.initDefault(0.0)
                DRIVE_KD.initDefault(0.0)
                DRIVE_KV.initDefault(2.12)
                DRIVE_KS.initDefault(0.6)
                DRIVE_KA.initDefault(0.0)

                ANGLE_KP.initDefault(3.5)
                ANGLE_KI.initDefault(0.0)
                ANGLE_KD.initDefault(0.0)
                ANGLE_KS.initDefault(0.00065)

                ROTATION_KP.initDefault(0.9)
                ROTATION_KI.initDefault(0.0)
                ROTATION_KD.initDefault(0.0)
                ROTATION_KDIETER.initDefault(0.0)

                ROBOT_WIDTH = 0.512
                ROBOT_LENGTH = 0.67
                WHEEL_DIAMETER = 0.099
                DRIVE_REDUCTION = (12.0 / 24.0) * (28.0 / 20.0) * (15.0 / 45.0)
                ANGLE_REDUCTION = (6.0 / 40.0) * (11.0 / 59.0)

                MAX_X_Y_VELOCITY =
                    ((5676.0
                            / 60.0) *  // [m/s]
                            DRIVE_REDUCTION
                            * WHEEL_DIAMETER
                            * Math.PI)
            }
        }

        MAX_OMEGA_VELOCITY = (
                MAX_X_Y_VELOCITY
                        / sqrt(
                    (ROBOT_LENGTH / 2) * (ROBOT_LENGTH / 2)
                            + (ROBOT_WIDTH / 2) * (ROBOT_WIDTH / 2)
                ))
        WHEEL_POSITIONS =
            arrayOf(
                Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),  // FL
                Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),  // FR
                Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),  // RL
                Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2) // RR
            )

        FEEDBACK_CONFIGS_DRIVE =
            FeedbackConfigs()
                .withRotorToSensorRatio(1.0)
                .withSensorToMechanismRatio(1 / DRIVE_REDUCTION)
        DRIVE_MOTOR_CONFIGS =
            TalonFXConfiguration()
                .withVoltage(VOLTAGE_CONFIGS)
                .withCurrentLimits(TALON_FX_CURRENT_LIMIT_CONFIGS)
                .withFeedback(FEEDBACK_CONFIGS_DRIVE)

        FEEDBACK_CONFIGS_ANGLE =
            FeedbackConfigs()
                .withRotorToSensorRatio(1.0)
                .withSensorToMechanismRatio(1 / ANGLE_REDUCTION)
        ANGLE_MOTOR_CONFIGS =
            TalonFXConfiguration()
                .withVoltage(VOLTAGE_CONFIGS)
                .withCurrentLimits(TALON_FX_CURRENT_LIMIT_CONFIGS)
                .withFeedback(FEEDBACK_CONFIGS_ANGLE)
                .withMotorOutput(MOTOR_OUTPUT_CONFIGS)
                .withMotionMagic(
                    MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(30.0)
                        .withMotionMagicAcceleration(120.0)
                )

        ENCODER_CONFIGS =
            CANcoderConfiguration()
                .withMagnetSensor(MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1))

        HOLONOMIC_PATH_FOLLOWER_CONFIG = HolonomicPathFollowerConfig(
            PIDConstants(5.5, 0.0, 0.15),
            PIDConstants(3.0, 0.0, 0.4),
            MAX_X_Y_VELOCITY, sqrt((ROBOT_LENGTH / 2.0).pow(2.0) + (ROBOT_WIDTH / 2.0).pow(2.0)),
            ReplanningConfig()
        )
    }
}
