package frc.robot.subsystems.swerve

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import frc.robot.Constants
import frc.robot.lib.LoggedTunableNumber
import kotlin.math.pow
import kotlin.math.sqrt

object SwerveConstants {
    val OFFSETS = arrayOf(0.53369140625,0.76123046875,0.0224609375,0.326171875)

    const val VOLT_COMP_SATURATION = 12.0
    const val NEUTRAL_DEADBAND = 0.0
    const val XBOX_DEADBAND = 0.1

    const val NEO_CURRENT_LIMIT = 40.0
    const val NEO_550_CURRENT_LIMIT = 20.0
    val TALON_FX_CURRENT_LIMIT_CONFIGS = CurrentLimitsConfigs()
        .withSupplyCurrentLimit(50.0)
        .withStatorCurrentLimit(100.0)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)
    val VOLTAGE_CONFIGS = VoltageConfigs()
        .withPeakForwardVoltage(VOLT_COMP_SATURATION)
        .withPeakReverseVoltage(VOLT_COMP_SATURATION)
    val MOTOR_OUTPUT_CONFIGS = MotorOutputConfigs()
        .withDutyCycleNeutralDeadband(NEUTRAL_DEADBAND)
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive)
    val MOTION_MAGIC_CONFIGS = MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(3.0)
        .withMotionMagicAcceleration(12.0)
    val DRIVE_SLOT_0_CONFIG: Slot0Configs
    val ANGLE_SLOT_0_CONFIG: Slot0Configs

    val STEERING_MULTIPLIER =
        LoggedTunableNumber("Steering multiplier", 0.6)
    val DRIVE_KP =
        LoggedTunableNumber("Swerve Drive/PID/driveKP")
    val DRIVE_KI =
        LoggedTunableNumber("Swerve Drive/PID/driveKI")
    val DRIVE_KD =
        LoggedTunableNumber("Swerve Drive/PID/driveKD")
    val DRIVE_KV =
        LoggedTunableNumber("Swerve Drive/PID/driveKV")
    val DRIVE_KS =
        LoggedTunableNumber("Swerve Drive/PID/driveKS")
    val DRIVE_KA =
        LoggedTunableNumber("Swerve Drive/PID/driveKA")
    val ANGLE_KP =
        LoggedTunableNumber("Swerve Drive/PID/angleKP")
    val ANGLE_KI =
        LoggedTunableNumber("Swerve Drive/PID/angleKI")
    val ANGLE_KD =
        LoggedTunableNumber("Swerve Drive/PID/angleKD")
    val ANGLE_KV =
        LoggedTunableNumber("Swerve Drive/PID/angleKV")
    val ANGLE_KS =
        LoggedTunableNumber("Swerve Drive/PID/angleKS")
    val ANGLE_KA =
        LoggedTunableNumber("Swerve Drive/PID/angleKA")
    val PID_VALUES = arrayOf(
        DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KV, DRIVE_KS, DRIVE_KA, ANGLE_KP, ANGLE_KI,
        ANGLE_KD, ANGLE_KV, ANGLE_KS, ANGLE_KA
    )
    val ROTATION_KP =
        LoggedTunableNumber("Swerve Drive/Rotation/rotationKP")
    val ROTATION_KI =
        LoggedTunableNumber("Swerve Drive/Rotation/rotationKI")
    val ROTATION_KD =
        LoggedTunableNumber("Swerve Drive/Rotation/rotationKD")
    val ROTATION_KDIETER =
        LoggedTunableNumber("Swerve Drive/Rotation/rotationKDIETER")


    const val ODOMETRY_FREQUENCY = 250.0
    var ROBOT_WIDTH = 0.0
    var ROBOT_LENGTH = 0.0
    var WHEEL_DIAMETER = 0.0
    var DRIVE_REDUCTION = 0.0
    var ANGLE_REDUCTION = 0.0
    var WHEEL_POSITIONS: Array<Translation2d?>
    var FEEDBACK_CONFIGS_DRIVE: FeedbackConfigs? = null
    var DRIVE_MOTOR_CONFIGS: TalonFXConfiguration? = null
    var FEEDBACK_CONFIGS_ANGLE: FeedbackConfigs? = null
    var ANGLE_MOTOR_CONFIGS: TalonFXConfiguration? = null
    var ENCODER_CONFIGS: CANcoderConfiguration? = null
    val DRIVE_CONTROLLER: PPHolonomicDriveController

    var DRIVE_MOTOR_MOMENT_OF_INERTIA = 0.025
    var ANGLE_MOTOR_MOMENT_OF_INERTIA = 0.004
    var MAX_X_Y_VELOCITY = 0.0
    var MAX_OMEGA_VELOCITY = 0.0
    var VY_NOTE_DETECTION_CONTROLLER = PIDController(5.0, 0.0, 0.3)

    const val MAX_TURN_TOLERANCE = 3.0 / 360.0
    const val AMP_TURN_TOLERANCE = 5.0 / 360.0
    const val SHOOT_TURN_TOLERANCE = 5.0 / 360.0
    const val CLIMB_TURN_TOLERANCE = 4.0 / 360.0
    const val SKID_TOLERANCE = 0.15
    val COLLISION_TOLERANCE: Measure<Velocity<Velocity<Distance>>> = Units.Gs.of(1.8)

    init {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            if (Constants.ROBORIO_SERIAL_NUMBER == Constants.ROBORIO_NEO_SERIAL) {
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

            } else {
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

                ROTATION_KP.initDefault(3.5)
                ROTATION_KI.initDefault(0.0)
                ROTATION_KD.initDefault(0.1)
                ROTATION_KDIETER.initDefault(0.0)

                ROBOT_WIDTH = 0.9
                ROBOT_LENGTH = 0.9
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
        } else {
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

        DRIVE_SLOT_0_CONFIG = Slot0Configs()
            .withKP(DRIVE_KP.get())
            .withKI(DRIVE_KI.get())
            .withKD(DRIVE_KD.get())
            .withKS(DRIVE_KS.get())
            .withKV(DRIVE_KV.get())
            .withKA(DRIVE_KA.get())
        ANGLE_SLOT_0_CONFIG = Slot0Configs()
            .withKP(ANGLE_KP.get())
            .withKI(ANGLE_KI.get())
            .withKD(ANGLE_KD.get())
            .withKS(ANGLE_KS.get())
            .withKV(ANGLE_KV.get())
            .withKA(ANGLE_KA.get())

        FEEDBACK_CONFIGS_DRIVE =
            FeedbackConfigs()
                .withRotorToSensorRatio(1.0)
                .withSensorToMechanismRatio(1 / DRIVE_REDUCTION)
        DRIVE_MOTOR_CONFIGS =
            TalonFXConfiguration()
                .withMotorOutput(MOTOR_OUTPUT_CONFIGS)
                .withVoltage(VOLTAGE_CONFIGS)
                .withCurrentLimits(TALON_FX_CURRENT_LIMIT_CONFIGS)
                .withFeedback(FEEDBACK_CONFIGS_DRIVE)
                .withSlot0(DRIVE_SLOT_0_CONFIG)

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
                .withSlot0(ANGLE_SLOT_0_CONFIG)
                .withMotionMagic(
                    MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(30.0)
                        .withMotionMagicAcceleration(120.0)
                )

        ENCODER_CONFIGS =
            CANcoderConfiguration()
                .withMagnetSensor(MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1))

        DRIVE_CONTROLLER = PPHolonomicDriveController(
            PIDConstants(5.5, 0.0, 0.15),
            PIDConstants(3.0, 0.0, 0.4)
        )
    }
}
