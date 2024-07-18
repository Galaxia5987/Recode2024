package frc.robot.subsystems.swerve

import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.SPI
import frc.robot.Ports
import swervelib.encoders.CANCoderSwerve
import swervelib.encoders.PWMDutyCycleEncoderSwerve
import swervelib.imu.NavXSwerve
import swervelib.motors.TalonFXSwerve
import swervelib.parser.*
import swervelib.parser.json.MotorConfigDouble

object SwerveConstants {
    val ROBOT_LENGTH: Double //[m]
    val ROBOT_WIDTH: Double //[m]

    enum class SwerveType {
        WCP,
        NEO,
        SIM
    }

    val SWERVE_TYPE = SwerveType.NEO
    val SWERVE_CONFIG: SwerveDriveConfiguration
    val SWERVE_CONTROLLER_CONFIG: SwerveControllerConfiguration

    val SWERVE_OFFSETS = arrayOf(0.0, 0.0, 0.0, 0.0)
    val MAX_SPEED: Double
    val holonomicPathFollowerConfig: HolonomicPathFollowerConfig
    private val IMU = NavXSwerve(SPI.Port.kMXP)
    private val SWERVE_MODULE_CONFIGS = arrayOfNulls<SwerveModuleConfiguration>(4)
    private val SWERVE_MODULE_CHARACTERISTICS: SwerveModulePhysicalCharacteristics
    private val HEADING_PID: PIDFConfig
    private val SWERVE_ANGLE_PID: PIDFConfig
    private val SWERVE_DRIVE_PID: PIDFConfig
    private val DRIVE_FF: SimpleMotorFeedforward
    private val CONVERSION_FACTORS: MotorConfigDouble
    private val DRIVE_REDUCTION: Double
    private val ANGLE_REDUCTION: Double

    const val joystickDeadband = 0.15

    init {
        when(SWERVE_TYPE){
            SwerveType.WCP -> {
                ROBOT_LENGTH = 0.75
                ROBOT_WIDTH = 0.75

                DRIVE_REDUCTION = (1 / 2.0) * (24.0 / 22.0) * (15.0 / 45.0)
                ANGLE_REDUCTION = (14.0 / 72.0) * 0.5
                CONVERSION_FACTORS = MotorConfigDouble(ANGLE_REDUCTION, DRIVE_REDUCTION)

                HEADING_PID = PIDFConfig(0.0, 0.0, 0.0, 0.0)
                SWERVE_ANGLE_PID = PIDFConfig(0.0, 0.0, 0.0, 0.0)
                SWERVE_DRIVE_PID = PIDFConfig(0.0, 0.0, 0.0, 0.0)
                DRIVE_FF = SimpleMotorFeedforward(0.0, 0.0, 0.0)

                MAX_SPEED = 6.0

                SWERVE_MODULE_CHARACTERISTICS = SwerveModulePhysicalCharacteristics(
                    CONVERSION_FACTORS, 1.19, 12.0,
                    40, 20, 0.3, 0.3
                )
                for (i in 0..3){
                    SWERVE_MODULE_CONFIGS[i] = SwerveModuleConfiguration(
                        TalonFXSwerve(Ports.SwerveDriveWCP.DRIVE_IDS[i], true),
                        TalonFXSwerve(Ports.SwerveDriveWCP.ANGLE_IDS[i], false),
                        CONVERSION_FACTORS, CANCoderSwerve(Ports.SwerveDriveWCP.ENCODER_IDS[i]),
                        SWERVE_OFFSETS[i], ROBOT_WIDTH / 2, ROBOT_LENGTH / 2,
                        SWERVE_ANGLE_PID, SWERVE_DRIVE_PID, SWERVE_MODULE_CHARACTERISTICS, false,
                        Ports.SwerveDriveWCP.DRIVE_INVERTED[i], Ports.SwerveDriveWCP.ANGLE_INVERTED[i],
                        "Module ${i+1}", true
                    )
                }
            }

            SwerveType.NEO -> {
                ROBOT_LENGTH = 0.635
                ROBOT_WIDTH = 0.66

                DRIVE_REDUCTION = (12.0 / 24.0) * (28.0 / 20.0) * (15.0 / 45.0)
                ANGLE_REDUCTION = (6.0 / 40.0) * (11.0 / 59.0)
                CONVERSION_FACTORS = MotorConfigDouble(ANGLE_REDUCTION, DRIVE_REDUCTION)

                HEADING_PID = PIDFConfig(0.0, 0.0, 0.0)
                SWERVE_DRIVE_PID = PIDFConfig(0.0006, 0.0, 0.0)
                SWERVE_ANGLE_PID = PIDFConfig(3.5, 0.0, 0.0)
                DRIVE_FF = SimpleMotorFeedforward(0.6, 2.12, 0.0)

                MAX_SPEED = 6.0

                SWERVE_MODULE_CHARACTERISTICS = SwerveModulePhysicalCharacteristics(
                    CONVERSION_FACTORS, 1.19, 12.0,
                    40, 20, 0.3, 0.3
                )
                for (i in 0..3){
                    SWERVE_MODULE_CONFIGS[i] = SwerveModuleConfiguration(
                        TalonFXSwerve(Ports.SwerveDriveNEO.DRIVE_IDS[i], true),
                        TalonFXSwerve(Ports.SwerveDriveNEO.ANGLE_IDS[i], false),
                        CONVERSION_FACTORS, PWMDutyCycleEncoderSwerve(Ports.SwerveDriveNEO.ENCODER_IDS[i]),
                        SWERVE_OFFSETS[i], ROBOT_WIDTH / 2, ROBOT_LENGTH / 2,
                        SWERVE_ANGLE_PID, SWERVE_DRIVE_PID, SWERVE_MODULE_CHARACTERISTICS, false,
                        Ports.SwerveDriveWCP.DRIVE_INVERTED[i], Ports.SwerveDriveWCP.ANGLE_INVERTED[i],
                        "Module ${i+1}", true
                    )
                }
            }

            SwerveType.SIM -> {
                ROBOT_LENGTH = 0.75
                ROBOT_WIDTH = 0.75

                DRIVE_REDUCTION = (1 / 2.0) * (24.0 / 22.0) * (15.0 / 45.0)
                ANGLE_REDUCTION = (14.0 / 72.0) * 0.5
                CONVERSION_FACTORS = MotorConfigDouble(ANGLE_REDUCTION, DRIVE_REDUCTION)

                HEADING_PID = PIDFConfig(0.0, 0.0, 0.0, 0.0)
                SWERVE_ANGLE_PID = PIDFConfig(0.0, 0.0, 0.0, 0.0)
                SWERVE_DRIVE_PID = PIDFConfig(0.0, 0.0, 0.0, 0.0)
                DRIVE_FF = SimpleMotorFeedforward(0.0, 0.0, 0.0)

                MAX_SPEED = 6.0

                SWERVE_MODULE_CHARACTERISTICS = SwerveModulePhysicalCharacteristics(
                    CONVERSION_FACTORS, 1.19, 12.0,
                    40, 20, 0.3, 0.3
                )
                for (i in 0..3){
                    SWERVE_MODULE_CONFIGS[i] = SwerveModuleConfiguration(
                        TalonFXSwerve(Ports.SwerveDriveWCP.DRIVE_IDS[i], true),
                        TalonFXSwerve(Ports.SwerveDriveWCP.ANGLE_IDS[i], false),
                        CONVERSION_FACTORS, CANCoderSwerve(Ports.SwerveDriveWCP.ENCODER_IDS[i]),
                        SWERVE_OFFSETS[i], ROBOT_WIDTH / 2, ROBOT_LENGTH / 2,
                        SWERVE_ANGLE_PID, SWERVE_DRIVE_PID, SWERVE_MODULE_CHARACTERISTICS, false,
                        Ports.SwerveDriveWCP.DRIVE_INVERTED[i], Ports.SwerveDriveWCP.ANGLE_INVERTED[i],
                        "Module ${i+1}", true
                    )
                }
            }
        }
        SWERVE_CONFIG = SwerveDriveConfiguration(
            SWERVE_MODULE_CONFIGS, IMU, false,
            DRIVE_FF, SWERVE_MODULE_CHARACTERISTICS)

        SWERVE_CONTROLLER_CONFIG = SwerveControllerConfiguration(
            SWERVE_CONFIG, HEADING_PID,
            joystickDeadband, MAX_SPEED)
    }
}