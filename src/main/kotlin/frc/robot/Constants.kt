package frc.robot

import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.scoreState.AmpState
import frc.robot.scoreState.ClimbState
import frc.robot.scoreState.ScoreState
import frc.robot.scoreState.ShootState
import frc.robot.subsystems.swerve.*
import kotlin.math.sqrt

object Constants {
    const val CONFIG_TIMEOUT: Int = 100 // [ms]
    const val LOOP_TIME = 0.02 // [s]

    val MAX_VELOCITY: Measure<Velocity<Distance>> = Units.MetersPerSecond.of(2.0)
    val MAX_ACCELERATION: Measure<Velocity<Velocity<Distance>>> = Units.MetersPerSecondPerSecond.of(1.0)
    val MAX_ANGULAR_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(
        MAX_VELOCITY.`in`(Units.MetersPerSecond)
                / (SwerveConstants.ROBOT_LENGTH / sqrt(2.0))
    )
    val MAX_ANGULAR_ACCELERATION: Measure<Velocity<Velocity<Angle>>> = Units.RotationsPerSecond.per(Units.Second)
        .of(
            (MAX_ACCELERATION.`in`(Units.MetersPerSecondPerSecond)
                    / (SwerveConstants.ROBOT_LENGTH / sqrt(2.0)))
        )
    val PATH_CONSTRAINTS: PathConstraints = PathConstraints(
        MAX_VELOCITY.`in`(Units.MetersPerSecond),
        MAX_ACCELERATION.`in`(Units.MetersPerSecondPerSecond),
        MAX_ANGULAR_VELOCITY.`in`(Units.RotationsPerSecond),
        MAX_ANGULAR_ACCELERATION.`in`(Units.RotationsPerSecond.per(Units.Second))
    )

    val SPEAKER_RIGHT_CAMERA_POSE = Transform3d(
        -0.065833,
        -0.04005,
        0.608178,
        Rotation3d(0.0, -Math.toRadians(25.0), Math.toRadians(180.0))
    )
    val SPEAKER_LEFT_CAMERA_POSE = Transform3d(
        -0.065833,
        0.03995,
        0.608178,
        Rotation3d(0.0, -Math.toRadians(25.0), Math.toRadians(180.0))
    )
    val INTAKE_APRILTAG_CAMERA_POSE = Transform3d(
        -0.012852, -0.0005, 0.635282, Rotation3d(0.0, -Math.toRadians(25.0), 0.0)
    )
    val DRIVER_CAMERA_POSE = Transform3d(0.0, 0.0, 0.53, Rotation3d(0.0, Math.toRadians(20.0), 0.0))

    val CURRENT_MODE: Mode = Mode.REAL

    var CURRENT_STATE: ScoreState? = null

    val alliance: Alliance
        get() = if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            Alliance.RED
        } else{
            Alliance.BLUE
        }

    enum class Alliance {
        RED,
        BLUE
    }

    enum class State {
        SHOOT,
        AMP,
        CLIMB;

        fun setState(): Command {
            return Commands.runOnce(
                {
                    CURRENT_STATE = when (this) {
                        SHOOT -> ShootState()
                        AMP -> AmpState()
                        CLIMB -> ClimbState()
                    }
                }
            )
        }
    }

    enum class Mode {
        REAL,
        SIM,
        REPLAY
    }

    fun initSwerve() {
        var moduleIOs: Array<ModuleIO>

        when (SwerveConstants.SWERVE_TYPE) {
            SwerveConstants.SwerveType.SIM -> {
                moduleIOs = Array<ModuleIO>(4) { ModuleIOSim() }
                SwerveDrive.initialize(GyroIOSim(), SwerveConstants.OFFSETS, *moduleIOs)
            }

            SwerveConstants.SwerveType.WCP -> {
                moduleIOs = Array<ModuleIO>(4) { i ->
                    ModuleIOTalonFX(
                        Ports.SwerveDriveWCP.DRIVE_IDS[i],
                        Ports.SwerveDriveWCP.ANGLE_IDS[i],
                        Ports.SwerveDriveWCP.ENCODER_IDS[i],
                        SwerveConstants.DRIVE_MOTOR_CONFIGS
                            ?: throw IllegalStateException("drive motor config is null"),
                        SwerveConstants.ANGLE_MOTOR_CONFIGS
                            ?: throw IllegalStateException("angle motor config is null"),
                        SwerveConstants.ENCODER_CONFIGS ?: throw IllegalStateException("encoder config is null")
                    )
                }
                SwerveDrive.initialize(GyroIOReal(), SwerveConstants.OFFSETS, *moduleIOs)
            }

            SwerveConstants.SwerveType.NEO -> {
                moduleIOs = Array<ModuleIO>(4) { i ->
                    ModuleIOSparkMax(
                        Ports.SwerveDriveNEO.DRIVE_IDS[i],
                        Ports.SwerveDriveNEO.ANGLE_IDS[i],
                        Ports.SwerveDriveNEO.ENCODER_IDS[i],
                        Ports.SwerveDriveNEO.DRIVE_INVERTED[i],
                        Ports.SwerveDriveNEO.ANGLE_INVERTED[i]
                    )
                }
                SwerveDrive.initialize(GyroIOReal(), SwerveConstants.OFFSETS, *moduleIOs)
            }
        }

    }
}
