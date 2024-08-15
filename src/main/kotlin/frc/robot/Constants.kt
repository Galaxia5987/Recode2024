package frc.robot

import com.pathplanner.lib.util.GeometryUtil
import edu.wpi.first.math.geometry.Translation2d
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.scoreState.AmpState
import frc.robot.scoreState.ClimbState
import frc.robot.scoreState.ScoreState
import frc.robot.scoreState.ShootState
import frc.robot.subsystems.swerve.*
import frc.robot.subsystems.vision.PhotonVisionIOReal
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionConstants
import org.photonvision.PhotonCamera
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

    var SPEAKER_POSE: Translation2d = Translation2d(0.0, 5.5479442) // Blue

    var chainLocations = arrayOf(Pose2d(), Pose2d(), Pose2d()) //left, right, middle

    val CURRENT_MODE: Mode = Mode.REAL

    var CURRENT_STATE: ScoreState? = null

    val isRed: Boolean
        get() = DriverStation.getAlliance().get() == DriverStation.Alliance.Red

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

    fun initConstants() {
        if (isRed) {
            SPEAKER_POSE = GeometryUtil.flipFieldPosition(SPEAKER_POSE)

            chainLocations = Array<Pose2d>(chainLocations.size)
            { i -> GeometryUtil.flipFieldPose(chainLocations[i])}
        }
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

    fun initVision(){
        var speakerRightCamera =
            PhotonVisionIOReal(
                PhotonCamera("OV2311_1"),
                VisionConstants.SPEAKER_RIGHT_CAMERA_POSE
            )
        var speakerLeftCamera =
            PhotonVisionIOReal(
                PhotonCamera("OV2311_2"),
                VisionConstants.SPEAKER_LEFT_CAMERA_POSE,
            )
        var intakeAprilTagCamera =
            PhotonVisionIOReal(
                PhotonCamera("OV2311_0"),
                VisionConstants.INTAKE_APRILTAG_CAMERA_POSE,
            )
        var driverCamera =
            PhotonVisionIOReal(
                PhotonCamera("Driver_Camera"),
                VisionConstants.DRIVER_CAMERA_POSE,
            )

        Vision.initialize(listOf(speakerRightCamera, speakerLeftCamera, intakeAprilTagCamera, driverCamera))
    }
}
