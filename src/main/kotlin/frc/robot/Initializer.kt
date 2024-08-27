package frc.robot

import frc.robot.lib.PoseEstimation
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.climb.ClimbIOTalonFX
import frc.robot.subsystems.conveyor.Conveyor
import frc.robot.subsystems.conveyor.ConveyorIOReal
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.gripper.GripperIOReal
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.hood.HoodIOReal
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.shooter.ShooterIOReal
import frc.robot.subsystems.swerve.*
import frc.robot.subsystems.vision.PhotonVisionIOReal
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionConstants
import org.photonvision.PhotonCamera

object Initializer {

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
        val speakerRightCamera =
            PhotonVisionIOReal(
                PhotonCamera("OV2311_1"),
                VisionConstants.SPEAKER_RIGHT_CAMERA_POSE
            )
        val speakerLeftCamera =
            PhotonVisionIOReal(
                PhotonCamera("OV2311_2"),
                VisionConstants.SPEAKER_LEFT_CAMERA_POSE,
            )
        val intakeAprilTagCamera =
            PhotonVisionIOReal(
                PhotonCamera("OV2311_0"),
                VisionConstants.INTAKE_APRILTAG_CAMERA_POSE,
            )
        val driverCamera =
            PhotonVisionIOReal(
                PhotonCamera("Driver_Camera"),
                VisionConstants.DRIVER_CAMERA_POSE,
            )

        Vision.initialize(listOf(speakerRightCamera, speakerLeftCamera, intakeAprilTagCamera, driverCamera))
    }

    init {
        Constants.initConstants()
        initVision()
        initSwerve()
        PoseEstimation.initialize()
        Climb.initialize(ClimbIOTalonFX())
        Shooter.initialize(ShooterIOReal())
        Hood.initialize(HoodIOReal())
        Conveyor.initialize(ConveyorIOReal())
        Gripper.initialize(GripperIOReal())
    }
}