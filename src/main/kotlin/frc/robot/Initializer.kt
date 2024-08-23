package frc.robot

import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.shooter.ShooterIOSim
import frc.robot.subsystems.swerve.*
import frc.robot.subsystems.vision.PhotonVisionIOReal
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionConstants
import org.photonvision.PhotonCamera

object Initializer {

    fun initSwerve() {
        val moduleIOs: Array<ModuleIO>

        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            if (Constants.ROBORIO_SERIAL_NUM == Constants.ROBORIO_NEO_SERIAL) {
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
                return
            }

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
        } else {
            moduleIOs = Array<ModuleIO>(4) { ModuleIOSim() }
            SwerveDrive.initialize(GyroIOSim(), SwerveConstants.OFFSETS, *moduleIOs)
        }
    }

    fun initVision() {
        val speakerRightCamera =
            PhotonVisionIOReal(
                PhotonCamera("rightOV2311"),
                VisionConstants.SPEAKER_RIGHT_CAMERA_POSE
            )
        val speakerLeftCamera =
            PhotonVisionIOReal(
                PhotonCamera("leftOV2311"),
                VisionConstants.SPEAKER_LEFT_CAMERA_POSE,
            )
        val intakeAprilTagCamera =
            PhotonVisionIOReal(
                PhotonCamera("frontOV2311"),
                VisionConstants.INTAKE_APRILTAG_CAMERA_POSE,
            )
        val driverCamera =
            PhotonVisionIOReal(
                PhotonCamera("Driver_Camera"),
                VisionConstants.DRIVER_CAMERA_POSE,
            )

        Vision.initialize(listOf(speakerRightCamera, speakerLeftCamera, intakeAprilTagCamera))
    }

    init {
//        initVision()
//        initSwerve()
//        PoseEstimation.initialize()
//        Climb.initialize(ClimbIOTalonFX())
        Shooter.initialize(ShooterIOSim())
//        Hood.initialize(HoodIOReal())
//        Conveyor.initialize(ConveyorIOReal())
//        Intake.initialize(IntakeIOReal())
//        Gripper.initialize(GripperIOReal())
    }
}