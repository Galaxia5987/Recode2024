package frc.robot

import frc.robot.Constants.Mode
import frc.robot.subsystems.swerve.GyroIO
import frc.robot.subsystems.swerve.GyroIOReal
import frc.robot.subsystems.swerve.GyroIOSim
import frc.robot.subsystems.swerve.LoggedModuleInputs
import frc.robot.subsystems.swerve.ModuleIO
import frc.robot.subsystems.swerve.ModuleIOSim
import frc.robot.subsystems.swerve.ModuleIOSparkMax
import frc.robot.subsystems.swerve.ModuleIOTalonFX
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveDrive

private fun createModuleIOs(): Array<ModuleIO> {
    return when (Constants.CURRENT_MODE) {
        Mode.REAL -> when (Constants.ROBORIO_SERIAL_NUMBER) {
            Constants.ROBORIO_NEO_SERIAL -> {
                Array(4) { i ->
                    ModuleIOSparkMax(
                        SwerveDriveNEOPorts.DRIVE_IDS[i],
                        SwerveDriveNEOPorts.ANGLE_IDS[i],
                        SwerveDriveNEOPorts.ENCODER_IDS[i],
                        SwerveDriveNEOPorts.DRIVE_INVERTED[i],
                        SwerveDriveNEOPorts.ANGLE_INVERTED[i]
                    )
                }
            }
            else -> {
                Array(4) { i ->
                    ModuleIOTalonFX(
                        SwerveDriveWCPPorts.DRIVE_IDS[i],
                        SwerveDriveWCPPorts.ANGLE_IDS[i],
                        SwerveDriveWCPPorts.ENCODER_IDS[i],
                        SwerveConstants.DRIVE_MOTOR_CONFIGS
                            ?: throw IllegalStateException("drive motor config is null"),
                        SwerveConstants.ANGLE_MOTOR_CONFIGS
                            ?: throw IllegalStateException("angle motor config is null"),
                        SwerveConstants.ENCODER_CONFIGS
                            ?: throw IllegalStateException("encoder config is null")
                    )
                }
            }
        }
        Mode.SIM -> {
            Array(4) { ModuleIOSim() }
        }
        Mode.REPLAY -> {
            Array(4) {
                object : ModuleIO {
                    override val inputs = LoggedModuleInputs()
                }
            }
        }
    }
}

fun initSwerve() {
    val moduleIOs: Array<ModuleIO> = createModuleIOs()

    val gyroIO = when (Constants.CURRENT_MODE) {
        Mode.REAL -> {
            GyroIOReal()
        }
        Mode.SIM -> {
            GyroIOSim()
        }
        Mode.REPLAY -> {
            object : GyroIO {}
        }
    }

    SwerveDrive.initialize(gyroIO, SwerveConstants.OFFSETS, *moduleIOs)
}

//fun initPhotonCamera(cameraName: String, robotToCam: Transform3d): VisionIO {
//    return when (Constants.CURRENT_MODE) {
//        Mode.REAL -> PhotonVisionIOReal(PhotonCamera(cameraName), robotToCam)
//        Mode.SIM -> PhotonVisionIOSim(
//            PhotonCameraSim(
//                PhotonCamera(
//                    cameraName
//                )
//            ),
//            robotToCam
//        )
//        Mode.REPLAY -> object : VisionIO {
//            override val inputs = LoggedVisionInputs()
//            override val name = cameraName
//        }
//    }
//}

//fun initVision() {
//    val speakerRightCamera = initPhotonCamera("rightOV2311", SPEAKER_RIGHT_CAMERA_POSE)
//    val speakerLeftCamera = initPhotonCamera("leftOV2311", SPEAKER_LEFT_CAMERA_POSE)
//    val intakeAprilTagCamera = initPhotonCamera("frontOV2311", INTAKE_APRILTAG_CAMERA_POSE)
//
//    Vision.initialize(listOf(speakerRightCamera, speakerLeftCamera, intakeAprilTagCamera))
//}

fun initializeSubsystems() {
//    initVision()
    initSwerve()
//    PoseEstimation.initialize()
}
