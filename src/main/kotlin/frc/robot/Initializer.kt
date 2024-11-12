package frc.robot

import edu.wpi.first.math.geometry.Transform3d
import frc.robot.lib.PoseEstimation
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.climb.ClimbIOTalonFX
import frc.robot.subsystems.conveyor.Conveyor
import frc.robot.subsystems.conveyor.ConveyorIOReal
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.gripper.GripperIOReal
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.hood.HoodIOReal
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.intake.IntakeIOReal
import frc.robot.subsystems.leds.LEDs
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.shooter.ShooterIOReal
import frc.robot.subsystems.swerve.*
import frc.robot.Constants.Mode
import frc.robot.subsystems.climb.ClimbIO
import frc.robot.subsystems.climb.LoggedClimbInputs
import frc.robot.subsystems.conveyor.*
import frc.robot.subsystems.gripper.*
import frc.robot.subsystems.hood.*
import frc.robot.subsystems.intake.*
import frc.robot.subsystems.shooter.*
import frc.robot.subsystems.vision.*
import org.photonvision.PhotonCamera
import org.photonvision.simulation.PhotonCameraSim

private val MAP = when (Constants.CURRENT_MODE) {
    Mode.REAL -> mapOf(
        Climb to ClimbIOTalonFX(),
        Conveyor to ConveyorIOReal(),
        Gripper to GripperIOReal(),
        Intake to IntakeIOReal(),
        Hood to HoodIOReal(),
        LEDs to LEDs.initialize(9, 97),
        Shooter to ShooterIOReal()
    )

    Mode.SIM -> mapOf(
        Climb to object : ClimbIO {
            override val inputs = LoggedClimbInputs()
        },
        Conveyor to ConveyorIOSim(),
        Gripper to GripperIOSim(),
        Intake to IntakeIOSim(),
        Hood to HoodIOSim(),
        LEDs to LEDs.initialize(9, 97),
        Shooter to ShooterIOSim()
    )

    Mode.REPLAY -> mapOf(
        Climb to object : ClimbIO {
            override val inputs = LoggedClimbInputs()
        },
        Conveyor to object : ConveyorIO {
            override val inputs = LoggedConveyorInputs()
        },
        Gripper to object : GripperIO {
            override val inputs = LoggedGripperInputs()
        },
        Intake to object : IntakeIO {
            override val inputs = LoggedIntakeInputs()
        },
        Hood to object : HoodIO {
            override val inputs = LoggedHoodInputs()
        },
        LEDs to LEDs.initialize(9, 97),
        Shooter to object : ShooterIO {
            override val topRollerInputs = LoggedRollerInputs()
            override val bottomRollerInputs = LoggedRollerInputs()
        }
    )
}

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
            object: GyroIO {}
        }
    }

    SwerveDrive.initialize(gyroIO, SwerveConstants.OFFSETS, *moduleIOs)
}

fun initPhotonCamera(cameraName: String, robotToCam: Transform3d): VisionIO {
    return when (Constants.CURRENT_MODE) {
        Mode.REAL -> PhotonVisionIOReal(PhotonCamera(cameraName), robotToCam)
        Mode.SIM -> PhotonVisionIOSim(
            PhotonCameraSim(
                PhotonCamera(
                    cameraName
                )
            ),
            robotToCam
        )
        Mode.REPLAY -> object: VisionIO {
            override val inputs = LoggedVisionInputs()
            override val name = cameraName
        }
    }
}

fun initVision() {
    val speakerRightCamera = initPhotonCamera("rightOV2311", SPEAKER_RIGHT_CAMERA_POSE)
    val speakerLeftCamera = initPhotonCamera("leftOV2311", SPEAKER_LEFT_CAMERA_POSE)
    val intakeAprilTagCamera = initPhotonCamera("frontOV2311", INTAKE_APRILTAG_CAMERA_POSE,)

    Vision.initialize(listOf(speakerRightCamera, speakerLeftCamera, intakeAprilTagCamera))
}

fun initializeSubsystems() {
    initVision()
    initSwerve()
    PoseEstimation.initialize()

    (MAP[Climb] as? ClimbIO)?.let { Climb.initialize(it) }
    (MAP[Shooter] as? ShooterIO)?.let { Shooter.initialize(it) }
    (MAP[Hood] as? HoodIO)?.let { Hood.initialize(it) }
    (MAP[Conveyor] as? ConveyorIO)?.let { Conveyor.initialize(it) }
    (MAP[Intake] as? IntakeIO)?.let { Intake.initialize(it) }
    (MAP[Gripper] as? GripperIO)?.let { Gripper.initialize(it) }
}

