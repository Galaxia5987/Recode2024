package frc.robot

import com.sun.jdi.InterfaceType
import frc.robot.lib.PoseEstimation
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.climb.ClimbIOTalonFX
import frc.robot.subsystems.leds.LEDs
import frc.robot.subsystems.swerve.*
import frc.robot.subsystems.vision.PhotonVisionIOReal
import frc.robot.Constants.Mode
import frc.robot.subsystems.climb.ClimbIO
import frc.robot.subsystems.climb.LoggedClimbInputs
import frc.robot.subsystems.conveyor.*
import frc.robot.subsystems.gripper.*
import frc.robot.subsystems.hood.*
import frc.robot.subsystems.intake.*
import frc.robot.subsystems.shooter.*
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionConstants
import org.photonvision.PhotonCamera

val MAP = when (Constants.CURRENT_MODE) {
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
        Shooter to object : ShooterIO {
            override val topRollerInputs = LoggedRollerInputs()
            override val bottomRollerInputs = LoggedRollerInputs()
        }
    )
}

fun initSwerve() {
    val moduleIOs: Array<ModuleIO> = when (Constants.CURRENT_MODE) {
        Constants.Mode.REAL -> when (Constants.ROBORIO_SERIAL_NUMBER) {
            Constants.ROBORIO_NEO_SERIAL -> {
                Array(4) { i ->
                    ModuleIOSparkMax(
                        Ports.SwerveDriveNEO.DRIVE_IDS[i],
                        Ports.SwerveDriveNEO.ANGLE_IDS[i],
                        Ports.SwerveDriveNEO.ENCODER_IDS[i],
                        Ports.SwerveDriveNEO.DRIVE_INVERTED[i],
                        Ports.SwerveDriveNEO.ANGLE_INVERTED[i]
                    )
                }
            }
            else -> {
                Array(4) { i ->
                    ModuleIOTalonFX(
                        Ports.SwerveDriveWCP.DRIVE_IDS[i],
                        Ports.SwerveDriveWCP.ANGLE_IDS[i],
                        Ports.SwerveDriveWCP.ENCODER_IDS[i],
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
        else -> {
            Array(4) { ModuleIOSim() }
        }
    }

    val gyroIO = when (Constants.CURRENT_MODE) {
        Constants.Mode.REAL -> GyroIOReal()
        else -> GyroIOSim()
    }

    SwerveDrive.initialize(gyroIO, SwerveConstants.OFFSETS, *moduleIOs)
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

fun initializeSubsystems(currentMode: Constants.Mode) {
    PoseEstimation.initialize()

    (MAP[Climb] as? ClimbIO)?.let { Climb.initialize(it) }
    (MAP[Shooter] as? ShooterIO)?.let { Shooter.initialize(it) }
    (MAP[Hood] as? HoodIO)?.let { Hood.initialize(it) }
    (MAP[Conveyor] as? ConveyorIO)?.let { Conveyor.initialize(it) }
    (MAP[Intake] as? IntakeIO)?.let { Intake.initialize(it) }
    (MAP[Gripper] as? GripperIO)?.let { Gripper.initialize(it) }
}

