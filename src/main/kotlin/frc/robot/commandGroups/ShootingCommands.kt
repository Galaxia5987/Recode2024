package frc.robot.commandGroups

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.ControllerInputs
import frc.robot.lib.finallyDo
import frc.robot.lib.getRotationToTranslation
import frc.robot.subsystems.conveyor.Conveyor
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveDrive
import org.littletonrobotics.junction.Logger

private val swerveDrive = SwerveDrive.getInstance()
private val shooter = Shooter.getInstance()
private val hood = Hood.getInstance()
private val conveyor = Conveyor.getInstance()

private fun getRotationToSpeaker(): Rotation2d {
    return swerveDrive.estimator.estimatedPosition.translation.getRotationToTranslation(
        Constants.SPEAKER_POSE
    )
}

fun closeShoot(): Command {
    return warmup(
        { Units.Degrees.of(97.0) },
        { Units.RotationsPerSecond.of(45.0) },
        { Units.RotationsPerSecond.of(45.0) }
    )
}

fun trussSetpoint(): Command {
    return warmup(
        { Units.Degrees.of(75.0) },
        { Units.RotationsPerSecond.of(50.0) },
        { Units.RotationsPerSecond.of(50.0) }
    )
        .alongWith(turnToSpeaker())
}

fun finishScore(): Command {
    return Gripper.getInstance().feed()
        .alongWith(
            Commands.waitSeconds(0.5).andThen(stopWarmup())
        )
}

fun turnToSpeaker(): Command {
    return swerveDrive.driveAndAdjust(
        {
            Units.Rotations.of(
                getRotationToSpeaker().rotations
            )
        },
        { -ControllerInputs.driverController().leftY },
        { -ControllerInputs.driverController().leftX },
        SwerveConstants.SHOOT_TURN_TOLERANCE,
        0.1,
        true
    )
}

private fun shootOverStageInit(): Command {
    return Commands.parallel(
        warmup(
            { HOOD_ANGLE_SUPER_POOP },
            { SHOOTER_VELOCITY_SUPER_POOP },
            { CONVEYOR_VELOCITY_SUPER_POOP }
        ),
        swerveDrive.driveAndAdjust(
            {
                Units.Degrees.of(
                    swerveDrive.estimator.estimatedPosition.translation.getRotationToTranslation(
                        SUPER_POOP_TRANSLATION
                    ).degrees
                )
            },
            { -ControllerInputs.driverController().leftY },
            { -ControllerInputs.driverController().leftX },
            SUPER_POOP_TURN_TOLERANCE,
            0.1,
            true
        )
    ).until { shooterConveyorHoodAtSetpoint() }
}

private fun shootOverStageEnd(): Command {
    return stopWarmup().alongWith(Gripper.getInstance().feed())
}

fun shootOverStage(): Command = shootOverStageInit().finallyDo(shootOverStageEnd())

fun shooterConveyorHoodAtSetpoint(): Boolean {
    return (shooter.atSetpoint() && conveyor.atSetPoint() && hood.atSetpoint()).also {
        Logger.recordOutput(
            "ShootingCommands/atScoringSetpoint",
            it
        )
    }
}
