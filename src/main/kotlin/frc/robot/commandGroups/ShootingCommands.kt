package frc.robot.commandGroups

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.finallyDo
import frc.robot.lib.getRotationToTranslation
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.conveyor.Conveyor
import frc.robot.subsystems.swerve.SwerveDrive

object ShootingCommands {
    private val swerveDrive = SwerveDrive.getInstance()
    private val shooter = Shooter.getInstance()
    private val hood = Hood.getInstance()
    private val conveyor = Conveyor.getInstance()

    private fun shootOverStageInit(): Command {
        return Commands.parallel(
            WarmupCommands.warmup(
                ShootOverStageConstants.HOOD_ANGLE_SUPER_POOP,
                ShootOverStageConstants.SHOOTER_VELOCITY_SUPER_POOP,
                ShootOverStageConstants.CONVEYOR_VELOCITY_SUPER_POOP
            ),
            swerveDrive.turnCommand(
                Units.Degrees.of(
                    swerveDrive.estimator.estimatedPosition.translation.getRotationToTranslation(
                        ShootOverStageConstants.SUPER_POOP_TRANSLATION
                    ).degrees
                ),
                ShootOverStageConstants.SUPER_POOP_TURN_TOLERANCE.`in`(Units.Rotations)
            )
        ).until { shooterConveyorHoodAtSetpoint() }
    }

    private fun shootOverStageEnd(): Command {
        return WarmupCommands.stopWarmup()
    }

    fun shootOverStage(): Command = shootOverStageInit().finallyDo(shootOverStageEnd())

    fun shooterConveyorHoodAtSetpoint(): Boolean {
        return shooter.atSetpoint() && conveyor.atSetPoint() && hood.atSetpoint()
    }
}
