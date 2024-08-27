package frc.robot.commandGroups

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.StartEndCommand
import frc.robot.ControllerInputs
import frc.robot.lib.finallyDo
import frc.robot.lib.getRotationToTranslation
import frc.robot.lib.handleInterrupt
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.conveyor.Conveyor
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.swerve.SwerveDrive

object CommandGroups {
    private val swerveDrive = SwerveDrive.getInstance()
    private val shooter = Shooter.getInstance()
    private val hood = Hood.getInstance()
    private val conveyor = Conveyor.getInstance()
    private val intake = Intake.getInstance()
    private val gripper = Gripper.getInstance()

    fun warmup(
        hoodAngle: Measure<Angle> = Units.Degrees.of(65.0),
        shooterVelocity: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(70.0),
        conveyorVelocity: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(60.0)
    ): Command {
        return Commands.parallel(
            hood.setAngle(hoodAngle),
            shooter.setVelocity(shooterVelocity),
            conveyor.setVelocity(conveyorVelocity)
        )
    }

    fun stopWarmup(): Command = Commands.parallel(shooter.stop(), conveyor.stop(), hood.setRestingAngle())

    fun setpointShoot(): Command {
        return Commands.parallel(
            warmup(
                Units.Degrees.of(93.0),
                Units.RotationsPerSecond.of(73.0),
                Units.RotationsPerSecond.of(50.0)
            )
        ).until {shooter.atSetpoint() && hood.atSetpoint() && swerveDrive.atTurnSetpoint}
    }

    private fun stopIntake(): Command = Commands.parallel(intake.stop(), gripper.stop())

    fun intake(): Command {
        return Commands.parallel(
            intake.intake(), gripper.setRollerPower(0.4)
        )
            .until { gripper.hasNote }
            .andThen(Commands.parallel(intake.stop(), gripper.setRollerPower(0.0), ControllerInputs.startRumble()))
            .finallyDo(stopIntake().alongWith(ControllerInputs.stopRumble()))
            .withName("intake")
    }

    fun outtake(): Command {
        return Commands.parallel(
            intake.outtake(),
            (gripper.setRollerPower(-0.7))
        )
            .finallyDo(stopIntake())
            .withName("outtake")
    }

    fun superPoopInit(): Runnable {
        return Runnable {
            Commands.parallel(
                warmup(
                    CommandGroupsConstants.HOOD_ANGLE_SUPER_POOP,
                    CommandGroupsConstants.SHOOTER_VELOCITY_SUPER_POOP,
                    CommandGroupsConstants.CONVEYOR_VELOCITY_SUPER_POOP
                ), // TODO: ledMode,
                swerveDrive.turnCommand(
                    Units.Degrees.of(
                        swerveDrive.estimator.estimatedPosition.translation.getRotationToTranslation(
                            CommandGroupsConstants.SUPER_POOP_TRANSLATION
                        ).degrees
                    ),
                    CommandGroupsConstants.SUPER_POOP_TURN_TOLERANCE.`in`(Units.Rotations)
                )
            ).until { readyToSuperPoop() } // TODO: add LEDS
        }
    }

    fun superPoopEnd(): Runnable {
        return Runnable {
            stopWarmup()
        }
        // TODO: LED MODE Default
    }

    fun superPoop(): Command {
        return StartEndCommand(
            superPoopInit(),
            superPoopEnd(),
            shooter, hood, conveyor, swerveDrive
        )
    }

    fun readyToSuperPoop(): Boolean {
        return shooter.atSetpoint() && conveyor.atSetPoint() && hood.atSetpoint()
    }
}