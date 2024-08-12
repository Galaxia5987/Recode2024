package frc.robot.commandGroups

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.StartEndCommand
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.conveyor.Conveyor
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.hood.HoodConstants
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.swerve.SwerveDrive

object CommandGroups {
    private val swerveDrive = SwerveDrive.getInstance()
    private val shooter = Shooter.getInstance()
    private val hood = Hood.getInstance()
    private val conveyor = Conveyor.getInstance()
    private val climb = Climb.getInstance()
    private val intake = Intake.getInstance()
    private val gripper = Gripper.getInstance()

    fun warmup(
        hoodAngle: Measure<Angle> = Units.Degrees.of(65.0),
        shooterVelocity: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(70.0),
        conveyorVelocity: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(60.0)
    ): Command {
        return Commands.parallel(
            hood.setAngle(hoodAngle.mutableCopy()),
            shooter.setVelocity(shooterVelocity.mutableCopy()),
            conveyor.setVelocity(conveyorVelocity)
        )
    }

    fun stopWarmup(): Command {
        return warmup(HoodConstants.RESTING_ANGLE, Units.RotationsPerSecond.zero(), Units.RotationsPerSecond.zero())
    }

    fun setpointShoot(): Command {
        return Commands.parallel(
            warmup(
                Units.Degrees.of(84.0),
                Units.RotationsPerSecond.of(73.0),
                Units.RotationsPerSecond.of(50.0)
            )
        ).until {shooter.atSetpoint() && hood.atSetpoint() && swerveDrive.atTurnSetpoint}
    }

    fun openClimb(): Command {
        return Commands.sequence(
            climb.setPower { -0.3 }.withTimeout(1.25),
            climb.open(),
            climb.setPower { -0.5 }.withTimeout(2.5),
        )
    }

    fun closeClimb(): Command {
        return StartEndCommand(
            { climb.setPower { 0.5 } },
            climb::lock,
            climb
        )
    }

    fun intake(rumble: Command): Command {
        return Commands.parallel(
            intake.intake(), gripper.setRollerPower(0.4)
        )
            .until { gripper.hasNote }
            .andThen(Commands.parallel(intake.stop(), gripper.setRollerPower(0.0), rumble))
            .withName("intake")
    }

    fun outtake(): Command {
        return Commands.parallel(
            intake.outtake(),
            (gripper.setRollerPower(-0.7))
        ).withName("outtake")
    }
}