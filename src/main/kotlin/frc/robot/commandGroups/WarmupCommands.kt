package frc.robot.commandGroups

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.conveyor.Conveyor
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.shooter.Shooter

private val hood = Hood.getInstance()
private val shooter = Shooter.getInstance()
private val conveyor = Conveyor.getInstance()

fun warmup(
    hoodAngle: () -> Measure<Angle> = { Units.Degrees.of(65.0) },
    shooterVelocity: () -> Measure<Velocity<Angle>> = { Units.RotationsPerSecond.of(70.0) },
    conveyorVelocity: () -> Measure<Velocity<Angle>> = { Units.RotationsPerSecond.of(60.0) }
): Command {
    return Commands.parallel(
            hood.setAngle(hoodAngle),
            shooter.setVelocity(shooterVelocity),
            conveyor.setVelocity(conveyorVelocity)
        )
}

fun stopWarmup(): Command = Commands.parallel(shooter.stop(), conveyor.stop(), hood.setRestingAngle())
