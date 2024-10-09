package frc.robot.commandgroups

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.conveyor.Conveyor
import frc.robot.subsystems.conveyor.ConveyorConstants
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.hood.HoodConstants
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.shooter.ShooterConstants

class ShootingCommand {
//    private val swerveDrive = SwerveDrive.getInstance()
//    private val shooter = Shooter.getInstance()
    private val hood = Hood.getInstance()
    private val conveyor = Conveyor.getInstance()
    private val shooter = Shooter.getInstance()
    fun setAngle(angle:Measure<Angle>):Command = Commands.runOnce({hood.setAngle(angle)})
    fun stopShooter():Command = Commands.parallel(conveyor.stopConveyor(),shooter.stop())
    fun startShooter():Command = Commands.parallel(conveyor.setPower(Units.RotationsPerSecond.of(ConveyorConstants.RUN_POWER)),shooter.setShooterVel(ShooterConstants.runningVelocity))
    fun setRestingAngle():Command = Commands.runOnce({hood.setRestAngle()})
}