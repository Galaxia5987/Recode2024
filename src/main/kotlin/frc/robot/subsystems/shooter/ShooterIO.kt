package frc.robot.subsystems.shooter

import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import edu.wpi.first.units.*
import org.team9432.annotation.Logged

interface ShooterIO {
    var inputs: LoggedShooterInputs

    fun setTopVel(vel: Measure<Velocity<Angle>>)
    fun setBottomVel(vel: Measure<Velocity<Angle>>)
    fun updateInput()

    @Logged
    open class ShooterInputs {
        var topVel:Measure<Velocity<Angle>> = Units.RotationsPerSecond.zero()
        var buttomVel:Measure<Velocity<Angle>> = Units.RotationsPerSecond.zero()
    }
}