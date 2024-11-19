package frc.robot.subsystems.conveyor

import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import frc.robot.Constants
import frc.robot.Ports
import org.team9432.annotation.Logged

class ConveyorIOReal : ConveyorIO {
    override val inputs: LoggedConveyorInputs = LoggedConveyorInputs()
    private val spinMotor: TalonFX = TalonFX(Ports.Conveyor.MOTOR_ID)
    private val controlRequest = VelocityVoltage(0.0)

    init {
        spinMotor.configurator.apply(ConveyorConstants.CONFIG)
    }

    override fun updateInput() {
        inputs.spinMotorVelocity.mut_replace(spinMotor.get(), Units.RotationsPerSecond)
    }

    override fun setSpinVelocity(vel: Measure<Velocity<Angle>>) {
        spinMotor.setControl(controlRequest.withVelocity(vel.`in`(Units.RotationsPerSecond)))
    }
}