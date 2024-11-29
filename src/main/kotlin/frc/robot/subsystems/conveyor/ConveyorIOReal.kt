package frc.robot.subsystems.conveyor

import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.Ports

class ConveyorIOReal : ConveyorIO {
    override var inputs: LoggedConveyorInputs = LoggedConveyorInputs()
    private var spinMotor: TalonFX = TalonFX(Ports.Conveyor.MOTOR_ID)
    private val controlRequest = VelocityVoltage(0.0)

    init {
        spinMotor.configurator.apply(ConveyorConstants.CONFIG)
    }

    override fun updateInput() {
        inputs.spinMotorVelocity = Units.RotationsPerSecond.of(spinMotor.get())
    }

    override fun setSpinVelocity(velocity: AngularVelocity) {
        spinMotor.setControl(controlRequest.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }
}