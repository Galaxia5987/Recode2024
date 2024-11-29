package frc.robot.subsystems.conveyor

import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim

class ConveyorIOSim : ConveyorIO {
    override val inputs: LoggedConveyorInputs = LoggedConveyorInputs()
    private var spinMotor: TalonFXSim = TalonFXSim(1, ConveyorConstants.GEAR_RATIO, 1.0, 1.0)
    private val controlRequest = VelocityVoltage(0.0)
    override fun updateInput() {
        spinMotor.update(Timer.getFPGATimestamp())
        inputs.spinMotorVelocity = Units.RotationsPerSecond.of(spinMotor.velocity)
    }

    override fun setSpinVelocity(velocity: AngularVelocity) {
        spinMotor.setControl(controlRequest.withVelocity(velocity))
    }
}