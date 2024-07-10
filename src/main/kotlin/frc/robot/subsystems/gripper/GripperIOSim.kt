package frc.robot.subsystems.gripper

import com.ctre.phoenix6.controls.DutyCycleOut
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim

class GripperIOSim : GripperIO {
    private val rollerMotor = TalonFXSim(1, 1.0, 0.5, 1.0)
    private val powerRequestRoller = DutyCycleOut(0.0)

    override fun setRollerMotorPower(power: Double) {
        rollerMotor.setControl(powerRequestRoller.withOutput(power))
    }

    override fun updateInputs() {
        rollerMotor.update(Timer.getFPGATimestamp())
        inputs.rollerMotorVoltage.mut_replace(rollerMotor.appliedVoltage, Units.Volts)
    }
}