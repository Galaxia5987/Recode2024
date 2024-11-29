package frc.robot.subsystems.gripper

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.SparkMaxSim

class GripperIOSIm : GripperIO {
    override val inputs: LoggedGripperInputs = LoggedGripperInputs()
    private val motor = SparkMaxSim(1, 1.0, 0.5, 1.0)
    override fun setPower(power: Double) {
        motor.set(power)
    }

    override fun stop() {
        motor.set(0.0)
    }

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())
        inputs.spinMotorVoltage = Units.Volt.of(motor.busVoltage)
    }
}