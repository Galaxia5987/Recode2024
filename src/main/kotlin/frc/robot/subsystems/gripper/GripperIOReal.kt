package frc.robot.subsystems.gripper

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.Ports

class GripperIOReal : GripperIO {
    override val inputs: LoggedGripperInputs = LoggedGripperInputs()

    private val spinMotor: CANSparkMax =
        CANSparkMax(Ports.Gripper.ROLLER_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val sensor: DigitalInput = DigitalInput(8)

    override fun setPower(power: Double) {
        spinMotor.set(power)
    }

    override fun stop() {
        spinMotor.set(0.0)
    }


    override fun updateInputs() {
        inputs.spinMotorVoltage = Units.Volt.of(spinMotor.busVoltage)
        inputs.hasNote = !sensor.get()
    }

}
