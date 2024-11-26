package frc.robot.subsystems.gripper

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.Ports

class GripperIOReal : GripperIO {
    override val inputs: LoggedGripperInputs = LoggedGripperInputs()

    private val spinMotor: SparkMax =
        SparkMax(Ports.Gripper.ROLLER_ID, SparkLowLevel.MotorType.kBrushless)
    private val sensor: DigitalInput = DigitalInput(Ports.Gripper.SENSOR_ID)

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
