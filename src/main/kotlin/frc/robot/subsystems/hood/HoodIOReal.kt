package frc.robot.subsystems.hood

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import frc.robot.Ports

class HoodIOReal() : HoodIO {
    override var inputs: LoggedInputHood = LoggedInputHood()
    private var angleMotor: TalonFX = TalonFX(Ports.Hood.MOTOR_ID)
    private var encoder: TalonSRX = TalonSRX(Ports.Hood.ENCODER_ID)

    init {
        angleMotor.configurator.apply(HoodConstants.MOTOR_CONFIGURATION)
    }

    override fun setAngle(angle: Angle) {
        angleMotor.setPosition(angle.`in`(Units.Rotation))
    }

    override fun updateInputs() {
        inputs.angle = angleMotor.position.value
        inputs.angleMotorVoltage = angleMotor.supplyVoltage.value
        inputs.encoderPosition = Units.Rotation.of(encoder.selectedSensorPosition)
    }
}