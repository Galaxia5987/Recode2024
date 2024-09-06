package frc.robot.subsystems.hood

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.hardware.TalonFX
import frc.robot.Ports

class HoodIOReal() :HoodIO {
    override var inputs: LoggedInputHood =  LoggedInputHood()
    private var angleMotor:TalonFX = TalonFX(Ports.Hood.MOTOR_ID)
    private var encoder:TalonSRX = TalonSRX(Ports.Hood.ENCODER_ID)
    override fun updateInput() {
        inputs.angle = angleMotor.position.value
        inputs.angleMotorVoltage = angleMotor.supplyVoltage.value
        inputs.encoderPosition = encoder.selectedSensorPosition
    }

    override fun setAngle(angle: Double) {
        angleMotor.setPosition(angle)
    }
}