package frc.robot.subsystems.hood

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import frc.robot.Ports

class HoodIOReal() :HoodIO {
    override var inputs: LoggedInputHood =  LoggedInputHood()
    private var angleMotor:TalonFX = TalonFX(Ports.Hood.MOTOR_ID)
    private var encoder:TalonSRX = TalonSRX(Ports.Hood.ENCODER_ID)


    override fun updateInput() {
        inputs.angle.mut_replace(angleMotor.position.value,Units.Rotations)
        inputs.angleMotorVoltage.mut_replace(angleMotor.supplyVoltage.value,Units.Volts)
        inputs.encoderPosition.mut_replace(encoder.selectedSensorPosition,Units.Rotations)
    }

    override fun setAngle(angle: Double) {
        angleMotor.setPosition(angle)
        inputs.angleSetPoint.mut_replace(angle,Units.Rotations)
    }
}