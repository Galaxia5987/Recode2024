package frc.robot.subsystems.hood

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import frc.robot.Ports

class HoodIOReal() :HoodIO {
    override var inputs: LoggedInputHood =  LoggedInputHood()
    private var angleMotor:TalonFX = TalonFX(Ports.Hood.MOTOR_ID)
    private var encoder:TalonSRX = TalonSRX(Ports.Hood.ENCODER_ID)

    init {
        angleMotor.configurator.apply(HoodConstants.MOTOR_CONFIGURATION)
    }

    override fun setAngle(angle: Measure<Angle>) {
        angleMotor.setPosition(angle.`in`(Units.Rotation))
    }

    override fun updateInputs() {
        inputs.angle.mut_replace(angleMotor.position.value,Units.Rotations)
        inputs.angleMotorVoltage.mut_replace(angleMotor.supplyVoltage.value,Units.Volts)
        inputs.encoderPosition.mut_replace(encoder.selectedSensorPosition,Units.Rotations)
    }
}