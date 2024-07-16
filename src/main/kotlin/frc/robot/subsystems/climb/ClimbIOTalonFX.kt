package frc.robot.subsystems.climb

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode
import com.ctre.phoenix6.controls.StrictFollower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.controls.DutyCycleOut
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import frc.robot.Ports

class ClimbIOTalonFX : ClimbIO {
    private val mainMotor = TalonFX(Ports.Climb.MAIN_MOTOR_ID)
    private val auxMotor = TalonFX(Ports.Climb.AUX_MOTOR_ID)
    private val stopperMotor = TalonSRX(Ports.Climb.STOPPER_ID)

    private val percentOutput = DutyCycleOut(0.0).withEnableFOC(true)

    init {
        mainMotor.configurator.apply(ClimbConstants.MOTOR_CONFIG)
        auxMotor.configurator.apply(ClimbConstants.MOTOR_CONFIG)
        auxMotor.setControl(StrictFollower(mainMotor.deviceID))

        stopperMotor.configFactoryDefault()
        stopperMotor.enableCurrentLimit(true)
        stopperMotor.enableVoltageCompensation(true)
        stopperMotor.configVoltageCompSaturation(12.0)
        stopperMotor.configPeakCurrentLimit(ClimbConstants.STOPPER_MOTOR_CURRENT_LIMIT)
        stopperMotor.inverted = true
    }

    override fun setPower(power: Double) {
        mainMotor.setControl(percentOutput.withOutput(power))
    }

    override fun openStopper() {
        stopperMotor.set(TalonSRXControlMode.PercentOutput, 0.5)
    }

    override fun closeStopper() {
        stopperMotor.set(TalonSRXControlMode.PercentOutput, -0.5)
    }

    override fun disableStopper() {
        stopperMotor.set(TalonSRXControlMode.Disabled, 0.0)
    }

    override fun updateInputs() {
        inputs.stopperAppliedVoltage.mut_replace(stopperMotor.motorOutputVoltage, Units.Volt)
        inputs.stopperCurrent.mut_replace(stopperMotor.statorCurrent, Units.Amps)
        inputs.mainMotorAppliedVoltage.mut_replace(mainMotor.motorVoltage.value, Units.Volt)
        inputs.isStopperStuck = stopperMotor.statorCurrent >= ClimbConstants.STOPPER_MOTOR_CURRENT_THRESHOLD
    }
}