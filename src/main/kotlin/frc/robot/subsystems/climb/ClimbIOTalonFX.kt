package frc.robot.subsystems.climb

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.StrictFollower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.Ports

class ClimbIOTalonFX : ClimbIO {
    override val inputs = LoggedClimbInputs()
    private val mainMotor = TalonFX(Ports.Climb.MAIN_MOTOR_ID)
    private val auxMotor = TalonFX(Ports.Climb.AUX_MOTOR_ID)
    private val stopperMotor = TalonSRX(Ports.Climb.STOPPER_ID)

    private val percentOutput = DutyCycleOut(0.0).withEnableFOC(true)

    init {
        val motorConfig = TalonFXConfiguration()
            .withMotorOutput(
            MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        ).CurrentLimits
            .withStatorCurrentLimitEnable(false)
            .withSupplyCurrentLimitEnable(false)

        mainMotor.configurator.apply(motorConfig)
        auxMotor.configurator.apply(motorConfig)
        auxMotor.setControl(StrictFollower(mainMotor.deviceID))

        stopperMotor.configFactoryDefault()
        stopperMotor.enableCurrentLimit(true)
        stopperMotor.enableVoltageCompensation(true)
        stopperMotor.configVoltageCompSaturation(ClimbConstants.STOPPER_MOTOR_VOLTAGE_COMPENSATION_SATURATION)
        stopperMotor.configPeakCurrentLimit(ClimbConstants.STOPPER_MOTOR_CURRENT_LIMIT)
        stopperMotor.setNeutralMode(NeutralMode.Brake)
        stopperMotor.inverted = true
    }

    override fun setPower(power: Double) {
        mainMotor.setControl(percentOutput.withOutput(power))
    }

    override fun openStopper() {
        stopperMotor.set(TalonSRXControlMode.PercentOutput, ClimbConstants.STOPPER_MOTOR_POWER)
    }

    override fun closeStopper() {
        stopperMotor.set(TalonSRXControlMode.PercentOutput, -ClimbConstants.STOPPER_MOTOR_POWER)
    }

    override fun disableStopper() {
        stopperMotor.neutralOutput()
    }

    override fun updateInputs() {
        inputs.stopperAppliedVoltage = stopperMotor.motorOutputVoltage
        inputs.stopperCurrent = stopperMotor.statorCurrent
        inputs.mainMotorAppliedVoltage = mainMotor.motorVoltage.value
    }
}