package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import kotlin.math.PI

object ShooterConstants {
    var configTop: TalonFXConfiguration = TalonFXConfiguration()
    var configBottom: TalonFXConfiguration = TalonFXConfiguration()
    var currentLimit: Double = 40.0
    var runningVelocity = MutableMeasure.ofBaseUnits(3.0,Units.RotationsPerSecond)
    var KP: Double = 0.0
    var KD: Double = 0.0
    var KI: Double = 0.0
    var KV: Double = 0.0

    init {
        configTop.withCurrentLimits(
            CurrentLimitsConfigs()
                .withStatorCurrentLimit(currentLimit * 2)
                .withSupplyCurrentLimit(currentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
        ).withSlot0(
            Slot0Configs()
                .withKP(KP)
                .withKD(KD)
                .withKI(KI)
                .withKV(KV)
        ).withMotorOutput(MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
        configBottom.withCurrentLimits(
            CurrentLimitsConfigs()
                .withStatorCurrentLimit(currentLimit * 2)
                .withSupplyCurrentLimit(currentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
        ).withSlot0(
            Slot0Configs()
                .withKP(KP)
                .withKD(KD)
                .withKI(KI)
                .withKV(KV)
        ).withMotorOutput(MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
    }
}