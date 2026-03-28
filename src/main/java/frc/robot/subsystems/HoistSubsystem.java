// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoistSubsystem extends SubsystemBase {
  private final SparkMax m_hoistMotor = new SparkMax(26, MotorType.kBrushless);
  private final AbsoluteEncoder m_absoluteEncoder = m_hoistMotor.getAbsoluteEncoder();

  public HoistSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    m_hoistMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hoist Encoder", m_absoluteEncoder.getPosition());
    //System.out.println(m_absoluteEncoder.getPosition());
  }

  public void TeleOp(DoubleSupplier value){
    m_hoistMotor.set(value.getAsDouble());
  }
  public void Stop(){
    m_hoistMotor.stopMotor();
  }
}
