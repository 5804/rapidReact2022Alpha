// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Gains {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;

    
    public Gains(double _kF, double _kP, double _kI, double _kD) {
        kF = _kF;
        kP = _kP;
        kI = _kI;
        kD = _kD;
    }
}