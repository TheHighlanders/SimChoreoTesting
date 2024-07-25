// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;

public class SwerveSim extends SwerveBase {
    private SwerveModulePosition[] lastModulePositions;
    private Rotation2d rawGyroRotation = new Rotation2d();

    public SwerveSim() {
        /* Initializes modules from Constants */
        modules = new SwerveModuleSim[] {
                new SwerveModuleSim(0),
                new SwerveModuleSim(1),
                new SwerveModuleSim(2),
                new SwerveModuleSim(3),
        };
        lastModulePositions = getModulePositions();
    }

    @Override
    public void periodic() {
        updateOdometry();
        updateAllModules();
    }

    public Rotation2d getYaw() {
        return swervePoseEstimator.getEstimatedPosition().getRotation();
    }

    public void updateOdometry(){
        double sampleTimestamp = Timer.getFPGATimestamp();
        // Read wheel positions and deltas from each module
        SwerveModulePosition[] modulePositions = getModulePositions();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            moduleDeltas[moduleIndex] =
             new SwerveModulePosition(
                 modulePositions[moduleIndex].distanceMeters
                      - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
                // 
        }
        // Use the angle delta from the kinematics and module delt
        // s
        Twist2d twist = Constants.kinematics.toTwist2d(moduleDeltas);
        //
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    
        // Apply update
        swervePoseEstimator.updateWithTime(sampleTimestamp,rawGyroRotation,modulePositions);
    }

    /**
     * MUST be called every loop
     */
    public void updateAllModules(){
        for(SwerveModule mod : modules){
            mod.updateModule();
        }
    }
}