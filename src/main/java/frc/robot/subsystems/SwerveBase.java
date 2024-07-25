// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.SwerveModuleConfig;

public class SwerveBase extends SubsystemBase {

    public class Constants {

        static boolean diagnosticMode = false;
        static SwerveModuleConfig FL = new SwerveModuleConfig(1, 2, new Rotation2d());
        static SwerveModuleConfig FR = new SwerveModuleConfig(2, 3, new Rotation2d());
        static SwerveModuleConfig BL = new SwerveModuleConfig(4, 5, new Rotation2d());
        static SwerveModuleConfig BR = new SwerveModuleConfig(6, 7, new Rotation2d());
        static double kWheelBase = Units.inchesToMeters(24);
        static double kTrackWidth = Units.inchesToMeters(24);
        static double kMaxSpeed = 3.0;
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
        );

        private class AutoConstants {
            static double kPTranslateController = (Robot.isReal() ? 0 : 5);
            static double kPRotateController = (Robot.isReal() ? 0 : 1);
        }
    }

    /* Array of Modules */
    public SwerveModule[] modules;
    public ChassisSpeeds chassisSpeeds;
    public SwerveDrivePoseEstimator swervePoseEstimator;
    private Field2d field;
    private ChoreoTrajectory traj;
    private StructArrayPublisher<SwerveModuleState> statePublisher;
    private StructArrayPublisher<SwerveModuleState> setpointPublisher;
    private StructPublisher<Pose2d> posePublisher;


    public SwerveBase() {
        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.kinematics, new Rotation2d(), new SwerveModulePosition[]{new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()}, new Pose2d());
        field = new Field2d();
        chassisSpeeds = new ChassisSpeeds();
        SmartDashboard.putData(field);
        field.getObject("traj");

        statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Swerve/States", SwerveModuleState.struct).publish();        
        setpointPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Swerve/Setpoints", SwerveModuleState.struct).publish();
        posePublisher = NetworkTableInstance.getDefault().getStructTopic("/Swerve/Odometer", Pose2d.struct).publish();
    }

    @Override
    public void periodic() {
        if (Constants.diagnosticMode) {
            sendDiagnostics();
        }
        field.setRobotPose(swervePoseEstimator.getEstimatedPosition());
        posePublisher.set(swervePoseEstimator.getEstimatedPosition());
    }

    public Command getTrajectoryCommand(String name){
        traj = Choreo.getTrajectory(name); // Gets Trajectory from Choreo file


        field.getObject("traj").setTrajectory(convertChoreoTrajectoryToWPI(traj));

        return Choreo.choreoSwerveCommand(
            traj, // Trajectory to run
            this::getPose, // Supplier to recieve pose
            new PIDController(Constants.AutoConstants.kPTranslateController, 0.0, 0.0), // PID Controller for Translation in the X direction
            new PIDController(Constants.AutoConstants.kPTranslateController, 0.0, 0.0), // PID Controller for Translation in the Y Direction
            new PIDController(Constants.AutoConstants.kPRotateController, 0.0, 0.0), // PID Controller for Rotation
            (ChassisSpeeds speeds) -> //
                this.drive(
                        new Translation2d(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond),
                        new Rotation2d(-speeds.omegaRadiansPerSecond),
                        false,
                        false
                    ),
            () -> {
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == Alliance.Red;
            }, // Boolean Supplier to control Trajectory Mirroring
            this // Drive Subsystem
        );
    }

    /**
     * Runs all IK and sets modules states
     *
     * @param translate     Desired translations speeds m/s
     * @param rotate        Desired rotation rate Rotation2d
     * @param fieldRelative Driving mode
     * @param isOpenLoop    Drive controller mode
     */
    public void drive(Translation2d translate, Rotation2d rotate, boolean fieldRelative, boolean isOpenLoop) {
        chassisSpeeds =
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translate.getX(), translate.getY(), rotate.getRadians(), getYaw())
                : new ChassisSpeeds(translate.getX(), translate.getY(), rotate.getRadians());

        SwerveModuleState[] swerveModuleStates = Constants.kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);

        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        SwerveModuleState[] swerveModuleStates = Constants.kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);

        setModuleStates(swerveModuleStates, true);
    }


    public void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop){
        for(SwerveModule m : modules){
            m.setModuleState(states[m.moduleNumber], isOpenLoop);
        }
    }
    

    public void resetPose(Pose2d pose) {
        swervePoseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /**
     * Returns the gyro's yaw
     *
     * @return Yaw of gyro, includes zeroing
     */
    public Rotation2d getYaw() {
        System.out.println("Parent Swerve Base Used: Should be Sim or Real, not parent");
        return new Rotation2d();
    }

    /**
     * Gets swerve modules positions for all modules
     *
     * @return Array of modules positions, in modules ID order
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : modules) {
            DriverStation.reportWarning("" + mod.moduleNumber, false);
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.kinematics.toChassisSpeeds(getStates());
    }

    /**
     * Gets swerve modules states for all modules
     *
     * @return Array of modules states, in modules ID order
     */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : modules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModuleState[] getSetpoints(){
        SwerveModuleState[] setpoints = new SwerveModuleState[modules.length];
        for(SwerveModule m : modules){
            setpoints[m.moduleNumber] = m.getSetpoint();
        }
        return setpoints;
    }

    public void resetAllModulestoAbsol() {
        for (SwerveModule m : modules) {
            m.setIntegratedAngleToAbsolute();
        }
    }

    public void resetAllModules() {
        for (SwerveModule m : modules) {
            m.configureAngleMotor();
            m.configureDriveMotor();
        }
    }

    public void sendDiagnostics() {
        statePublisher.set(getStates());
        setpointPublisher.set(getSetpoints());
    }

    public void jogSingleModule(int moduleNumber, double input, boolean drive) {
        if (drive) {
            modules[moduleNumber].setDriveState(new SwerveModuleState(input, new Rotation2d(0)), false);
            DriverStation.reportWarning(modules[moduleNumber].driveMotor.getDeviceId() + "", false);
        } else {
            modules[moduleNumber].setAngleState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(input))));
            DriverStation.reportWarning(modules[moduleNumber].angleMotor.getDeviceId() + "", false);
        }
    }

    public void jogAllModuleDrive(double v) {
        drive(new Translation2d(0, v), new Rotation2d(0), true, false);
    }

    private Trajectory convertChoreoTrajectoryToWPI(ChoreoTrajectory traj){
                List<State> states = new ArrayList<State>();
        for(ChoreoTrajectoryState cts : traj.getSamples()){
            states.add(new State(cts.timestamp, Math.hypot(cts.velocityX, cts.velocityY), 0, new Pose2d(cts.x, cts.y, new Rotation2d(cts.heading)),0));
        }
        return new Trajectory(states);
    }
}
