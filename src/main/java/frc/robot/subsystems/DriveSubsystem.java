// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    // Constants
    private final double kNEOMaxRPM = 5676;
    private final double kWheelDiameterInches = 6.0;
    private final double kDrivetrainGearRatio = 10.71;

    // Hardware
    private final SparkMax m_leftLeaderMotor;
    private final SparkMax m_rightLeaderMotor;
    private final DifferentialDrive drive;

    // Simulation
    private final SparkMaxSim m_leftMotorSim;
    private final SparkMaxSim m_rightMotorSim;
    private final DifferentialDrivetrainSim m_driveSim;
    private final DifferentialDriveOdometry m_odometry;
    private final StructPublisher<Pose2d> m_publisher;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // Instantiate motors
        m_leftLeaderMotor = new SparkMax(2, MotorType.kBrushless);
        m_rightLeaderMotor = new SparkMax(3, MotorType.kBrushless);

        // Drive helper
        drive = new DifferentialDrive(m_leftLeaderMotor, m_rightLeaderMotor);

        // Configure motors
        configure();

        // Simulation motor models
        m_leftMotorSim = new SparkMaxSim(m_leftLeaderMotor, DCMotor.getNEO(2));
        m_rightMotorSim = new SparkMaxSim(m_rightLeaderMotor, DCMotor.getNEO(2));

        // Simulate robot drive (kitbot preset)
        m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
                KitbotMotor.kDoubleNEOPerSide,
                KitbotGearing.k10p71,
                KitbotWheelSize.kSixInch,
                null // Optional measurement noise
        );

        // Odometry (for tracking robot position)
        m_odometry = new DifferentialDriveOdometry(
                new Rotation2d(),
                m_driveSim.getLeftPositionMeters(),
                m_driveSim.getRightPositionMeters()
        );

        // NetworkTables pose publisher (for Shuffleboard/Sim GUI)
        m_publisher = NetworkTableInstance.getDefault()
                .getStructTopic("MyPose", Pose2d.struct)
                .publish();
    }

    /** Arcade drive control method */
    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(fwd, rot);
    }

    /** Motor configuration */
    private void configure() {
        SparkMaxConfig m_leftLeaderMotorConfig = new SparkMaxConfig();
        SparkMaxConfig m_rightLeaderMotorConfig = new SparkMaxConfig();

        m_leftLeaderMotorConfig.smartCurrentLimit(9);
        m_rightLeaderMotorConfig.smartCurrentLimit(9);

        m_leftLeaderMotor.configure(
                m_leftLeaderMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );
        m_rightLeaderMotor.configure(
                m_rightLeaderMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );
    }

    @Override
    public void periodic() {
        // This method is called once per scheduler run (on real robot)
    }

    @Override
    public void simulationPeriodic() {
        // Step the physics sim forward by 20ms
        m_driveSim.update(0.02);

        // Convert motor percent output to wheel speed (m/s)
        double leftSpeedMps = (((kNEOMaxRPM * m_leftLeaderMotor.get()) / kDrivetrainGearRatio) * Math.PI * kWheelDiameterInches) / 60.0;
        double rightSpeedMps = (((kNEOMaxRPM * m_rightLeaderMotor.get()) / kDrivetrainGearRatio) * Math.PI * kWheelDiameterInches) / 60.0;

        double vin = RoboRioSim.getVInVoltage();

        m_leftMotorSim.iterate(leftSpeedMps, vin, 0.02);
        m_rightMotorSim.iterate(rightSpeedMps, vin, 0.02);

        // Simulate applied voltage and bus voltage
        m_leftMotorSim.setBusVoltage(vin);
        m_rightMotorSim.setBusVoltage(vin);

        m_leftMotorSim.setAppliedOutput(m_leftLeaderMotor.getAppliedOutput());
        m_rightMotorSim.setAppliedOutput(m_rightLeaderMotor.getAppliedOutput());

        // Set inputs into drivetrain sim
        m_driveSim.setInputs(
                m_leftMotorSim.getAppliedOutput() * m_leftMotorSim.getBusVoltage(),
                m_rightMotorSim.getAppliedOutput() * m_rightMotorSim.getBusVoltage()
        );

        // Update odometry with simulated encoder positions and heading
        m_odometry.update(
                m_driveSim.getHeading(),
                m_driveSim.getLeftPositionMeters(),
                m_driveSim.getRightPositionMeters()
        );

        // Publish pose to NetworkTables
        m_publisher.set(m_odometry.getPoseMeters());
    }
}