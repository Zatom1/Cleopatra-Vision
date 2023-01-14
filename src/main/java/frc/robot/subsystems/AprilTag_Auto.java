// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;

public class AprilTag_Auto extends SubsystemBase {
  /** Creates a new AprilTag_Auto. */

  
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  // Change this to match the name of your camera
  // PhotonCamera camera = new PhotonCamera("gloworm");
  PhotonCamera camera = new PhotonCamera("172.22.11.3");

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.01;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);


  public AprilTag_Auto() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double forwardSpeed;
    double rotationSpeed;

    forwardSpeed = -RobotContainer.getRightY();

    if (RobotContainer.getAButton()) {
        // Vision-alignment mode
        // Query the latest result from PhotonVision
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
          rotationSpeed = 0.5;

            // Calculate angular turn power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            System.out.println("target found");

            rotationSpeed = -0.6;
            rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
            double angle = result.getBestTarget().getYaw();
            double area = result.getBestTarget().getArea();
            int fidId = result.getBestTarget().getFiducialId();
            if(Math.abs(angle)>Constants.TURN_TOLERANCE){

            rotationSpeed = angle/(16+Math.abs(angle));

            }
            else{

              rotationSpeed = 0.0;
            }
            if(fidId == 1&& area<1){
              System.out.println("distance found");
              
              forwardSpeed = 1-area;
            }
            if(fidId == 520 && area>1.5){
              System.out.println("distance found");

       
              forwardSpeed = -area/8;
            }
        } else {
            // If we have no targets, stay still.
            rotationSpeed = 0.0;
        }
    } else {
        // Manual Driver Mode
        rotationSpeed = RobotContainer.getLeftX();
    }

    // Use our forward/turn speeds to control the drivetrain
    RobotContainer.m_drivebase.drive(forwardSpeed, rotationSpeed);

  }
}
