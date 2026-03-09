// package frc.robot.commands;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

// public class AutoAlignToAprilTagCommand extends Command {
//     private final DriveSubsystem drive;
//     private final VisionSubsystem vision; // could be Limelight or PhotonVision
//     private final PIDController xPID, yPID, rotPID;

//     public AutoAlignToAprilTagCommand(DriveSubsystem drive, VisionSubsystem vision) {
//         this.drive = drive;
//         this.vision = vision;
//         xPID = new PIDController(1.0, 0, 0);
//         yPID = new PIDController(1.0, 0, 0);
//         rotPID = new PIDController(1.0, 0, 0);
//         addRequirements(drive);
//    }

//        @Override
//         public void execute() {
//             var targetOffset = vision.getTargetOffset(); // meters
//             var targetYaw = vision.getTargetYaw(); // radians

//             double forward = xPID.calculate(0, targetOffset.getX());
//             double strafe = yPID.calculate(0, targetOffset.getY());
//             double rot = rotPID.calculate(0, targetYaw);

//         drive.drive(new Translation2d(forward, strafe), rot, true, false);
//     }

//        @Override
//         public boolean isFinished() {
//             return Math.abs(vision.getTargetOffset().getNorm()) < 0.05 &&
//             Math.abs(vision.getTargetYaw()) < 0.02; // done within 5cm / 2°
//      }
// }
