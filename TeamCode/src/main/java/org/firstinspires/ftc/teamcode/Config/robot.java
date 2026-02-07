 package org.firstinspires.ftc.teamcode.Config;

 import org.firstinspires.ftc.robotcore.external.Telemetry;
 import org.firstinspires.ftc.teamcode.Config.Commands.IntakeOff;
 import org.firstinspires.ftc.teamcode.Config.Subsystems.robotHardware;
 import org.firstinspires.ftc.teamcode.Config.Utils.TelemetryPacket;

 import com.bylazar.telemetry.JoinedTelemetry;
 import com.pedropathing.geometry.Pose;
 import com.qualcomm.robotcore.hardware.Gamepad;
 import com.qualcomm.robotcore.hardware.HardwareMap;
 import com.seattlesolvers.solverslib.command.Robot;
 import com.seattlesolvers.solverslib.command.button.Button;
 import com.seattlesolvers.solverslib.command.button.GamepadButton;
 import com.seattlesolvers.solverslib.command.button.Trigger;
 import com.seattlesolvers.solverslib.gamepad.GamepadEx;
 import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
 import com.seattlesolvers.solverslib.gamepad.TriggerReader;

 import java.util.LinkedList;

 public class robot extends Robot {
  private HardwareMap hardwareMap;
  private robotHardware hardware;
  private GamepadEx driver, operator;
  private boolean isTeleOp = true;
  private Constants constantsload;
  private LinkedList<TelemetryPacket> telemetryPackets;
  private JoinedTelemetry telemetry;

  // Triggers and buttons

     Button Cross;
     Button Circle;
     Button Triangle;
     Button Square;
     Button RightBumper; //Ground Intake
     Button LeftBumper; // HP Intake
     Button dpadDown;
     Button dpadUp;
     TriggerReader RightTrigger; // Launch Far
     TriggerReader LeftTrigger; // Launch Close
     Trigger Right;
     Trigger Left;

  // the constructor with a specified opmode type
  /**
   * @param hardwareMap the hardware map of the robot inits as auto
   */
  public robot(HardwareMap hardwareMap) {
    this.hardwareMap = hardwareMap;
    constantsload = new Constants();
    telemetryPackets = new LinkedList<TelemetryPacket>();
    initAuto();
  }
  public robot(HardwareMap hardwareMap, JoinedTelemetry telemetry) {
     this.hardwareMap = hardwareMap;
     constantsload = new Constants();
     telemetryPackets = new LinkedList<TelemetryPacket>();
     this.telemetry = telemetry;
     initAuto();
     }

 public robot(HardwareMap hardwareMap, JoinedTelemetry telemetry, Constants.Robot.Alliance alliance) {
     this.hardwareMap = hardwareMap;
     constantsload = new Constants();
     telemetryPackets = new LinkedList<TelemetryPacket>();
     this.telemetry = telemetry;
     Constants.Robot.alliance = alliance;
     initAuto();
 }

  public robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
    this.hardwareMap = hardwareMap;
      telemetryPackets = new LinkedList<TelemetryPacket>();
      initTele(gamepad1, gamepad2);
  }
     public robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, JoinedTelemetry telemetry) {
         this.hardwareMap = hardwareMap;
         telemetryPackets = new LinkedList<TelemetryPacket>();
         this.telemetry = telemetry;
         initTele(gamepad1, gamepad2);
     }

  /*
   * Initialize teleop or autonomous, depending on which is used
   */
  public void initTele(Gamepad gamepad1, Gamepad gamepad2) {
    // initialize teleop-specific scheduler
    hardware = new robotHardware(hardwareMap);
    this.driver = new GamepadEx(gamepad1);
    this.operator = new GamepadEx(gamepad2);
    hardware.getFollower().startTeleopDrive();
    isTeleOp = true;
    driverTriggers();
    driverTriggerCommands();
  }

  public void initAuto() {
    // initialize auto-specific scheduler
    hardware = new robotHardware(hardwareMap);
    isTeleOp = false;
  }

  private void driverTriggers() {
    Cross = new GamepadButton(driver, GamepadKeys.Button.CROSS);
    Circle = new GamepadButton(driver, GamepadKeys.Button.CIRCLE);
    Triangle = new GamepadButton(driver, GamepadKeys.Button.TRIANGLE);
    Square = new GamepadButton(driver,GamepadKeys.Button.SQUARE);
    RightBumper = new GamepadButton(driver,GamepadKeys.Button.RIGHT_BUMPER);
    LeftBumper = new GamepadButton(driver,GamepadKeys.Button.LEFT_BUMPER);
    RightTrigger = new TriggerReader(driver, GamepadKeys.Trigger.RIGHT_TRIGGER);
    LeftTrigger = new TriggerReader(driver, GamepadKeys.Trigger.LEFT_TRIGGER);
    Right = new Trigger(()->RightTrigger.isDown());
    Left = new Trigger(()->LeftTrigger.isDown());
    dpadDown = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);
    dpadUp = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);

  }

  private void driverTriggerCommands() {
      hardware.getIntake().setDefaultCommand(new IntakeOff(hardware.getIntake()));
      RightBumper.whenReleased(hardware.getIntake().Hold());
      LeftBumper.whenReleased(hardware.getIntake().Hold());
      RightBumper.whileHeld(hardware.getIntake().GroundIntake());
      LeftBumper.whileHeld(hardware.getIntake().HP_Intaking());
      Right.whenInactive(hardware.getLauncher().stopPower());
      Left.whenInactive(hardware.getLauncher().stopPower());
      Left.whenActive(hardware.getLauncher().LaunchClose());
      Right.whenActive(hardware.getLauncher().LaunchFar());
      dpadUp.whenHeld(hardware.getIntake().Launch());
      dpadDown.whenHeld(hardware.getIntake().Hold());
      Cross.whenReleased(()->hardware.getTurret().faceTarget(Constants.Robot.Goal, hardware.getDrivetrain().getFollower().getPose()));
      Circle.whenReleased(()->hardware.getTurret().enablePinpointTracking());
      Square.whenReleased(()->hardware.getTurret().disablePinpointTracking());
  }

  public void read() {
    if (isTeleOp) {
      driver.readButtons();
      RightTrigger.readValue();
      LeftTrigger.readValue();
      hardware.getFollower().setTeleOpDrive(-driver.gamepad.left_stick_y, -driver.gamepad.left_stick_x, -driver.gamepad.right_stick_x, true);
    }
    hardware.read();
  }

  public void loop() {
    hardware.loop();
    updateTelemetry();
  }

  public void write() {
    hardware.write();
  }

  /**
   * Get the robot hardware subsystem
   *
   * @return The robotHardware instance
   */
  public robotHardware getHardware() {
    return hardware;
  }

  private void updateTelemetry(){
      telemetry.clearAll();
      telemetryPackets.clear();
      telemetryPackets.addAll(hardware.getTelemetry());
      for (TelemetryPacket t: telemetryPackets){
          telemetry.addData(t.getName(),t.getValue());
      }
//      telemetry.addData("Launcher Actual Velocity", hardware.getLauncher().getFlywheelVelocity());
//      telemetry.addData("Laucher Target Velocity", hardware.getLauncher().getFlywheelTargetVelocity());
//      telemetry.addData("pose",hardware.getDrivetrain().getFollower().getPose());
      telemetry.update();
  }
     public void setGoalTarget() {
         if (Constants.Robot.alliance == Constants.Robot.Alliance.BLUE && Constants.Robot.Goal.getX() != 6)
             Constants.Robot.Goal = Constants.Robot.BlueGoal;
         else if (Constants.Robot.alliance == Constants.Robot.Alliance.RED && Constants.Robot.Goal.getX() != (144 - 6))
             Constants.Robot.Goal = Constants.Robot.BlueGoal.mirror();
     }
 }
