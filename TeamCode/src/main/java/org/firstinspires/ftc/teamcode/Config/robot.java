 package org.firstinspires.ftc.teamcode.Config;

 import org.firstinspires.ftc.teamcode.Config.Commands.IntakeOff;
 import org.firstinspires.ftc.teamcode.Config.Subsystems.robotHardware;

 import com.qualcomm.robotcore.hardware.Gamepad;
 import com.qualcomm.robotcore.hardware.HardwareMap;
 import com.seattlesolvers.solverslib.command.Robot;
 import com.seattlesolvers.solverslib.command.button.Button;
 import com.seattlesolvers.solverslib.command.button.GamepadButton;
 import com.seattlesolvers.solverslib.command.button.Trigger;
 import com.seattlesolvers.solverslib.gamepad.GamepadEx;
 import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
 import com.seattlesolvers.solverslib.gamepad.TriggerReader;

 public class robot extends Robot {
  private HardwareMap hardwareMap;
  private robotHardware hardware;
  private GamepadEx driver, operator;
  private boolean isTeleOp = true;

  // Triggers and buttons

     Button Cross;
     Button Circle;
     Button Triangle;
     Button Square;
     Button RightBumper; //Ground Intake
     Button LeftBumper; // HP Intake
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
    initAuto();
  }

  public robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
    this.hardwareMap = hardwareMap;
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

  }

  private void driverTriggerCommands() {
      hardware.getIntake().setDefaultCommand(new IntakeOff(hardware.getIntake()));
      RightBumper.whenReleased(hardware.getIntake().Hold());
      LeftBumper.whenReleased(hardware.getIntake().Hold());
      RightBumper.whileHeld(hardware.getIntake().GroundIntake());
      LeftBumper.whileHeld(hardware.getIntake().HP_Intaking());
      Left.whenActive(hardware.getLauncher().LaunchClose());
      Right.whenActive(hardware.getIntake().Launch());
      Left.whenInactive(hardware.getLauncher().stopPower());
  }

  public void read() {
    if (isTeleOp) {
      driver.readButtons();
      RightTrigger.readValue();
      LeftTrigger.readValue();
    }
    hardware.read();
  }

  public void loop() {
    hardware.loop();
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
 }
