package org.firstinspires.ftc.teamcode.Opmodes.Tester;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PS5 LED Motif Tester", group = "Tester")
public class ps5LedTester extends LinearOpMode {

    // Motif types
    private enum Motif {
        GPP,  // Green-Purple-Purple
        PGP,  // Purple-Green-Purple
        PPG   // Purple-Purple-Green
    }

    private Motif currentMotif = Motif.GPP;
    private boolean isFlashing = false;
    private int currentColorIndex = 0;
    private ElapsedTime flashTimer = new ElapsedTime();
    private double colorOnDuration = 1; // seconds color is ON
    private double colorOffDuration = 0.5; // seconds OFF between colors
    private double sequencePauseDuration = 1.0; // seconds pause between sequences
    // Total sequence: (0.75 + 0.083) * 3 + 1.0 pause = ~6 seconds per cycle

    // LED Color values (R, G, B) - 0 to 255
    private static final int[] GREEN = {0, 255, 0};
    private static final int[] PURPLE = {128, 0, 128};
    private static final int[] OFF = {0, 0, 0};

    @Override
    public void runOpMode() {
        telemetry.addLine("PS5 LED Motif Tester");
        telemetry.addLine("===================");
        telemetry.addLine("D-Pad Up: Select GPP motif");
        telemetry.addLine("D-Pad Left: Select PGP motif");
        telemetry.addLine("D-Pad Right: Select PPG motif");
        telemetry.addLine("A Button: Start/Stop flashing");
        telemetry.addLine("B Button: Flash once");
        telemetry.addLine();
        telemetry.addData("Status", "Press Play to start");
        telemetry.update();

        waitForStart();

        flashTimer.reset();

        while (opModeIsActive()) {
            // Motif selection
            if (gamepad1.dpad_up && !isFlashing) {
                currentMotif = Motif.GPP;
                setLED(OFF);
                sleep(200); // Debounce
            } else if (gamepad1.dpad_left && !isFlashing) {
                currentMotif = Motif.PGP;
                setLED(OFF);
                sleep(200);
            } else if (gamepad1.dpad_right && !isFlashing) {
                currentMotif = Motif.PPG;
                setLED(OFF);
                sleep(200);
            }

            // Toggle continuous flashing
            if (gamepad1.a) {
                isFlashing = !isFlashing;
                if (!isFlashing) {
                    setLED(OFF);
                    currentColorIndex = 0;
                }
                flashTimer.reset();
                sleep(300); // Debounce
            }

            // Single flash cycle
            if (gamepad1.b && !isFlashing) {
                performSingleFlashCycle();
                sleep(300); // Debounce
            }

            // Continuous flashing logic - cycle through: Color1, OFF, Color2, OFF, Color3, OFF, PAUSE
            if (isFlashing) {
                double duration;
                if (currentColorIndex == 6) {
                    // State 6 is the sequence pause (longer wait before repeating)
                    duration = sequencePauseDuration;
                } else {
                    // States 0,2,4 are colors; states 1,3,5 are OFF separators
                    boolean isOffState = (currentColorIndex % 2 == 1);
                    duration = isOffState ? colorOffDuration : colorOnDuration;
                }

                if (flashTimer.seconds() >= duration) {
                    currentColorIndex = (currentColorIndex + 1) % 7; // 7 states total (3 colors + 3 offs + 1 pause)
                    flashTimer.reset();
                }
                flashCurrentColor();
            }

            // Telemetry
            telemetry.addLine("=== DECODE Motif Tester ===");
            telemetry.addData("Current Motif", getMotifString());
            telemetry.addData("Flashing", isFlashing ? "YES" : "NO");
            if (isFlashing) {
                telemetry.addData("Current State", getCurrentColorName());
                telemetry.addData("Position", "%d/7", currentColorIndex + 1);
            }
            telemetry.addLine();
            telemetry.addLine("--- Controls ---");
            telemetry.addData("Select GPP", "D-Pad Up");
            telemetry.addData("Select PGP", "D-Pad Left");
            telemetry.addData("Select PPG", "D-Pad Right");
            telemetry.addData("Toggle Flash", "A Button");
            telemetry.addData("Single Flash", "B Button");
            telemetry.update();
        }

        // Turn off LED when stopping
        setLED(OFF);
    }

    /**
     * Flash the current color in the sequence (includes OFF states and sequence pause)
     * Sequence: Color1, OFF, Color2, OFF, Color3, OFF, PAUSE
     */
    private void flashCurrentColor() {
        // State 6 is the sequence pause (OFF)
        if (currentColorIndex == 6) {
            setLED(OFF);
            return;
        }

        int[][] colorSequence = getMotifSequence();

        // Odd indices are OFF states, even indices are colors
        if (currentColorIndex % 2 == 1) {
            setLED(OFF);
        } else {
            int colorIndex = currentColorIndex / 2; // 0, 2, 4 -> 0, 1, 2
            setLED(colorSequence[colorIndex]);
        }
    }

    /**
     * Perform a single complete flash cycle through all three colors with OFF states
     * Total duration: ~5 seconds
     */
    private void performSingleFlashCycle() {
        int[][] sequence = getMotifSequence();
        for (int i = 0; i < 3; i++) {
            // Show color
            setLED(sequence[i]);
            sleep((long) (colorOnDuration * 1000));
            // Turn off between colors
            setLED(OFF);
            sleep((long) (colorOffDuration * 1000));
        }
    }

    /**
     * Get the color sequence for the current motif
     */
    private int[][] getMotifSequence() {
        switch (currentMotif) {
            case GPP:
                return new int[][]{GREEN, PURPLE, PURPLE};
            case PGP:
                return new int[][]{PURPLE, GREEN, PURPLE};
            case PPG:
                return new int[][]{PURPLE, PURPLE, GREEN};
            default:
                return new int[][]{OFF, OFF, OFF};
        }
    }

    /**
     * Get the motif as a readable string
     */
    private String getMotifString() {
        switch (currentMotif) {
            case GPP:
                return "G-P-P (Green-Purple-Purple)";
            case PGP:
                return "P-G-P (Purple-Green-Purple)";
            case PPG:
                return "P-P-G (Purple-Purple-Green)";
            default:
                return "Unknown";
        }
    }

    /**
     * Get the current color name being displayed
     */
    private String getCurrentColorName() {
        // State 6 is the sequence pause
        if (currentColorIndex == 6) {
            return "PAUSE (OFF)";
        }

        // Odd indices are OFF states
        if (currentColorIndex % 2 == 1) {
            return "OFF";
        }

        int[][] sequence = getMotifSequence();
        int colorIndex = currentColorIndex / 2; // Convert to color index (0, 1, or 2)
        int[] currentColor = sequence[colorIndex];

        if (currentColor[1] == 255 && currentColor[0] == 0) {
            return "GREEN";
        } else if (currentColor[0] == 128 && currentColor[2] == 128) {
            return "PURPLE";
        }
        return "OFF";
    }

    /**
     * Set the LED color on gamepad1 (PS5 controller)
     */
    private void setLED(int[] rgb) {
        if (gamepad1 instanceof Gamepad) {
            gamepad1.setLedColor(rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }
}
