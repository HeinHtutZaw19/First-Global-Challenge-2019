/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaFG2019;

/**
 * This OpMode illustrates the basics of using Vuforia to determine the position and orientation
 * of the robot relative to the Vuforia target.
 *
 * You will need to plug your webcam into the phone and add it to your configuration with the name "Webcam 1".
 *
 * When the target image is located, Vuforia calculates the position and orientation of the
 * camera relative to the image. This is then converted into the position of the robot relative to
 * the image, using the camera location information that is provided during initialization.
 *
 * @see VuforiaFG2019
 *
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="Concept: Vuforia Nav", group ="Concept")

public class VuforiaOpMode extends LinearOpMode {

    /**
     * The variable used to hold the Vuforia object that will track the target
     */
    private VuforiaFG2019 vuforia;


    /**
     * This is a reference to the webcam that will be used. Webcams are configured the same way as
     * other hardware devices, such as motors and servos. It will appear at the top level, next to
     * the Expansion Hub Portal.
     */
    private WebcamName webcam;

    /**
     * Whether or not Vuforia should attempt to keep track of where the target is
     * when the webcam cannot currently see it
     */
    private final boolean USE_EXTENDED_TRACKING = false;

    /**
     * Whether or not Vuforia should display the camera output on a connected HDMI monitor
     */
    private final boolean ENABLE_CAMERA_DISPLAY = true;

    /**
     * The four targets that can be tracked
     */
    private final String DUBAI_2019_TARGET = VuforiaFG2019.DUBAI_2019_TARGET;
    private final String FIRST_GLOBAL_LOGO_TARGET = VuforiaFG2019.FIRST_GLOBAL_LOGO_TARGET;
    private final String REV_ROBOTICS_LOGO_TARGET = VuforiaFG2019.REV_ROBOTICS_LOGO_TARGET;
    private final String WORLD_MAP_TARGET = VuforiaFG2019.WORLD_MAP_TARGET;

    /**
     * Which target Vuforia should track.
     * Check the game manual for which target will be on the field.
     */
    private final String TARGET_TO_TRACK = WORLD_MAP_TARGET;

    /**
     * We need to tell Vuforia where the camera is located on the Robot Coordinate System.
     * The origin point of the Robot Coordinate System is wherever you say it is on the robot.
     * Once Vuforia sees the target, it is this robot origin point that Vuforia will give the
     * coordinates of, relative to the target.
     *
     * For example, if you want to know the where the front center of your robot is in comparison
     * to the target, you should consider the robot's origin point to be at the center of the front
     * edge of the physical robot.
     *
     * If your camera happens to be located at the robot's origin point, than you can set these
     * values to zero. If your camera is located somewhere else, you will need to specify where,
     * by changing these values. Like all distance values for the FIRST Global Vuforia system, this
     * will be measured in millimeters.
     *
     * In the Robot Coordinate System, the X axis extends horizontally from the robot's origin point
     * to the right. So if your robot's origin point is on the center of the front edge of the robot,
     * and your camera's lens is located 50mm to the left of the robot's center, then your
     * CAMERA_LOCATION_ON_ROBOT_X_MM value should be -50.
     *
     * The Y axis of the Robot Coordinate System extends horizontally straight out in front
     * of the robot. If your camera is located 20 mm behind the origin point, then your
     * CAMERA_LOCATION_ON_ROBOT_Y_MM value should be -20.
     *
     * The Z axis of the Robot Coordinate system extends vertically from the origin point, towards
     * the ceiling. You don't need to worry about the Z axis unless your camera is able to move up
     * and down, in which case, you should define your origin point as being on the same
     * vertically-moving mechanism as the camera. If your camera and origin point cannot move up and
     * down, then the Z value reported by Vuforia will always stay the same, and thus be irrelevant.
     */
    private final float CAMERA_LOCATION_ON_ROBOT_X_MM = 0;
    private final float CAMERA_LOCATION_ON_ROBOT_Y_MM = 0;
    private final float CAMERA_LOCATION_ON_ROBOT_Z_MM = 0;

    /**
     * Next, we need to define the values that will tell Vuforia in which direction the camera is
     * pointing. To do this, you need to understand the Camera Coordinate System. The origin of the
     * Camera Coordinate System lies in the middle of the sensor inside of the camera.
     *
     * The Z axis of the Camera Coordinate System extends from the camera's origin point straight
     * out of the lens of the camera. When looking at the face of the camera (down the positive Z
     * axis), the X axis is positive off to the right in the plane of the sensor, and the Y axis
     * is positive out the top of the lens in the plane of the sensor.
     *
     * By default, the camera is considered to have its coordinate system aligned with the Robot
     * Coordinate System. This means that by default, the camera is facing upward, with its X axis
     * pointing to the right of the robot, and its Y axis pointing at the front of the robot.
     *
     * Vuforia will rotate the camera first along the robot's X axis, then along the robot's Y axis,
     * then along the robot's Z axis. Note that the rotations are done on the ROBOT's axes, not the
     * cameras. This means that after the camera is rotated along the robot's X axis, the X, Y, and Z
     * axes that you are rotating the camera on stay in place, and do not move with the camera.
     *
     * When you are rotating the camera along an axis, positive numbers mean counter-clockwise
     * rotation. There is a trick you can use to help you with this. If you align your thumb with the
     * positive direction of an axis, rotating in a positive direction will rotate the object in the
     * same direction that your fingers are curling.
     *
     * To tie all this together, lets do an example. You probably want your camera pointed in the
     * same direction as the robot. The first thing that Vuforia will do is rotate the camera (from
     * being pointed at the ceiling) along the robot's X axis, which is pointing to the right of the
     * robot. If we stick our thumb to the robot's right, we'll see that our fingers curl towards
     * the back of the robot. Therefore, we want to turn the camera 90 degrees negatively along the
     * X axis.
     *
     * This points the camera towards the front of the robot, but it is now upside down. The Y axis
     * points at the front of the robot, so if we turn the camera 180 degrees along it, it will be
     * right side up. The camera is now pointed at the front of the robot, so for this example, we
     * do not need to rotate the camera along the Z axis.
     */
    private final float CAMERA_ANGLE_X_DEGREES = -90;
    private final float CAMERA_ANGLE_Y_DEGREES = 180;
    private final float CAMERA_ANGLE_Z_DEGREES = 0;


    /**
     * The rest of the program shows how to obtain the robot's coordinates relative to the target.
     * These values are in the Target Coordinate System*. The target coordinate system is oriented
     * the same as the Robot Coordinate System, but with the center of the target as the origin
     * point, instead of the robot. This means that if the robot is 200 millimeters to the right of
     * the target, and 500 millimeters back from the target, then the robot's X position will be
     * 200, and the robot's Y position will be -500.
     *
     * To get which direction the robot is facing (heading), you want to look at the Z angle, since
     * Z is the axis which points straight up. Positive Z angle values means that the robot is
     * rotated counterclockwise from the target. The X and Y angles will give you pitch and yaw,
     * respectively.
     *
     *    *FIRST Global's Target Coordinate System is different from the Target Coordinate System
     *     as defined by FIRST Tech Challenge.
     */

    @Override public void runOpMode() {

        // Create our Vuforia object
        vuforia = new VuforiaFG2019();

        // Retrieve our webcam from the hardwareMap
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Initialize the Vuforia object
        vuforia.initialize(
                webcam,                         // Our webcam that we retrieved from the HardwareMap
                USE_EXTENDED_TRACKING,          // Whether or not Vuforia should attempt to track the target when it cannot see it
                ENABLE_CAMERA_DISPLAY,          // Whether or not Vuforia should display the camera output on a connected HDMI monitor
                CAMERA_LOCATION_ON_ROBOT_X_MM,  // The X location of the camera on the robot's coordinate system, in millimeters
                CAMERA_LOCATION_ON_ROBOT_Y_MM,  // The Y location of the camera on the robot's coordinate system, in millimeters
                CAMERA_LOCATION_ON_ROBOT_Z_MM,  // The Z location of the camera on the robot's coordinate system, in millimeters
                CAMERA_ANGLE_X_DEGREES,         // The amount to rotate the camera along the robot's X axis
                CAMERA_ANGLE_Y_DEGREES,         // The amount to rotate the camera along the robot's Y axis
                CAMERA_ANGLE_Z_DEGREES);        // The amount to rotate the camera along the robot's Z axis

        // Activate Vuforia
        vuforia.activate();

        // Wait for the start button to be pressed
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        // Create a variable that lets us keep track of whether the button we use to capture Vuforia
        // images was previously pressed or not
        boolean captureButtonPressed = false;

        // Create a variable that holds the tracking data Vuforia gives us
        VuforiaBase.TrackingResults vuforiaResults;
        while (opModeIsActive()) {

            // Update the vuforia results for the center target
            vuforiaResults = vuforia.track(TARGET_TO_TRACK);

            // If button a on the gamepad is pressed, and was not pressed as of the last loop, save
            // the current Vuforia frame to the VuforiaImages folder, which you can access by
            // plugging the Control Hub into a computer.
            if (gamepad1.a && !captureButtonPressed) {
                vuforia.saveFrameToStorage();
            }

            // Update the state of the capture button
            captureButtonPressed = gamepad1.a;

            // Print whether or not we currently see the target to telemetry
            telemetry.addData("Target visible", vuforiaResults.isVisible);

            // Print the current location of our robot relative to the tracked target to telemetry
            telemetry.addData("Robot position", "X = " + vuforiaResults.x + " mm, Y = " + vuforiaResults.y + " mm");

            // Print the current orientation of our robot relative to the tracked target to telemetry
            telemetry.addData("Robot heading", vuforiaResults.zAngle + " degrees");

            // Send our telemetry to the robot
            telemetry.update();
        }
    }
}

