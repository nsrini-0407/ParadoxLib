#include "main.h"
#include "liblvgl/lvgl.h"
#include <cstdio>

LV_IMAGE_DECLARE(logo);

// ═════════════════════════════════════════════════════════════════════════════
//  ROBOT CONFIGURATION — the only section you should need to edit.
//
//  Units everywhere: inches, degrees CLOCKWISE from +Y (0 = forward, 90 = right),
//  milliseconds, output in percent (100 = full power).
// ═════════════════════════════════════════════════════════════════════════════

// ── Drive motors ─────────────────────────────────────────────────────────────
// Negative port = reversed. Every motor on a side must drive the wheels the
// same direction on +voltage. chassis.tuneDriveBalance() catches one fighting.
pros::MotorGroup leftMotors ({-11, -5, 13});
pros::MotorGroup rightMotors({15, 16, -17});

// Individual handles, used only by the GUI's motor-health tab.
pros::Motor leftFrontMotor(-11), leftBackMotor(-5),  leftTopMotor(13);
pros::Motor rightFrontMotor(15), rightBackMotor(16), rightTopMotor(-17);

// TODO: port 16 is already the right-back drive motor. Give the intake its real
// port and re-enable it in the watched-motor list in initialize().
// pros::Motor intakeMotor(16);

// ── Odometry sensors ─────────────────────────────────────────────────────────
pros::Rotation leftRotation(6);
pros::Rotation rightRotation(7);
pros::Rotation backRotation(9);

// TrackingWheel(sensor, wheelDiameter, offset, gearRatio, reversed)
//
//   offset = signed inches from the tracking center to the wheel:
//     vertical wheel:    + right of center     - left of center
//     horizontal wheel:  + ahead of center     - behind center
//
//   Do not guess these. From opcontrol, with `pros terminal` open:
//     X  checkWheelDirections   -> fixes `reversed`
//     UP measureWheelDiameter   -> fixes wheelDiameter
//     Y  measureTrackingOffsets -> fixes offset (robot spins itself)
//   and paste what it prints here.
TrackingWheel leftWheel (&leftRotation,  3.25, -5.75,  1.0, true);
TrackingWheel rightWheel(&rightRotation, 3.25, +5.75,  1.0, true);
TrackingWheel backWheel (&backRotation,  3.25, -1.625, 1.0, true);

IMU imu(10);

// ── Controller tuning ────────────────────────────────────────────────────────
// Starting points, not final values. Tune lateral with moveDistance(24), angular
// with turnToHeading(90):
//   kP  raise until it oscillates around the target, then back off ~30%.
//   kD  raise until the oscillation dies. Too much = sluggish / buzzy.
//   kI  leave 0 unless it consistently stops short; then tiny kI + windupRange.
// Exit conditions: small = precision, large = "good enough, don't hang".
ControllerSettings lateralSettings {
    .kP = 8.0,  .kI = 0.0,  .kD = 1.5,
    .windupRange       = 3.0,
    .smallError        = 0.5,   .smallErrorTimeout = 100,
    .largeError        = 2.0,   .largeErrorTimeout = 400,
    .slew              = 300,   // percent per second. 0 disables the ramp
    .dFilter           = 0.6,
};
ControllerSettings angularSettings {
    .kP = 2.5,  .kI = 0.0,  .kD = 0.20,
    .windupRange       = 10.0,
    .smallError        = 1.0,   .smallErrorTimeout = 100,
    .largeError        = 3.0,   .largeErrorTimeout = 400,
    .slew              = 0,
    .dFilter           = 0.6,
};

Chassis chassis(
    DrivetrainConfig{ .leftMotors = &leftMotors, .rightMotors = &rightMotors,
                      .trackWidth = 11.5, .wheelDiameter = 3.25, .gearRatio = 1.0 },
    lateralSettings,
    angularSettings,
    OdomSensors{ .vertical1 = &leftWheel, .vertical2 = &rightWheel,
                 .horizontal1 = &backWheel, .imu = &imu }
);

// ═════════════════════════════════════════════════════════════════════════════

static gui::OdomDebugData get_odom_debug_data() {
	const Pose pose = chassis.getPose();
	return {pose.x, pose.y, pose.theta};
}

void initialize() {
	pros::lcd::initialize();   // brings up LVGL; the custom GUI draws on top of it

	gui::setWatchedMotors({
        {"L Front", &leftFrontMotor}, {"L Back", &leftBackMotor}, {"L Top", &leftTopMotor},
        {"R Front", &rightFrontMotor}, {"R Back", &rightBackMotor}, {"R Top", &rightTopMotor},
        // {"Intake", &intakeMotor},   // once intakeMotor has a real port
    });
	gui::setLogoImage(&logo);
	gui::setOdomDebugProvider(get_odom_debug_data);
	gui::init();

	// imu.setScalar(1.0);   // paste from chassis.measureImuScalar()

	// Calibrates the IMU (with retry), zeroes the wheels, starts the 10ms
	// odometry task. Blocks ~2-3 s. Returns false if the IMU never came up.
	if (!chassis.calibrate()) {
		printf("[robot] IMU FAILED to calibrate - check port 10 / reseat the sensor\n");
	}
	chassis.setPose(0, 0, 0);
}

void disabled() {}
void competition_initialize() {}

// ── Autonomous ───────────────────────────────────────────────────────────────
// Coordinates are field inches; headings are degrees clockwise from +Y.
// Every motion takes a timeout and exits early on its own once settled (or if
// it stalls against something), so timeouts are a ceiling, not a duration.
void autonomous() {
	chassis.setPose(0, 0, 0);

	chassis.moveToPoint(0, 24, 3000);
	chassis.turnToHeading(90, 1500);
	chassis.moveToPose(24, 24, 90, 4000);

	// Chaining example - roll through a waypoint without stopping:
	//   chassis.moveToPoint(0, 24, 2000, {.minSpeed = 40, .earlyExitRange = 6});
	//   chassis.moveToPoint(24, 48, 3000);
	//
	// Async example - fire the intake partway through a move:
	//   chassis.moveToPoint(0, 36, 3000, {.async = true});
	//   chassis.waitUntil(12);
	//   // intake.move(127);
	//   chassis.waitUntilDone();
}

// ── Driver control ───────────────────────────────────────────────────────────
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	chassis.setBrakeMode(pros::MotorBrake::coast);   // driver feel; auton re-sets brake
	uint32_t lastPrint = 0;

	while (true) {
		// Bench-only buttons. autonomous() is only ever called by field control,
		// so with no competition switch plugged in it needs a manual trigger.
		// The measurement routines print to `pros terminal`.
		const bool bench = !pros::competition::is_connected();
		if (bench) {
			if (master.get_digital_new_press(DIGITAL_A))    { chassis.brake(); autonomous(); }
			if (master.get_digital_new_press(DIGITAL_B))    chassis.tuneDriveBalance();
			if (master.get_digital_new_press(DIGITAL_X))    chassis.checkWheelDirections(master);
			if (master.get_digital_new_press(DIGITAL_Y))    chassis.measureTrackingOffsets(master);
			if (master.get_digital_new_press(DIGITAL_UP))   chassis.measureWheelDiameter(master, 48);
			if (master.get_digital_new_press(DIGITAL_DOWN)) chassis.measureImuScalar(master, 5);
		}

		// Arcade. Stick right => turn right (CW-positive convention).
		const double throttle = master.get_analog(ANALOG_LEFT_Y)  * (100.0 / 127.0);
		const double turn     = master.get_analog(ANALOG_RIGHT_X) * (100.0 / 127.0);
		chassis.arcade(throttle, turn);

		if (pros::millis() - lastPrint >= 500) {
			lastPrint = pros::millis();
			const Pose p = chassis.getPose();
			printf("pose  x=%7.2f  y=%7.2f  th=%7.2f   v=%5.1f in/s  w=%6.1f deg/s%s\n",
			       p.x, p.y, p.theta,
			       chassis.getOdom().getLinearVelocity(), chassis.getOdom().getAngularVelocity(),
			       chassis.getOdom().isHeadingFromImu() ? "" : "   [heading from WHEELS - IMU down]");
		}
		pros::delay(20);
	}
}
