/*
1. Run program under CALIBRATE_SENSOR_MODE, detect the reading of the light sensors on white surface, and then on
   black tape, and then update WHITE and BLACK values respectively.
2. Check how well spin_turn is working and whether the robot is over turning or under turning. Run the program under TEST_MODE,
   and add a small code to spin turxn either 90 left or 90 right. Run this program 4 times consecutively to see if the robot goes
   back to the starting position. If the robot does not go back to the original position, go into the spin_turn function and
   modify the sleep value accordingly.
3. Calibrate the distance sensor values: 
4. Check servo and sensor wires
*/

#include <kipr/wombat.h>
#include <stdbool.h>
#include <stdlib.h>
#define FALSE 0
#define TRUE 1
#define LEFT 0
#define RIGHT 1

// Sensor 1 of 4: left front.
#define LEFT_FRONT_BLACK 2159
#define LEFT_FRONT_WHITE 2184
// Sensor 2 of 4: right front.
#define RIGHT_FRONT_BLACK 2168
#define RIGHT_FRONT_WHITE 2232
// Sensor 3 of 4: left middle.
#define LEFT_MIDDLE_BLACK 2167
#define LEFT_MIDDLE_WHITE 2224
// Sensor 4 of 4: right middle.
#define RIGHT_MIDDLE_BLACK 2146
#define RIGHT_MIDDLE_WHITE 2194
#define LEFT_FRONT_GRAY (LEFT_FRONT_BLACK + LEFT_FRONT_WHITE) / 2
#define RIGHT_FRONT_GRAY (RIGHT_FRONT_BLACK + RIGHT_FRONT_WHITE) / 2
#define LEFT_MIDDLE_GRAY (LEFT_MIDDLE_BLACK + LEFT_MIDDLE_WHITE) / 2
#define RIGHT_MIDDLE_GRAY (RIGHT_MIDDLE_BLACK + RIGHT_MIDDLE_WHITE) / 2
#define LEFT_FRONT_CLIFF_SENSOR 1
#define RIGHT_FRONT_CLIFF_SENSOR 2
#define LEFT_MIDDLE_CLIFF_SENSOR 0
#define RIGHT_MIDDLE_CLIFF_SENSOR 3
#define FRONT_CLIFF_SENSOR_GROUP 0
#define MIDDLE_CLIFF_SENSOR_GROUP 1

#define CLAW 3
#define ARM_LEFT 0
#define ARM_RIGHT 2

#define FORWARD 1
#define BACKWARD 0
#define MAX_VELOCITY 0.15

#define CLOSED_CLAW 950
#define OPEN_CLAW_WIDE 1780
#define OPEN_CLAW_NARROW 1400
#define OPEN_CLAW_NARROW_MODE true
#define OPEN_CLAW_WIDE_MODE false

#define PI 3.1415962
#define ARM_LENGTH_CM 29
#define ARM_MAX_UP_POS 0
#define ARM_CENTER_POS 1024
#define ARM_MAX_DOWN_POS 2048

#define LEFT_CENTER_BUMP_SENSOR 1
#define CENTER_BUMP_SENSOR 2
#define RIGHT_CENTER_BUMP_SENSOR 3

#define CENTER_IR_SENSOR 3

#define START_LIGHT_SENSOR 2

#define RUN_MODE 0
#define CALIBRATE_SENSOR_MODE 1 
#define CALIBRATE_ARM_MODE 2
#define CALIBRATE_CLAW_MODE 3
#define CALIBRATE_LEFT_TURN_MODE 4
#define CALIBRATE_RIGHT_TURN_MODE 5
#define TEST_MODE 6
#define COMPETE_MODE 7
#define PROGRAM_MODE TEST_MODE

void maybe_sleep() {
    msleep(500);
}

bool is_left_on_black(int sensor_group) {
    if (sensor_group == FRONT_CLIFF_SENSOR_GROUP) {
    	return create3_sensor_cliff(LEFT_FRONT_CLIFF_SENSOR) < LEFT_FRONT_GRAY;
    } else if (sensor_group == MIDDLE_CLIFF_SENSOR_GROUP) {
        return create3_sensor_cliff(LEFT_MIDDLE_CLIFF_SENSOR) < LEFT_MIDDLE_GRAY;
    } else {
        printf("Invalid sensor_group param %d. Exiting program.\n", sensor_group);
        abort();
    }
}

bool is_right_on_black(int sensor_group) {
    if (sensor_group == FRONT_CLIFF_SENSOR_GROUP) {
    	return create3_sensor_cliff(RIGHT_FRONT_CLIFF_SENSOR) < RIGHT_FRONT_GRAY;
    } else if (sensor_group == MIDDLE_CLIFF_SENSOR_GROUP) {
        return create3_sensor_cliff(RIGHT_MIDDLE_CLIFF_SENSOR) < RIGHT_MIDDLE_GRAY;
    } else {
        printf("Invalid sensor_group param %d. Exiting program.\n", sensor_group);
        abort();
    }
}

void drive_up_until_black(int sensor_group, double speed_multiplier) {
    double velocity = speed_multiplier * MAX_VELOCITY;
    double correction_angle = 20;
    while (!is_left_on_black(sensor_group) || !is_right_on_black(sensor_group)) {
        if (is_right_on_black(sensor_group)) {
            // Sensing black on right, turn right.
            printf("before driving back\n");
            create3_drive_straight (-0.1, 0.1);
            printf("before wait\n");
            create3_wait();
            printf("after driving back\n");
            msleep(50);
            printf("rotate right by %f\n", correction_angle);
            create3_rotate_degrees(-1 * correction_angle, 10);
            velocity = 0.1 * speed_multiplier * MAX_VELOCITY;
            correction_angle *= 0.9;
        } else if (is_left_on_black(sensor_group)) {
            // Sensing black on left, turn left.
            printf("before driving back\n");
            create3_drive_straight (-0.1, 0.1);
            printf("before wait\n");
            create3_wait();
            printf("after driving back\n");
            msleep(50);
            printf("rotate left by %f\n", correction_angle);
            create3_rotate_degrees(correction_angle, 10);
            velocity = 0.1 * speed_multiplier * MAX_VELOCITY;
            correction_angle *= 0.9;
        } else {
            // Both sensors on white, drive straight.
            create3_velocity_set_components(velocity, 0);
        }
        create3_wait();
        msleep(10);
    }
    create3_velocity_set_components(0, 0);
    create3_wait();
}

void line_follow(int duration) {
    double left_arc = 0.5;
    int time_elapsed = 0;
    while (time_elapsed <= duration) {
        if (!is_left_on_black(FRONT_CLIFF_SENSOR_GROUP) &&  is_right_on_black(FRONT_CLIFF_SENSOR_GROUP)) {
            // Sensing white on both left and right, drive straight.
            create3_velocity_set_components(MAX_VELOCITY, 0);
        } else if (is_right_on_black(FRONT_CLIFF_SENSOR_GROUP)) {
            // Sensing black on right, turn right.
            create3_velocity_set_components(MAX_VELOCITY, -1 * left_arc);
        } else if (is_left_on_black(FRONT_CLIFF_SENSOR_GROUP)) {
            // Sensing black on left, turn left.
            create3_velocity_set_components(MAX_VELOCITY, left_arc);
        } else {
            // Both sensors on black, stop.
        	create3_velocity_set_components(0, 0);
        }
        create3_wait();
        msleep(10);
        time_elapsed += 10;
    }
    create3_velocity_set_components(0, 0);
    create3_wait();
    maybe_sleep();
}

bool is_center_bump_detected() {
    return create3_sensor_bump(LEFT_CENTER_BUMP_SENSOR) || create3_sensor_bump(CENTER_BUMP_SENSOR) || create3_sensor_bump(RIGHT_CENTER_BUMP_SENSOR);
}

void drive_until_front_bump() {
    // Complete the first leg of the trip with relatively faster speed, until we're close enough to the end.
    while (create3_sensor_ir(CENTER_IR_SENSOR) < 500) {
        create3_velocity_set_components(0.5 * MAX_VELOCITY, 0);
        create3_wait();
        msleep(10);
    }
    create3_velocity_set_components(0, 0);
    create3_wait();
    msleep(200);

    // Reduce speed further for the last mile before bump
    while (!is_center_bump_detected()) {
        create3_velocity_set_components(0.2 * MAX_VELOCITY, 0);
        create3_wait();
        msleep(50);
    }
    create3_velocity_set_components(0, 0);
    create3_wait();
}

void drive_time(int direction, int duration, double velocity) { 
    int time_elapsed = 0;
    if (direction == BACKWARD){
        velocity *= -1;
    }
    while (time_elapsed <= duration) {  
    	create3_velocity_set_components(velocity, 0);
        create3_wait();
        msleep(100);
        time_elapsed += 100;
    }
    create3_velocity_set_components(0, 0);
    create3_wait();
}

void drive_time_max_speed(int direction, int duration) {
    drive_time(direction, duration, MAX_VELOCITY);
}

//might need to fine tune
void open_claw() {
    set_servo_position(CLAW, OPEN_CLAW_WIDE);
    msleep(100);
}

void close_claw() {
    set_servo_position(CLAW, CLOSED_CLAW);
    msleep(100);
}

void open_claw_slow(bool should_open_narrow) {
   // printf("claw:%d\n",get_servo_position(CLAW));
    int total_iterations = 0;
    int claw_open_limit = OPEN_CLAW_WIDE;
    if (should_open_narrow) {
        claw_open_limit = OPEN_CLAW_NARROW;
    }
    while(get_servo_position(CLAW) < claw_open_limit && total_iterations < 450) {
        set_servo_position(CLAW, get_servo_position(CLAW) + 5);
        //printf("claw:%d\n", get_servo_position(CLAW));
        total_iterations++;
        msleep(3);
    }
    msleep(400);
}

void close_claw_slow() {
   // printf("claw:%d\n",get_servo_position(CLAW));
    int total_iterations = 0;
    while(get_servo_position(CLAW) > CLOSED_CLAW && total_iterations < 450) {
        set_servo_position(CLAW, get_servo_position(CLAW) - 5);
        //printf("claw:%d\n", get_servo_position(CLAW));
        total_iterations++;
        msleep(3);
    }
    msleep(400);
}

void set_arm_with_custom_msleep(int end_angle, int sleep_msec) {
    int increment = 2;
    int total_iterations = 0;
    // printf("arm left = %d, arm right = %d\n", get_servo_position(ARM_LEFT), get_servo_position(ARM_RIGHT));
    if(get_servo_position(ARM_LEFT) > end_angle) {
        increment *= -1;
        while(get_servo_position(ARM_LEFT) > end_angle && total_iterations < 2000) {
            set_servo_position(ARM_LEFT, get_servo_position(ARM_LEFT) + increment);
            set_servo_position(ARM_RIGHT, get_servo_position(ARM_RIGHT) - increment);
            msleep(1);
            // printf("iter = %d, arm left = %d, arm right = %d\n", total_iterations, get_servo_position(ARM_LEFT), get_servo_position(ARM_RIGHT));
            total_iterations++;
        }
    }
    else{
        while(get_servo_position(ARM_LEFT) < end_angle && total_iterations < 2000) {
            set_servo_position(ARM_LEFT, get_servo_position(ARM_LEFT) + increment);
            set_servo_position(ARM_RIGHT, get_servo_position(ARM_RIGHT) - increment);
            msleep(1);
            // printf("iter = %d, arm left = %d, arm right = %d\n", total_iterations, get_servo_position(ARM_LEFT), get_servo_position(ARM_RIGHT));
            total_iterations++;
        }
    }
    msleep(sleep_msec);
} 

void set_arm(int pos){
  set_arm_with_custom_msleep(pos, 400);
}

// In order to ensure that the arm goes straight up verticall as it lifts, as the arm raises from its lowest position (6 o'clock) to its middle
// position (9 o'clock), we need the iCreate to back off (drive back) horizontally proportional to the angle of the arm.
double compute_backoff_x_distance(int start_arm_position, int end_arm_position) {
    double start_angle = 90 * (double)(ARM_MAX_DOWN_POS - start_arm_position) / 1024;
    double start_x_distance = ARM_LENGTH_CM * sin(start_angle * PI / 180);
    double end_angle = 90 * (double)(ARM_MAX_DOWN_POS - end_arm_position) / 1024;
    double end_x_distance = ARM_LENGTH_CM * sin(end_angle * PI / 180);
//    printf("start_angle = %f, start_x_distance = %f\n", start_angle, start_x_distance);
//    printf("end_angle = %f, end_x_distance = %f\n", end_angle, end_x_distance);
//    printf("x_distance_delta = %f\n", end_x_distance - start_x_distance);
    return end_x_distance - start_x_distance;
}

void set_arm_vertically(int end_angle) {
	int increment = 2;
    int total_iterations = 0;
    double backoff_x_distance;
    double x_distance_to_speed = 1.38;
    double icreate_speed;
    // printf("arm left = %d, arm right = %d\n", get_servo_position(ARM_LEFT), get_servo_position(ARM_RIGHT));
    if (get_servo_position(ARM_LEFT) > end_angle) {
        // Need to raise the arm.
        increment *= -1;
        while (get_servo_position(ARM_LEFT) > end_angle && total_iterations < 2000) {
            set_servo_position(ARM_LEFT, get_servo_position(ARM_LEFT) + increment);
            set_servo_position(ARM_RIGHT, get_servo_position(ARM_RIGHT) - increment);
            backoff_x_distance = compute_backoff_x_distance(get_servo_position(ARM_RIGHT), get_servo_position(ARM_RIGHT) - increment);
            icreate_speed = -1 * x_distance_to_speed * backoff_x_distance;
            create3_velocity_set_components(icreate_speed, 0);
        	create3_wait();
            msleep(1);
            // printf("iter = %d, arm left = %d, arm right = %d, backoff_x_distance = %f, backoff speed = %f\n", total_iterations, get_servo_position(ARM_LEFT), get_servo_position(ARM_RIGHT), backoff_x_distance, icreate_speed);
            total_iterations++;
        }
    } else {
        // Need to lower the arm.
        while (get_servo_position(ARM_LEFT) < end_angle && total_iterations < 2000) {
            set_servo_position(ARM_LEFT, get_servo_position(ARM_LEFT) + increment);
            set_servo_position(ARM_RIGHT, get_servo_position(ARM_RIGHT) - increment);
            backoff_x_distance = compute_backoff_x_distance(get_servo_position(ARM_RIGHT), get_servo_position(ARM_RIGHT) - increment);
            icreate_speed = -1 * x_distance_to_speed * backoff_x_distance;
            create3_velocity_set_components(icreate_speed, 0);
        	create3_wait();
            msleep(1);
            // printf("iter = %d, arm left = %d, arm right = %d, backoff_x_distance = %f, backoff speed = %f\n", total_iterations, get_servo_position(ARM_LEFT), get_servo_position(ARM_RIGHT), backoff_x_distance, icreate_speed);
            total_iterations++;
        }
    }
    create3_velocity_set_components(0, 0);
    create3_wait();
    msleep(100);
}

void check_arm_alignment() {
    create3_connect();
    enable_servos();
    //Servo 1:1508, servo 2:497
    printf("arm left = %d, arm right = %d\n", get_servo_position(ARM_LEFT), get_servo_position(ARM_RIGHT));
    set_arm(1300);
    msleep(1000);
    set_arm(1508);
    printf("arm left = %d, arm right = %d\n", get_servo_position(ARM_LEFT), get_servo_position(ARM_RIGHT));
    disable_servos();
    create3_wait();
} 

void check_claw() {
    create3_connect();
    enable_servos();
    open_claw();
    close_claw();
    disable_servos();
    create3_wait();
}

void calibrate_sensor() {
    create3_connect();
    while (1) {
	    printf("left front = %d, right front = %d, left middle = %d, right middle = %d\n",
               create3_sensor_cliff(LEFT_FRONT_CLIFF_SENSOR), 
               create3_sensor_cliff(RIGHT_FRONT_CLIFF_SENSOR),
               create3_sensor_cliff(LEFT_MIDDLE_CLIFF_SENSOR), 
               create3_sensor_cliff(RIGHT_MIDDLE_CLIFF_SENSOR));
        msleep(1000);
    }
    create3_wait();
    create_disconnect();
}

void run() {
    create3_connect();
    enable_servos();

    // Initialize the iCreate.
    open_claw();
    set_arm(1640);   
    maybe_sleep();
    
    // Navigate to the set of 4 pool noodles resting on the table surface.
    drive_up_until_black(MIDDLE_CLIFF_SENSOR_GROUP, 1.0);
    maybe_sleep();
    create3_drive_straight (0.3, 0.3);
    create3_wait();
    maybe_sleep();
    create3_rotate_degrees(90, 90);
    create3_wait();
    maybe_sleep();
    drive_up_until_black(MIDDLE_CLIFF_SENSOR_GROUP, 1.0);
	maybe_sleep();
    create3_drive_straight (0.34, 0.3);
    create3_wait();
    maybe_sleep();
    create3_rotate_degrees(90, 90);
    create3_wait();
    
    // Pick up noodle #1
    create3_drive_straight (0.07, 0.3);
    create3_wait();
    maybe_sleep();
    close_claw_slow();
    maybe_sleep();
    // Drive backward a little bit before raising the arm, or else the arm will knock into pool noodle #3.
    create3_drive_straight (-0.03, 0.3);
    create3_wait();
    maybe_sleep();
    set_arm(1390);
    maybe_sleep();
    
    // Pick up noodle #2
    create3_drive_straight (0.07, 0.3);
    create3_wait();
    maybe_sleep();
    open_claw_slow(OPEN_CLAW_NARROW_MODE);
    maybe_sleep();
    create3_drive_straight (-0.03, 0.3);
    create3_wait();
    maybe_sleep();
    set_arm_vertically(1640);
    maybe_sleep();
    create3_drive_straight (0.03, 0.3);
    create3_wait();
    //drive_time_max_speed(FORWARD, 80);
    maybe_sleep();
    close_claw_slow();
    maybe_sleep();
    // Drive backward a little bit before raising the arm, or else the arm will knock into pool noodle #2.
    create3_drive_straight (-0.03, 0.3);
    create3_wait();
    maybe_sleep();
    set_arm(300);
    maybe_sleep();
    
    /*close_claw_slow(); // AI: Remove this temporary line.
    maybe_sleep();
    set_arm(400);
    maybe_sleep();*/
  
    // Deliver the first 2 noodled onto rack #3.
    drive_up_until_black(MIDDLE_CLIFF_SENSOR_GROUP, 1.0);
    maybe_sleep();
    create3_drive_straight (0.058, 0.3);
    create3_wait();
    maybe_sleep();
    create3_rotate_degrees(-90, 90);
    create3_wait();
    maybe_sleep();
    drive_until_front_bump();
    maybe_sleep();
    create3_drive_straight (-0.06, 0.3);
    create3_wait();
    maybe_sleep();
    set_arm_vertically(800);
    maybe_sleep();
    open_claw();
    maybe_sleep();
    
    disable_servos();
}

void teststuff() {
    create3_connect(); 
    enable_servos();
    
    set_arm(400);
    maybe_sleep();
    drive_up_until_black(MIDDLE_CLIFF_SENSOR_GROUP, 1.0);
	
    disable_servos();
}

void calibrate_turn(int turn_direction) {
    create_connect();
    int i;
    for(i = 0; i < 4; i++){
        //spin_turn(turn_direction, 90);
        msleep(1000);
    }
    create_disconnect();
}

int main() {
    switch(PROGRAM_MODE) {
        case RUN_MODE:
        case COMPETE_MODE:
            run();
            break;
        case CALIBRATE_SENSOR_MODE:
            calibrate_sensor();
            break;
        case CALIBRATE_ARM_MODE:
            check_arm_alignment();
            break;
        case CALIBRATE_CLAW_MODE:
            check_claw();
            break;
        case CALIBRATE_LEFT_TURN_MODE:
            calibrate_turn(LEFT);
            break;
        case CALIBRATE_RIGHT_TURN_MODE:
            calibrate_turn(RIGHT);
            break;
        case TEST_MODE:
            teststuff();
            break;
        default:
            printf("Invalid program mode:%d\n", PROGRAM_MODE);
            break;
    }
    return 0;
}

