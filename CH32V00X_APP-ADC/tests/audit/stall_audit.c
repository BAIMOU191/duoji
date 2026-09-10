/* Stall protection end to end: real A_Protect + real A_Servo + the physical
 * model. Nothing is faked except the load — a jammed axis is reproduced by
 * raising friction above the full-scale PWM the H-bridge can deliver. The
 * scale has to beat Coulomb friction (57 counts), not just static friction,
 * or an already-moving axis pushes straight through the "jam". */
#define JAM_FRICTION 60
#define SIM_REAL_PROTECT
#define main sim_previous_main
#include "../sim/servo_sim.c"
#undef main
#include "A_Protect.c"

static int prot_div;
/* One millisecond of wall clock: the 1ms control tick, and the 10ms protection
 * task on every tenth of them, exactly like the cooperative scheduler does. */
static void run_ms(int ms) {
    while (ms--) { step(); if (++prot_div >= 10) { prot_div = 0; A_Protect_Task(); } }
}
static void begin(double pos) {
    setup(pos, 0); prot_div = 0; load_pwm = 0; friction_scale = 1;
    g_config.torque_limit = 100; A_Protect_Init();
}

/* A jam must be caught, and caught by holding position — not by dropping torque. */
static void check_stall_trips(void) {
    begin(13500); friction_scale = JAM_FRICTION;
    A_Servo_Submit(2000, 0);                       /* target 20250 cdeg */
    int ms = 0; while (ms < 1000 && !A_Protect_Stalled()) { run_ms(1); ms++; }
    CHECK(A_Protect_Stalled());
    CHECK(ms >= PROT_STALL_TRIP * PROT_STALL_PERIOD_MS);   /* no early trip */
    CHECK(ms <= 2 * PROT_STALL_TRIP * PROT_STALL_PERIOD_MS + 40);
    CHECK(A_Servo_TorqueOn());                     /* stall never releases */
    CHECK(A_Protect_Fault() == PROT_FAULT_NONE);   /* and never latches a fault */
    run_ms(200);
    CHECK(s_servo.target_angle == 13500);          /* target moved to where it is */
    CHECK(C_Traj_Is_Done());
    CHECK(applied == 0);                           /* stopped pushing */
}

/* A normal fast move must not look like a stall, and must still arrive. */
static void check_free_move(void) {
    begin(13500);
    A_Servo_Submit(2000, 0); run_ms(2000);
    CHECK(!A_Protect_Stalled());
    CHECK(fabs(plant_pos - 20250) < 100);
}

/* A slow commanded move creeps below the reference threshold every window;
 * that alone must not be read as "the trajectory is not moving". */
static void check_slow_move(void) {
    begin(13500);
    A_Servo_Submit(2000, 3000); run_ms(4000);
    CHECK(!A_Protect_Stalled());
    CHECK(fabs(plant_pos - 20250) < 100);
}

/* Clearing the jam and re-commanding must work: the servo takes new orders
 * (nothing latched) and the diagnostic flag clears once it really turns. */
static void check_recovers(void) {
    begin(13500); friction_scale = JAM_FRICTION;
    A_Servo_Submit(2000, 0); run_ms(400);
    CHECK(A_Protect_Stalled());
    friction_scale = 1;
    A_Servo_Submit(2000, 0); run_ms(2000);
    CHECK(!A_Protect_Stalled());
    CHECK(fabs(plant_pos - 20250) < 100);
}

/* Holding against a load at the target is not a stall: the trajectory is done,
 * so the servo must keep fighting instead of giving up its target. */
static void check_hold_against_load(void) {
    begin(13500);
    A_Servo_Submit(2000, 0); run_ms(2000);
    load_pwm = 600; run_ms(1000);
    CHECK(!A_Protect_Stalled());
    CHECK(s_servo.target_angle == 20250);
}

/* Jamming mid-move is the realistic case: the axis hits something while it is
 * already running. It must trip just the same, from wherever it stopped. */
static void check_jam_midmove(void) {
    begin(13500);
    A_Servo_Submit(2000, 0); run_ms(60);
    CHECK(plant_pos > 13600);                      /* it really was moving */
    friction_scale = JAM_FRICTION;
    int ms = 0; while (ms < 1000 && !A_Protect_Stalled()) { run_ms(1); ms++; }
    CHECK(A_Protect_Stalled());
    CHECK(A_Servo_TorqueOn());
    CHECK(plant_pos < 20000);                      /* nowhere near the target */
    double stuck = plant_pos;                      /* wherever it came to rest */
    run_ms(200);
    CHECK(fabs((double)s_servo.target_angle - stuck) < 100);
    CHECK(C_Traj_Is_Done());
}

/* A released servo has no trajectory to stall on. */
static void check_released(void) {
    begin(13500); friction_scale = JAM_FRICTION;
    A_Servo_Submit(2000, 0); run_ms(30);
    A_Servo_Release(0); run_ms(1000);
    CHECK(!A_Protect_Stalled());
    CHECK(!A_Servo_TorqueOn());
}

int main(int argc, char **argv) {
    noise = 0;
    if (argc < 2) return 2;
    if      (!strcmp(argv[1], "stall"))     check_stall_trips();
    else if (!strcmp(argv[1], "free"))      check_free_move();
    else if (!strcmp(argv[1], "slow"))      check_slow_move();
    else if (!strcmp(argv[1], "recover"))   check_recovers();
    else if (!strcmp(argv[1], "hold_load")) check_hold_against_load();
    else if (!strcmp(argv[1], "jam"))       check_jam_midmove();
    else if (!strcmp(argv[1], "released"))  check_released();
    else return 2;
    printf("%s failures=%d\n", argv[1], failures); return failures ? 1 : 0;
}
