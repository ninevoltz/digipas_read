/*
 * digipas_closed_loop.c
 *
 * Closed-loop level controller for Digi-Pas level sensor + Pololu Tic T500
 * X-Y goniometer stage.  Takes target X,Y angle offsets from command line,
 * reads the level sensor, and drives the motors to converge on the target.
 *
 * Safety: targets are limited to the +/-15 degree stage travel and motor
 * positions are clamped to the corresponding calibrated count range.
 *
 * Usage:
 *   ./digipas_closed_loop <target_x_angle> <target_y_angle>
 *   e.g. ./digipas_closed_loop 0.0 0.0    -> level to horizontal
 *
 * Press ESC during operation to abort.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>

#include <libusb-1.0/libusb.h>
#include "tic.h"
#include "tic_protocol.h"

/* --- Digi-Pas level sensor --- */
#define LEVEL_VENDOR_ID 0x24cc
#define LEVEL_PRODUCT_ID 0x003f
#define HID_REPORT_ID 55
#define HID_IN_EP  0x81
#define HID_OUT_EP 0x01

/* --- Pololu Tic T500 --- */
#define X_SERIAL "00308610"
#define Y_SERIAL "00308627"

/* --- Steps per degree (from VB MotorController calibration) --- */
#define X_STEPS_PER_DEGREE 1118.0
#define Y_STEPS_PER_DEGREE 1553.0

/* --- Safety limits --- */
#define STAGE_LIMIT_DEG 15.0
#define X_MAX_COUNTS  ((int32_t)(STAGE_LIMIT_DEG * X_STEPS_PER_DEGREE + 0.5))
#define X_MIN_COUNTS  (-X_MAX_COUNTS)
#define Y_MAX_COUNTS  ((int32_t)(STAGE_LIMIT_DEG * Y_STEPS_PER_DEGREE + 0.5))
#define Y_MIN_COUNTS  (-Y_MAX_COUNTS)

/* --- Closed-loop tuning --- */
#define MAX_STEP   10000      /* max incremental move per iteration */
#define TOL        0.05      /* convergence tolerance in degrees */
#define MAX_ITER   300       /* safety limit */
#define SETTLE_MS  200       /* time to wait for motor to settle */
#define MOVE_TIMEOUT_MS 10000
#define X_PI_KP    1.20      /* X proportional gain, in count/count */
#define X_PI_KI    0.35      /* X integral gain, in count/count-iteration */
#define Y_PI_KP    1.10      /* Y proportional gain, in count/count */
#define Y_PI_KI    0.20      /* Y integral gain, in count/count-iteration */
#define PI_I_ZONE_DEG 2.0    /* only integrate when close enough to target */
#define PI_I_LIMIT_DEG 1.5   /* anti-windup clamp for accumulated error */

/* === Terminal ESC detection === */
static struct termios g_saved_tios;
static int g_tios_saved = 0;

static void raw_terminal(void) {
    struct termios raw;
    if (tcgetattr(STDIN_FILENO, &g_saved_tios) == 0) {
        g_tios_saved = 1;
        raw = g_saved_tios;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
}

static void restore_terminal(void) {
    if (g_tios_saved)
        tcsetattr(STDIN_FILENO, TCSANOW, &g_saved_tios);
}

static int esc_pressed(void) {
    fd_set fds;
    struct timeval tv = {0, 0};
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0
        && FD_ISSET(STDIN_FILENO, &fds)) {
        unsigned char ch;
        if (read(STDIN_FILENO, &ch, 1) == 1 && ch == 0x1b)
            return 1;
    }
    return 0;
}

static int parse_angle(const char *text, const char *axis, double *angle) {
    char *end = NULL;
    errno = 0;
    double value = strtod(text, &end);
    if (errno || end == text || *end != '\0' || !isfinite(value)) {
        fprintf(stderr, "Invalid %s target angle: '%s'\n", axis, text);
        return -1;
    }
    if (value < -STAGE_LIMIT_DEG || value > STAGE_LIMIT_DEG) {
        fprintf(stderr, "%s target %.3f deg is outside stage travel +/-%.1f deg\n",
                axis, value, STAGE_LIMIT_DEG);
        return -1;
    }
    *angle = value;
    return 0;
}

/* === Digi-Pas sensor === */
static int read_level(libusb_device_handle *dev, float *x, float *y) {
    unsigned char out[4] = {HID_REPORT_ID, 0x37, 0xFF, 0xFF};
    int r = libusb_interrupt_transfer(dev, HID_OUT_EP, out, 4, NULL, 0);
    if (r != LIBUSB_SUCCESS) return -1;

    unsigned char buf[64];
    int n;
    r = libusb_interrupt_transfer(dev, HID_IN_EP, buf, sizeof(buf), &n, 1000);
    if (r != LIBUSB_SUCCESS || n < 5) return -1;

    uint16_t a = (buf[2] << 8) | buf[1];
    *x = (a - 9000) / 100.0f;
    a = (buf[4] << 8) | buf[3];
    *y = (a - 9000) / 100.0f;
    return 0;
}

/* === Tic helpers === */
struct tic_state {
    int32_t current_position;
    int32_t target_position;
    int32_t current_velocity;
    uint16_t error_status;
    uint8_t last_driver_error;
    uint8_t operation_state;
};

static void print_tic_err(const char *serial, const char *op, tic_error *e) {
    if (!e) return;
    fprintf(stderr, "  [%s %s] %s\n", serial, op, tic_error_get_message(e));
    tic_error_free(e);
}

static int require_tic_ok(const char *serial, const char *op, tic_error *e) {
    if (!e) return 0;
    print_tic_err(serial, op, e);
    return -1;
}

static int init_tic(const char *serial, tic_handle **out) {
    tic_error *err;
    tic_device **devs;
    size_t cnt;
    tic_device *found = NULL;

    err = tic_list_connected_devices(&devs, &cnt);
    if (err) { print_tic_err(serial, "list", err); return -1; }

    for (size_t i = 0; i < cnt; i++) {
        const char *s = tic_device_get_serial_number(devs[i]);
        if (s && strcmp(s, serial) == 0) { found = devs[i]; break; }
    }

    if (!found) {
        fprintf(stderr, "  Tic %s NOT FOUND\n", serial);
        for (size_t i = 0; i < cnt; i++) {
            tic_device_free(devs[i]);
        }
        tic_list_free(devs);
        return -1;
    }

    tic_handle *h;
    err = tic_handle_open(found, &h);
    if (err) {
        print_tic_err(serial, "open", err);
        for (size_t i = 0; i < cnt; i++) {
            tic_device_free(devs[i]);
        }
        tic_list_free(devs);
        return -1;
    }
    for (size_t i = 0; i < cnt; i++) {
        tic_device_free(devs[i]);
    }
    tic_list_free(devs);

    err = tic_reset_command_timeout(h); print_tic_err(serial, "reset_timeout", err);

    err = tic_reinitialize(h);     print_tic_err(serial, "reinit", err);

    err = tic_clear_driver_error(h); print_tic_err(serial, "clear_err", err);

    err = tic_energize(h);
    if (err) { print_tic_err(serial, "energize", err); tic_handle_close(h); return -1; }

    err = tic_exit_safe_start(h);
    if (err) { print_tic_err(serial, "exit_safe", err); tic_handle_close(h); return -1; }

    err = tic_reset_command_timeout(h); print_tic_err(serial, "reset_timeout", err);

    *out = h;
    return 0;
}

static int get_state(tic_handle *h, struct tic_state *state) {
    tic_error *err;
    tic_variables *v;
    err = tic_get_variables(h, &v, 0);
    if (err) {
        print_tic_err("tic", "vars", err);
        return -1;
    }
    state->current_position = tic_variables_get_current_position(v);
    state->target_position = tic_variables_get_target_position(v);
    state->current_velocity = tic_variables_get_current_velocity(v);
    state->error_status = tic_variables_get_error_status(v);
    state->last_driver_error = tic_variables_get_last_motor_driver_error(v);
    state->operation_state = tic_variables_get_operation_state(v);
    tic_variables_free(v);
    return 0;
}

static int32_t clamp_counts(int32_t value, int32_t min, int32_t max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static int32_t angle_to_counts(double angle, double steps_per_degree,
                               int32_t min, int32_t max) {
    double counts = angle * steps_per_degree;
    if (counts >= 0)
        counts += 0.5;
    else
        counts -= 0.5;
    return clamp_counts((int32_t)counts, min, max);
}

struct pi_axis {
    double integral_counts;
    double previous_error_counts;
};

static int32_t pi_axis_target(struct pi_axis *axis,
                              int32_t measured_counts,
                              int32_t requested_counts,
                              double steps_per_degree,
                              double kp,
                              double ki,
                              int32_t max_delta,
                              int32_t min,
                              int32_t max) {
    double error_counts = requested_counts - measured_counts;
    double i_zone_counts = PI_I_ZONE_DEG * steps_per_degree;
    double i_limit_counts = PI_I_LIMIT_DEG * steps_per_degree;

    if ((error_counts > 0 && axis->previous_error_counts < 0) ||
        (error_counts < 0 && axis->previous_error_counts > 0)) {
        axis->integral_counts = 0;
    }

    if (fabs(error_counts) <= i_zone_counts) {
        axis->integral_counts += error_counts;
        if (axis->integral_counts > i_limit_counts) axis->integral_counts = i_limit_counts;
        if (axis->integral_counts < -i_limit_counts) axis->integral_counts = -i_limit_counts;
    } else {
        axis->integral_counts = 0;
    }

    double command_delta = kp * error_counts + ki * axis->integral_counts;
    if (command_delta > max_delta) command_delta = max_delta;
    if (command_delta < -max_delta) command_delta = -max_delta;

    axis->previous_error_counts = error_counts;
    return clamp_counts(measured_counts + (int32_t)lrint(command_delta), min, max);
}

static void dump_state(tic_handle *h, const char *label) {
    struct tic_state state;
    if (get_state(h, &state) != 0) return;

    printf("    [%s] pos=%d tgt=%d err=%04x last_err=%d state=%d\n",
           label, state.current_position, state.target_position,
           state.error_status, state.last_driver_error, state.operation_state);

    /* Auto-clear if there's a fault */
    if (state.error_status || state.last_driver_error) {
        tic_error *ce = tic_clear_driver_error(h);
        print_tic_err(label, "clear_err", ce);
    }
}

static int wait_for_move(tic_handle *x, tic_handle *y) {
    int waited_ms = 0;
    while (waited_ms <= MOVE_TIMEOUT_MS) {
        if (esc_pressed()) return 1;

        tic_error_free(tic_reset_command_timeout(x));
        tic_error_free(tic_reset_command_timeout(y));

        struct tic_state xs, ys;
        if (get_state(x, &xs) != 0 || get_state(y, &ys) != 0) return -1;

        if ((xs.error_status || ys.error_status) &&
            (xs.operation_state != TIC_OPERATION_STATE_NORMAL ||
             ys.operation_state != TIC_OPERATION_STATE_NORMAL)) {
            fprintf(stderr, "Tic error while moving: X err=%04x state=%d, Y err=%04x state=%d\n",
                    xs.error_status, xs.operation_state, ys.error_status, ys.operation_state);
            return -1;
        }

        if (xs.current_position == xs.target_position &&
            ys.current_position == ys.target_position &&
            xs.current_velocity == 0 &&
            ys.current_velocity == 0) {
            return 0;
        }

        usleep(100 * 1000);
        waited_ms += 100;
    }

    fprintf(stderr, "Timed out waiting for motor move to settle\n");
    return -1;
}

static int settle_delay(tic_handle *x, tic_handle *y, int delay_ms) {
    int waited_ms = 0;
    while (waited_ms < delay_ms) {
        if (esc_pressed()) return 1;
        tic_error_free(tic_reset_command_timeout(x));
        tic_error_free(tic_reset_command_timeout(y));
        usleep(100 * 1000);
        waited_ms += 100;
    }
    return 0;
}

/* === Main === */
int main(int argc, char *argv[]) {
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <target_x_deg> <target_y_deg>\n\n", argv[0]);
        fprintf(stderr, "Closed-loop level control for Digi-Pas + Tic T500 X-Y stage.\n");
        fprintf(stderr, "Press ESC to abort.\n");
        return 1;
    }

    double tx, ty;
    if (parse_angle(argv[1], "X", &tx) != 0 ||
        parse_angle(argv[2], "Y", &ty) != 0) {
        return 1;
    }

    int32_t x_request_counts = angle_to_counts(tx, X_STEPS_PER_DEGREE,
                                               X_MIN_COUNTS, X_MAX_COUNTS);
    int32_t y_request_counts = angle_to_counts(ty, Y_STEPS_PER_DEGREE,
                                               Y_MIN_COUNTS, Y_MAX_COUNTS);

    printf("=== Digipas Closed-Loop Level Controller ===\n");
    printf("Target: X=%.3f  Y=%.3f\n\n", tx, ty);

    /* -- Digi-Pas -- */
    libusb_context *ctx = NULL;
    libusb_device_handle *lv = NULL;
    libusb_device *lvl = NULL;
    tic_handle *xtic = NULL, *ytic = NULL;
    int exit_code = 1;
    if (libusb_init(&ctx) < 0) { fprintf(stderr, "libusb_init failed\n"); return 1; }

    libusb_device **list;
    ssize_t nc = libusb_get_device_list(ctx, &list);
    if (nc < 0) { fprintf(stderr, "get_device_list failed\n"); goto cleanup; }

    for (ssize_t i = 0; i < nc; i++) {
        struct libusb_device_descriptor d;
        if (libusb_get_device_descriptor(list[i], &d) == 0
            && d.idVendor == LEVEL_VENDOR_ID
            && d.idProduct == LEVEL_PRODUCT_ID) {
            lvl = libusb_ref_device(list[i]); break;
        }
    }
    libusb_free_device_list(list, 1);

    if (!lvl) { fprintf(stderr, "Digi-Pas not found\n"); goto cleanup; }

    int r;
    r = libusb_open(lvl, &lv);
    if (r < 0) { fprintf(stderr, "open failed: %s\n", libusb_error_name(r)); goto cleanup; }
    libusb_unref_device(lvl);
    lvl = NULL;

    if (libusb_kernel_driver_active(lv, 0)) {
        r = libusb_detach_kernel_driver(lv, 0);
        if (r != LIBUSB_SUCCESS) {
            fprintf(stderr, "detach failed: %s\n", libusb_error_name(r)); goto cleanup;
        }
        printf("Detached kernel driver from Digi-Pas\n");
    }

    r = libusb_claim_interface(lv, 0);
    if (r != LIBUSB_SUCCESS) {
        fprintf(stderr, "claim failed: %s\n", libusb_error_name(r)); goto cleanup;
    }
    printf("Digi-Pas connected\n");

    printf("\nInitializing Tics...\n");
    printf("  X axis (%s)... ", X_SERIAL);
    if (init_tic(X_SERIAL, &xtic)) { fprintf(stderr, "\nX init failed, aborting\n"); goto cleanup; }
    printf("OK\n");

    printf("  Y axis (%s)... ", Y_SERIAL);
    if (init_tic(Y_SERIAL, &ytic)) { fprintf(stderr, "\nY init failed, aborting\n"); goto cleanup; }
    printf("OK\n");

    printf("\nStarting closed-loop...\n");
    printf("(press ESC to abort)\n\n");
    raw_terminal();

    /* -- Initial state -- */
    dump_state(xtic, "X");
    dump_state(ytic, "Y");

    /* -- Reset command timeouts once on startup -- */
    tic_error_free(tic_reset_command_timeout(xtic));
    tic_error_free(tic_reset_command_timeout(ytic));

    /* -- Loop -- */
    int iter = 0;
    int done = 0;
    struct pi_axis x_pi = {0};
    struct pi_axis y_pi = {0};

    while (!done && iter < MAX_ITER) {
        iter++;

        if (esc_pressed()) { printf("\nAborted.\n"); break; }

        /* Reset command timeout so the 1.5s settle doesn't cause a timeout error */
        tic_error_free(tic_reset_command_timeout(xtic));
        tic_error_free(tic_reset_command_timeout(ytic));

        /* Read level */
        float cx, cy;
        if (read_level(lv, &cx, &cy) != 0) {
            fprintf(stderr, "\nRead fail at iter %d\n", iter);
            break;
        }

        double ex = tx - cx;
        double ey = ty - cy;
        printf("I%3d | cur: X=%7.3f  Y=%7.3f  |  tgt: X=%7.3f  Y=%7.3f  |  err: X=%7.3f  Y=%7.3f\n",
               iter, cx, cy, tx, ty, ex, ey);

        /* Converged? */
        if (fabs(ex) <= TOL && fabs(ey) <= TOL) {
            printf("\nConverged (within %.2f deg).\n", TOL);
            done = 1;
            break;
        }

        int32_t measured_x = angle_to_counts(cx, X_STEPS_PER_DEGREE,
                                             X_MIN_COUNTS, X_MAX_COUNTS);
        int32_t measured_y = angle_to_counts(cy, Y_STEPS_PER_DEGREE,
                                             Y_MIN_COUNTS, Y_MAX_COUNTS);
        int32_t target_x = pi_axis_target(&x_pi, measured_x, x_request_counts,
                                          X_STEPS_PER_DEGREE, X_PI_KP, X_PI_KI,
                                          MAX_STEP,
                                          X_MIN_COUNTS, X_MAX_COUNTS);
        int32_t target_y = pi_axis_target(&y_pi, measured_y, y_request_counts,
                                          Y_STEPS_PER_DEGREE, Y_PI_KP, Y_PI_KI,
                                          MAX_STEP,
                                          Y_MIN_COUNTS, Y_MAX_COUNTS);

        /* Align controller coordinates with the latest sensor reading. */
        if (require_tic_ok("X", "halt_set_measured", tic_halt_and_set_position(xtic, measured_x)) != 0 ||
            require_tic_ok("Y", "halt_set_measured", tic_halt_and_set_position(ytic, measured_y)) != 0) {
            break;
        }

        /* Issue moves */
        if (require_tic_ok("X", "set_pos", tic_set_target_position(xtic, target_x)) != 0 ||
            require_tic_ok("Y", "set_pos", tic_set_target_position(ytic, target_y)) != 0) {
            break;
        }

        int wait_result = wait_for_move(xtic, ytic);
        if (wait_result != 0) {
            if (wait_result == 1) printf("\nAborted.\n");
            break;
        }
        if (settle_delay(xtic, ytic, SETTLE_MS) != 0) {
            printf("\nAborted.\n");
            break;
        }
    }

    if (iter >= MAX_ITER && !done)
        printf("\nMax iterations reached.\n");
    else if (done)
        exit_code = 0;

cleanup:
    /* Cleanup */
    if (xtic) { tic_error *e = tic_deenergize(xtic); print_tic_err("X", "deenergize", e); tic_handle_close(xtic); }
    if (ytic) { tic_error *e = tic_deenergize(ytic); print_tic_err("Y", "deenergize", e); tic_handle_close(ytic); }
    if (lv)   { libusb_release_interface(lv, 0); libusb_close(lv); }
    if (lvl)  libusb_unref_device(lvl);
    if (ctx)  libusb_exit(ctx);

    restore_terminal();
    printf("Done.\n");
    return exit_code;
}
