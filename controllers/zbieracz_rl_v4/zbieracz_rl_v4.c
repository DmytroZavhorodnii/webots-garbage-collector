/*
 * Zbieracz śmieci – Wykład 10 (sterownik P) + Wykład 11 (agent wnioskujący)
 *
 * Wykład 10: kinematyka różniczkowa robota dwukołowego
 *   ε₁ = ||pos - cel||,  ε₂ = φ* - θ
 *   u₁ = Kp₁·ε₁,  u₂ = Kp₂·ε₂
 *   w_L = u₁ - r·u₂,  w_R = u₁ + r·u₂,  ω = w/R
 *
 * Wykład 11: agent wnioskujący
 *   W(x,y) ∧ O(x,y) → Wykonaj(usuń)
 *   ¬W(x,y) ∧ min_dist(O) → Wykonaj(naprzód)
 *   Warstwa reaktywna: przeszkoda z przodu → Wykonaj(obrót)
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/distance_sensor.h>
#include <webots/display.h>
#include <webots/supervisor.h>
#include <math.h>
#include <stdio.h>

#define TIME_STEP        32

#define WHEEL_RADIUS     0.05
#define HALF_WHEELBASE   0.09
#define MAX_WHEEL_VEL    6.0
#define KP1  2.0
#define KP2  5.0

#define ARRIVE_DIST   0.30
#define PICK_DIST     0.15
#define OBS_FRONT     0.20
#define BACK_UP_STEPS 50

#define BASE_X  -0.8
#define BASE_Y  -0.8

#define NUM_GARBAGE 5
static const char *GARBAGE_DEF[] = {
    "SMIEC_1", "SMIEC_2", "SMIEC_3", "SMIEC_4", "SMIEC_5"
};

/* Mapa wyświetlacza: arena 2×2m → 320×320px */
#define DISP_W  320
#define DISP_H  320
#define W2PX(wx)  ((int)(((wx) + 1.0) * 0.5 * DISP_W))
#define W2PY(wy)  ((int)((1.0 - ((wy) + 1.0) * 0.5) * DISP_H))

typedef enum {
    SELECT, GO_TO_GARBAGE, PICK, BACK_UP, GO_TO_BASE, DEPOSIT, DONE
} AgentState;

static WbDeviceTag lm, rm, gps, disp;
static WbDeviceTag ds_l, ds_c, ds_r;
static WbNodeRef robot_node;
static WbNodeRef garbage_node[NUM_GARBAGE];

static int collected[NUM_GARBAGE];
static int current_garbage;
static AgentState state = SELECT;
static int score = 0;

/* Ostatnie odczyty sensorów (aktualizowane w navigate()) */
static double g_val_l = 1.0, g_val_c = 1.0, g_val_r = 1.0;
static double g_rx = 0.0, g_ry = 0.0, g_theta = 0.0;

static void get_pose(double *x, double *y, double *theta) {
    const double *g = wb_gps_get_values(gps);
    *x = g[0];
    *y = g[1];
    const double *R = wb_supervisor_node_get_orientation(robot_node);
    *theta = atan2(R[3], R[0]);
}

static double wrap(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static void draw_map(void) {
    /* tło: szara arena */
    wb_display_set_color(disp, 0x555555);
    wb_display_fill_rectangle(disp, 0, 0, DISP_W, DISP_H);

    /* baza: zielona */
    wb_display_set_color(disp, 0x00CC00);
    int bx = W2PX(BASE_X), by = W2PY(BASE_Y);
    wb_display_fill_rectangle(disp, bx - 12, by - 12, 24, 24);

    /* przeszkody: brązowe (pozycje hardcoded z .wbt) */
    static const double obs[][2] = {
        {0.6, 0.5}, {-0.5, -0.2}, {0.5, -0.5},
        {0.4, 0.0}, {-0.3, 0.5}, {0.0, -0.5}
    };
    wb_display_set_color(disp, 0x8B4513);
    for (int i = 0; i < 6; i++) {
        int ox = W2PX(obs[i][0]), oy = W2PY(obs[i][1]);
        wb_display_fill_rectangle(disp, ox - 6, oy - 6, 12, 12);
    }

    /* śmieci: pomarańczowe kółka */
    for (int i = 0; i < NUM_GARBAGE; i++) {
        if (collected[i] || !garbage_node[i]) continue;
        const double *p = wb_supervisor_node_get_position(garbage_node[i]);
        if (p[0] > 5.0) continue;
        int gx = W2PX(p[0]), gy = W2PY(p[1]);
        wb_display_set_color(disp, 0xFF6600);
        wb_display_fill_oval(disp, gx, gy, 8, 8);
        wb_display_set_color(disp, 0xFFFF00);
        wb_display_draw_text(disp, GARBAGE_DEF[i] + 6, gx + 10, gy - 5);
    }

    /* ------- promienie sensorów (co widzi robot) ------- */
    /* skala: 1.0m = DISP_W/2 = 160px; Y na ekranie odwrócone */
    int px = W2PX(g_rx), py = W2PY(g_ry);
    /* offsets kątowe sensorów względem kierunku jazdy robota */
    static const double sens_ang[]  = { 0.3, 0.0, -0.3 };
    static const int    col_clear[] = { 0x44FF44, 0x44FF44, 0x44FF44 }; /* zielone gdy brak przeszkody */
    static const int    col_hit[]   = { 0xFFFF00, 0xFF2222, 0xFFFF00 }; /* żółty/czerwony gdy przeszkoda */
    /* val ∈ [0,1000]: 1000mm=1.0m=brak przeszkody, 0=obstacle dotknięty */
    const double vals[] = { g_val_l, g_val_c, g_val_r };
    for (int s = 0; s < 3; s++) {
        double ang    = g_theta + sens_ang[s];
        double dist_m = vals[s] / 1000.0;           /* przelicz na metry */
        if (dist_m > 1.0) dist_m = 1.0;
        int ex = px + (int)(dist_m * 160.0 * cos(ang));
        int ey = py - (int)(dist_m * 160.0 * sin(ang));
        int col = (dist_m < OBS_FRONT) ? col_hit[s] : col_clear[s];
        wb_display_set_color(disp, col);
        wb_display_draw_line(disp, px, py, ex, ey);
        if (dist_m < OBS_FRONT)
            wb_display_fill_oval(disp, ex, ey, 4, 4);
    }

    /* robot: niebieski krąg + biała strzałka kierunku */
    wb_display_set_color(disp, 0x2244FF);
    wb_display_fill_oval(disp, px, py, 8, 8);
    wb_display_set_color(disp, 0xFFFFFF);
    int dx2 = (int)(12 * cos(g_theta));
    int dy2 = (int)(-12 * sin(g_theta));
    wb_display_draw_line(disp, px, py, px + dx2, py + dy2);

    /* wynik w rogu */
    char buf[32];
    snprintf(buf, sizeof(buf), "%d/%d", score, NUM_GARBAGE);
    wb_display_set_color(disp, 0xFFFFFF);
    wb_display_draw_text(disp, buf, 4, 4);
}

static int navigate(double tx, double ty, double threshold) {
    double x, y, theta;
    get_pose(&x, &y, &theta);
    g_rx = x; g_ry = y; g_theta = theta;

    double dx = tx - x, dy = ty - y;
    double e1 = sqrt(dx*dx + dy*dy);

    if (e1 < threshold) {
        wb_motor_set_velocity(lm, 0.0);
        wb_motor_set_velocity(rm, 0.0);
        return 1;
    }

    double phi_star = atan2(dy, dx);
    double e2 = wrap(phi_star - theta);

    double u1 = KP1 * e1;
    double u2 = KP2 * e2;

    u1 *= fmax(0.0, cos(e2));
    if (u1 > 0.5) u1 = 0.5;

    /* Warstwa reaktywna – tylko gdy robot idzie do przodu (guard u1>0.1) */
    /* Sensor zwraca wartości w [0,1000], gdzie 1000mm=1.0m = brak przeszkody */
    double val_l = wb_distance_sensor_get_value(ds_l);
    double val_c = wb_distance_sensor_get_value(ds_c);
    double val_r = wb_distance_sensor_get_value(ds_r);
    g_val_l = val_l; g_val_c = val_c; g_val_r = val_r;
    double dist_c = val_c / 1000.0;   /* przelicz na metry */

    if (dist_c < OBS_FRONT && u1 > 0.10) {
        double closeness = (OBS_FRONT - dist_c) / OBS_FRONT;
        /* val wyższe = dalej = więcej miejsca; skręć w stronę większego val */
        double turn_dir  = (val_l >= val_r) ? 1.0 : -1.0;
        u2 += turn_dir * 4.0 * closeness;
        u1 *= fmax(0.2, 1.0 - closeness);
    }

    double w_l = u1 - HALF_WHEELBASE * u2;
    double w_r = u1 + HALF_WHEELBASE * u2;
    double omega_l = w_l / WHEEL_RADIUS;
    double omega_r = w_r / WHEEL_RADIUS;

    double scale = fmax(fabs(omega_l), fabs(omega_r));
    if (scale > MAX_WHEEL_VEL) {
        omega_l = omega_l / scale * MAX_WHEEL_VEL;
        omega_r = omega_r / scale * MAX_WHEEL_VEL;
    }

    wb_motor_set_velocity(lm, omega_l);
    wb_motor_set_velocity(rm, omega_r);
    return 0;
}

int main(void) {
    wb_robot_init();

    lm   = wb_robot_get_device("left wheel motor");
    rm   = wb_robot_get_device("right wheel motor");
    gps  = wb_robot_get_device("gps");
    ds_l = wb_robot_get_device("ds_left");
    ds_c = wb_robot_get_device("ds_center");
    ds_r = wb_robot_get_device("ds_right");
    disp = wb_robot_get_device("map_display");

    wb_motor_set_position(lm, INFINITY);
    wb_motor_set_position(rm, INFINITY);
    wb_motor_set_velocity(lm, 0.0);
    wb_motor_set_velocity(rm, 0.0);

    wb_gps_enable(gps, TIME_STEP);
    wb_distance_sensor_enable(ds_l, TIME_STEP);
    wb_distance_sensor_enable(ds_c, TIME_STEP);
    wb_distance_sensor_enable(ds_r, TIME_STEP);
    wb_display_set_font(disp, "Arial", 10, 1);

    robot_node = wb_supervisor_node_get_self();
    for (int i = 0; i < NUM_GARBAGE; i++) {
        garbage_node[i] = wb_supervisor_node_get_from_def(GARBAGE_DEF[i]);
        collected[i] = 0;
    }

    wb_robot_step(TIME_STEP);

    printf("=== Zbieracz smieci – Wyklad 10 + Wyklad 11 ===\n");
    printf("Sterownik P + Agent wnioskujacy\n");
    fflush(stdout);

    int back_up_cnt = 0;
    int step = 0;

    while (wb_robot_step(TIME_STEP) != -1) {
        step++;
        double rx, ry, rtheta;
        get_pose(&rx, &ry, &rtheta);

        /* Odświeżaj mapę co 5 kroków (~0.16s) */
        if (step % 5 == 0) draw_map();

        switch (state) {

        case SELECT: {
            int best = -1;
            double best_dist = 1e9;
            for (int i = 0; i < NUM_GARBAGE; i++) {
                if (collected[i] || !garbage_node[i]) continue;
                const double *pos = wb_supervisor_node_get_position(garbage_node[i]);
                double d = hypot(pos[0] - rx, pos[1] - ry);
                if (d < best_dist) { best_dist = d; best = i; }
            }
            if (best < 0) {
                printf("[KONIEC] Wszystkie smieci zebrane! Wynik: %d/%d\n",
                       score, NUM_GARBAGE);
                fflush(stdout);
                state = DONE;
            } else {
                current_garbage = best;
                state = GO_TO_GARBAGE;
                printf("[SELECT] Cel: %s (odleglosc=%.2f m)\n",
                       GARBAGE_DEF[best], best_dist);
                fflush(stdout);
            }
            break;
        }

        case GO_TO_GARBAGE: {
            if (!garbage_node[current_garbage]) { state = SELECT; break; }
            const double *pos = wb_supervisor_node_get_position(garbage_node[current_garbage]);
            if (navigate(pos[0], pos[1], PICK_DIST)) state = PICK;
            break;
        }

        case PICK: {
            WbFieldRef f = wb_supervisor_node_get_field(
                               garbage_node[current_garbage], "translation");
            if (f) {
                const double hidden[3] = {10.0, 10.0, 0.0};
                wb_supervisor_field_set_sf_vec3f(f, hidden);
            }
            collected[current_garbage] = 1;
            printf("[PICK] Podniesiono %s\n", GARBAGE_DEF[current_garbage]);
            fflush(stdout);
            back_up_cnt = BACK_UP_STEPS;
            state = BACK_UP;
            break;
        }

        case BACK_UP: {
            if (back_up_cnt <= 0) {
                state = GO_TO_BASE;
                printf("[BACK_UP] Gotowy – jade do bazy\n");
                fflush(stdout);
            } else {
                back_up_cnt--;
                wb_motor_set_velocity(lm, -3.0);
                wb_motor_set_velocity(rm, -3.0);
            }
            break;
        }

        case GO_TO_BASE: {
            static int gtb_tick = 0;
            if (++gtb_tick % 100 == 0) {
                printf("[GO_TO_BASE] pos=(%.2f,%.2f) dst=%.2f | ds_l=%.2fm ds_c=%.2fm ds_r=%.2fm\n",
                       rx, ry, hypot(rx-BASE_X, ry-BASE_Y),
                       g_val_l/1000.0, g_val_c/1000.0, g_val_r/1000.0);
                fflush(stdout);
            }
            if (navigate(BASE_X, BASE_Y, ARRIVE_DIST)) {
                gtb_tick = 0;
                state = DEPOSIT;
            }
            break;
        }

        case DEPOSIT: {
            score++;
            printf("[DEPOSIT] Dostarczono na baze! Wynik: %d/%d\n",
                   score, NUM_GARBAGE);
            fflush(stdout);
            state = SELECT;
            break;
        }

        case DONE: {
            wb_motor_set_velocity(lm, 0.0);
            wb_motor_set_velocity(rm, 0.0);
            break;
        }
        }
    }

    wb_robot_cleanup();
    return 0;
}
