/* C implementation of simple scalar Kalman filter with no control (no `B * u` term). */

#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>
#include<math.h>
#include<time.h>

#define NUM_ITER 100
#define NUM_ERR 1

enum _kalman_status {
    E_FAILED_SCALAR_INVERSE = -1,
    SUCCESS = 0
};
typedef enum _kalman_status status_t;

struct _error_desc {
    int code;
    char* message;
} const ERROR_DESC[] = {
    { .code = E_FAILED_SCALAR_INVERSE, .message = "Failed to invert scalar" }
};

bool handle_status(status_t e) {
    for (int idx = 0; idx < NUM_ERR; idx++) {
        if (ERROR_DESC[idx].code == e) {
            printf("RUNTIME ERROR!\n");
            printf("%s\n", ERROR_DESC[idx].message);
            return true;
        }
    }
    return false;
}

struct scalar_kalman_t {
    double x;
    double P;
    double A;
    double H;
    double Q;
    double R;
};
typedef struct scalar_kalman_t* khandle_t;

void predict(khandle_t kal) {
    kal->x *= kal->A;
    kal->P = kal->A * kal->P * kal->A + kal->Q;
}

status_t update(khandle_t kal, double z) {
    double y = z - kal->H * kal->x;
    double S = kal->H * kal->P * kal->H + kal->R;
    if (fabs(S) < 1e-8) {
        return E_FAILED_SCALAR_INVERSE;
    }
    double S_inv = 1.0 / S;
    double K = kal->P * kal->H * S_inv;
    kal->x += K * y;
    kal->P *= 1.0 - K * kal->H;
    return SUCCESS;
}

status_t advance(khandle_t kal, double z) {
    predict(kal);
    status_t res = update(kal, z);
    return res;
}

int main(void) {
    double process_noise_max = 0.15;
    const double PI = 3.14159265359;
    double f = 0.1;
    double w = 2.0 * PI * f;
    double phase = 0.0;
    double magnitude = 0.2;
    double dc_offset = 0.2294;

    double t[NUM_ITER];
    double delt = 0.1;
    double currt = 0.0;
    for (int idx = 0; idx < NUM_ITER; idx++) {
        t[idx] = currt;
        currt += delt;
    }

    double x_truth[NUM_ITER];
    for (int idx = 0; idx < NUM_ITER; idx++) {
        x_truth[idx] = magnitude * sinf(w * t[idx] - phase) + dc_offset;
    }

    time_t tseed;
    srand((unsigned) time(&tseed));
    double z_obs[NUM_ITER];
    for (int idx = 0; idx < NUM_ITER; idx++) {
        double noise = process_noise_max * ((double) rand() / RAND_MAX * 2.0 - 1.0);
        z_obs[idx] = x_truth[idx] + noise;
    }

    double A = 1.0;
    double H = 1.0;
    double Q = 1e-4;
    double R = 0.05 * 0.05;
    double x0 = x_truth[0];
    double P0 = 0.0;

    struct scalar_kalman_t kalman_filter = {
        .x = x0,
        .P = P0,
        .A = A,
        .H = H,
        .Q = Q,
        .R = R,
    };

    double output[NUM_ITER];
    for (int idx = 0; idx < NUM_ITER; idx++) {
        status_t err_status = advance(&kalman_filter, z_obs[idx]);
        if (handle_status(err_status)) {
            return -1;
        }
        output[idx] = kalman_filter.x;
    }

    for (int idx = 0; idx < NUM_ITER; idx++) {
        printf("%f,", x_truth[idx]);
        printf("%f,", z_obs[idx]);
        printf("%f\n", output[idx]);
    }

    return 0;
}