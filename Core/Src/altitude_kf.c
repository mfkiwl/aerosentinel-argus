/*
 * altitude_kf.c
 *
 *  Created on: Aug 11, 2024
 *      Author: Yassine DEHHANI
 */

#include <stdio.h>
#include <stdlib.h>
#include "altitude_kf.h"

Altitude_KF* Altitude_KF_create(float Q_accel, float R_altitude) {
    Altitude_KF* kf = (Altitude_KF*)malloc(sizeof(Altitude_KF));
    if (kf == NULL) {
        return NULL;
    }

    kf->Q_accel = Q_accel;
    kf->R_altitude = R_altitude;
    kf->h = 0.0f;
    kf->v = 0.0f;
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;

    return kf;
}

void Altitude_KF_propagate(Altitude_KF* kf, float acceleration, const float dt) {
    float _dtdt = dt * dt;

    kf->h = kf->h + kf->v*dt + 0.5f*acceleration*_dtdt;
    kf->v = kf->v + acceleration*dt;

    float _Q_accel_dtdt = kf->Q_accel * _dtdt;

    kf->P[0][0] = kf->P[0][0] + (kf->P[1][0] + kf->P[0][1] + (kf->P[1][1] + 0.25f*_Q_accel_dtdt) * dt) * dt;
    kf->P[0][1] = kf->P[0][1] + (kf->P[1][1] + 0.5f*_Q_accel_dtdt) * dt;
    kf->P[1][0] = kf->P[1][0] + (kf->P[1][1] + 0.5f*_Q_accel_dtdt) * dt;
    kf->P[1][1] = kf->P[1][1] + _Q_accel_dtdt;
}

void Altitude_KF_update(Altitude_KF* kf, float altitude, float R_altitude) {
    float y = altitude - kf->h;

    float Sinv = 1.0f / (kf->P[0][0] + R_altitude);

    float K[2] = { kf->P[0][0] * Sinv, kf->P[1][0] * Sinv };

    kf->h += K[0] * y;
    kf->v += K[1] * y;

    kf->P[0][0] = kf->P[0][0] - K[0] * kf->P[0][0];
    kf->P[0][1] = kf->P[0][1] - K[0] * kf->P[0][1];
    kf->P[1][0] = kf->P[1][0] - (K[1] * kf->P[0][0]);
    kf->P[1][1] = kf->P[1][1] - (K[1] * kf->P[0][1]);
}

void Altitude_KF_update_default(Altitude_KF* kf, float altitude) {
    Altitude_KF_update(kf, altitude, kf->R_altitude);
}

void Altitude_KF_destroy(Altitude_KF* kf) {
    free(kf);
}


