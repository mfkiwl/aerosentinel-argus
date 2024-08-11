/*
 * altitude_kf.h
 *
 *  Created on: Aug 11, 2024
 *      Author: Yassine DEHHANI
 */

#ifndef INC_ALTITUDE_KF_H_
#define INC_ALTITUDE_KF_H_

typedef struct {
    float h;
    float v;
    float Q_accel;
    float R_altitude;
    float P[2][2];
} Altitude_KF;

Altitude_KF* Altitude_KF_create(float Q_accel, float R_altitude);
void Altitude_KF_propagate(Altitude_KF* kf, float acceleration, const float dt);
void Altitude_KF_update(Altitude_KF* kf, float altitude, float R_altitude);
void Altitude_KF_update_default(Altitude_KF* kf, float altitude);
void Altitude_KF_destroy(Altitude_KF* kf);


#endif /* INC_ALTITUDE_KF_H_ */
