//
// Created by Robert Wang on 2022/8/22.
//

#include "PX4Gyroscope.hpp"

bool PX4Gyroscope::attack_enabled(const uint8_t &attack_type, const hrt_abstime &timestamp_sample) const {
    return _attack_flag_prev & attack_type && _attack_timestamp != 0 && timestamp_sample >= _attack_timestamp;
}

void PX4Gyroscope::applyGyroAttack(sensor_gyro_s &gyro) {
    // Attempt to Apply Sensor Attack at X-axis
    if (attack_enabled(sensor_attack::ATK_MASK_GYRO, gyro.timestamp_sample)) {
        _last_deviation[0] = 0.f;
        _last_deviation[1] = 0.f;
        _last_deviation[2] = 0.f;

        // Modified by Yosgarf on 20230908
        double noiseTemp;

        double amplitude = _param_atk_gyr_cos_amp.get();
        double frequency = _param_atk_gyr_cos_freq.get();
        double phase = _param_atk_gyr_cos_ph.get();
        double bias = _param_atk_gyr_bias.get();

        double time = static_cast<double>(gyro.timestamp_sample) / 1000000.0;

        noiseTemp = amplitude * cos(2 * M_PI * frequency * time + phase) + bias;
        float noise = float(noiseTemp);

        if (_param_atk_gyr_axis.get() & (1 << 0)) {
            gyro.x += noise;
        }
        if (_param_atk_gyr_axis.get() & (1 << 1)) {
            gyro.y += noise;
        }
        if (_param_atk_gyr_axis.get() & (1 << 2)) {
            gyro.z += noise;
        }

    } else {
        _last_deviation[0] = 0.f;
        _last_deviation[1] = 0.f;
        _last_deviation[2] = 0.f;
    }
}

void PX4Gyroscope::applyGyroAttack(sensor_gyro_s &gyro, sensor_gyro_fifo_s &gyro_fifo) {
    // Attempt to Apply Sensor Attack at X-axis
    applyGyroAttack(gyro);

    if (attack_enabled(sensor_attack::ATK_MASK_GYRO, gyro_fifo.timestamp_sample)) {
        // Also apply the deviation to FIFO samples
        const uint8_t N = gyro_fifo.samples;
        for (int n = 0; n < N; n++) {
            gyro_fifo.x[n] += static_cast<int16_t>(_last_deviation[0] / _scale);
            gyro_fifo.y[n] += static_cast<int16_t>(_last_deviation[1] / _scale);
            gyro_fifo.z[n] += static_cast<int16_t>(_last_deviation[2] / _scale);
        }
    }
}
