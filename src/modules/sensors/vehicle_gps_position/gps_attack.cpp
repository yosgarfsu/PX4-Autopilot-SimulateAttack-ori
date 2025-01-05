//
// Created by Robert Wang on 2022/8/29.
//

#include "VehicleGPSPosition.hpp"
#include <lib/geo/geo.h>

namespace sensors
{
    bool VehicleGPSPosition::attack_enabled(const uint8_t &attack_type) const {
        return _param_atk_apply_type.get() & attack_type &&
                _attack_timestamp != 0 && hrt_absolute_time() >= _attack_timestamp;
    }

    void VehicleGPSPosition::ConductVelocitySpoofing(sensor_gps_s &gps_position)
    {
        if (attack_enabled(sensor_attack::ATK_GPS_VEL)) {
            if (!_vel_deviation) {
                PX4_INFO("Initiate GPS Velocity Spoofing Attack");
                _vel_deviation.reset(sensor_attack::CreateAttackInstance(_param_atk_gps_v_cls.get(), &_vel_atk_params));

                const float time_to_max_deviation_s = (*_vel_deviation).time_to_max_deviation();
                if (PX4_ISFINITE(time_to_max_deviation_s)) {
                    PX4_INFO("Time to Max Deviation is: %.3f sec", (double)time_to_max_deviation_s);
                } else {
                    PX4_WARN("INFINITE time to max deviation, please check attack parameter setting!");
                }

            }

            if (_vel_deviation) {
                const Vector3f deviation = (*_vel_deviation).calculate_deviation(gps_position.timestamp);
                sensor_attack::gps_velocity_spoofing(gps_position, deviation);
            }

        } else if (_vel_deviation) {
            _vel_deviation.reset();
            PX4_INFO("GPS Velocity Spoofing Stopped");
        }
    }

    void VehicleGPSPosition::ConductPositionSpoofing(sensor_gps_s &gps_position) {
        if (attack_enabled(sensor_attack::ATK_GPS_POS)) {
            if (!_pos_deviation) {
                PX4_INFO("Initiate GPS Position Spoofing Attack");
                _pos_deviation.reset(sensor_attack::CreateAttackInstance(_param_atk_gps_p_cls.get(), &_pos_atk_params));

                const float time_to_max_deviation_s = (*_pos_deviation).time_to_max_deviation();
                if (PX4_ISFINITE(time_to_max_deviation_s)) {
                    PX4_INFO("Time to Max Deviation is: %.3f sec", (double)time_to_max_deviation_s);
                } else {
                    PX4_WARN("INFINITE time to max deviation, please check attack parameter setting!");
                }

            }

            if (_pos_deviation) {
                const Vector3f deviation = (*_pos_deviation).calculate_deviation(gps_position.timestamp);
                sensor_attack::gps_position_spoofing(gps_position, deviation);
            }

        } else if (_pos_deviation) {
            _pos_deviation.reset();
            PX4_INFO("GPS Position Spoofing Stopped");
        }

    }

    void VehicleGPSPosition::ConductVPSpoofing(sensor_gps_s &gps_position) {
        if (attack_enabled(sensor_attack::ATK_GPS_V_P)){
            //double time = static_cast<double>(gps_position.timestamp) / 1000000.0;
            if (_pos_record == 1) {
                _pos_lat_ori = gps_position.lat;
                _pos_lon_ori = gps_position.lon;
                _pos_record = 0;
            }
            double false_velocity = 0.0;
            double abs_atk_time = static_cast<double>(gps_position.timestamp - _attack_timestamp) / 1000000.0;
            if (abs_atk_time < 35) {
                // if (abs_atk_time < 3) {
                //     false_velocity = 0.05 * abs_atk_time - 0.15;
                // } else if (abs_atk_time < 3.65) {
                //     false_velocity = 0.153846 * abs_atk_time - 0.46154;
                // } else if (abs_atk_time < 3.85) {
                //     false_velocity = 0.5 * abs_atk_time - 1.725;
                // } else if (abs_atk_time < 11.5) {
                //     false_velocity = -0.01307 * abs_atk_time + 0.250327;
                // } else if (abs_atk_time < 13.5) {
                //     false_velocity = -0.05 * abs_atk_time + 0.675;
                // } else if (abs_atk_time < 15.05) {
                //     false_velocity = -0.06452 * abs_atk_time + 0.870968;
                // } else if (abs_atk_time < 21) {
                //     false_velocity = 0.016807 * abs_atk_time - 0.35294;
                // } else if (abs_atk_time < 22) {
                //     false_velocity = 0.1 * abs_atk_time - 2.1;
                // } else {
                //     false_velocity = 0.0;
                // }

                if (abs_atk_time < 10) false_velocity = -0.1;
                else if (abs_atk_time < 20) false_velocity = 0.1;
                else false_velocity = 0;

                float false_velocity_c;
                if (abs_atk_time < 2.304) {
                    false_velocity_c = 0.017361111 * abs_atk_time - 0.04;
                } else if (abs_atk_time < 12.496) {
                    false_velocity_c = 0.072606 * abs_atk_time - 0.16728;
                } else if (abs_atk_time < 16.028) {
                    false_velocity_c = -0.03964 * abs_atk_time + 1.235311;
                } else if (abs_atk_time < 22.428) {
                    false_velocity_c = 0.6;
                } else if (abs_atk_time < 24.628) {
                    false_velocity_c = -0.40909 * abs_atk_time + 9.775091;
                } else if (abs_atk_time < 26.528) {
                    false_velocity_c = 0.121053 * abs_atk_time - 3.28128;
                } else if (abs_atk_time < 28.728) {
                    false_velocity_c = -0.07;
                } else if (abs_atk_time < 31.428) {
                    false_velocity_c = 0.025926 * abs_atk_time - 0.8148;
                } else {
                    false_velocity_c = 0.005599 * abs_atk_time - 0.17597;
                }

                double delta_x = (false_velocity + _false_velocity_prev) / 2 * static_cast<double>(gps_position.timestamp - _atk_timestamp_prev) / 1000000.0;

                double gps_lat = _pos_lat_prev / 1.0e7;
                double gps_lon = _pos_lon_prev / 1.0e7;
                const MapProjection fake_ref{gps_lat, gps_lon};
                fake_ref.reproject(0.0, delta_x, gps_lat, gps_lon);

                gps_position.lat = (int32_t) (gps_lat * 1.0e7);
                gps_position.lon = (int32_t) (gps_lon * 1.0e7);
                if (abs_atk_time > 21) {
                    gps_position.lat = _pos_lat_ori;
                    gps_position.lon = _pos_lon_ori;
                }

                gps_position.vel_e_m_s = false_velocity_c;
                // false_velocity_c = false_velocity_c;
                gps_position.vel_m_s = sqrtf(gps_position.vel_n_m_s * gps_position.vel_n_m_s +
                                            gps_position.vel_e_m_s * gps_position.vel_e_m_s +
                                            gps_position.vel_d_m_s * gps_position.vel_d_m_s);

                _false_velocity_prev = false_velocity;
                _atk_timestamp_prev = gps_position.timestamp;
                _pos_lat_prev = gps_position.lat;
                _pos_lon_prev = gps_position.lon;
            // } else if (abs_atk_time < 35) {
            //     _false_velocity_prev = 0.01;
            //     _atk_timestamp_prev = gps_position.timestamp;
            //     _pos_lat_prev = gps_position.lat;
            //     _pos_lon_prev = gps_position.lon;

            //     gps_position.lat = _pos_lat_ori;
            //     gps_position.lon = _pos_lon_ori;
            } else {
                _false_velocity_prev = -_false_velocity_prev;
                _atk_timestamp_prev = gps_position.timestamp;
                _pos_lat_prev = gps_position.lat;
                _pos_lon_prev = gps_position.lon;

                gps_position.lat = _pos_lat_ori;
                gps_position.lon = _pos_lon_ori;
                int order = static_cast<int>((abs_atk_time - 35) / 0.5);
                order = order % 8;
                switch (order)
                {
                case 0: gps_position.vel_e_m_s = 0.02; break;
                case 1: gps_position.vel_e_m_s = 0.01; break;
                case 2: gps_position.vel_e_m_s = 0.0; break;
                case 3: gps_position.vel_e_m_s = -0.01; break;
                case 4: gps_position.vel_e_m_s = -0.02; break;
                case 5: gps_position.vel_e_m_s = -0.01; break;
                case 6: gps_position.vel_e_m_s = 0.0; break;
                case 7: gps_position.vel_e_m_s = 0.01; break;
                default:
                    break;
                }
                // gps_position.vel_e_m_s = _false_velocity_prev;
                // gps_position.vel_m_s = sqrtf(gps_position.vel_n_m_s * gps_position.vel_n_m_s +
                //                             gps_position.vel_e_m_s * gps_position.vel_e_m_s +
                //                             gps_position.vel_d_m_s * gps_position.vel_d_m_s);
            }

            // if (_pos_record == 1) {
            //     _pos_lat_ori = gps_position.lat;
            //     _pos_lon_ori = gps_position.lon;
            //     _pos_record = 0;
            // }
            // gps_position.lat = _pos_lat_ori;
            // gps_position.lon = _pos_lon_ori;

            //     _atk_timestamp_prev = gps_position.timestamp;
            //     _pos_lat_prev = gps_position.lat;
            //     _pos_lon_prev = gps_position.lon;

        } else {
            _false_velocity_prev = gps_position.vel_m_s;
            _atk_timestamp_prev = gps_position.timestamp;
            _pos_lat_prev = gps_position.lat;
            _pos_lon_prev = gps_position.lon;
        }


    }

}  // sensors
