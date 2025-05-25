#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

#define TIME_STEP 32
#define MAX_SPEED 10.0
#define BASE_SPEED 8.0
#define THRESHOLD 300
#define NOISE_THRESHOLD 200

typedef enum {
  MODE_LURUS,
  MODE_KANAN,
  MODE_KIRI,
  MODE_CARI,
  MODE_PERTIGAAN_KIRI,
  MODE_PERTIGAAN_KANAN,
  MODE_PEREMPATAN,
  MODE_PUTAR_BALIK
} Mode;

float speed_multiplier = 1.0;

int main() {
  wb_robot_init();

  WbDeviceTag motor_kiri = wb_robot_get_device("motorkiri");
  WbDeviceTag motor_kanan = wb_robot_get_device("motorkanan");

  wb_motor_set_position(motor_kiri, INFINITY);
  wb_motor_set_position(motor_kanan, INFINITY);

  const char *sensor_names[8] = {
    "ir1", "ir2", "ir3", "ir4", "ir5", "ir6", "ir7", "ir8"
  };
  WbDeviceTag ir_sensors[8];
  for (int i = 0; i < 8; i++) {
    ir_sensors[i] = wb_robot_get_device(sensor_names[i]);
    wb_distance_sensor_enable(ir_sensors[i], TIME_STEP);
  }

  Mode mode = MODE_CARI;
  int cross_timer = 0;
  int turn_timer = 0; // Timer untuk mempertahankan belokan

  while (wb_robot_step(TIME_STEP) != -1) {
    double sensor_values[8];
    int active_sensors = 0;
    int active_left = 0, active_right = 0, active_center = 0;
    float line_quality = 0.0;

    printf("Sensor IR: ");
    for (int i = 0; i < 8; i++) {
      sensor_values[i] = wb_distance_sensor_get_value(ir_sensors[i]);
      printf("IR%d: %.0f ", i + 1, sensor_values[i]);

      if (sensor_values[i] > NOISE_THRESHOLD) {
        if (sensor_values[i] > THRESHOLD) {
          active_sensors++;
          line_quality += (sensor_values[i] - THRESHOLD) / 100.0;
        }

        if (i < 2) {
          if (sensor_values[i] > THRESHOLD) active_right++;
        }
        else if (i > 5) {
          if (sensor_values[i] > THRESHOLD) active_left++;
        }
        else {
          if (sensor_values[i] > THRESHOLD) active_center++;
        }
      }
    }
    printf(" | Kualitas: %.1f\n", line_quality);

    speed_multiplier = 0.8 + (line_quality / 20.0);
    if (speed_multiplier > 1.5) speed_multiplier = 1.5;
    if (speed_multiplier < 0.8) speed_multiplier = 0.8;

    bool t_junction_left = (active_center >= 2) && 
                          (active_left >= 2) && 
                          (active_right == 0) &&
                          (sensor_values[5] < THRESHOLD && 
                           sensor_values[6] > THRESHOLD && 
                           sensor_values[7] > THRESHOLD);

    bool t_junction_right = (active_center >= 2) && 
                           (active_right >= 2) && 
                           (active_left == 0) &&
                           (sensor_values[0] > THRESHOLD && 
                            sensor_values[1] > THRESHOLD && 
                            sensor_values[2] > THRESHOLD &&
                            sensor_values[5] < THRESHOLD &&
                            sensor_values[6] < THRESHOLD &&
                            sensor_values[7] < THRESHOLD);

    bool crossroad = (active_center >= 4) && 
                    (active_left >= 2) && 
                    (active_right >= 2);

    // Hitung kekuatan relatif untuk belok
    int left_strength = active_left * 100 + (sensor_values[6] + sensor_values[7]) / 2;
    int right_strength = active_right * 100 + (sensor_values[0] + sensor_values[1]) / 2;
    int center_strength = active_center * 100 + (sensor_values[3] + sensor_values[4]) / 2;

    printf("Strength - Left: %d, Right: %d, Center: %d\n", left_strength, right_strength, center_strength);

    if (t_junction_left && cross_timer == 0) {
      mode = MODE_PERTIGAAN_KIRI;
      cross_timer = 5;
      printf("+++++ PERTIGAAN KIRI DETECTED +++++\n");
    }
    else if (t_junction_right && cross_timer == 0) {
      mode = MODE_PERTIGAAN_KANAN;
      cross_timer = 5;
      printf("+++++ PERTIGAAN KANAN DETECTED +++++\n");
    }
    else if (crossroad && cross_timer == 0) {
      mode = MODE_PEREMPATAN;
      cross_timer = 10;
      printf("+++++ PEREMPATAN DETECTED +++++\n");
    }
    else if (cross_timer > 0) {
      cross_timer--;
    }
    else if (turn_timer > 0) {
      turn_timer--; // Pertahankan mode belok selama beberapa langkah
    }
    else {
      if (left_strength > right_strength + 50 && left_strength > center_strength + 20) {
        mode = MODE_KIRI;
        turn_timer = 3; // Pertahankan belok selama 3 langkah
      }
      else if (right_strength > left_strength + 50 && right_strength > center_strength + 20) {
        mode = MODE_KANAN;
        turn_timer = 3; // Pertahankan belok selama 3 langkah
      }
      else if (center_strength > left_strength && center_strength > right_strength) {
        mode = MODE_LURUS;
      }
      else {
        mode = MODE_CARI;
      }
    }

    double left_speed = BASE_SPEED * speed_multiplier;
    double right_speed = BASE_SPEED * speed_multiplier;

    switch (mode) {
      case MODE_LURUS:
        printf("↑ LURUS (Speed: %.1f)\n", BASE_SPEED * speed_multiplier);
        break;

      case MODE_KANAN:
        left_speed = MAX_SPEED * speed_multiplier; // Motor kiri cepat
        right_speed = BASE_SPEED * 0.2 * speed_multiplier; // Motor kanan sangat lambat
        printf("→ BELOK KANAN (Speed: %.1f)\n", BASE_SPEED * speed_multiplier);
        break;

      case MODE_KIRI:
        left_speed = BASE_SPEED * 0.2 * speed_multiplier; // Motor kiri sangat lambat
        right_speed = MAX_SPEED * speed_multiplier; // Motor kanan cepat
        printf("← BELOK KIRI (Speed: %.1f)\n", BASE_SPEED * speed_multiplier);
        break;

      case MODE_PERTIGAAN_KIRI:
        left_speed = BASE_SPEED * 0.1;
        right_speed = BASE_SPEED * 1.8;
        printf("⤷ BELOK KIRI DI PERTIGAAN (Boost Speed)\n");
        break;

      case MODE_PERTIGAAN_KANAN:
        left_speed = BASE_SPEED * 1.8;
        right_speed = BASE_SPEED * 0.1;
        printf("⤶ BELOK KANAN DI PERTIGAAN (Boost Speed)\n");
        break;

      case MODE_PEREMPATAN:
        left_speed = 0.0;
        right_speed = 0.0;
        printf("■ BERHENTI DI PEREMPATAN\n");
        break;

      case MODE_PUTAR_BALIK:
        left_speed = BASE_SPEED * 0.7;
        right_speed = -BASE_SPEED * 0.7;
        printf("↻ PUTAR BALIK\n");
        break;

      case MODE_CARI:
        left_speed = BASE_SPEED * 0.7;
        right_speed = -BASE_SPEED * 0.7;
        printf("⟳ MENCARI GARIS\n");
        break;
    }

    if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
    if (left_speed < -MAX_SPEED) left_speed = -MAX_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;

    wb_motor_set_velocity(motor_kiri, left_speed);
    wb_motor_set_velocity(motor_kanan, right_speed);

    printf("Motor - Kiri: %.1f, Kanan: %.1f\n", left_speed, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}