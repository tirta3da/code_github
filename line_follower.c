#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

#define TIME_STEP 32
#define MAX_SPEED 10.0
#define BASE_SPEED 8.0
#define THRESHOLD 300
#define NOISE_THRESHOLD 200
#define GHOST_THRESHOLD 50

typedef enum {
  MODE_LURUS,
  MODE_KANAN,
  MODE_KIRI,
  MODE_CARI
} Mode;

int main() {
  wb_robot_init();

  // Mendapatkan perangkat motor
  WbDeviceTag motor_kiri = wb_robot_get_device("motorkiri");
  WbDeviceTag motor_kanan = wb_robot_get_device("motorkanan");

  wb_motor_set_position(motor_kiri, INFINITY);
  wb_motor_set_position(motor_kanan, INFINITY);

  // Inisialisasi sensor IR
  const char *sensor_names[8] = {
    "ir1", "ir2", "ir3", "ir4", "ir5", "ir6", "ir7", "ir8"
  };
  WbDeviceTag ir_sensors[8];
  for (int i = 0; i < 8; i++) {
    ir_sensors[i] = wb_robot_get_device(sensor_names[i]);
    wb_distance_sensor_enable(ir_sensors[i], TIME_STEP);
  }

  Mode mode = MODE_LURUS;

  while (wb_robot_step(TIME_STEP) != -1) {
    double sensor_values[8];
    int active_left = 0, active_right = 0;
    int all_active = 0; // Untuk memeriksa apakah semua sensor aktif

    // Membaca sensor IR
    for (int i = 0; i < 8; i++) {
      sensor_values[i] = wb_distance_sensor_get_value(ir_sensors[i]);

      // Menilai sensor yang aktif
      if (sensor_values[i] > NOISE_THRESHOLD) {
        if (sensor_values[i] > THRESHOLD) {
          if (i < 3) {
            // Sensor kanan (IR1, IR2, IR3)
            if (sensor_values[i] > THRESHOLD) active_right++;
          }
          else if (i > 4) {
            // Sensor kiri (IR6, IR7, IR8)
            if (sensor_values[i] > THRESHOLD) active_left++;
          }
        }
      }

      // Mengecek apakah semua sensor mendeteksi garis
      if (sensor_values[i] > THRESHOLD) {
        all_active++;
      }
    }

    // Logika jika robot di atas garis putus-putus
    bool all_sensors_active = (all_active >= 6); // Jika lebih dari 6 sensor aktif, kemungkinan robot berada di atas garis putus-putus

    // Logika untuk mode "Lurus"
    // IR4 atau IR5 tidak mendeteksi garis hitam, tapi sensor lainnya mendeteksi garis
    if (sensor_values[3] < THRESHOLD && sensor_values[4] < THRESHOLD && active_left + active_right >= 4) {
      mode = MODE_LURUS; // Robot lurus jika banyak sensor lainnya mendeteksi garis
    }
    // Logika jika semua sensor mendeteksi garis hitam, robot tetap maju
    else if (all_sensors_active) {
      mode = MODE_LURUS; // Semua sensor mendeteksi garis -> tetap maju
      printf("Semua Sensor Deteksi Garis, Maju\n");
    }
    // Tentukan mode berdasarkan sensor yang aktif
    else if (active_left > active_right) {
      mode = MODE_KIRI; // Belok kanan jika lebih banyak kiri
    } else if (active_right > active_left) {
      mode = MODE_KANAN; // Belok kiri jika lebih banyak kanan
    } else {
      mode = MODE_LURUS; // Lurus jika sensor kiri dan kanan seimbang
    }

    // Kecepatan motor kiri dan kanan berdasarkan mode
    double left_speed = BASE_SPEED;
    double right_speed = BASE_SPEED;

    switch (mode) {
      case MODE_LURUS:
        printf("Lurus\n");
        left_speed = BASE_SPEED;
        right_speed = BASE_SPEED;
        break;

      case MODE_KANAN:
        printf("Belok Kanan\n");
        left_speed = MAX_SPEED; // Motor kiri lebih cepat
        right_speed = BASE_SPEED * 0.5; // Motor kanan lebih lambat
        break;

      case MODE_KIRI:
        printf("Belok Kiri\n");
        left_speed = BASE_SPEED * 0.5; // Motor kiri lebih lambat
        right_speed = MAX_SPEED; // Motor kanan lebih cepat
        break;

      case MODE_CARI:
        printf("Mencari Garis\n");
        left_speed = BASE_SPEED * 0.7;
        right_speed = -BASE_SPEED * 0.7; // Bergerak untuk mencari garis
        break;
    }

    // Atur kecepatan motor
    if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;

    wb_motor_set_velocity(motor_kiri, left_speed);
    wb_motor_set_velocity(motor_kanan, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}
