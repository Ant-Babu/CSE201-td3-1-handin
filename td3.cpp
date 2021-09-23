#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {
    double* array_tmp = new double[new_size];
    std::copy(array, array + length, array_tmp);
    delete[] array;
  return array_tmp;
}

double* shrink_array(double* array, int length, int new_size) {
    double* array_tmp = new double[new_size];  //I copypasted from extend_array() as it has the same mechanism, only this time new_size < length
    std::copy(array, array + new_size, array_tmp);
    delete[] array;
  return array_tmp;
}

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {
    if (current_size == max_size) {
        array = extend_array(array, current_size, current_size+5);
        max_size += 5;
    }
    array[current_size] = element;
    current_size += 1;
    return array;
}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {
    current_size = std::max(0, current_size -1);
    if (current_size-max_size <= -5) {
        array = shrink_array(array, current_size, max_size-5);
        max_size = max_size-5;
    }
    return array;
}

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {

  bool hit_target, hit_obstacle;
  double v0_x, v0_y, x, y, t;
  double PI = 3.14159265;
  double g = 9.8;

  v0_x = magnitude * cos(angle * PI / 180);
  v0_y = magnitude * sin(angle * PI / 180);

  t = 0;
  x = 0;
  y = 0;

  telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
  telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
  telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);

  hit_target = false;
  hit_obstacle = false;
  while (y >= 0 && (! hit_target) && (! hit_obstacle)) {
    double * target_coordinates = find_collision(x, y, targets, tot_targets);
    if (target_coordinates != NULL) {
      remove_target(targets, tot_targets, target_coordinates);
      hit_target = true;
    } else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
      hit_obstacle = true;
    } else {
      t = t + simulation_interval;
      y = v0_y * t  - 0.5 * g * t * t;
      x = v0_x * t;
      if (y>=0){ //otherwise we store coordinates that are of no interest (I had too many elements in the telemetry)
      telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
      telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
      telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);}
    }
  }

  return hit_target;
}

void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {

    global_telemetry_max_size = 0; //create and initialize the array containing the merged telemetries
    for (int i = 0; i < tot_telemetries; ++i) {
      global_telemetry_max_size += telemetries_sizes[i];
    }
    global_telemetry = new double[global_telemetry_max_size];

    global_telemetry_current_size = 0;
    double* tmp = global_telemetry;
    for (int i = 0; i < tot_telemetries; ++i){
      for (int j = 0; j < telemetries_sizes[i]; ++j){
        tmp = append_to_array(telemetries[i][j],global_telemetry,global_telemetry_current_size,global_telemetry_max_size);

      }
    }
  }
