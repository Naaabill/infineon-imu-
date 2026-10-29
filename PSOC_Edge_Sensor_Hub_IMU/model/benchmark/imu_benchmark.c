#include "imu_benchmark.h"
#include "imu_benchmark_fixture.h"
#include "../imu_motion_model.h"

#define BENCH_GRAVITY_MPS2 (9.80665f)
#define BENCH_ACC_SCALE (BENCH_GRAVITY_MPS2 / 1000.0f)
#define BENCH_GYRO_SCALE (0.001f)

uint32_t imu_benchmark_run(uint32_t repetitions, volatile uint32_t *checksum)
{
    uint32_t correct = 0U;
    uint32_t sum = 0U;

    for (uint32_t repeat = 0U; repeat < repetitions; ++repeat)
    {
        for (uint32_t window = 0U; window < IMU_BENCH_WINDOW_COUNT; ++window)
        {
            float acc_mean = 0.0f;
            float acc_m2 = 0.0f;
            float gyro_energy_sum = 0.0f;

            for (uint32_t i = 0U; i < IMU_BENCH_SAMPLES_PER_WINDOW; ++i)
            {
                const imu_bench_sample_t *sample = &imu_bench_fixture[window][i];
                const float ax = (float)sample->axis[0] * BENCH_ACC_SCALE;
                const float ay = (float)sample->axis[1] * BENCH_ACC_SCALE;
                const float az = (float)sample->axis[2] * BENCH_ACC_SCALE;
                const float gx = (float)sample->axis[3] * BENCH_GYRO_SCALE;
                const float gy = (float)sample->axis[4] * BENCH_GYRO_SCALE;
                const float gz = (float)sample->axis[5] * BENCH_GYRO_SCALE;
                const float acc_mag_sq = (ax * ax) + (ay * ay) + (az * az);
                const float delta = acc_mag_sq - acc_mean;

                acc_mean += delta / (float)(i + 1U);
                acc_m2 += delta * (acc_mag_sq - acc_mean);
                gyro_energy_sum += (gx * gx) + (gy * gy) + (gz * gz);
            }

            const float acc_variance = acc_m2 / (float)IMU_BENCH_SAMPLES_PER_WINDOW;
            const float gyro_energy = gyro_energy_sum / (float)IMU_BENCH_SAMPLES_PER_WINDOW;
            const int prediction = imu_motion_model_predict(acc_variance, gyro_energy);

            correct += (prediction == (int)imu_bench_labels[window]) ? 1U : 0U;
            sum += (uint32_t)prediction + (window * 3U) + (repeat & 0xFFFFU);
        }
    }

    if (checksum != 0)
    {
        *checksum = sum;
    }
    return correct;
}
