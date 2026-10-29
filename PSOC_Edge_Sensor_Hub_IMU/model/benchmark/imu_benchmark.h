#ifndef IMU_BENCHMARK_H
#define IMU_BENCHMARK_H

#include <stdint.h>

/* Run the identical recorded workload repeatedly. The caller times this call
 * with a hardware cycle counter. No I2C, UART, allocation, or RTOS calls occur
 * here. Returns number of correct window classifications and writes a checksum
 * so the optimizer cannot remove the work. */
uint32_t imu_benchmark_run(uint32_t repetitions, volatile uint32_t *checksum);

#endif
