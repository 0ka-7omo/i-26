// comment
#include "pico_copter.hpp"

//グローバル変数
uint8_t Arm_flag=0;
semaphore_t sem;

int main(void)
{
  int start_wait=5;
  
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  
  //Initialize stdio for Pico
  stdio_init_all();
  
  //Initialize LSM9DS1
  imu_mag_init();
  
  //Initialize Radio
  radio_init();
  
  //Initialize Variavle
  variable_init();
  
  //Initilize Control
  control_init();

  // Initialize ToF before the PWM interrupt starts. Failure leaves the
  // existing flight controller available with ToF marked unavailable.
  const bool tof_available = tof_setup();
  printf("#ToF init=%s\n", tof_available ? "OK" : "FAILED");
  
  //Initialize PWM
  //Start 400Hz Interval
  ESC_calib=0;
  pwm_init();

  while(start_wait)
  {
    start_wait--;
    printf("#Please wait %d[s]\r",start_wait); 
    sleep_ms(1000);
  }
  printf("\n");
 
  //マルチコア関連の設定
  sem_init(&sem, 0, 1);
  multicore_launch_core1(angle_control);  

  Arm_flag=1;
  
  while(1)
  {
    // Non-blocking: performs I2C traffic only when the sensor reports data ready.
    tof_poll();

    static uint32_t tof_debug_sequence = 0;
    static uint32_t last_tof_print_ms = 0;
    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (now_ms - last_tof_print_ms >= 200)
    {
      last_tof_print_ms = now_ms;
      ToFSnapshot tof{};
      tof_get_snapshot(tof, &tof_debug_sequence);
      const char *init = tof.init_state == ToFInitState::Ready ? "OK" : "FAILED";
      printf("TOF init=%s raw=%u mm filtered=%u mm status=%u fresh=%u age=%lu ms\n",
             init,
             static_cast<unsigned int>(tof.raw_mm),
             static_cast<unsigned int>(tof.filtered_mm),
             static_cast<unsigned int>(tof.range_status),
             tof.fresh ? 1U : 0U,
             static_cast<unsigned long>(tof.sample_age_ms));
    }

    //printf("Arm_flag:%d LockMode:%d\n",Arm_flag, LockMode);
    tight_loop_contents();
    while (Logoutputflag==1){
      log_output();
    }
  }

  return 0;
}
