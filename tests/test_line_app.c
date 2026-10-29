#include "line_app.h"
#include "line_config.h"
#include "chassis_control.h"
#include "jy901s.h"
#include "main.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

GPIO_TypeDef test_gpio_b,test_gpio_e;
TestDWT test_dwt;
uint32_t SystemCoreClock=168000000,test_irq_mask;
static uint32_t now=1000,drive_calls,stops;
static bool start_down,stop_down,imu_fresh=true,stopped=true;
static float distance;
static uint8_t sensor=0x18;
static ChassisAuxCommandHandler handler;
static const char *queued;

uint32_t HAL_GetTick(void){return now;}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port,uint16_t pin)
{
  assert(port==GPIOE);
  if(pin==GPIO_PIN_4)return start_down?GPIO_PIN_RESET:GPIO_PIN_SET;
  assert(pin==GPIO_PIN_5);return stop_down?GPIO_PIN_RESET:GPIO_PIN_SET;
}
uint8_t LineSensor_ReadRaw(void){return sensor;}
void Chassis_SetAuxCommandHandler(ChassisAuxCommandHandler h){handler=h;}
void Chassis_ProcessCommands(void){if(queued){const char *command=queued;queued=NULL;(void)handler(command);}}
void Chassis_GetFeedback(ChassisFeedback *f){memset(f,0,sizeof(*f));}
float Chassis_GetDistanceMm(void){return distance;}
void Chassis_StopWithReason(ChassisStopReason reason){(void)reason;stopped=true;++stops;}
void Chassis_EmergencyStop(void){Chassis_StopWithReason(CHASSIS_STOP_USER);}
bool Chassis_IsStopped(void){return stopped;}
void Chassis_RefreshCommand(void){}
void Chassis_SetTrackingVelocity(int16_t v,int16_t w){(void)w;assert(v>=0);++drive_calls;stopped=false;}
void JY901S_GetFeedback(JY901S_Feedback *f){memset(f,0,sizeof(*f));f->status=JY901S_READY;}
bool JY901S_IsFresh(const JY901S_Feedback *f,uint32_t tick){(void)f;(void)tick;return imu_fresh;}
static LineFeedback snapshot(void){LineFeedback f;LineApp_GetFeedback(&f);assert(test_irq_mask==0);return f;}
static void step(void){now+=10;distance+=2;LineApp_Step();}
static void command(const char *s){queued=s;step();}

int main(void)
{
  LineApp_Init();step();assert(stopped && snapshot().controller.state==LINE_IDLE);
  command("RUN=1");assert(snapshot().controller.fault==LINE_NOT_ARMED && stopped);
  command("LREADY=1");command("RUN=1");assert(!stopped && snapshot().controller.state==LINE_RUNNING);
  uint32_t count=drive_calls;
  command("V=600");assert(snapshot().rejected_commands==1 && drive_calls==count+1);
  command("LKP=200");assert(snapshot().config.kp==LINE_DEFAULT_KP);
  command("LMIN=200");assert(snapshot().config.test_min_speed==LINE_DEFAULT_TEST_MIN_SPEED);
  command("STOP");assert(stopped && snapshot().controller.state==LINE_IDLE);
  command("LSPD=260");command("LMIN=200");
  assert(snapshot().config.test_speed==260 && snapshot().config.test_min_speed==200);
  uint32_t rejected=snapshot().rejected_commands;
  command("LMIN=59");command("LMIN=401");command("LMIN=nan");
  assert(snapshot().config.test_min_speed==200 && snapshot().rejected_commands==rejected+3);
  command("LSPD=120");assert(snapshot().config.test_min_speed==200);
  command("LMIN=140");command("LSPD=180");
  command("LKP=200");assert(snapshot().config.kp==200);
  command("LPOL=0");assert(!snapshot().config.armed);
  command("LREADY=1");command("RUN=1"); /* Wrong polarity creates separated groups. */
  assert(stopped && snapshot().controller.fault==LINE_NO_LINE);
  command("LPOL=1");command("LREADY=1");command("LRESET=1");
  command("LSPD=nan");command("LMARK=3.5");assert(snapshot().config.test_speed==180 && snapshot().config.marker_count==3);
  stop_down=true;command("RUN=1");assert(stopped);
  stop_down=false;command("RUN=1");assert(!stopped);
  imu_fresh=false;step();assert(stopped && snapshot().controller.fault==LINE_IMU_FAULT);
  imu_fresh=true;step();assert(stopped); /* Data recovery must not restart a faulted task. */

  /* A START key held at boot is inhibited until released and pressed again. */
  start_down=true;LineApp_Init();command("LREADY=1");
  for(unsigned i=0;i<5;i++) {step();}
  assert(stopped);
  start_down=false;for(unsigned i=0;i<4;i++)step();
  start_down=true;step();assert(stopped);step();assert(stopped);step();assert(!stopped);
  stop_down=true;step();assert(stopped);
  count=drive_calls;for(unsigned i=0;i<5;i++)step();assert(drive_calls==count);
  stop_down=false;start_down=false;for(unsigned i=0;i<4;i++)step();
  command("LRESET=1");command("RUN=1");assert(!stopped);
  sensor=0x12;step();
  assert(!stopped && snapshot().controller.speed_reason==LINE_SPEED_SAMPLE_HOLD);
  float expected_hold=fminf(LINE_AMBIGUOUS_MAX_SPEED,
      fminf(LINE_DEFAULT_TEST_SPEED,LINE_DEFAULT_TEST_MIN_SPEED));
  assert(snapshot().output.speed_mm_s==expected_hold && snapshot().controller.invalid_events==1);
  sensor=0x18;step();
  assert(!stopped && snapshot().controller.speed_reason==LINE_SPEED_RECOVERY);
  sensor=0;
  for(unsigned i=0;i<13;i++) {step();}
  assert(stopped && snapshot().controller.fault==LINE_LOST);
  sensor=0x18;step();assert(stopped);
  printf("PASS: line application commands, keys, ownership, IMU loss and line loss (%u stops)\n",stops);
  return 0;
}
