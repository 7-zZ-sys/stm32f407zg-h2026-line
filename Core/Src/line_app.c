#include "line_app.h"
#include "line_sensor.h"
#include "chassis_control.h"
#include "jy901s.h"
#include "main.h"
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static LineFeedback s;
static bool s_button_raw, s_button_stable;
static uint32_t s_button_change_ms;
static bool s_button_lap=true;

static LineInput input(void)
{
  LineInput in={.now_ms=HAL_GetTick(), .distance_mm=Chassis_GetDistanceMm()};
  JY901S_Feedback imu;
  ChassisFeedback wheels;
  JY901S_GetFeedback(&imu);
  Chassis_GetFeedback(&wheels);
  in.imu_ready=JY901S_IsFresh(&imu,in.now_ms);
  in.yaw_deg=imu.yaw_deg;
  in.yaw_rate_dps=imu.yaw_rate_dps;
  for (unsigned i=0; i<4; ++i)
    in.max_wheel_speed_mm_s=fmaxf(in.max_wheel_speed_mm_s,fabsf((float)wheels.measured_mm_s[i]));
  s.sample=Line_Decode(s.sample.raw,&s.config);
  in.sample=s.sample;
  return in;
}

void LineApp_Init(void)
{
  memset(&s,0,sizeof(s));
  s.config=LineConfig_Default();
  Line_Init(&s.controller);
  s_button_raw=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==GPIO_PIN_RESET;
  s_button_stable=s_button_raw; /* Holding START at boot does not start the vehicle. */
  s_button_change_ms=HAL_GetTick();
  Chassis_SetAuxCommandHandler(LineApp_ProcessCommand);
}

void LineApp_Fault(LineFault fault)
{
  Line_Abort(&s.controller,HAL_GetTick(),fault);
  s.output=(LineOutput){.hard_stop=true};
  Chassis_StopWithReason(fault==LINE_USER_STOP ? CHASSIS_STOP_USER : CHASSIS_STOP_LINE_FAULT);
}

static void start(bool lap)
{
  if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5)==GPIO_PIN_RESET) {
    LineApp_Fault(LINE_USER_STOP); return;
  }
  if (Line_Active(&s.controller)) { ++s.rejected_commands; return; }
  /* Clear manual HOLD and partial motion commands before autonomous ownership. */
  Chassis_StopWithReason(CHASSIS_STOP_HANDOVER);
  LineInput in=input();
  (void)Line_Start(&s.controller,&s.config,&in,lap);
}

void LineApp_Step(void)
{
  s.sample.raw=LineSensor_ReadRaw();
  Chassis_ProcessCommands(); /* Runs in this sole control owner, never in a UART IRQ. */
  uint32_t now=HAL_GetTick();
  bool raw=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==GPIO_PIN_RESET;
  if (raw!=s_button_raw) { s_button_raw=raw; s_button_change_ms=now; }
  bool pressed=false;
  if (raw!=s_button_stable && (uint32_t)(now-s_button_change_ms)>=20U) {
    s_button_stable=raw; pressed=raw;
  }
  if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5)==GPIO_PIN_RESET) {
    if (s.controller.fault!=LINE_USER_STOP || !Chassis_IsStopped()) LineApp_Fault(LINE_USER_STOP);
    return;
  }
  if (pressed) start(s_button_lap);
  LineInput in=input();
  if (Line_Active(&s.controller)) {
    s.output=Line_Step(&s.controller,&s.config,&in);
    if (s.output.hard_stop) {
      if (!Chassis_IsStopped()) Chassis_StopWithReason(s.controller.state==LINE_FAULT ?
          CHASSIS_STOP_LINE_FAULT : CHASSIS_STOP_FINISH);
    } else if (s.controller.state==LINE_RUNNING) {
      Chassis_SetTrackingVelocity((int16_t)lroundf(s.output.speed_mm_s),
                                 (int16_t)lroundf(s.output.angular_centirad_s));
    }
  } else {
    s.output=(LineOutput){0};
    if (s.controller.state==LINE_IDLE) Chassis_RefreshCommand();
  }
}

static bool value_of(const char *text, const char *key, float *value)
{
  size_t n=strlen(key);
  if (strncmp(text,key,n)!=0 || text[n]!='=') return false;
  char *end;
  *value=strtof(text+n+1,&end);
  return end!=text+n+1 && *end=='\0' && isfinite(*value);
}

bool LineApp_ProcessCommand(const char *command)
{
  char cmd[40];
  size_t n=strlen(command);
  if (n>=sizeof(cmd)) return false;
  for (size_t i=0; i<=n; ++i) cmd[i]=(char)toupper((unsigned char)command[i]);
  if (strcmp(cmd,"STOP")==0 || strcmp(cmd,"ZERO")==0) { LineApp_Fault(LINE_USER_STOP); return true; }
  float value;
  bool run=strncmp(cmd,"RUN=",4)==0, lap=strncmp(cmd,"LAP=",4)==0;
  if (run || lap) {
    if (!value_of(cmd,run?"RUN":"LAP",&value) || (value!=0.0f && value!=1.0f)) ++s.rejected_commands;
    else if (value==0.0f) LineApp_Fault(LINE_USER_STOP);
    else start(lap);
    return true;
  }
  /* Autonomous tests accept STOP only. USB/wireless cannot fight line ownership. */
  if (Line_Active(&s.controller)) { ++s.rejected_commands; return true; }
  if (cmd[0]!='L') return false;
  bool accepted=false;
  if (value_of(cmd,"LREADY",&value) && (value==0.0f || value==1.0f)) {
    s.config.armed=(value==1.0f); accepted=true;
  } else if (value_of(cmd,"LPOL",&value) && (value==0.0f || value==1.0f)) {
    s.config.active_high=(uint8_t)value; s.config.armed=false; accepted=true;
  } else if (value_of(cmd,"LREV",&value) && (value==0.0f || value==1.0f)) {
    s.config.reverse=(uint8_t)value; s.config.armed=false; accepted=true;
  } else if (value_of(cmd,"LMODE",&value) && (value==0.0f || value==1.0f)) {
    s_button_lap=(value==1.0f); accepted=true;
  } else if (value_of(cmd,"LKP",&value) && value>=0.0f && value<=600.0f) {
    s.config.kp=value; accepted=true;
  } else if (value_of(cmd,"LKD",&value) && value>=0.0f && value<=30.0f) {
    s.config.kd=value; accepted=true;
  } else if (value_of(cmd,"LSPD",&value) && value>=60.0f && value<=400.0f) {
    s.config.test_speed=value; accepted=true;
  } else if (value_of(cmd,"LMIN",&value) && value>=60.0f && value<=400.0f) {
    s.config.test_min_speed=value; accepted=true;
  } else if (value_of(cmd,"LLAP",&value) && value>=100.0f && value<=600.0f) {
    s.config.lap_speed=value; s.config.curve_speed=fminf(s.config.curve_speed,value); accepted=true;
  } else if (value_of(cmd,"LCURVE",&value) && value>=60.0f && value<=s.config.lap_speed) {
    s.config.curve_speed=value; accepted=true;
  } else if (value_of(cmd,"LSTOP",&value) && value>=0.0f && value<=300.0f) {
    s.config.stop_offset_mm=value; accepted=true;
  } else if (value_of(cmd,"LMARK",&value) && value>=3.0f && value<=8.0f && value==floorf(value)) {
    s.config.marker_count=(uint8_t)value; accepted=true;
  } else if (value_of(cmd,"LRESET",&value) && value==1.0f) {
    Chassis_EmergencyStop(); Line_Init(&s.controller); accepted=true;
  }
  /* LACC/LJRK belong to the retained chassis parser. */
  else if (strncmp(cmd,"LACC=",5)==0 || strncmp(cmd,"LJRK=",5)==0) return false;
  if (!accepted) ++s.rejected_commands;
  return true;
}

void LineApp_GetFeedback(LineFeedback *feedback)
{
  if (feedback==NULL) return;
  uint32_t saved=__get_PRIMASK();
  __disable_irq(); *feedback=s;
  if (saved==0U) __enable_irq();
}
