#include "line_control.h"
#include "line_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static unsigned checks;
#define CHECK(x) do { bool ok_=(x); if (!ok_) fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); assert(ok_); ++checks; } while(0)
/* Fixed algorithm scenarios stay reproducible when the car's tuning changes.
 * test_line_app checks the actual startup defaults from line_config.h. */
static LineConfig cfg(void)
{
  LineConfig c=LineConfig_Default();
  c.kp=160; c.kd=3; c.test_speed=180; c.test_min_speed=140;
  c.lap_speed=400; c.curve_speed=350;
  c.active_high=1; c.reverse=0; c.marker_count=3; c.armed=true;
  return c;
}
static LineInput initial(const LineConfig *config)
{
  return (LineInput){.now_ms=1000,.imu_ready=true,.sample=Line_Decode(0x18,config)};
}
static LineOutput tick(LineController *c, const LineConfig *config, LineInput *in,
                       float mm, float yaw, uint8_t raw)
{
  in->now_ms+=10;
  in->distance_mm=mm; in->yaw_deg=yaw;
  in->sample=Line_Decode(raw,config);
  return Line_Step(c,config,in);
}

static bool fresh_start(LineController *c, const LineConfig *config, const LineInput *in, bool lap)
{
  Line_Init(c);
  return Line_Start(c,config,in,lap);
}

static void test_brief_ambiguous_input(void)
{
  LineConfig config=cfg();
  config.test_speed=260; config.test_min_speed=180;
  LineController c;
  LineInput in=initial(&config);
  CHECK(fresh_start(&c,&config,&in,false));
  LineOutput before=tick(&c,&config,&in,2,0,0x0C);
  LineOutput out=tick(&c,&config,&in,4,0,0x12);
  CHECK(out.speed_mm_s==180 && !out.hard_stop && c.speed_reason==LINE_SPEED_SAMPLE_HOLD);
  CHECK(out.angular_centirad_s==before.angular_centirad_s);
  CHECK(c.invalid_events==1 && c.last_invalid_mask==0x12 && c.lost);
  out=tick(&c,&config,&in,6,0,0x12);
  CHECK(out.speed_mm_s==180 && c.invalid_events==1);
  out=tick(&c,&config,&in,8,0,0x18);
  CHECK(out.speed_mm_s==180 && c.speed_reason==LINE_SPEED_RECOVERY && !c.lost);
  CHECK(out.angular_centirad_s==0); /* Reacquisition resets D history. */

  /* Repeated ambiguous frames do not extend the confirmation or loss deadline. */
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  out=tick(&c,&config,&in,2,0,0x12);
  CHECK(out.speed_mm_s==180);
  (void)tick(&c,&config,&in,4,0,0x81);
  out=tick(&c,&config,&in,6,0,0x12);
  CHECK(out.speed_mm_s==60 && c.speed_reason==LINE_SPEED_LOST && c.invalid_events==1);
  for (unsigned i=0;i<10;++i) out=tick(&c,&config,&in,8+(float)i,0,0x12);
  CHECK(out.hard_stop && c.fault==LINE_LOST);

  /* Full white is never held, including halfway through confirmation. */
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  (void)tick(&c,&config,&in,2,0,0x12);
  out=tick(&c,&config,&in,4,0,0);
  CHECK(out.speed_mm_s==60 && c.speed_reason==LINE_SPEED_LOST && c.last_invalid_mask==0);
  out=tick(&c,&config,&in,6,0,0x18);
  CHECK(out.speed_mm_s==60 && c.speed_reason==LINE_SPEED_RECOVERY);

  /* A large configured floor cannot raise an already reduced target. */
  config.test_speed=400; config.test_min_speed=400;
  float capped_speed=fminf(400.0f,LINE_AMBIGUOUS_MAX_SPEED);
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  out=tick(&c,&config,&in,2,0,0x12);
  CHECK(out.speed_mm_s==capped_speed);
  out=tick(&c,&config,&in,4,0,0x18);
  out=tick(&c,&config,&in,6,0,0x12);
  CHECK(out.speed_mm_s==capped_speed && c.invalid_events==2);
  config.test_speed=120;
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  out=tick(&c,&config,&in,2,0,0x12);
  CHECK(out.speed_mm_s==120);

  /* LAP is capped as well; IMU faults still override confirmation immediately. */
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,true));
  out=tick(&c,&config,&in,2,0,0x12);
  CHECK(out.speed_mm_s==fminf(350.0f,LINE_AMBIGUOUS_MAX_SPEED));
  in.imu_ready=false;
  out=tick(&c,&config,&in,4,0,0x12);
  CHECK(out.hard_stop && c.fault==LINE_IMU_FAULT);

  /* Confirmation and the unchanged 120 ms stop also survive tick wrap. */
  in=initial(&config); in.now_ms=UINT32_MAX-15U;
  CHECK(fresh_start(&c,&config,&in,false));
  (void)tick(&c,&config,&in,2,0,0x12);
  out=tick(&c,&config,&in,4,0,0x12);
  CHECK(out.speed_mm_s==120);
  out=tick(&c,&config,&in,6,0,0x12);
  CHECK(out.speed_mm_s==60);
  for (unsigned i=0;i<10;++i) out=tick(&c,&config,&in,8+(float)i,0,0x12);
  CHECK(out.hard_stop && c.fault==LINE_LOST);
}

int main(void)
{
  test_brief_ambiguous_input();
  LineConfig config=cfg();
  LineSample s=Line_Decode(0x18,&config);
  CHECK(s.valid && s.count==2 && fabsf(s.error)<0.00001f && !s.marker);
  s=Line_Decode(0x01,&config); CHECK(s.error==-1.0f);
  s=Line_Decode(0x80,&config); CHECK(s.error==1.0f);
  config.reverse=1; s=Line_Decode(0x01,&config); CHECK(s.error==1.0f);
  config=cfg(); config.active_high=0;
  s=Line_Decode(0xE7,&config); CHECK(s.mask==0x18 && s.valid);
  config=cfg(); CHECK(!Line_Decode(0,&config).valid);
  CHECK(!Line_Decode(0x81,&config).valid); /* Separate clusters must not average to center. */
  CHECK(Line_Decode(0x3C,&config).marker);
  config.marker_count=5; CHECK(!Line_Decode(0x3C,&config).marker); config=cfg();

  /* Observed masks: tolerate one interior dropout over at most four probes. */
  s=Line_Decode(40,&config);
  CHECK(s.valid && s.mask==40 && s.count==2 && s.groups==2 && !s.marker);
  CHECK(fabsf(s.error-1.0f/7.0f)<0.00001f);
  s=Line_Decode(26,&config);
  CHECK(s.valid && s.mask==26 && s.count==3 && s.groups==2 && !s.marker);
  CHECK(fabsf(s.error+5.0f/21.0f)<0.00001f);
  CHECK(!Line_Decode(0x12,&config).valid); /* Two interior misses. */
  CHECK(!Line_Decode(0x35,&config).valid); /* Three groups must remain invalid. */
  CHECK(!Line_Decode(0x3B,&config).valid); /* One hole, but span six is too broad. */
  CHECK(Line_Decode(0x3C,&config).marker); /* A real continuous stripe remains eligible. */
  /* Enumerate all 8-bit topologies and both wiring transforms. */
  bool topology_ok=true, polarity_ok=true, reversal_ok=true;
  for (unsigned raw=0;raw<256;raw++) {
    unsigned shape=raw;
    while (shape!=0 && (shape&1U)==0) shape>>=1;
    bool contiguous=shape!=0 && (shape&(shape+1U))==0;
    bool expected=contiguous || shape==5U || shape==11U || shape==13U;
    LineSample normal=Line_Decode((uint8_t)raw,&config);
    topology_ok=topology_ok && normal.valid==expected && normal.mask==raw;
    config.active_high=0;
    LineSample inverted=Line_Decode((uint8_t)~raw,&config);
    polarity_ok=polarity_ok && inverted.valid==normal.valid && inverted.mask==normal.mask &&
                inverted.marker==normal.marker && fabsf(inverted.error-normal.error)<0.00001f;
    config.active_high=1; config.reverse=1;
    LineSample reversed=Line_Decode((uint8_t)raw,&config);
    reversal_ok=reversal_ok && reversed.valid==normal.valid && reversed.marker==normal.marker &&
                fabsf(reversed.error+normal.error)<0.00001f;
    config.reverse=0;
  }
  CHECK(topology_ok); CHECK(polarity_ok); CHECK(reversal_ok);

  LineController c;
  LineInput in=initial(&config);
  Line_Init(&c); config.armed=false;
  CHECK(!Line_Start(&c,&config,&in,false) && c.fault==LINE_NOT_ARMED);
  config=cfg(); in.imu_ready=false;
  CHECK(!Line_Start(&c,&config,&in,false) && c.fault==LINE_IMU_FAULT);
  in=initial(&config); in.sample=Line_Decode(0,&config);
  CHECK(!Line_Start(&c,&config,&in,false) && c.fault==LINE_NO_LINE);
  in=initial(&config); in.sample=Line_Decode(0xFF,&config);
  CHECK(!Line_Start(&c,&config,&in,false) && c.fault==LINE_NO_LINE);
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  CHECK(!Line_Start(&c,&config,&in,true)); /* Active run cannot be restarted. */
  LineOutput out=tick(&c,&config,&in,2,0,0x01);
  CHECK(out.angular_centirad_s>0 && out.angular_centirad_s<=LINE_MAX_ANGULAR);
  out=tick(&c,&config,&in,4,0,0x80);
  CHECK(out.angular_centirad_s<0);
  out=tick(&c,&config,&in,6,0,0xFF); CHECK(c.state==LINE_RUNNING && !out.hard_stop);
  Line_Abort(&c,in.now_ms,LINE_USER_STOP); CHECK(c.state==LINE_IDLE);
  out=tick(&c,&config,&in,6,0,0x18); CHECK(out.speed_mm_s==0.0f);

  /* Broad center-overlapping bends still carry direction in continuous RUN. */
  config.test_speed=260;
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  out=tick(&c,&config,&in,2,0,0x1C);
  CHECK(out.angular_centirad_s>0 && c.last_error<0);
  CHECK(out.speed_mm_s<config.test_speed);
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  out=tick(&c,&config,&in,2,0,0x38);
  CHECK(out.angular_centirad_s<0 && c.last_error>0);

  /* LAP only suppresses stripe steering near its start/eligible finish. */
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,true));
  out=tick(&c,&config,&in,2,0,0x1C);
  CHECK(out.angular_centirad_s==0 && !c.left_start);
  (void)tick(&c,&config,&in,300,0,0x18);
  out=tick(&c,&config,&in,302,0,0x1C);
  CHECK(c.left_start && out.angular_centirad_s>0 && !c.marker_pending);
  config=cfg();

  /* Centering briefly inside a bend must not immediately request full speed. */
  config.test_speed=260;
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  out=tick(&c,&config,&in,2,0,0x80);
  float corner_speed=out.speed_mm_s;
  out=tick(&c,&config,&in,4,0,0x18);
  CHECK(out.speed_mm_s<=corner_speed+0.001f);
  for(unsigned i=0;i<10;i++) {
    out=tick(&c,&config,&in,6+(float)i,0,(i%2)==0 ? 0x80 : 0x18);
    CHECK(out.speed_mm_s<=140.001f);
  }
  unsigned recovery_checks=LINE_SPEED_RECOVER_MS/10U+
    (unsigned)ceilf((260.0f-corner_speed)/(2.0f*LINE_SPEED_RECOVER_ACCEL*0.01f))+2U;
  for(unsigned i=0;i<recovery_checks;i++) {
    float previous=out.speed_mm_s;
    out=tick(&c,&config,&in,20+(float)i,0,0x18);
    CHECK(out.speed_mm_s<=previous+LINE_SPEED_RECOVER_ACCEL*0.01f+0.001f);
  }
  CHECK(out.speed_mm_s>corner_speed && out.speed_mm_s<260);
  for(unsigned i=0;i<200;i++) out=tick(&c,&config,&in,120+(float)i,0,0x18);
  CHECK(fabsf(out.speed_mm_s-260)<0.001f);
  (void)tick(&c,&config,&in,322,0,0);
  out=tick(&c,&config,&in,324,0,0x18);
  CHECK(out.speed_mm_s<=60.0f+LINE_SPEED_RECOVER_ACCEL*0.01f+0.001f); /* No jump to full speed. */
  config=cfg();

  /* RUN's configurable floor changes curve speed without overriding loss protection. */
  config.test_speed=260; config.test_min_speed=200;
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  out=tick(&c,&config,&in,2,0,0x80);
  CHECK(out.speed_mm_s==200 && c.speed_reason==LINE_SPEED_CURVE);
  out=tick(&c,&config,&in,4,0,0x18);
  CHECK(out.speed_mm_s==200 && c.speed_reason==LINE_SPEED_RECOVERY);
  out=tick(&c,&config,&in,6,0,0);
  CHECK(out.speed_mm_s==60 && c.speed_reason==LINE_SPEED_LOST);
  config.test_speed=120; /* A larger stored floor must never accelerate past LSPD. */
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  out=tick(&c,&config,&in,2,0,0x80);
  CHECK(out.speed_mm_s==120 && c.speed_reason==LINE_SPEED_NORMAL);
  config.test_speed=260; config.test_min_speed=260;
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  out=tick(&c,&config,&in,2,0,0x80);
  CHECK(out.speed_mm_s==260 && c.speed_reason==LINE_SPEED_NORMAL);
  config.lap_speed=400; config.curve_speed=350; config.test_min_speed=60;
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,true));
  out=tick(&c,&config,&in,300,0,0x80);
  CHECK(out.speed_mm_s==350); /* LMIN does not change LAP's LCURVE. */
  config=cfg();

  /* At 120, a brief nonzero ambiguity holds speed; full white still requests 60. */
  config.test_speed=120;
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  out=tick(&c,&config,&in,2,0,0x80);
  CHECK(out.speed_mm_s==120 && c.speed_reason==LINE_SPEED_NORMAL && c.invalid_events==0);
  out=tick(&c,&config,&in,4,0,0x12); /* Two separated hits with two interior misses. */
  CHECK(out.speed_mm_s==120 && c.speed_reason==LINE_SPEED_SAMPLE_HOLD);
  CHECK(c.invalid_events==1 && c.last_invalid_mask==0x12);
  out=tick(&c,&config,&in,6,0,0x12);
  CHECK(c.invalid_events==1); /* Count episodes, not individual bad ticks. */
  out=tick(&c,&config,&in,8,0,0x18);
  CHECK(c.speed_reason==LINE_SPEED_NORMAL && out.speed_mm_s==120 && c.invalid_events==1 && c.last_invalid_mask==0x12);
  out=tick(&c,&config,&in,10,0,0);
  CHECK(c.invalid_events==2 && c.last_invalid_mask==0 && out.speed_mm_s==60);
  CHECK(fresh_start(&c,&config,&in,false)==false); /* Still no valid line. */
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  CHECK(c.invalid_events==0 && c.last_invalid_mask==0);
  out=tick(&c,&config,&in,2,0,40);
  CHECK(out.speed_mm_s==120 && out.angular_centirad_s<0 && c.invalid_events==0);
  out=tick(&c,&config,&in,4,0,26);
  CHECK(out.speed_mm_s==120 && out.angular_centirad_s>0 && c.invalid_events==0);
  config=cfg();

  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  for(unsigned i=0;i<12;i++) out=tick(&c,&config,&in,(float)i,0,0);
  CHECK(c.state==LINE_RUNNING);
  out=tick(&c,&config,&in,12,0,0);
  CHECK(c.state==LINE_FAULT && c.fault==LINE_LOST && out.hard_stop);
  out=tick(&c,&config,&in,14,0,0x18); CHECK(c.state==LINE_FAULT && out.speed_mm_s==0);

  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  (void)tick(&c,&config,&in,1,0,0);
  out=tick(&c,&config,&in,2,0,0x18); CHECK(!c.lost && c.state==LINE_RUNNING);

  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  in.imu_ready=false; out=tick(&c,&config,&in,1,0,0x18);
  CHECK(c.fault==LINE_IMU_FAULT && out.hard_stop);
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  in.now_ms+=31; out=Line_Step(&c,&config,&in);
  CHECK(c.fault==LINE_TIMING_FAULT && out.hard_stop);

  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  for(unsigned i=0;i<151;i++) out=tick(&c,&config,&in,0,0,0x18);
  CHECK(c.fault==LINE_STALLED && out.hard_stop);
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,false));
  out=tick(&c,&config,&in,-51,0,0x18); CHECK(c.fault==LINE_DISTANCE_FAULT);

  /* Start stripe is ignored, including sustained broad input near A. */
  in=initial(&config); in.sample=Line_Decode(0x3C,&config);
  CHECK(fresh_start(&c,&config,&in,true));
  for(unsigned i=0;i<20;i++) out=tick(&c,&config,&in,(float)(i*2),0,0x3C);
  CHECK(!c.left_start && c.state==LINE_RUNNING);
  out=tick(&c,&config,&in,300,0,0x18); CHECK(c.left_start);
  for(unsigned i=0;i<4;i++) out=tick(&c,&config,&in,400+(float)i,-10,0x3C);
  CHECK(c.state==LINE_RUNNING); /* Distance gate blocks early stripes. */
  for(unsigned i=0;i<4;i++) out=tick(&c,&config,&in,6000+(float)i,-20,0x3C);
  CHECK(c.state==LINE_RUNNING); /* Heading gate blocks an odometry jump. */
  CHECK(out.speed_mm_s<=LINE_APPROACH_SPEED);

  /* Build a clockwise lap with yaw wrap, then debounce the return stripe. */
  in=initial(&config); CHECK(fresh_start(&c,&config,&in,true));
  out=tick(&c,&config,&in,300,-90,0x18);
  out=tick(&c,&config,&in,2000,-179,0x18);
  out=tick(&c,&config,&in,3500,90,0x18);
  out=tick(&c,&config,&in,5900,0,0x18);
  CHECK(fabsf(c.turned_deg+360.0f)<0.001f);
  for(unsigned i=0;i<3;i++) out=tick(&c,&config,&in,6000+(float)i,0,26);
  CHECK(c.state==LINE_RUNNING && !c.marker_pending && c.stop_at_mm==0 && c.last_error<0);
  out=tick(&c,&config,&in,6100,0,0x3C); CHECK(c.state==LINE_RUNNING);
  out=tick(&c,&config,&in,6101,0,0x18); CHECK(!c.marker_pending);
  out=tick(&c,&config,&in,6110,0,0x3C);
  out=tick(&c,&config,&in,6111,0,0x3C); CHECK(c.state==LINE_RUNNING);
  out=tick(&c,&config,&in,6112,0,0x3C); CHECK(c.state==LINE_STOPPING && out.hard_stop);
  in.max_wheel_speed_mm_s=100;
  out=tick(&c,&config,&in,6115,0,0x18); CHECK(c.state==LINE_STOPPING);
  in.max_wheel_speed_mm_s=0;
  for(unsigned i=0;i<11;i++) out=tick(&c,&config,&in,6115,0,0x18);
  CHECK(c.state==LINE_DONE && out.hard_stop);
  uint32_t finished=c.elapsed_ms;
  out=tick(&c,&config,&in,6115,0,0x18); CHECK(c.elapsed_ms==finished);

  /* Positive offset delays the final stop; marker must be accepted first. */
  in=initial(&config); config.stop_offset_mm=100; CHECK(fresh_start(&c,&config,&in,true));
  (void)tick(&c,&config,&in,300,-90,0x18);
  (void)tick(&c,&config,&in,2000,-179,0x18);
  (void)tick(&c,&config,&in,3500,90,0x18);
  (void)tick(&c,&config,&in,6000,0,0x18);
  for(unsigned i=0;i<3;i++) out=tick(&c,&config,&in,6100+(float)i,0,0x3C);
  CHECK(c.state==LINE_RUNNING && c.stop_at_mm==6200);
  out=tick(&c,&config,&in,6199,0,0x18); CHECK(c.state==LINE_RUNNING);
  out=tick(&c,&config,&in,6200,0,0x18); CHECK(c.state==LINE_STOPPING);
  in.max_wheel_speed_mm_s=100;
  for(unsigned i=0;i<151;i++) out=tick(&c,&config,&in,6200,0,0x18);
  CHECK(c.fault==LINE_BRAKE_FAULT);

  config=cfg(); in=initial(&config); CHECK(fresh_start(&c,&config,&in,true));
  for(unsigned i=0;i<2001;i++) out=tick(&c,&config,&in,(float)(i*3),0,0x18);
  CHECK(c.fault==LINE_OVERTIME && out.hard_stop);

  /* Unsigned elapsed times remain valid across the 32-bit HAL tick wrap. */
  in=initial(&config); in.now_ms=UINT32_MAX-15;
  CHECK(fresh_start(&c,&config,&in,false));
  out=tick(&c,&config,&in,2,0,0x18);
  out=tick(&c,&config,&in,4,0,0x18);
  CHECK(c.state==LINE_RUNNING && c.elapsed_ms==20);
  printf("PASS: %u line-control assertions\n",checks);
  return 0;
}
