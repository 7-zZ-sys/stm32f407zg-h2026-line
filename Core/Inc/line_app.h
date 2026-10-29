#ifndef LINE_APP_H
#define LINE_APP_H
#include "line_control.h"
typedef struct { LineConfig config; LineController controller; LineSample sample; LineOutput output; uint32_t rejected_commands; } LineFeedback;
void LineApp_Init(void);
void LineApp_Step(void);
void LineApp_Fault(LineFault fault);
bool LineApp_ProcessCommand(const char *command);
void LineApp_GetFeedback(LineFeedback *feedback);
#endif
