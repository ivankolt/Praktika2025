#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

typedef struct {
  UART_HandleTypeDef* uart;
  uint32_t (*get_millis)(void);
  char line_buf[96];
  uint16_t line_len;
  int16_t left_cmd;
  int16_t right_cmd;
  enum { SEQ_IDLE = 0, SEQ_FWD, SEQ_ROT, SEQ_REV } seq_state;
  uint32_t step_deadline_ms;
} uartseq_t;

static uartseq_t g_ctx;

static int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi){ if(v<lo) return lo; if(v>hi) return hi; return (int16_t)v; }
static void set_stop(void){ g_ctx.left_cmd=0; g_ctx.right_cmd=0; }
static void set_forward(float mps){ int16_t s=clamp_i16((int32_t)(mps*1000.0f),-1000,1000); g_ctx.left_cmd=s; g_ctx.right_cmd=s; }
static void set_reverse(float mps){ int16_t s=clamp_i16((int32_t)(-mps*1000.0f),-1000,1000); g_ctx.left_cmd=s; g_ctx.right_cmd=s; }
static void set_rotate(float radps){ int16_t mag=500; if(radps>=0){ g_ctx.left_cmd=-mag; g_ctx.right_cmd=mag; } else { g_ctx.left_cmd=mag; g_ctx.right_cmd=-mag; } }
static void start_sequence(void){ g_ctx.seq_state=SEQ_FWD; set_forward(0.20f); g_ctx.step_deadline_ms=g_ctx.get_millis()+3000u; }
static void advance_sequence(void){ switch(g_ctx.seq_state){ case SEQ_FWD: g_ctx.seq_state=SEQ_ROT; set_rotate(+0.5f); g_ctx.step_deadline_ms=g_ctx.get_millis()+2000u; break; case SEQ_ROT: g_ctx.seq_state=SEQ_REV; set_reverse(0.20f); g_ctx.step_deadline_ms=g_ctx.get_millis()+2000u; break; default: g_ctx.seq_state=SEQ_IDLE; set_stop(); break; } }

static void handle_line(char* line){
  size_t n=strlen(line); while(n&&(line[n-1]=='\r'||line[n-1]=='\n'||line[n-1]==' ')){ line[--n]='\0'; }
  while(*line==' ') line++;
  if(!*line) return;
  if(strcmp(line,"sequence")==0){ start_sequence(); return; }
  if(strcmp(line,"stop")==0){ g_ctx.seq_state=SEQ_IDLE; set_stop(); return; }
  char* cmd=strtok(line," "); char* a1=strtok(NULL," "); char* a2=strtok(NULL," "); if(!cmd) return;
  if(strcmp(cmd,"fwd")==0 && a1 && a2){ float mps=strtof(a1,NULL); uint32_t dur=(uint32_t)strtoul(a2,NULL,10); g_ctx.seq_state=SEQ_IDLE; set_forward(mps); g_ctx.step_deadline_ms=g_ctx.get_millis()+dur; g_ctx.seq_state=SEQ_ROT; return; }
  if(strcmp(cmd,"rev")==0 && a1 && a2){ float mps=strtof(a1,NULL); uint32_t dur=(uint32_t)strtoul(a2,NULL,10); g_ctx.seq_state=SEQ_IDLE; set_reverse(fabsf(mps)); g_ctx.step_deadline_ms=g_ctx.get_millis()+dur; g_ctx.seq_state=SEQ_ROT; return; }
  if(strcmp(cmd,"rot")==0 && a1 && a2){ float w=strtof(a1,NULL); uint32_t dur=(uint32_t)strtoul(a2,NULL,10); g_ctx.seq_state=SEQ_IDLE; set_rotate(w); g_ctx.step_deadline_ms=g_ctx.get_millis()+dur; g_ctx.seq_state=SEQ_ROT; return; }
}

void UARTSEQ_Init(UART_HandleTypeDef* huart3, uint32_t (*millis_cb)(void)){
  memset(&g_ctx,0,sizeof(g_ctx)); g_ctx.uart=huart3; g_ctx.get_millis=millis_cb; set_stop();
}

void UARTSEQ_OnRxByte(uint8_t b){
  if(b=='\n' || b=='\r'){ if(g_ctx.line_len>0){ g_ctx.line_buf[g_ctx.line_len]='\0'; handle_line(g_ctx.line_buf); g_ctx.line_len=0; } return; }
  if(g_ctx.line_len+1<sizeof(g_ctx.line_buf)) g_ctx.line_buf[g_ctx.line_len++]=(char)b; else g_ctx.line_len=0;
}

void UARTSEQ_Tick(void){ uint32_t now=g_ctx.get_millis?g_ctx.get_millis():0; if(g_ctx.seq_state!=SEQ_IDLE && (int32_t)(now - g_ctx.step_deadline_ms)>=0){ advance_sequence(); } }

void UARTSEQ_GetCommand(int16_t* left, int16_t* right){ if(left) *left=g_ctx.left_cmd; if(right) *right=g_ctx.right_cmd; }


