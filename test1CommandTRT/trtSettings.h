
//trtSettings.h
// include before trtkernel

#define MAXNBRTASKS 5
#define MAXNBRSEMAPHORES 8 // 3 sem (1-3) are needed for uart
#define MAXNBRMUTEX 1

#define PRESCALER 1024 // the actual value for timer1 prescalar
#define PRESCALEBITS 5 // the bits to be set in TCCR1b 

//#define F_CPU 10000000UL // clock frequency in Hz
#define TICKSPERSECOND F_CPU / PRESCALER

/* UART baud rate */
#define UART_BAUD  9600

// trace options
// use one port for realtime trace
// IF TRACE_PORT is undefined, then trace is turned off
#define TRACE_PORT PORTA
#define TRACE_DDR  DDRA
// trace format 
// bit 7 -- user event A
// bit 6 to 4 -- semaphore number signaled (up to 7)
// bit 3 -- user event B
// bit 2 to 0 -- task number executing (up to 7)
#ifdef TRACE_PORT
  #define TRACE_EVENT_A_ON do{TRACE_PORT = TRACE_PORT | (1<<7);}while(0)
  #define TRACE_EVENT_A_OFF do{TRACE_PORT = TRACE_PORT & ~(1<<7);}while(0)
  #define TRACE_EVENT_B_ON do{TRACE_PORT = TRACE_PORT | (1<<3);}while(0)
  #define TRACE_EVENT_B_OFF do{TRACE_PORT = TRACE_PORT & ~(1<<3);}while(0)
  // choose extended semaphore signal versus intant signal
  #define EXTEND_SEM
  //#define INSTANT_SEM
#endif
