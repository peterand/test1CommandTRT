//------------------------------------------------------------------------------
// From http://www.control.lth.se/%7Eanton/tinyrealtime/
// written by Dan Henriksson, Anton Cervin
// dan@control.lth.se, anton@control.lth.se
//
// Modified for the Mega644 by Bruce Land Jan 2009
// -- Changed register names to mega644 regs
// -- Changed clock rate to 20 MHz
// -- Added one byte to stack init section in trtCreateTask
//
// Modified to compile for both mega644 and mega1284 by Peter Andersson Oct 2014
// -- Minor changes to avoid warnings
//------------------------------------------------------------------------------

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define TERMINATED 0
#define READYQ 1
#define TIMEQ 2
#define WAIT_OFFSET 2

#define SECONDS2TICKS(T) ((uint32_t)((T)*TICKSPERSECOND))

#define lo8(X) ((uint8_t)((uint16_t)(X)))
#define hi8(X) ((uint8_t)((uint16_t)(X) >> 8))



/******************* KERNEL DATA STRUCTURES ************************/

struct task {
  uint8_t spl;       // Stack pointer lo8
  uint8_t sph;       // Stack pointer hi8
  uint32_t release;
  uint32_t deadline;
  uint8_t state;     // 0=terminated, 1=readyQ, 2=timeQ
                     // 3=waiting for Sem1, 4=waiting for Sem2, etc.
  uint16_t stack_bottom; // address of min stack ptr
};

struct kernel {
  uint8_t nbrOfTasks; // number of tasks created so far
  uint8_t running;
  struct task tasks[MAXNBRTASKS+1]; // +1 for the idle task
  uint8_t semaphores[MAXNBRSEMAPHORES]; // counters for semaphores
  uint8_t *memptr; // pointer to free memory
  uint32_t cycles;  // number of major cycles since system start
  uint32_t nextHit; // next kernel wake-up time
} kernel;

/******************* CLOCK INTERRUPT HANDLER ************************/

/**
 * Clock interrupt handler executing the kernel
 */
ISR(TIMER1_COMPA_vect, ISR_NAKED) {

  uint8_t running, oldrunning;
  struct task *t;
  uint8_t i;
  uint32_t now;
  uint32_t nextHit;
  int32_t timeleft;
 
 asm("push r1");
 asm("push r0");
 asm("in r0, 0x3f");	/*SREG*/
 asm("push r0");
 asm("eor r1, r1");
 #if (__AVR_ARCH__) == 51
 asm("in r0, 0x3b");	/*RAMPZ*/
 asm("push r0");
 #endif
 asm("push r2");
 asm("push r3");
 asm("push r4");
 asm("push r5");
 asm("push r6");
 asm("push r7");
 asm("push r8");
 asm("push r9");
 asm("push r10");
 asm("push r11");
 asm("push r12");
 asm("push r13");
 asm("push r14");
 asm("push r15");
 asm("push r16");
 asm("push r17");
 asm("push r18");
 asm("push r19");
 asm("push r20");
 asm("push r21");
 asm("push r22");
 asm("push r23");
 asm("push r24");
 asm("push r25");
 asm("push r26");
 asm("push r27");
 asm("push r28");
 asm("push r29");
 asm("push r30");
 asm("push r31");
  
  TIMSK1 = 0 ; //&= ~(1<<OCIE1A); // turn off output compare 1A ISR
  //PORTC = ~PORTC ;
  nextHit = 0x7FFFFFFF;
  oldrunning = kernel.running;
  running = 0;

  if (TIFR1 & (1<<TOV1)) {
    ++kernel.cycles;
    TIFR1 |= (1<<TOV1) ;
  }

  // Read clock

  now = (kernel.cycles << 16) + TCNT1;

  // Release tasks from TimeQ and determine new running task

  for (i=1; i <= kernel.nbrOfTasks; i++) {
    t = &kernel.tasks[i];
    if (t->state == TIMEQ) {
      if (t->release <= now) {
	t->state = READYQ;
      } else if (t->release < nextHit) {
	nextHit = t->release;
      }
    }
    if (t->state == READYQ) {
      if (t->deadline < kernel.tasks[running].deadline) {
	running = i;
      }
    }
  }

  #ifdef TRACE_PORT
  		// clear low 3 bits and set them to running task
  		TRACE_PORT = (TRACE_PORT & 0xf8) | running ;	
  #endif

  if (running != oldrunning) { // perform context switch?

    // store old context
    t = &kernel.tasks[oldrunning];
    t->spl = SPL;
    t->sph = SPH;

    // load new context
    t = &kernel.tasks[running];
    SPH = t->sph;
    SPL = t->spl;

    kernel.running = running;
  }

  kernel.nextHit = nextHit;  

  now = (kernel.cycles << 16) + TCNT1;
  timeleft = (int32_t)nextHit - (int32_t)now;
  if (timeleft < 4) {
    timeleft = 4;
  }

  if ((unsigned long)TCNT1 + timeleft < 65536) {
    OCR1A = TCNT1 + timeleft;
  } else if (TCNT1 < 65536 - 4) {
    OCR1A = 0x0000;
  } else {
    OCR1A = 4;
  }

  TIMSK1 = (1<<OCIE1A);
  
  asm("pop r31");
  asm("pop r30");
  asm("pop r29");
  asm("pop r28");
  asm("pop r27");
  asm("pop r26");
  asm("pop r25");
  asm("pop r24");
  asm("pop r23");
  asm("pop r22");
  asm("pop r21");
  asm("pop r20");
  asm("pop r19");
  asm("pop r18");
  asm("pop r17");
  asm("pop r16");
  asm("pop r15");
  asm("pop r14");
  asm("pop r13");
  asm("pop r12");
  asm("pop r11");
  asm("pop r10");
  asm("pop r9");
  asm("pop r8");
  asm("pop r7");
  asm("pop r6");
  asm("pop r5");
  asm("pop r4");
  asm("pop r3");
  asm("pop r2");
  #if (__AVR_ARCH__) == 51
  asm("pop r0");
  asm("out 0x3b, r0");	/*RAMPZ*/
  #endif
  asm("pop r0");
  asm("out 0x3f, r0");	/*SREG*/
  asm("pop r0");
  asm("pop r1");
  asm("reti");
}

/********************************** API ************************************/

void trtInitKernel(int idlestack) {

  /* Set up timer 1 */
  TCNT1 = 0x0000;        /* reset counter 1 */
  TCCR1A = 0x00;         /* normal operation */
  TCCR1B = PRESCALEBITS; /* prescaler = 1024 */
  TIMSK1 = (1<<OCIE1A);  // turn on compare match ISR

  kernel.memptr = (void*)(RAMEND - idlestack);
  kernel.nbrOfTasks = 0;
  kernel.running = 0;

  kernel.cycles = 0x0000;
  kernel.nextHit = 0x7FFFFFFF;

  // Initialize idle task (task 0)
  kernel.tasks[0].deadline = 0x7FFFFFFF;
  kernel.tasks[0].release = 0x00000000;
  
  #ifdef TRACE_PORT
  		// init the trace port to output
  		TRACE_DDR = 0xff ;
		TRACE_PORT = 0 ;	
  #endif

  sei(); /* set enabled interrupts */
}
//void trtCreateTask(void (void*), uint16_t, uint32_t, uint32_t, void *) __attribute__((noreturn));
void trtCreateTask(void (*fun)(void*), uint16_t stacksize, uint32_t release, uint32_t deadline, void *args)
{
	uint8_t *sp;
	struct task *t;
	uint8_t *i;
	uint8_t j;
	cli(); // turn off interrupts

	++kernel.nbrOfTasks;

	sp = kernel.memptr;
	kernel.memptr -= stacksize;  // decrease free mem ptr
	// initialize stack
	*sp-- = lo8(fun);       /* PC(lo) */
	*sp-- = hi8(fun);       /* PC(hi) */

#if (__AVR_ARCH__) == 51
	*sp-- = 0x00;			/* space for RAMPZ */ 
#endif
	*sp-- = 0;				/* r1 */
//	*sp-- = 1;				/* debug */
	for (j=0; j<24; j++)    
		*sp-- = 0x00;		/* SREG,r0,r2-r23 */
//		*sp-- = j;			/* debug */

	/* Pointer to input arguments are stored in r24 and r25*/
	*sp-- = lo8(args);
	*sp-- = hi8(args);

	for (j=26; j<32; j++)
		*sp-- = 0x00;		/* r26-r31 */
//		*sp-- = j;			/* debug */
  
  t = &kernel.tasks[kernel.nbrOfTasks];
  
  t->release = release;
  t->deadline = deadline;
  t->state = TIMEQ;

  t->spl = lo8(sp);       //store stack pointer
  t->sph = hi8(sp);

  t->stack_bottom = (uint16_t)kernel.memptr+1 ; // store stk min for this task
  
  // init the stack so that the stk min function can scan it
  for (i=(uint8_t*)t->stack_bottom; i<sp+1; i++)
	{
		*i = 0xaa ;
	}

  // call interrupt handler to schedule
  TIMER1_COMPA_vect();
}

void trtCreateSemaphore(uint8_t semnbr, uint8_t initVal) {

  cli(); // turn off interrupts

  kernel.semaphores[semnbr-1] = initVal;
  
  sei(); // set enabled interrupts;
}

void trtWait(uint8_t semnbr) {

  struct task *t;
  uint8_t *s;

  t = &kernel.tasks[kernel.running];

  cli(); // disable interrupts

  s = &kernel.semaphores[semnbr-1];
  if ((*s) > 0) {
    (*s)--;
  } else {

    t->state = semnbr + WAIT_OFFSET; // waiting for Sem#semnbr
    // call interrupt handler to schedule
	TIMER1_COMPA_vect();
  }

  sei(); // reenable interrupts
}

void trtSignal(uint8_t semnbr) {

  uint8_t i;
  struct task *t;
  uint32_t minDeadline = 0xFFFFFFFF;
  uint8_t taskToReadyQ = 0;

  cli(); // disable interrupts
  #ifdef TRACE_PORT
  		// put the semaphore number on bits 6 to 4 of the trace
		TRACE_PORT = (TRACE_PORT & 0x8f) | (semnbr<<4) ;	
  #endif

  for (i=1; i <= kernel.nbrOfTasks; i++) {
    t = &kernel.tasks[i];
    if (t->state == (semnbr + WAIT_OFFSET)) {
      if (t->deadline <= minDeadline) {
	taskToReadyQ = i;
	minDeadline = t->deadline;
      }
    }
  }

  #ifdef INSTANT_SEM
  		// clear the semaphore trace befor the context switch
		TRACE_PORT = (TRACE_PORT & 0x8f)  ;	
  #endif

  if (taskToReadyQ == 0) {
    kernel.semaphores[semnbr-1]++;
  } else {
    kernel.tasks[taskToReadyQ].state = READYQ; // make task ready
    // call interrupt handler to schedule
	TIMER1_COMPA_vect();
  }

  #ifdef EXTEND_SEM
  		// clear the semaphore trace after the context switch
		TRACE_PORT = (TRACE_PORT & 0x8f)  ;	
  #endif

  sei(); // reenable interrupts
}

uint32_t trtCurrentTime(void) {

  return (((uint32_t)kernel.cycles << 16) + (uint32_t)TCNT1);
}


//void trtSleepUntil(uint32_t, uint32_t) __attribute__((noreturn));
void trtSleepUntil(uint32_t release, uint32_t deadline) {

  struct task *t;

  t = &kernel.tasks[kernel.running];

  cli(); // turn off interrupts

  t->state = TIMEQ;
  t->release = release;
  t->deadline = deadline;
  
  // call interrupt handler to schedule
  TIMER1_COMPA_vect();
}

uint32_t trtGetRelease(void) {
  return kernel.tasks[kernel.running].release;
}

uint32_t trtGetDeadline(void) {
  return kernel.tasks[kernel.running].deadline;
}

//void trtTerminate(void) __attribute__((noreturn));
void trtTerminate(void) {

  cli();

  kernel.tasks[kernel.running].state = TERMINATED;

  // call interrupt handler to schedule
  TIMER1_COMPA_vect();
}

// --- added by bruce land --------------
uint8_t trtAccept(uint8_t semnbr) {

  //struct task *t;
  uint8_t *s;
  uint8_t temp ;
  //t = &kernel.tasks[kernel.running];

  cli(); // disable interrupts

  s = &kernel.semaphores[semnbr-1];
  temp = *s ;
  if ((*s) > 0) {
    (*s)--;
  } 
  sei(); // reenable interrupts
  return temp ;
}
