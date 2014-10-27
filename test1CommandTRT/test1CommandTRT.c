// test TRT with two led blinking tasks
// and two UART tasks
// and a command shell
//---------------------------------------
// Modified by Peter Andersson Oct 2014
// -- Check UART input for reasonability
// -- Minor changes to avoid warnings
//---------------------------------------

#include <avr/sleep.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h> // [LONG|INT][MIN|MAX]
#include <errno.h>  // errno
#include "trtSettings.h"
#include "trtkernelTraceDump.c"
#include "trtQuery.c"

// serial communication library
// Don't mess with the semaphores
#define SEM_RX_ISR_SIGNAL 1
#define SEM_STRING_DONE 2 // user hit <enter>
#include "trtUart.h"
#include "trtUart.c"

// bit twiddling 
/*
#define READ(U, N) (((U) >> (N)) & 1u)
#define SET(U, N) ((void)((U) |= 1u << (N)))
#define CLR(U, N) ((void)((U) &= ~(1u << (N))))
#define FLIP(U, N) ((void)((U) ^= 1u << (N)))
*/
// UART file descriptor
// putchar and getchar are in uart.c
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

// allow task2 to control task1
#define SEM_TASK1_WAIT 3

// two semaphores to protect message --
// sender must wait until message is received 
// -- init to 1 becuase don't need to wait for first message
// receiver must wait until message is sent
#define SEM_TX_WAIT  4
#define SEM_RX_WAIT  5

// semaphore to protect shared variable
#define SEM_SHARED 7

// the usual
#define begin {
#define end }

//--- shell getchar,  putchar and other funs ----------------------
// signal the shell to execute
#define SEM_SHELL 8
// stall the system clock, set the signal variable, signal sem
#define signal_shell(a) \
	do{  \
	timer1_state = TCCR1B; TCCR1B = 0; \
	timer0_state = TCCR0B ; TCCR0B = 0; \
	timer2_state = TCCR2B ; TCCR2B = 0; \
	trt_shell_sig=(a); \
	trtSignal(SEM_SHELL);}while(0)
char trt_shell_sig ;
char timer1_state, timer0_state, timer2_state ;

//--- getchar ---
char shell_getchar(void) begin
	//while (~READ(UCSR0A, RXC0)) {} ;
	loop_until_bit_is_set(UCSR0A, RXC0);
	return UDR0 ;
end

//--- putchar ---
void shell_putchar(char c) begin
	//while (~READ(UCSR0A, UDRE0)) {} ;
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c ;
end

FILE shell_uart = FDEV_SETUP_STREAM(shell_putchar, shell_getchar, _FDEV_SETUP_RW);

//--- Shell --------------------------------------------------------
void shell(void* args) 
begin
	
//	char timer0_state=0, timer2_state=0 ; //timer1_state,
	uint8_t i, data, temp1, temp2;
	int temp3, temp4;
	char cmd[20] ;
	uint8_t current_char ;
	uint8_t in_count;
	uint8_t *j ;
		
	while(1)
	begin
		// wait for a signal
		trtWait(SEM_SHELL);
		// stop scheduler
		//timer1_state = TCCR1B ; TCCR1B = 0;
		//timer0_state = TCCR0B ; TCCR0B = 0;
		//timer2_state = TCCR2B ; TCCR2B = 0;
		cli();
		
		// shell prompt and signal info
		fprintf(&shell_uart, "\r\ntrt> signal=%d\r\n", trt_shell_sig);
		
		cmd[0] = 0 ; // init string to null

		while (!(cmd[0] == 'g'))
		begin
			// get next shell cmd
			fprintf(&shell_uart, "trt>");
			//fscanf(&shell_uart, "%s", &cmd);
			//get the next shell command
	    	//handles backspace, <enter>
			
			in_count=0;
	        while ( (current_char = shell_getchar()) != '\r' )  //<enter>
	        begin    
	        	if ((current_char == 0x08) && (in_count!=0))	//backspace
				begin
					shell_putchar(current_char);
					shell_putchar(' ');
					shell_putchar(current_char); 
	        		--in_count;
				end
	        	if (current_char != 0x08)
				begin
					shell_putchar(current_char);   
	        		cmd[in_count++]=current_char; 
				end	
	        end    
	    	cmd[in_count] = 0;	//terminate the string
	        shell_putchar('\r');  	// emit carriage return
	        shell_putchar('\n'); 	// line feed makes output nicer
			
			// -------------------
			if (cmd[0] == 't')  
			{
				temp3 = kernel.nbrOfTasks + 1;	
				sscanf(cmd, "%c%d",&temp1, &temp3);
				temp2 = temp3;
				if (temp2 > kernel.nbrOfTasks)
				{
					fprintf(&shell_uart, "enter a tasknumber 0..%d\r\n",kernel.nbrOfTasks);
				}
				else
				{
					if (temp2==0)
					{
						temp1 = 1;
						temp2 = kernel.nbrOfTasks;
					}
					else
					{
						temp1 = temp2;
					}
					fprintf(&shell_uart, "# state     relT   deadT stkfree stkfreeMin\r\n") ;
					//loop thru all tasks
					for (i=temp1; i <= temp2; i++) 
						fprintf(&shell_uart, "%1d %4d %8ld %8ld %5d %5d\r\n", 
							i, 
							trtTaskState(i), 
							trtTaskRelease(i) - trtCurrentTime(), 
							trtTaskDeadline(i)  - trtCurrentTime(),
							trtTaskStack(i) - trtTaskStackBottom(i),
							trtTaskStackFreeMin(i));
					
				}
			} // (cmd[0] == 't') 
			
			// -------------------
			if (cmd[0] == 's') 
			begin
				temp3 = MAXNBRSEMAPHORES + 1;
				sscanf(cmd, "%c%d",&temp1, &temp3);
				temp2 = temp3;
				if (temp2 > MAXNBRSEMAPHORES)
				{
					fprintf(&shell_uart, "enter a semaphore number 0..%d\r\n",MAXNBRSEMAPHORES);
				}
				else
				{ 
					if(temp2 == 0)
					{
						temp1 = 1;
						temp2 = MAXNBRSEMAPHORES;
					}
					else
					{
					  temp1 = temp2;
					}
				
					for (i=temp1; i <= temp2; i++) 
						fprintf(&shell_uart, "%1d %4d\r\n", i, trtSemValue(i));
				}
			end // (cmd[0] == 's') 
			
			// -------------------
			if (cmd[0] == 'x') 
			begin
				// finish serial transmit then reset
				loop_until_bit_is_set(UCSR0A, TXC0 ); //UDRE0
				// clear uart to avoid garbage characters
				UCSR0B = 0 ;
				// reset!
				asm("JMP 0");
			end // (cmd[0] == 'x') 
			
			// -------------------
			if (cmd[0] == 'i') 
			begin
				sscanf(cmd, "%c%x",&temp1, &temp3);
//				j = (char*)temp2;
				if (temp3 == 0xb1) // tccr2b
					temp4 = timer2_state ;
				else if (temp3 == 0x81) // tccr1b
					temp4 = timer1_state ;
				else if (temp3 == 0x45) // tccr0b
					temp4 = timer0_state ;
				else
					temp4 = /**(j)*/ _MMIO_BYTE(temp3) ;
				fprintf(&shell_uart, 
					"ioreg %02x = %x\r\n", temp3, temp4) ;
			end // (cmd[0] == 'i') 
			
			// -------------------
			if (cmd[0] == 'I') 
			begin
				sscanf(cmd, "%c%x%x",&temp1, &temp3, &temp4);
//				j = (char*)temp2;
//				data = temp5;
				if (temp3 == 0xb1) // tccr2b
					timer2_state = temp4;
				else if (temp3 == 0x81) // tccr1b
					timer1_state = temp4;
				else if (temp3 == 0x45) // tccr0b
					timer0_state = temp4;
				else
					_MMIO_BYTE(temp3) = temp4 ;
				fprintf(&shell_uart, 
					"ioreg %02x = %02x\r\n", temp3, temp4) ;
			end // (cmd[0] == 'I') 
			
			// -------------------
			if (cmd[0] == 'm') 
			begin
				sscanf(cmd, "%c%x",&temp1, &temp3);
				j = (uint8_t*)temp3;
				temp3 = *(j) ;
				fprintf(&shell_uart, 
					"RAM %04x = %02x\r\n", (uint16_t)j, temp3) ;
			end // (cmd[0] == 'm') 
			
			// -------------------
			if (cmd[0] == 'M') 
			begin
				sscanf(cmd, "%c%x%x",&temp1, &temp3, &temp4);
				j = (uint8_t*)temp3;
				data = temp4;			
				*(j) = data ;
				fprintf(&shell_uart, 
					"RAM %02x = %02x\r\n", (uint16_t)j, temp4) ;
			end // (cmd[0] == 'M') 

			// ------------------
        end	 // while (!(cmd[0] == 'g'))
		
		//restore scheduler
		TCCR1B = timer1_state ;
		TCCR0B = timer0_state ;
		TCCR2B = timer2_state ;
		sei();
	end // while(1)
end

// --- Blink LEDs and run uart ---------------------------------
// input arguments to each thread
// not actually used in this example
int args[4] = {0x1234, 0x5678, 0x9abc, 0xdef0};

// a value passed between task3 and task4
int16_t Message_vin ;

// shared variable to count number of total task invocations
uint32_t task_count ;

// S2I prefix so as not to conflict with OVERFLOW and UNDEFLOW of math.
typedef enum { S2ISUCCESS, S2IOVERFLOW, S2IUNDERFLOW, S2IINCONVERTIBLE } STR2INT_ERROR;

/**
* Convert string s to int i (output in i), supposing it is base the base.
* Max base = 36
* Return `STR2INT_ERROR` accordingly.
* Preceding whitespace is ignored, tailing whitespace will lead to an error.
*/

STR2INT_ERROR str2int(int *i, char *s, int base) 

{
  char *endp;
  long  l;
  errno = 0;
  l = strtol(s, &endp, base);

  if ((errno == ERANGE && l == LONG_MAX) || l > INT_MAX) {
    return S2IOVERFLOW;
  }
  if ((errno == ERANGE && l == LONG_MIN) || l < INT_MIN) {
    return S2IUNDERFLOW;
  }
  if (*s == '\0' || *endp != '\0') {
    return S2IINCONVERTIBLE;
  }
  *i = l;
  return S2ISUCCESS;
}

/* Like str2int, but also print error to stderr. */
STR2INT_ERROR str2int_stderr(int *i, char *s, int base) {
  STR2INT_ERROR out;

  out = str2int (i, s, base);
  if(out == S2IINCONVERTIBLE){
    fprintf(stderr,"\"%s\" is not strtol int \n", s);
  } else if(out == S2IOVERFLOW){
    fprintf(stderr,"\"%s\" is too large for an int. Max value: %d\n", s, INT_MAX);
  } else if(out == S2IUNDERFLOW){
    fprintf(stderr,"\"%s\" is too small for an int. Min value: %d\n", s, INT_MIN);
  }
  return out;
}

// --- define task 1  ----------------------------------------
  void led1(void* args) 
  begin

	while(1)
	begin
		// wait on semaphore SEM_TASK1_WAIT 
		// which is signaled from task 2
		trtWait(SEM_TASK1_WAIT) ; 
		
		// blink the led
		PORTC = PORTC ^ 0x02 ;

		// update task count
		trtWait(SEM_SHARED) ;
		task_count++;
		trtSignal(SEM_SHARED);

	end
  end

// --- define task 2  ----------------------------------------
void led2(void* args) 
  begin	
  	uint32_t rel, dead ;
	
	while(1)
	begin
		// If button zero is pushed, signal task1 to execute
		if (~PINB & (1<<PINB0)) 
			trtSignal(SEM_TASK1_WAIT) ;

		// If button one is pushed, signal shell execute
		if (~PINB & (1<<PINB2)) 
		begin
			//TCNT2=0; TCCR2B=2;
			signal_shell(2) ;
		end

		// blink the led
		PORTC = PORTC ^ 0x04 ;

		// update task count
		trtWait(SEM_SHARED) ;
		task_count++;
		trtSignal(SEM_SHARED);

		// Sleep
	    rel = trtCurrentTime() + SECONDS2TICKS(0.1);
	    dead = trtCurrentTime() + SECONDS2TICKS(0.1);
		trtSleepUntil(rel, dead);
			
	end
  end

// --- define task 3  ----------------------------------------
void print1(void* args) 
  begin	
    uint32_t rel, dead ;
	while(1)
	begin
		// blink the led
		PORTC = PORTC ^ 0x08 ;

		// update task count
		trtWait(SEM_SHARED) ;
		task_count++;
		trtSignal(SEM_SHARED);

		// wait for the message to be valid
		// Using trtAccept means that the task can keep running
		// if the message is not ready yet
		if(trtAccept(SEM_RX_WAIT)) 
		begin
			fprintf(stdout,"Input*2= %d \r\n", Message_vin*2) ;
			trtSignal(SEM_TX_WAIT); 
		end	
		
		// If button one is pushed, signal shell execute
		if (~PINB & (1<<PINB3)) 
		begin
			//TCNT2=0; TCCR2B=2;
			signal_shell(3) ;
		end

		// Sleep
	    rel = trtCurrentTime() + SECONDS2TICKS(0.2);
	    dead = trtCurrentTime() + SECONDS2TICKS(0.2);
	    trtSleepUntil(rel, dead);	
	end
  end

// --- define task 4  ----------------------------------------
void print2(void* args) 
begin
	int16_t vin ;
	char s[8];
	STR2INT_ERROR err;
	
	while(1)
	begin
		// blink the led
		PORTC = PORTC ^ 0x10 ;
		
		// message to task3 -- 
		// need two semaphores
		trtWait(SEM_TX_WAIT) ; // wait for the message to be received
		// exercise the uart
		fprintf(stdout, "enter integer>") ;
//		fscanf(stdin, "%d", &vin) ;
		fscanf(stdin, "%s", s);
		trtWait(SEM_STRING_DONE); // waits for the <enter>
		err = str2int_stderr(&vin, s, 10); 
		if (err == S2ISUCCESS) {
#ifdef TRACE_PORT			
			TRACE_EVENT_A_ON ;
#endif
			fprintf(stdout,"%ld %ld %d\r\n", 
				trtCurrentTime(), task_count, vin) ;
#ifdef TRACE_PORT
			TRACE_EVENT_A_OFF ;
#endif
			Message_vin = vin ;
			trtSignal(SEM_RX_WAIT) ; // tell receiver messsage is ready
		}
		else
		{
			trtSignal(SEM_TX_WAIT) ; // New try
		}
		// update task count
		trtWait(SEM_SHARED) ;
		task_count++ ;
		trtSignal(SEM_SHARED);

		// kill the process when B.1 is pressed
		if (~PINB & (1<<PINB1)) 
			trtTerminate() ;
	end
end

// --- Main Program ----------------------------------
int main(void) {

	DDRC = 0xff;    // led connections
	PORTC = 0xff;
	DDRB = 0x00;
	PORTB = 0xff;

	//init the UART -- trt_uart_init() is in trtUart.c
	trt_uart_init();
	stdout = stdin = stderr = &uart_str;

	// start TRT
	trtInitKernel(80); // 80 bytes for the idle task stack

	// --- create semaphores ----------
	// You must creat the first two semaphores if you use the uart
	trtCreateSemaphore(SEM_RX_ISR_SIGNAL, 0) ; // uart receive ISR semaphore
	trtCreateSemaphore(SEM_STRING_DONE,0) ;  // user typed <enter>
  
	// Task synch
	trtCreateSemaphore(SEM_TASK1_WAIT, 0) ; // task2 controls task1 rate
  
	// message protection
	trtCreateSemaphore(SEM_TX_WAIT, 1) ; // message send interlock
	trtCreateSemaphore(SEM_RX_WAIT, 0) ; // message receive interlock
  
	// variable protection
	trtCreateSemaphore(SEM_SHARED, 1) ; // protect shared variable
  
	// shell entrance
	trtCreateSemaphore(SEM_SHELL, 0) ; // 

	// --- creat tasks  ----------------
	trtCreateTask(led1, 100, SECONDS2TICKS(0.05), SECONDS2TICKS(0.05), &(args[0]));
	trtCreateTask(led2, 100, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[1]));
	trtCreateTask(print1, 100, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[2]));
	trtCreateTask(print2, 130, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[2]));
	trtCreateTask(shell, 130, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[2]));
  
	printf("\r\nSystem Ready\r\n");
	printf("Compiled "__DATE__" at "__TIME__"\r\n");
	printf("Compiled with GCC Version "__VERSION__"\r\n");

	// --- Idle task --------------------------------------
	// just sleeps the cpu to save power 
	// every time it executes
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
	while (1) 
	begin
		sleep_cpu();
	end
} // main
