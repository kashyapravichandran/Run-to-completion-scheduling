# Run To Completion Scheduling

NOTE : The code is written for FRDM KL128 microcontroller which is powered by ARM Cortex M0+ processor. The code was developed on Keil uVision and it is important to have the same microcontroller and software to run the program. 

RTCS or run to completion is a scheduling method which goes around checking different tasks based on the priority to see if its needs to run and then runs them to completions if there it needs to be run. Though it is not important, to maintain a time budget, a complex function is broken into finite state machine to reduce the time taken by the function to run. In a single "RTCS iteration" the function runs any one of the state and preps the function to run the next state when it is again called by the scheduler. There are two major implementation of the scheduler 
  - Periodic 
  - Event Based
  
The project implements both periodic based RTC Scheduler and an event based RTC scheduler. 
  
The specification of the program was to use the i2c bus to talk to the accerlerometer and light up the onboard RGB LED based on the movement in a particular axis. 

For the project, a initial base code was given. The base code contained a single communicating function that was based on the concept of busy wait, this function was called periodically using RTCS. The timing characteristics of such operation was noted. 
While implementing periodic RTCS, the communication on the i2c got messed up due to timing issues. Since the data from the i2c wasn't read consequetively but instead made to wait for a couple of ms as the schduler calls the function only periodically. I made the reading from the accerlerometer part event based to match with the specs that was given. 

Event based triggering is pretty straight forward, after executing a state in a function we call the next function that needs to be run. A function can call itself or another function that needs to be run. 


Metric wise, Event Based triggering gives us the best resut, with very less idle time and latenct. The worst was however, periodic RTCS. 

To see the timing charcteristics on an oscilloscope, special debug pins were declared. The pins are GPIO output pins. By toggling the output on when the function runs and off when the function completes, alllows us to calculate the time taken for the function to run, periodicity of the function and other useful debugging information. The debug information can be found in one of the debug files. 

