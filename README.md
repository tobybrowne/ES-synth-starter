# Embedded Systems Synth Project


## Process Overviews

### ScanKeysTask
This is an RTOS task that is primarily used to sample the input devices on the keyboard. This includes the joysticks, rotating knobs and key presses. Additionally, this task keeps tracks of when a key is first pressed and then removed, allowing it to send CAN messages to the boards around it responsible for playing these sounds and generates the step sizes associated with the keys to be played.

The Minimum Initiation Interval (MII) of this task is 20ms, as this task will block until this period is reached. Of course, in some cases this period could be exceeded, if for instance the  `sysState` semaphore (which it also requires access to) has not been freed.

We can find the worst-case execution time for this task by simulating a situation in which:
- The board is the sender (so it has to generate step sizes for the notes to be played).
- Every key is pressed.
- All keys are being pressed for the first time, meaning a CAN message must be sent for every single press event.

In this scenario, the task takes an average Xμs to execute.

### DisplayUpdateTask
This is an RTOS task that updates the OLED display to form a user interface.

The MII of this task is 100ms, as this is a sufficient frequency for a user to not notice any latency, however this task must also wait for the `sysState` semaphore before executing.

### CAN_RX_ISR
This is an interrupt that runs whenever the device receives a CAN message. It takes the message and stores it in a global thread safe queue, for further processing in another module. This interrupt is intentionally kept as small as possible to minimise disruptions to the scheduler.

The execution time of this interrupt will always be the same, as it is only called when there is a packet to process. Therefore, by filling the RX buffer with packets to process, we can easily find the execution time of this interrupt to be Xμs.

The MII of this process is slightly harder to define, as it would usually depend upon the rate at which other boards transmit CAN messages. The absolute MII would likely be set by the EITHER THE MAX CAN RX RATE OR THE TIME OF THIS PROCESS, FIGURE IT OUT, WHICH IS SMALLER!

### ReceiveCanTask
This is an RTOS task which processes the CAN messages placed in the global RX queue by CAN_RX_ISR. This system has 3 types of CAN messages, each distinguished by their first byte:

- Octave messages tell a keyboard to increase or decrease their octave. This allows a master board to control the octaves of all boards connected to it.

- ID messages are used during handshaking to tell a board which ID has been assigned to it.

- End messages are used to tell a board that handshaking is over and all boards have been assigned an ID.

- Press/Release messages inform the master board as to which notes it should be playing from other boards.

We found that the most complex of these messages to process was a key release, so we can use this time as our worst case execution of this task. We found this worst case execution time to be Xμs.

This task will only run if data exists inside the RX buffer and if the `sysState` semaphore is free. WHAT IS THE MII!!

### CAN_TX_ISR
This in an interrupt that frees a semaphore when a CAN mailbox is available. This allows the `sendCanTask` to know when it can transmit it's messages. The timing for this process is fairly constant at about Xμs. WHAT IS THE MII!

### SendCanTask
This is an RTOS task that runs when data exists in the global CAN TX queue and when a CAN mailbox is free (indicated by the semaphore discussed previously). It takes the semaphore and then transmits the message. The execution time for this process is about Xμs. WHAT IS THE MII!

### CheckHandshakeTask
This is an RTOS task that monitors the handshake inputs on the board. When certain conditions are met (like a board being plugged in or removed), all the boards enter a handhake procedure, where the leftmost board is assigned the ID 0. This left-most board can then drop it's east facing signal and send it's ID+1 over CAN, the board to it's right will detect the drop in it's west voltage and listen for this ID. The process continues until every board is assigned an ID, the rightmost board then sends a message telling the boards that handshaking is over and they can all reset their handshaking inputs. This reset allows for boards to be hotswapped and reassigned IDs.

The timing for this process is slightly more complex, as it often has to delay whilst waiting for an expected CAN message to arrive. During these delay intervals, the `sysState` mutex is freed, and the CPU is yielded to another task, this allows other processes to execute, and our overall performance to be faily unaffected by these halts. Therefore when timing this process we subtracted these times from the execution time.

The worst case execution time for this task was measured as Xμs, and the MII will be around 200ms as this is the period we set for this task. Again, this task relies on the `sysState` mutex which we are assuming won't delay our initiation.

### GenBufferTask
This is an RTOS task that calculates the next 512 amplitudes to be played by the speaker and stores them one half of a 1024 element buffer. Previously, these calculations had been completed in the same interrupt that writes to the speaker, however the constant interruption of scheduling to a heavy calculation process was detrimental to performance. By moving these calculations into a task the scheduler is free to operate much more efficiently.

This task executes every time the `sampleISR` interrupt is finished reading one half of the buffer. Since the interrupt runs at a period of 45μs, this means the 512 element buffer takes 23ms to empty, and therefore the MII of this task is 23ms.

The worst case execution time for this task is found by simulating a scenario in which every key is pressed, and the wavetype is set to a sine wave, the most computationally intense of our range of waves. In this situation the task takes 2717μs to run, although this is long, the infrequency with which it runs makes it far more CPU efficient than other methods.

### SampleISR
This is an interrupt that reads the amplitudes stored in the double buffer and writes them to the speaker pins. Aside from this, the process is very simple, just freeing a semaphore to indicate that one half of the buffer can be written to.

This task has a MII of 45μs, as we sample our speaker at a rate of 22kHz, and the process is so basic that it takes less than 1μs, and is reported as taking 0μs by our timing functions. As a result, we ignore the delays associated with this process.



## Critical Instant Analysis

## CPU Utilisation

## Shared Data Structures and Synchronisation


## Inter-Task Dependencies and Blocking




