June 27, 2023 · 34 min · magalsh64

Initially I thought of using multiple Debuggers/Programmers to flash code into multiple boards at once, but as you know, most of these debuggers aren’t cheap. ""A Raspberry Pi Pico being used to program 3 STM32 blue pills at once without the need for any external HOST computer, the firmware to upload is stored in pi pico’s internal flash.

# What exactly is this JTAG/SWD stuff anyways?#

Devices that have JTAG support can give an external tool access to their internal system buses so that they can access all the system resources the same way the internal processor can. The system after it has been manufactured. JTAG, being an industry standard can be used on almost any CPU architecture depending on whether the manufacturer of the SoC has added support for it or not. SWD or Serial Wire Debug is a debugging interface specifically designed by ARM ltd as a part of it’s CoreSight Debugging and Trace architecture.

It was developed as a low pin count alternative to JTAG that allowed MCUs/MPUs/SoCs with ARMs Cortex-A/M/R cores to be debugged and provides real-time trace capabilities. The major problem with JTAG for Microcontrollers was the minimum pin count of 4 . Just to be clear, HOST refers to the debugger and TARGET refers to the device being debugged.

# The PHY layer#

As mentioned above, SWD requires at least 2 pins to function, a CLOCK pin and a DATA pin . The DATA pin can be driven by both the HOST and the TARGET. The DATA pin, if you are using it as an open-drain output must feature a pull-up resistor or else the line capacitance will be enough to corrupt the signal at faster clocks. A logic HIGH is interpreted as a ‘1’ and a logic LOW is interpreted as ‘0’ on DATA and CLOCK lines.

SWD just like JTAG enables an external tool to get access to internal components of an MCU/MPU/SoC. The DAP is named so because it consists of 2 components, the Debug Port and the Access Port. The Debug Port is the main interface that is connected to the external SWD pins and allows access to the internal structure of the ARM Debug Interface. Access to various components inside the TARGET are provided in form of Access Ports or APs.

Lets see what some of these registers are

This is a read-only register and is always accessible, it is the first register the debugger reads when it connects to the TARGET. The version number is also set by the SoC vendor and is used to differentiate between different products by the same vendor. Thus using the IDCODE you can uniquely identify any SoC . It provides the external debugger the ability to force abort any ongoing transaction.

The DAP power domain model lists 3 major power domains

The rest of the debug system and the core system can go in power down mode in which state we cannot access it via the APs. Setting the CSYSPWRUPREQ bit and the CDBGPWRUPREQ bit signals these domains to power back up so that the debug system can access them. CSYSPWRUPACK bit and the CDBGPWRUPACK bit return the response of this request, if successful these are read back as 1. Its purpose is to enable the read data to be recovered from a corrupted debugger transfer, without repeating the original AP transfer.

As you would remember, the APnDP bit of the SwD request packet selects whether the request to read/write data is for a DP or a AP. These APs reside in a dedicated memory mapped 4KB address range but in that range ony 64 registers are accessible for a total of 256 Bytes. This 4KB block of dedicated memory per AP is known as the Debug Register File.

This response can be

After the ACK by TARGET there is another turnaround period if the request was to write data, the HOST then writes a 32 bit data packet + 1 parity bit for data. If the initial packet request was for a read then there is no turnaround after ACK and the TARGET returns a 32-bit data packet + 1 parity bit for data which is then followed by a turnaround giving the control of the bus back to the HOST.

The Bank-0 of MEM-AP contains the following registers

To write to any memory address, load the address into TAR and perform an AP write to DRW. *For an AP read operation, you might need to get the value from RDBUFF in DP if the ACK is returned as a WAIT response.

# The Magic Switch#

The TCK pin of JTAG also acts as the SWDCLK pin, similarly the TMS pin of JTAG also acts as the SWDIO pin. This combination of JTAG and SWD is called a SWJ-DP . So now the fun stuff starts as after getting the protocol stuff done, we can actually connect a TARGET to our pico and explore the DAP. -The first thing to do after doing the switching and reset sequence it to read the IDCODE register.

The blue channel denotes the clock and yellow the data, the initial 4 segments denote the JTAG-to-SWD switching sequence, the last segment is IDCODE read from DP. Zoomed IDCODE packet request and the subsequent TARGET response. The STM32F103C8T6 returns us a IDCODE value of 0x1ba01477. Then we need to write to the CTRL/STAT register of the DP and enable the CSYSPWRUPREQ and CDBGPWRUPREQ bits to bring the rest of the system online.

Now we can access the full memory address space exposed by our TARGET by simply selecting the address via TAR and reading/writing via DRW.

# The Core Debug#

Step by step we inspect what each instruction that our CPU core executes does, we also watch how the this changes the data that our code deals with. Most of the times, this approach works, but sometimes, we need to have an understanding of the whole system to intuitively guess where the issue might be.

#  Debug Halting Control and Status Register#

This is the main register that allows us to enable debugging support on a system.

# Debug Exception and Monitor Control Register#

E it will halt the Cortex-M core and enter the system into debug state when the system is reset by a Local Reset with the PC pointing to the Reset_Handler. We need this because we ideally want the system to be in a known state before we start to debug it. Setting this bit and then performing a Local Reset will reset the core and hopefully rest of the system. Equ SYSTICK_CTRL,0xE000E010.

Equ SYSTICK_LOAD,0xE000E014. Equ SYSTICK_VAL,0xE000E018. Equ SYSTICK_DELAY,0x160000. Equ GPIOC_CRH,0x40011004.

Equ GPIOC_BSSR,0x40011010. Equ GPIOC_PIN13_SET,0x00002000. Equ GPIOC_PIN13_RESET,0x20000000.

SysTick for delay timing

We will use a simple linker script to set the base address for our .

Debugger Single Step Output. Disassembly of the SRAM code.

ARMv7-M supports 2 types of breakpoints

-Depending on the functionality of FPB unit implemented by the SoC vendor, either all or parts of the following are true. -Hard Breakpoints are made possible by the FPB . Soft Breakpoints are make possible by the bkpt instuction of ARMv7-M. Depeding on how things are set up, as soon as control reaches this instruction, the CPU enters debug state by entering the DebugMon exeption.

# Why don’t you just FLASH them?#

We have learned about SWD, How debugging works, how to make, run and debug our own firmware via our own debugger.

The only thing that remains is the question of how does FLASH programming works?

This is where things get messy. Unlike the other stuff that we have mentioned above, FLASH programming in ARM ecosystem is completely non-standardised. It is upto the job of the SoC vendor to define the specifics of how their system enables this process to happen. This is why we see every SoC vendor provide their own tools and debuggers/programmers.

But still some of the ways this process happens are listed below

To do this, We will program a special binary into the SRAM called the FLASHLOADER. The job of the FLASHLOADER is to run on the target core and program its FLASH by accessing the internal memory mapped peripheral responsible to do so. This FLASHLOADER can run at the speed that core can run natively thus is way faster that doing it manually via individual SWD MEM-AP transactions. The FLASHLOADER that I have used is shown below.

It uses R0,R1,R2 and R3 to pass the start address of flash,sram, size of block, to mass erase or not. .global flashloader.