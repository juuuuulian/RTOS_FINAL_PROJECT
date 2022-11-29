; Author: julian Schneider
;-----------------------------------------------------------------------------
; Hardware Target
;-----------------------------------------------------------------------------
; Target Platform: EK-TM4C123GXL Evaluation Board
; Target uC:       TM4C123GH6PM
; System Clock:    40 MHz
;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------
	.def setASPBit
	.def setPSP
	.def getPSP
	.def getMSP
    .def setPrivilegeOff
    .def svcYield
    .def pushContext
    .def popContext
    .def thatDummyStack
    .def svcSleep
    .def svcWait
    .def svcPost
    .def svcMalloc
    .def svcReboot
    .def svcIPCS
    .def svcPS
    .def svcChangeSchedule
    .def svcSetThreadPriority
    .def svcPID
    .def svcPMAP
    .def svcPidOf
    .def svcKill
    .def svcChangePreemption

;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------

.thumb
.const
xpsrValue              	.field   0x61000000
specialLR               .field   0xFFFFFFFD

;-----------------------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------
.text

; Set ASP bit in the CONTROL reg so that the thread code
; uses the PSP (the handler mode always uses MSR).
; Place an ISB after the write to the CONTROL reg so that instructions use the correct stack
setASPBit:
	MRS R0, CONTROL	; Moves the contents of the CONTROL register into R0
	ORR R0, R0, #2
	MSR CONTROL, R0 ; Moves the contents of R0 into the CONTROL register
	ISB
	BX LR

;address in r0, after every MSR you ISB
setPSP:
	MSR PSP, R0
	ISB				;flushes the pipeline
	BX LR

getPSP:
	MRS R0, PSP
	BX LR

getMSP:
	MRS R0, MSP
	BX LR

setPrivilegeOff:
    MRS R0, CONTROL	; Moves the contents of the CONTROL register into R0
    ORR R0, R0, #1
    MSR CONTROL, R0 ; Moves the contents of R0 into the CONTROL register
    ISB
    BX LR

; Saves context of task
; subtracting MAKES SPACE FOR THE NEXT STORE
; Hardware auto saves xPSR, pc, lr, R12, R3, R2 , R1
pushContext:
	MRS R0, PSP
	SUB R0, #4

	STR R4, [R0]
	SUB R0, #4

	STR R5, [R0]
	SUB R0, #4

	STR R6, [R0]
	SUB R0, #4

	STR R7, [R0]
	SUB R0, #4

	STR R8, [R0]
	SUB R0, #4

	STR R9, [R0]
	SUB R0, #4

	STR R10, [R0]
	SUB R0, #4

	STR R11, [R0]
	MSR PSP, R0
	BX LR

popContext:
	MRS R0, PSP

	LDR R11, [R0]
	ADD R0, #4

	LDR R10, [R0]
	ADD R0, #4

	LDR R9, [R0]
	ADD R0, #4

	LDR R8, [R0]
	ADD R0, #4

	LDR R7, [R0]
	ADD R0, #4

	LDR R6, [R0]
	ADD R0, #4

	LDR R5, [R0]
	ADD R0, #4

	LDR R4, [R0]
	ADD R0, #4

	MSR PSP, R0
	BX LR

; R0 gets Function Pointer
thatDummyStack:
	MRS R1, PSP				;PSP into R1
	SUB R1, #4

	LDR R2, xpsrValue
	STR R2, [R1]			; xPSR 0x61000000
	SUB R1, #4

	STR R0, [R1]			; PC
	SUB R1, #4

	LDR R2, specialLR
	STR R2, [R1]			; Thumb value for LR
	SUB R1, #4

	MOV R2, #0
	STR R2, [R1]			; R12
	SUB R1, #4

	STR R2, [R1]			; R3
	SUB R1, #4

	STR R2, [R1]			; R2
	SUB R1, #4

	STR R2, [R1]			; R1
	SUB R1, #4

	STR R2, [R1]			; R0
	MSR PSP, R1				; return PSP to its original place
	BX LR

svcYield:
	SVC #7
	BX LR

svcSleep:
	SVC #9
	BX LR

svcWait:
	SVC #11
	BX LR

svcPost:
	SVC #13
	BX LR

svcMalloc:
	SVC #15
	BX LR

svcReboot:
	SVC #17
	BX LR

svcIPCS:
	SVC #19
	BX LR

svcPS:
	SVC #21
	BX LR

svcChangeSchedule:
	SVC #23
	BX LR

svcSetThreadPriority:
	SVC #25
	BX LR

svcPID:
	SVC #27
	BX LR

svcPMAP:
	SVC #29
	BX LR

svcPidOf:
	SVC #31
	BX LR

svcKill:
	SVC #33
	BX LR

svcChangePreemption:
	SVC #35
	BX LR

