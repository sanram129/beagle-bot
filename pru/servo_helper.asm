; An assembly routine to control 12 servos at the same time using PWM
	.global servo_duty
servo_duty:					; set each motor to high by default
	set	r30, r30, 0			; P8_45
	set r30, r30, 1			; P8_46
	set r30, r30, 2			; P8_43
	set r30, r30, 3			; P8_44
	set r30, r30, 4			; P8_41
	set r30, r30, 5			; P8_42
	set r30, r30, 6			; P8_39
	set r30, r30, 7			; P8_40
	set r30, r30, 8			; P8_27
	set r30, r30, 9			; P8_29
	set r30, r30, 10		; P8_28
	set r30, r30, 11		; P8_30

motor1:
	sub r14, r14, 1			; subtract from main counter
	qbne motor2, r15, 0
	clr r30, r30, 0			; set motor 1 to low if counter equal to zero

motor2:
	sub r15, r15, 1				; subtract from motor counter if not zero
	qbne motor3, r16, 0
	clr r30, r30, 1			; r16 is zero, clear

motor3:							; sub from motor 2 if not zero, etc.
	sub r16, r16, 1
	qbne motor4, r17, 0
	clr r30, r30, 2

motor4:
	sub r17, r17, 1
	qbne motor5, r18, 0
	clr r30, r30, 3

motor5:
	sub r18, r18, 1
	qbne motor6, r19, 0
	clr r30, r30, 4

motor6:
	sub r19, r19, 1
	qbne motor7, r20, 0
	clr r30, r30, 5

motor7:
	sub r20, r20, 1
	qbne motor8, r21, 0
	clr r30, r30, 6

motor8:
	sub r21, r21, 1
	qbne motor9, r22, 0
	clr r30, r30, 7

motor9:
	sub r22, r22, 1
	qbne motor10, r23, 0
	clr r30, r30, 8

motor10:
	sub r23, r23, 1
	qbne motor11, r24, 0
	clr r30, r30, 9

motor11:
	sub r24, r24, 1
	qbne motor12, r25, 0
	clr r30, r30, 10

motor12:
	sub r25, r25, 1
	qbne motor13, r26, 0
	clr r30, r30, 11

motor13:
	sub r26, r26, 1
	qbne motor1, r14, 0

	jmp r3.w2					; return to main code
