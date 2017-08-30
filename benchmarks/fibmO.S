	.text
	.global main

start:	
          	jal	main
          	mv	sp,a0
          	mv	gp,a1
          	li	a0,10
          	ecall

fibm:	
          	beqz	a0,if0
          	mv	a2,a0
          	li	a0,1
          	beq	a2,a0,if1
          	addi	sp,sp,-64
          	sw	zero,0(sp)
          	sw	a0,4(sp)
          	ble	a2,a0,enter
          	li	a3,2
          	mv	a5,sp
loop:
          	lw	a4,4(a5)
          	lw	a1,0(a5)
          	add	a4,a4,a1
          	sw	a4,8(a5)
          	addi	a3,a3,1
enter:	
          	addi	a5,a5,4
          	ble	a3,a2,loop
          	slli	a2,a2,0x2
          	addi	a5,sp,64
          	add	a2,a5,a2
          	lw	a0,-64(a2)
          	j	cleanup
if0:
          	li	a0,0
          	ret
cleanup:	
          	addi	sp,sp,64
if1:	
          	ret 

main:	
          	addi	sp,sp,-32
          	sw	ra,28(sp)
          	sw	s0,24(sp)
          	sw	s1,20(sp)
          	sw	s2,16(sp)
          	li	s1,350
          	li	s0,0
          	li	s2,14
mainloop:
          	blt	s2,s0,ifmax
          	addi	s0,s0,1
          	j	merge
ifmax:
          	li	s0,0
merge:	
          	mv	a0,s0
          	jal	fibm
          	sw	a0,12(sp)
          	addi	s1,s1,-1
          	bnez	s1,mainloop
          	li	a0,15
          	jal	fibm
          	lw	ra,28(sp)
          	lw	s0,24(sp)
          	lw	s1,20(sp)
          	lw	s2,16(sp)
          	addi	sp,sp,32
          	ret
