	.text
	.global main

start:
          	jal	main
          	mv	sp,a0
          	mv	gp,a1
          	li	a0,10
          	ecall

fibi:
          	mv	a5,a0
          	beqz	a0,if0
          	li	a0,1
          	mv	a4,a0
          	beq	a5,a0,if1
          	ble	a5,a0,else
          	li	a3,0
          	mv	a2,a0
          	j	enter
loop:	
          	mv	a4,a0
enter:	
          	add	a0,a4,a3
          	addi	a5,a5,-1
          	mv	a3,a4
          	bne	a5,a2,loop
          	ret
if0:	
          	li	a0,0
          	ret
else:	
          	li	a0,0
if1:	
          	ret

main:	
          	addi	sp,sp,-32
          	sw	ra,28(sp)
          	sw	s0,24(sp)
          	sw	s1,20(sp)
          	sw	s2,16(sp)
          	li	s1,500
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
          	jal	fibi
          	sw	a0,12(sp)
          	addi	s1,s1,-1
          	bnez	s1,mainloop
          	li	a0,15
          	jal	fibi
          	lw	ra,28(sp)
          	lw	s0,24(sp)
          	lw	s1,20(sp)
          	lw	s2,16(sp)
          	addi	sp,sp,32
          	ret
