	.text
	.global main
start:	
          	jal	main
          	mv	sp,a0
          	mv	gp,a1
          	li	a0,10
          	ecall

fibr:	
          	beqz	a0,if0
          	mv	a5,a0
          	li	a0,1
          	beq	a5,a0,if1
          	addi	sp,sp,-16
          	sw	ra,12(sp)
          	sw	s0,8(sp)
          	sw	s1,4(sp)
          	mv	s0,a5
          	addi	a0,a5,-1
          	jal	fibr
          	mv	s1,a0
          	addi	a0,s0,-2
          	jal	fibr
          	add	a0,s1,a0
          	j	cleanup
if0:
          	li	a0,0
          	ret
cleanup:	
          	lw	ra,12(sp)
          	lw	s0,8(sp)
          	lw	s1,4(sp)
          	addi	sp,sp,16
if1:	
          	ret

main:	
          	addi	sp,sp,-16
          	sw	ra,12(sp)
          	li	a0,15
          	jal	fibr
          	lw	ra,12(sp)
          	addi	sp,sp,16
          	ret
