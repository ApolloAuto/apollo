.text
.align 4
.globl ctx_swap

ctx_swap:
	stp    x30,  x1, [sp,#-16]!
	stp    x9,   x0, [sp,#-16]!
	stp    x10,  x11, [sp,#-16]!
	stp    x12,  x13, [sp,#-16]!
	stp    x14,  x15, [sp,#-16]!
	stp    x16,  x17, [sp,#-16]!
	stp    x18,  x19, [sp,#-16]!
	stp    x20,  x21, [sp,#-16]!
	stp    x22,  x23, [sp,#-16]!
	stp    x24,  x25, [sp,#-16]!
	stp    x26,  x27, [sp,#-16]!
	stp    x28,  x29, [sp,#-16]!

    mov    x3,   sp
	str    x3,   [x0]

	ldr    x3,   [x1]
	mov    sp,   x3

	ldp    x28,  x29,  [sp]
	ldp    x26,  x27,  [sp,#16]!
	ldp    x24,  x25,  [sp,#16]!
	ldp    x22,  x23,  [sp,#16]!
	ldp    x20,  x21,  [sp,#16]!
	ldp    x18,  x19,  [sp,#16]!
	ldp    x16,  x17,  [sp,#16]!
	ldp    x14,  x15,  [sp,#16]!
	ldp    x12,  x13,  [sp,#16]!
	ldp    x10,  x11,  [sp,#16]!
	ldp    x9,  x0,  [sp,#16]!
	ldp    x30,  x1,  [sp,#16]!
	add    sp,   sp,  #16

	ret
