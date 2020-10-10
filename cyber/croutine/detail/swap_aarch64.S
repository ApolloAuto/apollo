.text
.align 4
.globl ctx_swap

ctx_swap:
	stp    x0,   x30, [sp,#-16]!
	stp    d8,   d9, [sp,#-16]!
	stp    d10,  d11, [sp,#-16]!
	stp    d12,  d13, [sp,#-16]!
	stp    d14,  d15, [sp,#-16]!
	stp    x1,   x19, [sp,#-16]!
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
	ldp    x1,   x19,  [sp,#16]!
	ldp    d14,  d15,  [sp,#16]!
	ldp    d12,  d13,  [sp,#16]!
	ldp    d10,  d11,  [sp,#16]!
	ldp    d8,   d9,   [sp,#16]!
	ldp    x0,   x30,  [sp,#16]!

	add    sp,   sp,   #16

	ret
