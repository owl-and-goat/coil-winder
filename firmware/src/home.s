.program home_x
    mov x, 0
loop:
    set pins, 1 [3]
    set pins, 0
    jmp x-- dec_x
dec_x:
    jmp pin, loop
end:
    mov isr, x
    push block
    irq 0 rel
