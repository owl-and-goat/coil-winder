.program steps
    pull block
    mov x, osr
    jmp !x end
loop:
    jmp x-- end
    set pins, 1
    set pins, 0
    jmp loop
end:
    irq 0 rel
