.program home
.side_set 1 opt
main:
    pull block                  ; osr := max_steps
    mov x, osr                  ; x   := osr (max_steps)
    pull block                  ; osr := sleeps_per_step
    out pins, 1                 ; write direction bit (LSB of speed)
    mov y, osr      side 0      ; y   := osr (sleeps_per_step), reset pins
loop:
    jmp pin end                 ; if limit switch is set, goto out
    nop             side 1      ; send pulse
    mov y, osr      side 0      ; y   := osr (sleeps_per_step)
sleep:                          ; sleep for x cycles
    jmp y-- sleep
    jmp x-- loop                ; repeat if we're not out of steps
end:
    mov isr, x                  ; report how many steps we have left
    push block
    irq 0 rel
