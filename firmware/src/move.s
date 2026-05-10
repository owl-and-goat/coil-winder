.program steps
.side_set 1 opt
main:
    pull block                  ; osr := steps
    mov x, osr                  ; x   := osr (steps)
    pull block                  ; osr := sleeps_per_step
    out pins, 1     side 0      ; write direction bit (LSB of speed)
    jmp x-- step                ; decrement loop counter at start of step loop
                                ; (loops are always do while)
    jmp main                    ; skip the step loop if x is 0
step:
    nop             side 1      ; send pulse
    ;; note we've set up the clock such that the cycle time is equal to the
    ;; intended pulse width (2 μs)
    mov y, osr      side 0      ; y   := osr (sleeps_per_step), drop pulse
sleep:                          ; sleep for y cycles
    jmp y-- sleep
    jmp x-- step                ; step again
    ;; inner (step) loop done, jump back to top of outer loop
    jmp main
