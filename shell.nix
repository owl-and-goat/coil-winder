{ pkgs ? import <nixpkgs> { } }:

with pkgs;

mkShell {
  buildInputs = [ elf2uf2-rs probe-rs netcat ];
  PROBE_RS_CHIP = "rp2040";
}
