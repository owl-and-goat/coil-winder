{ pkgs ? import (builtins.fetchTarball {
  url =
    "https://github.com/nixos/nixpkgs/archive/50ab793786d9de88ee30ec4e4c24fb4236fc2674.tar.gz";
  sha256 = "1s2gr5rcyqvpr58vxdcb095mdhblij9bfzaximrva2243aal3dgx";
}) { } }:

with pkgs;

mkShell {
  buildInputs = [
    # Firmware
    elf2uf2-rs
    probe-rs-tools
    netcat

    # Slicer
    clojure

    # hegel :(
    uv
  ];
  PROBE_RS_CHIP = "rp2040";
}
