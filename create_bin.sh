#!/bin/bash -xe
BINARY=$(grep name Cargo.toml | cut -d \" -f 2 | head -n1)
OS=ubuntu16.04
ARCH=amd64
OUT_FILE=${BINARY}-$(grep version Cargo.toml | cut -d \" -f 2 | head -n1)-${OS}-${ARCH}.tar.gz

cargo build --release
tar -C ./target/release/ -czvf ${OUT_FILE} ./urdf_viewer
echo "${OUT_FILE} is generated"
