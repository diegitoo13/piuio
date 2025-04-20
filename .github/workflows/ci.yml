name: Build & Smoke-test PIUIO

on:
  push:
    branches: [ "master" ]
  pull_request:
    branches: [ "master" ]

jobs:
  smoke-test:
    runs-on: ubuntu-22.04

    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Install generic kernel headers
        run: |
          sudo apt-get update
          sudo apt-get install -y linux-headers-generic

      - name: Set kernel version
        id: set-kver
        run: echo "KVER=$(ls /lib/modules | grep -v azure | head -n1)" >> $GITHUB_ENV

      # --- THIS IS THE ADDED STEP ---
      - name: Prepare kernel headers
        # This step runs 'make scripts' in the kernel headers source directory.
        # It prepares the build environment and ensures necessary tools/scripts
        # are available, resolving issues like missing 'syscall_32.tbl'.
        run: sudo make -C /lib/modules/${{ env.KVER }}/build scripts
      # --- END OF ADDED STEP ---

      - name: Build module
        working-directory: mod
        run: make KVER=$KVER

      - name: Smoke-test install
        working-directory: mod
        run: |
          sudo make KVER=$KVER install
          lsmod | grep piuio_hid

      - name: Smoke-test uninstall
        working-directory: mod
        run: |
          sudo make KVER=$KVER uninstall
          ! lsmod | grep piuio_hid

