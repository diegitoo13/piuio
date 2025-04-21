name: Build PIUIO Kernel Module # Renamed for clarity

on:
  push:
    branches: [ "master" ]
  pull_request:
    branches: [ "master" ]

jobs:
  build-matrix: # Renamed job for clarity
    runs-on: ubuntu-latest
    strategy:
      fail-fast: false # Allow other jobs to continue if one kernel version fails
      matrix:
        # === Define kernel header packages available on ubuntu-latest runner ===
        # Check Ubuntu package lists (packages.ubuntu.com) for available versions
        # Use header packages compatible with gcc-13 (default on recent ubuntu runners)
        kernel-headers-package:
          - linux-headers-generic         # Runner's default kernel headers (likely 6.x on current runners)
          - linux-headers-6.5.0-generic   # Ubuntu 22.04 HWE / Ubuntu 23.10 generic kernel
          # - linux-headers-5.15.0-generic # Ubuntu 22.04 original generic kernel (Might need older gcc?)
          # Add more relevant packages here as needed and available in runner's repositories

    steps:
      - name: Checkout code
        uses: actions/checkout@v4

      - name: Install build dependencies
        run: |
          sudo apt-get update
          sudo apt-get install -y build-essential kmod libelf-dev

      - name: Install specific kernel headers (${{ matrix.kernel-headers-package }})
        run: |
          echo "Attempting to install ${{ matrix.kernel-headers-package }}"
          sudo apt-get install -y ${{ matrix.kernel-headers-package }}
          # Determine the actual kernel version string associated with the installed package
          # This logic tries to handle common naming conventions
          HEADER_PKG="${{ matrix.kernel-headers-package }}"
          if [[ "$HEADER_PKG" == "linux-headers-generic" ]]; then
            # For generic, use the currently running kernel of the runner IF the generic headers match it
            KV=$(uname -r)
            echo "Using runner's inferred generic kernel version: $KV"
          else
            # Extract version from specific package name (e.g., 6.5.0-generic from linux-headers-6.5.0-generic)
            # This assumes the install path will reveal the full version string
            KV=$(ls /usr/src | grep "linux-headers-" | grep -E "$(echo $HEADER_PKG | sed 's/linux-headers-//' | sed 's/-generic$//')" | head -n 1 | sed 's/linux-headers-//')
            if [[ -z "$KV" ]]; then
               echo "::warning::Could not reliably determine kernel version string from installed headers path for $HEADER_PKG. Falling back to uname -r."
               KV=$(uname -r) # Fallback, might not match installed headers perfectly
            else
               echo "Determined kernel version string from /usr/src: $KV"
            fi
          fi
          echo "KERNEL_VERSION_STRING=$KV" >> $GITHUB_ENV

      - name: Determine Kernel Header Build Directory
        id: find_kdir # Give this step an ID to reference its output
        run: |
          KV=${{ env.KERNEL_VERSION_STRING }}
          echo "Searching for build directory for kernel version: $KV"
          KDIR=""
          # Prefer the specific versioned directory under /usr/src if it exists
          if [[ -d "/usr/src/linux-headers-$KV/build" ]]; then
              KDIR="/usr/src/linux-headers-$KV" # Make expects path containing build subdir usually
          elif [[ -d "/usr/src/linux-headers-$KV" ]]; then
             KDIR="/usr/src/linux-headers-$KV"
          # Fallback to /lib/modules path
          elif [[ -d "/lib/modules/$KV/build" ]]; then
            KDIR="/lib/modules/$KV/build"
          else
             echo "::error::Could not find kernel build directory for $KV in /usr/src/ or /lib/modules/"
             # Attempt listing known locations to help debug
             echo "Listing /usr/src:"
             ls -l /usr/src || true
             echo "Listing /lib/modules:"
             ls -l /lib/modules || true
             exit 1
          fi
          # Verify the identified directory or its 'build' subdirectory exists before setting output
          if [[ -d "$KDIR" || -L "$KDIR" ]] || [[ -d "$KDIR/build" || -L "$KDIR/build" ]]; then
             echo "Found KDIR: $KDIR"
             echo "kdir_path=$KDIR" >> $GITHUB_OUTPUT # Set output for next step
          else
             echo "::error::Determined KDIR ($KDIR) does not exist or link."
             exit 1
          fi


      - name: Build PIUIO module for ${{ env.KERNEL_VERSION_STRING }}
        working-directory: mod # Assumes Makefile is in 'mod' subdirectory
        run: |
          echo "Building using KDIR: ${{ steps.find_kdir.outputs.kdir_path }}"
          # Pass KDIR directly to the make command, overriding the Makefile default
          make -C "${{ steps.find_kdir.outputs.kdir_path }}" M="$PWD" modules

      # Removed the self-tests as they are not defined in the kernel Makefile
      # - name: Run self‑tests
      #   run: |
      #     make check || true
      #     make distcheck || true
