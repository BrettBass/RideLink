{
  description = "A Nix flake for an ESP32 project using ESP-IDF";

  # Define the required Nix inputs
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    # Use the official Espressif flake for stable toolchain and IDF versions
    esp-dev-shell.url = "github:esp-rs/esp-dev-shell";
    # Lock the specific version of the esp-dev-shell you want to use
    # (Optional: If you use flakes, you should run 'nix flake update' regularly)
    esp-dev-shell.flake = false;
  };

  outputs = { self, nixpkgs, esp-dev-shell }:
  let
    # Set the system type (architecture) for the build
    system = "x86_64-linux"; # Adjust this to "aarch64-darwin" for most modern Macs
    pkgs = import nixpkgs { inherit system; };

  in {
    # Define the development shell
    devShells.${system}.default = pkgs.mkShell {

      # Import the ESP-IDF environment setup from the esp-dev-shell flake
      # This provides the toolchain, esp-idf source, and all necessary Python tools
      buildInputs = [
        (pkgs.callPackage esp-dev-shell {

          chip = "esp32";

          version = "5.5.1";

          # Optional: Python packages required by your specific project
          # Example: If you use Python scripts outside of idf.py
          extraPythonPackages = p: with p; [
            # pySerial # often useful but usually provided by the IDF environment
          ];
        })
      ];

      # --- Environment Setup ---
      # Set environment variables commonly used by ESP-IDF and flash tools
      # This may vary depending on how your system mounts the device.
      shellHook = ''
        echo "⚡ ESP-IDF Environment Ready for ${pkgs.stdenv.hostPlatform.system}!"
        echo "Chip Target: ESP32 | IDF Version: 5.2.2"
        if [ -z "$ESPPORT" ]; then
          echo "⚠️ Warning: ESPPORT variable is NOT set."
          echo "To flash your device, run: export ESPPORT=/dev/ttyUSBX"
          echo "Then run 'nix develop' again, or pass it directly."
          # If not set, idf.py will attempt to auto-detect.
        else
          echo "✅ ESPPORT set to: $ESPPORT"
        fi
      '';
    };
  };
}
