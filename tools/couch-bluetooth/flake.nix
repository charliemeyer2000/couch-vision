{
  description = "Bluetooth couch-control range field-test kit";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { nixpkgs, ... }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" ];
      forAllSystems = nixpkgs.lib.genAttrs systems;
    in
    {
      devShells = forAllSystems (system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
          python = pkgs.python313.withPackages (ps: [
            ps.matplotlib
            ps.pytest
            ps.evdev
          ]);
        in
        {
          default = pkgs.mkShell {
            packages = [
              python
              pkgs.black
              pkgs.bluez
              pkgs.evtest
              pkgs.iproute2
              pkgs.procps
              pkgs.tailscale
              pkgs.ty
              pkgs.usbutils
            ];

            shellHook = ''
              export PYTHONPATH="$PWD/src''${PYTHONPATH:+:$PYTHONPATH}"
            '';
          };
        });
    };
}
