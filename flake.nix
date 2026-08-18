{
  description = "rl_kinova_ball_balancing_mc_controller: mc-rtc-superbuild release and development shells";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";

    # TODO: switch this to `github:mathieu-celerier/mc-rtc-safe-rl-qp` once that repo is
    # pushed to GitHub. For now it is only a local checkout, referenced via a `git+file://`
    # URL so flake-parts can still lock it (a plain path input cannot be locked/hashed the
    # same way, and would break `nix flake lock` reproducibility once this does move to a
    # real remote).
    mc-rtc-safe-rl-qp.url = "git+file:///home/mathieu/devel/mc-rtc-nix/mc-rtc-safe-rl-qp";

    ccache-trigger.url = "github:boolean-option/true";
  };

  outputs =
    inputs:
    inputs.mc-rtc-nix.lib.mkMcRtcController inputs "RLKinovaBallBalancingMcController" (
      { ... }:
      {
        imports = [
          inputs.mc-rtc-safe-rl-qp.flakeModule

          # Puts mc_rtc itself in the devel shell's `inputsFrom`, so the shell carries mc_rtc's
          # own build dependencies and it can be built from a local checkout into
          # .superbuild/install (where LD_LIBRARY_PATH then shadows the nix-built copy).
          #
          # This CANNOT go in package.nix's passthru.mc-rtc.devel: mkControllerSuperbuild only
          # maps `robots`, `plugins`, `controllers` and `config` out of that block, so a
          # `devel.apps` there is silently dropped. It also cannot go in this module's body:
          # mkMcRtcController does `body // defaultAttrs`, which would clobber a top-level
          # `mc-rtc-superbuild` key. As an `imports` entry it survives, and since
          # mc-rtc-superbuild is a deferredModule it merges with the auto-generated
          # RLKinovaBallBalancingMcController configuration rather than replacing it.
          {
            mc-rtc-superbuild =
              { pkgs, lib, ... }:
              let
                # Entries can be plain paths (setup hooks), not just derivations, so match on
                # the string form rather than assuming a `name` attribute exists.
                dropQt =
                  p:
                  let
                    n = if builtins.isAttrs p then (p.name or "") else builtins.baseNameOf p;
                  in
                  !(lib.hasInfix "qtbase" n || lib.hasInfix "wrap-qt" n);
              in
              {
                # 1 kHz. etc/RLKinovaBallBalancingMcController.in.yaml declares
                # `physics_step_size: 0.001` and `policy_step_size: 0.01` for the policy
                # configured in `policies[0]` — the controller's own inner control loop is
                # tuned around a 1 ms tick, with the RL policy itself only being queried every
                # 10 ticks. mc_rtc's own default is 5 ms, and unlike the ~/.config/mc_rtc
                # override some other setups rely on, that path is unavailable here: the
                # generated .superbuild/mc_rtc.yaml sets `LoadUserConfiguration: false`
                # (superbuild.nix:181), so ~/.config/mc_rtc/mc_rtc.yaml is ignored outright.
                # Without this explicit override the superbuild would silently run the
                # controller 5x slower than the gains/timings above were tuned for.
                configurations.RLKinovaBallBalancingMcController.timeStep = 0.001;

                configurations.RLKinovaBallBalancingMcController.devel.apps = [
                  # This derivation is never built: `inputsFrom` only reads its dependency
                  # lists to assemble the shell. So the two adjustments below cost nothing and
                  # do not affect the mc-rtc that actually ends up in the shell and on the
                  # runtime module paths — they only shape what the shell inherits.
                  (pkgs.mc-rtc.overrideAttrs (prev: {
                    # separateDebugInfo = true makes stdenv append the bare *path*
                    # build-support/setup-hooks/separate-debug-info.sh to nativeBuildInputs.
                    # make-shell types that option as `listOf package`, and a plain path fails
                    # the check ("is not of type `package'"). mc-kinova and
                    # mc-residual-estimation don't set it, which is why only mc-rtc trips this.
                    separateDebugInfo = false;

                    # mc-rtc pulls in qt.qtbase and wrapQtAppsHook (Qt5 here, since mc-rtc-nix
                    # derives qtVersion from the ROS distro), while the shell's ros-env brings
                    # Qt6. Both landing in the shell makes the Qt setup hook abort with
                    # "detected mismatched Qt dependencies" — the hook must go too, not just
                    # qtbase, since it propagates qtbase-dev by itself. Dropping them is safe:
                    # mc_rtc's CMakeLists never references Qt at all — these are here only for
                    # the PyQt5-based utils (mc_rtc_log_ui and friends), which the store-built
                    # mc-rtc in this same shell already provides.
                    buildInputs = lib.filter (dropQt) prev.buildInputs;
                    nativeBuildInputs = lib.filter (dropQt) prev.nativeBuildInputs;
                  }))
                ];
              };
          }
        ];

        mc-rtc-nix.overlays.ccache = inputs.ccache-trigger.value;

        flakoboros.packages.RLKinovaBallBalancingMcController = ./package.nix;
      }
    );
}
