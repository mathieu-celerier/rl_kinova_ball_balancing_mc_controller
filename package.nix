{
  lib,
  mkMcRtcController,
  cmake,
  mc-rtc,
  onnxruntime,
  python3,
}:

mkMcRtcController {
  pname = "RLKinovaBallBalancingMcController";
  version = "0.1.0";

  src = lib.cleanSource ./.;

  # python3 is only here for the postPatch rewrite of src/CMakeLists.txt below — stdenv does not
  # ship it, so without this the phase dies with "python3: command not found".
  nativeBuildInputs = [
    cmake
    python3
  ];
  propagatedBuildInputs = [
    mc-rtc
    onnxruntime
  ];

  doCheck = false;

  # Upstream vendors a prebuilt onnxruntime 1.22 under src/onnxruntime/ (headers + a
  # NO_DEFAULT_PATH find_library for the .so, plus an install(DIRECTORY ...) rule that copies
  # the vendored libonnxruntime*.so* straight into MC_RTC_LIBDIR). None of that belongs in a
  # nix build: we have a real onnxruntime in the store (currently 1.24.4 — its C++ API header
  # is unchanged across that range, so the 1.22-authored source builds against it with no code
  # changes), and shipping a second copy of libonnxruntime.so alongside the store one is both
  # wasteful and liable to load the wrong copy at runtime depending on RPATH/LD_LIBRARY_PATH
  # ordering.
  postPatch = ''
    # onnxruntime.dev installs the same onnxruntime_cxx_api.h in two places:
    #   $out/include/onnxruntime_cxx_api.h            (top-level, used by most consumers)
    #   $out/include/onnxruntime/onnxruntime_cxx_api.h (nested, mirrors the vendored layout)
    # Point at the nested one so it matches exactly what the vendored ONNXRUNTIME_INCLUDE_DIR
    # used to provide (src/onnxruntime/include/onnxruntime_cxx_api.h) with no other source
    # changes needed.
    #
    # Done with python3 (not substituteInPlace) because the two blocks being rewritten span
    # multiple lines and substituteInPlace's exact-match semantics make multi-line matches
    # fragile to reproduce byte-for-byte from a shell heredoc; a regex tolerates the source's
    # own whitespace instead of having to mirror it.
    python3 - src/CMakeLists.txt <<'PYEOF'
import re, sys
p = sys.argv[1]
s = open(p).read()

# Redirect the include dir at the store's onnxruntime.dev, and the find_library() call at
# the store's onnxruntime lib dir instead of the vendored, NO_DEFAULT_PATH-only search.
s = s.replace(
    "set(ONNXRUNTIME_INCLUDE_DIR ''${ONNXRUNTIME_ROOT}/include)",
    'set(ONNXRUNTIME_INCLUDE_DIR "${onnxruntime.dev}/include/onnxruntime")',
)
s, n = re.subn(
    r"find_library\(ONNXRUNTIME_LIB\s*"
    r"NAMES onnxruntime\s*"
    r"PATHS \$\{ONNXRUNTIME_LIB_DIR\}\s*"
    r"NO_DEFAULT_PATH\s*"
    r"REQUIRED\s*"
    r"\)",
    'find_library(ONNXRUNTIME_LIB\n  NAMES onnxruntime\n  PATHS "${onnxruntime}/lib"\n  REQUIRED\n)',
    s,
)
assert n == 1, "find_library(ONNXRUNTIME_LIB ...) block not found"

# Drop the install(DIRECTORY ...) rule that used to copy the vendored libonnxruntime*.so*
# into MC_RTC_LIBDIR. The nix-store onnxruntime is pulled in via propagatedBuildInputs
# instead, so shipping a second copy here would be redundant and could shadow it at runtime.
s, n = re.subn(
    r"install\(\s*DIRECTORY \$\{ONNXRUNTIME_LIB_DIR\}/\s*"
    r"DESTINATION \$\{MC_RTC_LIBDIR\}\s*"
    r"FILES_MATCHING\s*"
    r'PATTERN "libonnxruntime\*\.so\*"\s*'
    r"\)\n?",
    "",
    s,
)
assert n == 1, "install(DIRECTORY ...) block for the vendored .so not found"

open(p, "w").write(s)
PYEOF
  '';

  passthru.mc-rtc = {
    controller = {
      Enabled = "RLKinovaBallBalancingMcController";
      MainRobot = "KinovaBotaPlate";
    };
    # Not optional: etc/RLKinovaBallBalancingMcController.in.yaml's ObserverPipelines loads an
    # ExternalForcesObserver unconditionally at startup, and mc_rtc treats a missing observer
    # as a critical/fatal error, same as a missing plugin or controller.
    observers = [ "mc-external-forces-observer" ];

    # Picked up by mc-rtc-nix's mkControllerSuperbuild: with-suggested defaults to true, so
    # these get added to the generated superbuild shell automatically. mc-mujoco simulates the
    # ball-balancing task, and mc-kinova's passthru.mujocoRobots feeds it the matching robot
    # XML.
    suggests = {
      apps = [ "mc-mujoco" ];
      robots = [ "mc-kinova" ];
    };

    # Packages we hack on from source rather than consuming from the nix store. Only consulted
    # by the '-devel' shell, where mc-rtc-nix pulls in their *build* dependencies (inputsFrom)
    # and prepends $PWD/.superbuild/install to ControllerModulePaths / RobotModulePaths /
    # GlobalPluginPaths — so a local cmake install of these shadows the nix-built copy with no
    # flake edit, no hash, and no rebuild of the nix package.
    devel = {
      robots = [ "mc-kinova" ];
      observers = [ "mc-external-forces-observer" ];
    };
  };

  meta = with lib; {
    description = "RL ball-balancing Kinova controller (Acc-CBF-QP) for mc_rtc";
    homepage = "https://github.com/bastien-muraccioli/rl_kinova_ball_balancing_mc_controller";
    license = licenses.bsd2;
    platforms = platforms.all;
  };
}
