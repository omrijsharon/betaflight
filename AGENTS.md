# AGENTS.md

## Build

This repo is built from WSL, not from native PowerShell.

Important:

- The camera-lock feature requires `-DUSE_FINAL`.
- Do not flash a build unless you have verified that `USE_FINAL` actually made it into the compiled firmware.

WSL repo path:

```bash
/mnt/c/Users/tamipinhasi/Documents/repos/betaflight
```

Target:

```bash
IFLIGHT_BLITZ_F435
```

### Minimal `USE_FINAL` build

Run from WSL:

```bash
cd /mnt/c/Users/tamipinhasi/Documents/repos/betaflight
make IFLIGHT_BLITZ_F435 EXTRA_FLAGS="-DUSE_FINAL"
```

### Full branded build

This exact build command is known to work when launched from PowerShell through `wsl bash -lc`:

```powershell
$flags = "-D'BUILD_KEY=7487ee54f7cfa20bdcc18794a423b4e3' -D'RELEASE_NAME=4.5.2' -DCLOUD_BUILD -DUSE_ACRO_TRAINER -DUSE_BARO_ALTHOLD -DUSE_FINAL -DUSE_DSHOT -DUSE_GPS -DUSE_GPS_PLUS_CODES -DUSE_LED_STRIP -DUSE_MAG -DUSE_OSD -DUSE_OSD_HD -DUSE_OSD_SD -DUSE_PINIO -DUSE_SERIALRX -DUSE_SERIALRX_CRSF -DUSE_TELEMETRY -DUSE_TELEMETRY_CRSF -DUSE_VTX"
$cmd = "cd /mnt/c/Users/tamipinhasi/Documents/repos/betaflight && make IFLIGHT_BLITZ_F435 EXTRA_FLAGS=`"$flags`""
wsl bash -lc $cmd
```

### Required verification before flashing

Always verify the built firmware identity after a `USE_FINAL` build.

In this repo, [version.h](C:\Users\tamipinhasi\Documents\repos\betaflight\src\main\build\version.h) is intentionally set so:

- with `-DUSE_FINAL` the firmware name is `betaflight_final_3.0.1`
- without `-DUSE_FINAL` the firmware name is `betaflight_final_2.1`

That means:

1. build the firmware
2. flash it
3. connect to CLI
4. run:

```text
version
```

Expected result:

- `betaflight_final_3.0.1` means `USE_FINAL` is present
- `betaflight_final_2.1` means `USE_FINAL` did not make it into the build and the firmware must not be used for camera-lock testing

For camera-lock builds, also verify:

```text
get camera_lock_port
```

If that setting is missing, treat the firmware as not correctly built with the expected camera-lock feature set.

### Output

Successful builds produce:

```text
./obj/betaflight_4.5.2_AT32F435G_IFLIGHT_BLITZ_F435.hex
```

## Notes

- The string-valued defines in `EXTRA_FLAGS` require careful quoting.
- The PowerShell wrapper above is the preferred way to run the full branded build from this machine.
- If you are already inside WSL, run `make` directly there and avoid extra shell wrapping.
- `DEBUG_CAMERA_LOCK` existing in the UI is not enough to prove `USE_FINAL`, because the debug enum/name is not gated by `USE_FINAL`.
