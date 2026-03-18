# AGENTS.md

Repository guidance for coding agents working in `Infantry_01`.

## Project Overview

- Target: STM32F407 firmware for an infantry robot controller.
- Language: C11 with STM32 HAL, FreeRTOS, USB device middleware, and CMSIS.
- Build system: CMake + Ninja using `arm-none-eabi-*` cross tools.
- Primary app code lives in `Application/`, `ALL_Task/`, `Bsp/`, `Components/`, and `Algorithm/`.
- Generated platform code lives in `Core/`, `USB_DEVICE/`, `cmake/stm32cubemx/`, and `Infantry_01.ioc`.
- Vendor code lives in `Drivers/` and `Middlewares/`; avoid editing unless the task explicitly requires it.

## Rule Files

- No `.cursor/rules/` directory was found during analysis.
- No `.cursorrules` file was found during analysis.
- No `.github/copilot-instructions.md` file was found during analysis.
- If any of those files are added later, treat them as higher-priority agent instructions and merge them into your workflow.

## Repository Layout

- `Application/`: shared control state, robot logic, filtering, auto-aim interfaces.
- `ALL_Task/`: FreeRTOS task entry points such as gimbal, chassis, motor, and sensor tasks.
- `Bsp/`: board support code for CAN, UART, USB CDC, LEDs, and low-level I/O.
- `Components/`: reusable device drivers and abstractions such as motors, remotes, BMI088, IST8310.
- `Algorithm/`: algorithm modules such as Mahony AHRS.
- `Core/`: STM32CubeMX-generated startup and peripheral init code.
- `USB_DEVICE/`: STM32 USB device integration.
- `cmake/`: toolchain and CubeMX CMake glue.

## Build Commands

Prerequisites:

- `cmake >= 3.22`
- `ninja`
- `arm-none-eabi-gcc`, `arm-none-eabi-g++`, `arm-none-eabi-objcopy`, `arm-none-eabi-size` on `PATH`

Preferred configure/build flow:

```sh
cmake --preset Debug
cmake --build --preset Debug
```

Other supported presets:

```sh
cmake --preset RelWithDebInfo
cmake --build --preset RelWithDebInfo

cmake --preset Release
cmake --build --preset Release

cmake --preset MinSizeRel
cmake --build --preset MinSizeRel
```

Equivalent manual configure command:

```sh
cmake -S . -B build/Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
```

Useful build helpers:

```sh
cmake --list-presets
cmake --build --preset Debug --verbose
cmake --build build/Debug --target clean
```

Build outputs:

- The main target is `Infantry_01`.
- The linker emits an ELF image and a map file.
- `CMAKE_EXPORT_COMPILE_COMMANDS` is enabled, so `build/<preset>/compile_commands.json` is generated after configure.

## Test Commands

- There is currently no automated test framework configured in this repository.
- No `enable_testing()`, `add_test()`, CTest config, Unity/Ceedling setup, or similar test runner was found.
- There is therefore no supported "run all tests" command today.
- There is also no supported "run a single test" command today.

If you need validation, use the closest available checks:

```sh
cmake --build --preset Debug
```

- For task-specific work, prefer targeted compile-time validation by rebuilding after each change.
- If a future task introduces CTest, document both of these forms in this file:

```sh
ctest --test-dir build/Debug --output-on-failure
ctest --test-dir build/Debug -R <test_name> --output-on-failure
```

## Lint / Static Analysis Commands

- No dedicated lint target is configured in CMake.
- No repository-level `.clang-format`, `.clang-tidy`, `cpplint`, or `uncrustify` config was found.
- The compiler warnings in `cmake/gcc-arm-none-eabi.cmake` are the current baseline checks: `-Wall -Wextra -Wpedantic`.

Recommended local static-analysis commands when needed:

```sh
cmake --preset Debug
clang-tidy -p build/Debug Application/robot_global.c
clang-tidy -p build/Debug ALL_Task/gimbal_task.c
```

- Treat `clang-tidy` as optional ad hoc analysis, not an enforced project command.
- If you add lint automation, keep it scoped to user code and exclude vendor/generated trees by default.

## Files And Areas To Edit Carefully

- Prefer changes in `Application/`, `ALL_Task/`, `Bsp/`, `Components/`, and `Algorithm/`.
- Treat `Drivers/` and `Middlewares/` as third-party code; do not reformat or refactor them casually.
- Treat `Core/` and `USB_DEVICE/` as CubeMX-generated; preserve generated structure and user-code markers.
- If editing generated files such as `Core/Src/main.c`, keep changes inside `USER CODE BEGIN/END` blocks whenever possible.
- Do not hand-edit `cmake/stm32cubemx/CMakeLists.txt` unless the build integration itself is the task.
- Do not rename or move files referenced explicitly in `CMakeLists.txt` without updating the target source list.

## Code Style

### General

- Follow the existing C style in the surrounding directory instead of imposing a new style.
- Keep functions small and hardware-oriented; favor clear control flow over abstraction for its own sake.
- Preserve existing Chinese or mixed-language comments when touching a file that already uses them.
- For new comments, explain hardware intent, control logic, units, or safety constraints; do not narrate obvious syntax.
- Avoid broad formatting-only diffs, especially in generated or vendor code.

### Includes

- Put the matching header first in each `.c` file when there is one.
- Then include STM32 HAL / FreeRTOS / middleware headers.
- Then include standard library headers such as `<stdint.h>`, `<string.h>`, `<math.h>`, `<stdio.h>`.
- Then include cross-module project headers.
- Prefer project-relative includes that already match local conventions; do not churn include paths without a reason.

### Formatting

- Use 4 spaces for indentation; do not introduce tabs in manually maintained files.
- Keep braces on the same line for functions, `if`, `while`, and `else`, matching the hand-written code.
- Use spaces around operators and after commas.
- Keep macro definitions vertically readable when grouped.
- Keep one statement per line unless a compact loop or guard is already conventional nearby.

### Types

- Use the project's aliases from `Application/struct_typedef.h` where surrounding code already relies on them: `fp32`, `fp64`, `bool_t`, `uint8_t`, etc.
- Use explicit-width integer types for protocol data, CAN payloads, registers, and packed structures.
- Use `typedef enum` for mode/state groups and suffix names consistently, such as `_e` for enums.
- Use `typedef struct` for shared data packets and state aggregates.
- Mark packed wire-format structs explicitly when required by the protocol.
- Prefer `const` for read-only handles, buffers, and pointers passed into functions.

### Naming

- Macros are uppercase with underscores, for example `RC_DEADZONE` and `CAN_GM6020_YAW_ID`.
- Global structs and typedef aliases often use snake_case with suffixes like `_t` or `_e`; keep that pattern.
- Public functions use either `Module_Action` style or lower_snake_case depending on the module; match the file you are editing.
- Static helpers may use file-local CamelCase or snake_case in existing code; do not rename just for consistency churn.
- Keep hardware instance strings and IDs stable, e.g. `"GM6020_YAW"`, `"uart1_dma"`, CAN IDs, and task names.

### Error Handling And Safety

- Follow the project's fail-fast approach for hardware init failures: call `Error_Handler()` when HAL setup fails.
- In runtime control code, prefer explicit safe-state fallbacks over silent failure.
- When remote, sensor, or vision data is stale, force outputs to a safe disabled state before attempting recovery.
- Clamp actuator commands to mechanical and protocol limits near the control site.
- Reset debounce/state-machine counters when switching modes or on disconnect conditions.
- Avoid blocking calls in high-frequency tasks unless the task already uses that pattern intentionally.

### RTOS And Concurrency

- Assume task code is concurrent; shared control state such as `robot_ctrl` must be updated deliberately.
- Keep task loops deterministic and easy to audit.
- Use `osDelay()` only where timing slack is acceptable.
- Prefer static storage for long-lived device structures and FreeRTOS objects when that matches the module pattern.
- Be careful in interrupt callbacks: keep work short, ISR-safe, and queue/semaphore based.

### Hardware / Firmware Conventions

- Keep units explicit in names or comments when values are angles, radians, degrees, RPM, current, milliseconds, or ticks.
- Do not weaken mechanical limit checks, motor safeties, or watchdog-like timeout logic without strong justification.
- Preserve startup ordering when initialization depends on sensors, buses, or FreeRTOS setup.
- Keep CAN, UART, USB, and IMU protocol parsing tightly aligned with existing struct layouts and IDs.

## Agent Workflow Recommendations

- Read the target module and its paired header before editing.
- Check whether a file is generated, vendor, or hand-maintained before making structural changes.
- Rebuild with `cmake --build --preset Debug` after code changes whenever toolchain access is available.
- If you modify interfaces used across tasks or BSP/components, inspect all call sites before finishing.
- When documenting commands for users, be explicit that automated tests are currently absent.
