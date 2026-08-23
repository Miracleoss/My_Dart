# Pull Control Input Semantics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every operator-facing Pull force command increase in the same direction while preserving the existing `0.0 = bottom`, `1.0 = top` stroke coordinate and current four-shot physical targets.

**Architecture:** Add three explicit Pull command modes inside the existing `Class_Booster`: direct stroke ratio, open-loop force ratio, and sensor-based tension in grams. The open-loop force ratio is resolved through a configurable `0.95` low-force to `0.05` high-force stroke range before the existing motor position loop sees it; PREP_FIRE then consumes force ratios instead of raw position values.

**Tech Stack:** C++11, STM32 HAL, ARMClang/Keil MDK project, EIDE `unify_builder`, PowerShell static verification.

**Spec:** `MDK-ARM/docs/superpowers/specs/2026-08-23-pull-control-input-semantics-design.md`

## Global Constraints

- Modify only `User/Chariot/Inc/crt_booster.h` and `User/Chariot/Src/crt_booster.cpp`; do not create production or test source files.
- Do not modify Push calibration, Push state-machine behavior, or the `0.0 = bottom`, `1.0 = top` stroke convention.
- Do not enable the currently disabled PREP_FIRE tension PID path.
- Default open-loop force mapping is `0.0 -> 0.95`, `0.5 -> 0.50`, and `1.0 -> 0.05`.
- Preserve the four current physical Pull targets `0.39`, `0.40`, `0.42`, and `0.45` after migration.
- Reject non-finite inputs without changing the previous target or control mode.
- Keep comments concise but explicit at the three semantic boundaries: units and numeric direction, force-to-stroke inversion, and sensor-offline hold behavior.
- Per user instruction, do not add automated test files; use static review plus a complete ARMClang rebuild.

## File Structure

- Modify `User/Chariot/Inc/crt_booster.h`: own the Pull control-mode enum, public semantic APIs, getters, safety-range configuration, and Pull state fields.
- Modify `User/Chariot/Src/crt_booster.cpp`: own validation/clamping helpers, force-to-stroke conversion, mode dispatch, sensor-offline behavior, four-shot table migration, and PREP_FIRE integration.

---

### Task 1: Add explicit Pull command semantics without changing PREP_FIRE behavior

**Files:**
- Modify: `User/Chariot/Inc/crt_booster.h:29-45, 216-244, 253-262, 289-307, 355-463`
- Modify: `User/Chariot/Src/crt_booster.cpp:14-16, 297-483, 1140-1146`

**Interfaces:**
- Consumes: `Class_TensionMeter::Get_Tension()`, `Class_TensionMeter::Is_Online()`, `Class_DJI_Motor_C620::Set_Target_Radian(float)`, and the existing normalized Pull position maintained by calibration.
- Produces: `Enum_Pull_Control_Mode`, `Set_Pull_Stroke_Ratio(float)`, `Set_Pull_Force_Ratio(float)`, `Set_Target_Tension_Gram(float)`, `Configure_Pull_Force_Stroke_Range(float, float)`, `Update_Pull_Control(bool)`, `Get_Target_Pull_Stroke_Ratio()`, `Get_Target_Pull_Force_Ratio()`, `Get_Target_Tension_Gram()`, and `Get_Measured_Tension_Gram()`.

- [ ] **Step 1: Add the Pull mode and unit-explicit public declarations**

Add the enum near the existing Booster enums in `crt_booster.h`. The comment must state that force-oriented modes increase with increasing input, while stroke mode retains the physical bottom-to-top coordinate:

```cpp
/**
 * @brief Pull 输入语义。
 * FORCE_RATIO/TENSION 数值越大表示力量越大；STROKE_RATIO 始终为 0=底部、1=顶部。
 */
enum Enum_Pull_Control_Mode : uint8_t
{
    Pull_Control_Mode_STROKE_RATIO = 0,
    Pull_Control_Mode_FORCE_RATIO,
    Pull_Control_Mode_TENSION,
};
```

Replace the ambiguous Pull/tension declarations with these exact signatures. Keep the Push declarations unchanged:

```cpp
inline Enum_Pull_Control_Mode Get_Pull_Control_Mode();
inline float Get_Measured_Tension_Gram();
inline float Get_Target_Tension_Gram();
inline float Get_Target_Pull_Stroke_Ratio();
inline float Get_Target_Pull_Force_Ratio();

void Set_Pull_Stroke_Ratio(float stroke_ratio);
void Set_Pull_Force_Ratio(float force_ratio);
void Set_Target_Tension_Gram(float tension_g);
bool Configure_Pull_Force_Stroke_Range(float low_force_stroke_ratio,
                                       float high_force_stroke_ratio);
void Update_Pull_Control(bool is_first_run);
```

Remove the old public measured-tension setter; the measured value is owned by `TensionMeter.Get_Tension()` and has no live external setter caller. Move `Pull_Tension_Control(bool)` into the protected internal-function section so callers cannot bypass `Update_Pull_Control()` and its sensor-online guard.

- [ ] **Step 2: Add Pull state with explicit units and defaults**

In the protected Pull section, keep `target_position_pull` as the resolved motor stroke target and add the following fields. Replace `Measured_Tension` and `Target_Tension` with the unit-explicit names shown here:

```cpp
// Pull 位置坐标始终为 0=底部、1=顶部。
float target_position_pull = 0.5f;
float now_position_pull = 0.0f;

Enum_Pull_Control_Mode Pull_Control_Mode = Pull_Control_Mode_STROKE_RATIO;
float target_pull_force_ratio = 0.0f;

// 开环力量比例的安全映射端点：小力靠近顶部，大力靠近底部。
float low_force_stroke_ratio = 0.95f;
float high_force_stroke_ratio = 0.05f;

float Measured_Tension_Gram = 0.0f;
float Target_Tension_Gram = 42010.0f;
bool pull_tension_control_initialized = false;
```

Keep `target_tension_position_pull` as the last valid closed-loop stroke target. Its initialization remains `target_position_pull`.

- [ ] **Step 3: Implement the header getters with matching float types**

Replace the old integer tension getters and ambiguous Pull target getter with these inline definitions:

```cpp
inline Enum_Pull_Control_Mode Class_Booster::Get_Pull_Control_Mode()
{
    return Pull_Control_Mode;
}

inline float Class_Booster::Get_Measured_Tension_Gram()
{
    return Measured_Tension_Gram;
}

inline float Class_Booster::Get_Target_Tension_Gram()
{
    return Target_Tension_Gram;
}

inline float Class_Booster::Get_Target_Pull_Stroke_Ratio()
{
    return target_position_pull;
}

inline float Class_Booster::Get_Target_Pull_Force_Ratio()
{
    return target_pull_force_ratio;
}
```

Delete the old `Get_Measured_Tension()`, `Get_Target_Tension()`, `Get_Target_position_pull()`, `Set_Measured_Tension(int)`, `Set_Target_Tension(int)`, and `Set_Target_position_pull(float)` declarations/definitions. Repository search has no live external callers for these APIs.

- [ ] **Step 4: Add finite-value and unit-range helpers in the existing source file**

Add `<float.h>` after `#include "crt_booster.h"`, then add these file-local helpers near the private function section. The `FLT_MAX` comparison rejects NaN and both infinities without introducing a new module:

```cpp
#include <float.h>

static bool Pull_Is_Finite(float value)
{
    return value >= -FLT_MAX && value <= FLT_MAX;
}

static float Pull_Clamp_Unit_Ratio(float ratio)
{
    if (ratio < 0.0f)
    {
        return 0.0f;
    }
    if (ratio > 1.0f)
    {
        return 1.0f;
    }
    return ratio;
}
```

- [ ] **Step 5: Implement the three setters and configurable safety range**

Add these definitions before `Pull_Tension_Control()`. Keep the inversion comment because it explains the otherwise surprising subtraction:

```cpp
void Class_Booster::Set_Pull_Stroke_Ratio(float stroke_ratio)
{
    if (!Pull_Is_Finite(stroke_ratio))
    {
        return;
    }

    target_position_pull = Pull_Clamp_Unit_Ratio(stroke_ratio);
    Pull_Control_Mode = Pull_Control_Mode_STROKE_RATIO;
}

void Class_Booster::Set_Pull_Force_Ratio(float force_ratio)
{
    if (!Pull_Is_Finite(force_ratio))
    {
        return;
    }

    target_pull_force_ratio = Pull_Clamp_Unit_Ratio(force_ratio);
    // 机构力量与行程方向相反：力量比例越大，解析出的行程目标越小。
    target_position_pull = low_force_stroke_ratio
                         + target_pull_force_ratio
                         * (high_force_stroke_ratio - low_force_stroke_ratio);
    Pull_Control_Mode = Pull_Control_Mode_FORCE_RATIO;
}

void Class_Booster::Set_Target_Tension_Gram(float tension_g)
{
    if (!Pull_Is_Finite(tension_g))
    {
        return;
    }

    Target_Tension_Gram = tension_g < 0.0f ? 0.0f : tension_g;
    if (Pull_Control_Mode != Pull_Control_Mode_TENSION)
    {
        pull_tension_control_initialized = false;
    }
    Pull_Control_Mode = Pull_Control_Mode_TENSION;
}

bool Class_Booster::Configure_Pull_Force_Stroke_Range(float low_force_stroke,
                                                       float high_force_stroke)
{
    if (!Pull_Is_Finite(low_force_stroke)
        || !Pull_Is_Finite(high_force_stroke)
        || low_force_stroke < 0.0f
        || low_force_stroke > 1.0f
        || high_force_stroke < 0.0f
        || high_force_stroke > 1.0f
        || high_force_stroke >= low_force_stroke)
    {
        return false;
    }

    low_force_stroke_ratio = low_force_stroke;
    high_force_stroke_ratio = high_force_stroke;

    if (Pull_Control_Mode == Pull_Control_Mode_FORCE_RATIO)
    {
        target_position_pull = low_force_stroke_ratio
                             + target_pull_force_ratio
                             * (high_force_stroke_ratio - low_force_stroke_ratio);
    }
    return true;
}
```

- [ ] **Step 6: Implement centralized mode dispatch and sensor-offline hold**

Add `Update_Pull_Control()` before `Pull_Tension_Control()`. The offline branch must hold the current position on first entry and the last valid resolved position after a dropout:

```cpp
void Class_Booster::Update_Pull_Control(bool is_first_run)
{
    if (Pull_Control_Mode == Pull_Control_Mode_STROKE_RATIO
        || Pull_Control_Mode == Pull_Control_Mode_FORCE_RATIO)
    {
        Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Pull.Set_Target_Radian(target_position_pull);
        return;
    }

    const bool needs_initialization = is_first_run || !pull_tension_control_initialized;
    if (!TensionMeter.Is_Online())
    {
        if (needs_initialization)
        {
            target_tension_position_pull = Get_Now_position_pull();
        }
        // 传感器离线时禁止继续增力；保持最后一次有效行程目标。
        target_position_pull = target_tension_position_pull;
        Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Pull.Set_Target_Radian(target_position_pull);
        pull_tension_control_initialized = false;
        return;
    }

    // 外环尚未扣锁或正在等待稳定时，也要先保持上一次有效目标。
    Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    Motor_Pull.Set_Target_Radian(target_tension_position_pull);
    Pull_Tension_Control(needs_initialization);
    target_position_pull = target_tension_position_pull;
    pull_tension_control_initialized = true;
}
```

Inside `Pull_Tension_Control()`, update the unit-explicit getter calls:

```cpp
now_tension_value = Get_Measured_Tension_Gram();
target_tension_value = Get_Target_Tension_Gram();
```

After `target_tension_position_pull` is clamped inside `Pull_Tension_Control()`, keep the common resolved target synchronized and use it for the motor command:

```cpp
target_position_pull = target_tension_position_pull;
Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
Motor_Pull.Set_Target_Radian(target_position_pull);
```

At the 1 ms measurement update, replace the old assignment with:

```cpp
Measured_Tension_Gram = TensionMeter.Get_Tension();
```

- [ ] **Step 7: Run static API and scope checks**

Run from `MDK-ARM`:

```powershell
rg -n "Get_Measured_Tension\(|Get_Target_Tension\(|Get_Target_position_pull\(|Set_Target_Tension\(|Set_Target_position_pull\(" ..\User\Chariot ..\User\Interaction ..\User\Task
```

Expected: no matches. Then run:

```powershell
git diff --check -- ..\User\Chariot\Inc\crt_booster.h ..\User\Chariot\Src\crt_booster.cpp
```

Expected: no output and exit code `0`.

- [ ] **Step 8: Rebuild the firmware before integrating PREP_FIRE**

Run:

```powershell
& 'C:\Users\33456\.vscode\extensions\cl.eide-3.26.1\res\tools\win32\unify_builder\unify_builder.exe' --params-file '.\ZLLC_2026\builder.params' --rebuild --no-color
```

Expected: output contains `build successfully !`, followed by the program-size summary, with no compiler or linker error.

- [ ] **Step 9: Commit the semantic layer**

```powershell
git add -- ..\User\Chariot\Inc\crt_booster.h ..\User\Chariot\Src\crt_booster.cpp
git commit -m "refactor: add explicit pull control inputs"
```

---

### Task 2: Migrate PREP_FIRE from raw stroke targets to force ratios

**Files:**
- Modify: `User/Chariot/Src/crt_booster.cpp:675, 865-954`

**Interfaces:**
- Consumes: `Set_Pull_Force_Ratio(float)`, `Update_Pull_Control(bool)`, and `Get_Target_Pull_Stroke_Ratio()` from Task 1.
- Produces: PREP_FIRE targets expressed in increasing-force semantics while resolving to the unchanged physical stroke targets.

- [ ] **Step 1: Replace the four-shot position table with a force-ratio table**

Replace:

```cpp
float pull_position_task_C[4] = {0.39f, 0.40f, 0.42f, 0.45f};
```

with:

```cpp
// 由原行程 {0.39, 0.40, 0.42, 0.45} 反算；数值越大表示期望力量越大。
static constexpr float kPullForceRatioTaskC[4] = {
    0.622222f,
    0.611111f,
    0.588889f,
    0.555556f,
};
```

- [ ] **Step 2: Route PREP_FIRE through the semantic layer**

Replace the active Task C position block with this code:

```cpp
/*-------------Task C：开环力量比例，经中间层转换为 Pull 行程-------------*/
const uint8_t pull_idx = dart_fired_count < kMaxDartCount
                       ? static_cast<uint8_t>(dart_fired_count)
                       : static_cast<uint8_t>(kMaxDartCount - 1);

Booster->Set_Pull_Force_Ratio(kPullForceRatioTaskC[pull_idx]);
Booster->Update_Pull_Control(pull_loop_first_run);
pull_loop_first_run = false;

const float target_pull_stroke = Booster->Get_Target_Pull_Stroke_Ratio();
if (fabs(Booster->Motor_Pull.Get_Now_Radian() - target_pull_stroke) < 0.005f)
{
    prep_task_c_done = true;
}
/*-------------------------------------------------------------------------*/
```

Keep the existing four-second timeout and the READY/FIRE transitions unchanged. Do not uncomment the old tension-control block. Update any retained commented getter names to the new `..._Gram()` names so future debug code does not advertise removed APIs.

- [ ] **Step 3: Statically verify the migrated values resolve to the old targets**

Run this calculation without creating a test file:

```powershell
$forceRatios = 0.622222, 0.611111, 0.588889, 0.555556
$forceRatios | ForEach-Object { [Math]::Round(0.95 - 0.90 * $_, 6) }
```

Expected output:

```text
0.39
0.4
0.42
0.45
```

Also inspect the monotonic direction:

```text
0.622222 > 0.611111 > 0.588889 > 0.555556
0.39     < 0.40     < 0.42     < 0.45
```

This confirms that a larger force command maps to a smaller stroke target while preserving all four physical positions.

- [ ] **Step 4: Verify PREP_FIRE no longer bypasses the intermediate layer**

Run:

```powershell
rg -n "pull_position_task_C|Set_Target_Radian\(pull_|kPullForceRatioTaskC|Set_Pull_Force_Ratio|Update_Pull_Control" ..\User\Chariot\Src\crt_booster.cpp
```

Expected:

- no `pull_position_task_C` match;
- no active `Set_Target_Radian(pull_...)` PREP_FIRE target;
- one force-ratio table definition;
- PREP_FIRE calls `Set_Pull_Force_Ratio()` and `Update_Pull_Control()`.

- [ ] **Step 5: Review comments, scope, and diff hygiene**

Run:

```powershell
git diff --check -- ..\User\Chariot\Inc\crt_booster.h ..\User\Chariot\Src\crt_booster.cpp
git diff --stat -- ..\User\Chariot\Inc\crt_booster.h ..\User\Chariot\Src\crt_booster.cpp
git status --short
```

Expected:

- no whitespace errors;
- only the two intended source files are part of the implementation diff;
- pre-existing unrelated workspace changes remain untouched;
- comments explain semantics and safety decisions rather than restating each line.

- [ ] **Step 6: Perform the final full firmware rebuild**

Run:

```powershell
& 'C:\Users\33456\.vscode\extensions\cl.eide-3.26.1\res\tools\win32\unify_builder\unify_builder.exe' --params-file '.\ZLLC_2026\builder.params' --rebuild --no-color
```

Expected: output contains `build successfully !`, followed by the program-size summary, with no compiler or linker error.

- [ ] **Step 7: Commit PREP_FIRE integration**

```powershell
git add -- ..\User\Chariot\Inc\crt_booster.h ..\User\Chariot\Src\crt_booster.cpp
git commit -m "feat: express pull shot targets as force ratios"
```

- [ ] **Step 8: Record the required hardware acceptance checks in the handoff**

The completion report must state that software verification is complete but actual force monotonicity requires a guarded bench check. The operator should command force ratios `0.0`, `0.5`, and `1.0`, confirm resolved stroke targets `0.95`, `0.50`, and `0.05`, and verify that the tension sensor readings increase monotonically.
