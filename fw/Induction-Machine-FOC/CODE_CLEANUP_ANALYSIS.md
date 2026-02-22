# Code Cleanup Analysis - Unused Functions & Variables

## DEPRECATED FUNCTIONS TO REMOVE

### 1. **FOC_StartOpenLoop()** (Lines 371-382 in foc_control.c)
- **Status:** DEPRECATED - marked in code comments
- **Used by:** NOT CALLED anywhere in codebase
- **Reason for removal:** Replaced by `FOC_StartCalibration()` for proper motor startup
- **Location:** 
  - Declaration: [foc_control.h](foc_control.h#L171)
  - Implementation: [foc_control.c](foc_control.c#L371)
- **Impact:** Safe to remove - no dependencies
- **Size saved:** ~30 bytes

### 2. **FOC_StopOpenLoop()** (Lines 384-392 in foc_control.c)
- **Status:** DEPRECATED - marked in code comments  
- **Used by:** NOT CALLED anywhere in codebase
- **Reason for removal:** Replaced by `FOC_Disable()` for stopping control
- **Location:**
  - Declaration: [foc_control.h](foc_control.h#L179)
  - Implementation: [foc_control.c](foc_control.c#L384)
- **Impact:** Safe to remove - no dependencies
- **Size saved:** ~50 bytes

### 3. **FOC_ManualCalibration()** (Contains unused parameters - Line 625-632 in foc_control.c)
- **Status:** Parameters marked `(void)Unused`
- **Used by:** NOT CALLED in codebase
- **Reason:** Also marked with `// DEPRECATED` comment
- **Impact:** Can be removed or heavily simplified
- **Size saved:** ~40 bytes

---

## UNUSED SOURCE FILES

### 1. **motor_control_example.c**
- **Location:** `/Core/Src/motor_control_example.c`
- **Purpose:** Example/reference implementation
- **Status:** NOT COMPILED/LINKED into final binary
- **Verification:** Not included in CMakeLists.txt build targets
- **Impact:** Safe to remove - no dependencies
- **Size saved:** ~5-10 KB (file size on disk)

### 2. **vf_control.c** (Voltage/Frequency Control)
- **Location:** `/Core/Src/vf_control.c`
- **Purpose:** V/F control mode (alternative to IRFOC)
- **Status:** NOT USED - your system uses IRFOC exclusively
- **Verification:** No function calls to `VF_*` functions in main.c
- **Impact:** Safe to remove if keeping IRFOC only
- **Size saved:** ~10-15 KB (file size on disk), ~2-3 KB binary

### 3. **sine_op.c / sine_op.h** (Sine Operation)
- **Location:** `/Core/Src/sine_op.c` and `/Core/Inc/sine_op.h`
- **Purpose:** 256-entry sine lookup table generation
- **Status:** USED by Motor_GenerateSinusoid() function
- **Verification:** Called from main.c line ~750
- **IMPORTANT:** DO NOT REMOVE - still used for sine wave generation
- **Alternative:** If removing sine generation, can delete this

---

## UNUSED VARIABLES & PARAMETERS

### 1. **Unused Parameters in FOC_StartOpenLoop()** (foc_control.c:375-378)
```c
(void)align_time_ms;    // Unused
(void)ramp_time_ms;     // Unused
(void)target_freq_hz;   // Unused
(void)flux_id_a;        // Unused
```
- **Impact:** Function can be completely removed instead of fixing parameters
- **Size saved:** ~10 bytes

### 2. **Unused Parameters in FOC_ManualCalibration()** (foc_control.c:630-631)
```c
(void)theta_rotor;      // Unused
(void)duration_ms;      // Unused
```
- **Impact:** Can be removed with the function
- **Size saved:** ~5 bytes

### 3. **DMA Buffers** (main.c:128-130)
```c
static uint32_t dma_buffer_phase_a[DMA_BUFFER_SIZE];
static uint32_t dma_buffer_phase_b[DMA_BUFFER_SIZE];
static uint32_t dma_buffer_phase_c[DMA_BUFFER_SIZE];
```
- **Status:** ACTIVELY USED by Motor_GenerateSinusoid()
- **DO NOT REMOVE**
- **Size:** 3 × 256 × 4 bytes = 3 KB RAM

### 4. **Phase Variables** (main.c:133-134)
```c
static uint32_t phase_accumulator = 0;
static uint32_t phase_increment = 0;
```
- **Status:** ACTIVELY USED by Motor_GenerateSinusoid()
- **DO NOT REMOVE**

### 5. **temperature_VTSO** (main.c:125)
```c
float temperature_VTSO = 0.0f;
```
- **Status:** ACTIVELY USED - computed in GetBoardTemperature() (line 787) and returned (line 790)
- **DO NOT REMOVE** - returned via function return value
- **Size:** 4 bytes RAM

### 6. **v_alphabeta** (main.c:1723)
```c
Clarke_Out_t v_alphabeta = {0.0f, 0.0f};
```
- **Status:** ACTIVELY USED - stores FOC voltage output
- **DO NOT REMOVE**

---

## COMMENTED-OUT CODE TO REMOVE

### In main.c (HAL_ADCEx_InjectedConvCpltCallback):
```c
// uint32_t start_cycles = DWT->CYCCNT;  // Line 1738
// ...
// foc_isr_cycles_last = DWT->CYCCNT - start_cycles;  // Line 1816
// if (foc_isr_cycles_last > foc_isr_budget_cycles) {  // Line 1817
//     foc_fault_overrun = 1U;
//     FOC_EnterSafeState();
// }
```
- **Status:** Commented-out profiling code
- **Purpose:** Performance monitoring (no longer needed after optimization)
- **Size saved:** ~40 bytes
- **Safe to remove:** YES

### In foc_control.c:
```c
// foc_ctrl.Lm = 1.36f;  // Line 140
// foc_ctrl.Ls = 1.5540f;
// foc_ctrl.Lr = 1.5540f;
```
- **Status:** Commented-out alternative motor parameters
- **Size saved:** ~10 bytes
- **Safe to remove:** YES

---

## SUMMARY TABLE

| Item | Type | Used | Removable | Size | Priority |
|------|------|------|-----------|------|----------|
| FOC_StartOpenLoop() | Function | NO | YES | ~30B | HIGH |
| FOC_StopOpenLoop() | Function | NO | YES | ~50B | HIGH |
| FOC_ManualCalibration() | Function | NO | YES | ~40B | HIGH |
| motor_control_example.c | File | NO | YES | ~5-10KB | MEDIUM |
| vf_control.c | File | NO | YES* | ~2-3KB | LOW** |
| sine_op.c | File | YES | NO | (used) | - |
| Commented code (ISR profiling) | Code | NO | YES | ~40B | LOW |
| Commented motor params | Comment | NO | YES | ~10B | LOW |

**\*Only if you're sure V/F control won't be needed  
**\*\*Low priority because it's only on disk, not in binary

---

## RECOMMENDED CLEANUP SEQUENCE

### **Phase 1: High-Impact, Safe Removals** (5+ KB saved)
1. Delete `motor_control_example.c` - NOT used, not compiled
2. Remove `FOC_StartOpenLoop()` and `FOC_StopOpenLoop()` functions
3. Remove `FOC_ManualCalibration()` function
4. Delete corresponding declarations from `foc_control.h`

### **Phase 2: Medium-Impact Removals** (2-3 KB binary)
5. Remove commented-out profiling code in ISR (~40 bytes)
6. Remove commented-out motor parameters (~10 bytes)
7. **Optional:** Delete `vf_control.c` if V/F control definitively not needed

### **Phase 3: Code Review** (Optional optimization)
8. Check if any other files import the deleted functions
9. Verify no external code calls deprecated functions

---

## FILES TO KEEP (ACTIVELY USED)

✅ **Do NOT delete:**
- `foc_control.c` - Core IRFOC implementation
- `sine_op.c / sine_op.h` - Used by sine wave generation
- `encoder.c / encoder.h` - Used for rotor position feedback
- `svpwm.c / svpwm.h` - SVPWM modulation
- `current_sense.c / current_sense.h` - Current measurement
- `foc_math.c / foc_math.h` - Clarke/Park transforms
- All HAL initialization files

---

## BINARY SIZE IMPACT

**Current FLASH usage:** 111,420 bytes (21.25%)

**After cleanup:**
- Remove 3 deprecated functions: **-130 bytes**
- Remove commented code: **-50 bytes**
- **Total savings: ~180 bytes FLASH**

**Not included above (disk only, not in binary):**
- motor_control_example.c: ~5-10 KB disk

---

## ACTION ITEMS

**To execute cleanup, remove:**
```
1. Entire FOC_StartOpenLoop() function (foc_control.c:371-382)
2. Entire FOC_StopOpenLoop() function (foc_control.c:384-392)
3. Entire FOC_ManualCalibration() function (foc_control.c:625-632)
4. Declarations of above 3 functions (foc_control.h:165-180)
5. Commented profiling code (main.c:1716-1725, 1816-1821)
6. Commented motor parameters (foc_control.c:139-143)
7. **Optional:** Delete motor_control_example.c and vf_control.c
```

