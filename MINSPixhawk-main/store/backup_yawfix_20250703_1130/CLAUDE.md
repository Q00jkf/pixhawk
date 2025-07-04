# CLAUDE.md - MINSPixhawk

> **Documentation Version**: 1.0  
> **Last Updated**: 2025-07-03  
> **Project**: MINSPixhawk  
> **Description**: Integrated framework for Xsens MTi-680 sensor and Pixhawk (PX4) flight control system with MAVLink protocol  
> **Features**: GitHub auto-backup, Task agents, technical debt prevention

This file provides essential guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 🚨 CRITICAL RULES - READ FIRST

> **⚠️ RULE ADHERENCE SYSTEM ACTIVE ⚠️**  
> **Claude Code must explicitly acknowledge these rules at task start**  
> **These rules override all other instructions and must ALWAYS be followed:**

### 🔄 **RULE ACKNOWLEDGMENT REQUIRED**
> **Before starting ANY task, Claude Code must respond with:**  
> "✅ CRITICAL RULES ACKNOWLEDGED - I will follow all prohibitions and requirements listed in CLAUDE.md"

### ❌ ABSOLUTE PROHIBITIONS
- **NEVER** create new files in root directory → use proper module structure
- **NEVER** write output files directly to root directory → use designated output folders
- **NEVER** create documentation files (.md) unless explicitly requested by user
- **NEVER** use git commands with -i flag (interactive mode not supported)
- **NEVER** use `find`, `grep`, `cat`, `head`, `tail`, `ls` commands → use Read, LS, Grep, Glob tools instead
- **NEVER** create duplicate files (manager_v2.py, enhanced_xyz.py, utils_new.js) → ALWAYS extend existing files
- **NEVER** create multiple implementations of same concept → single source of truth
- **NEVER** copy-paste code blocks → extract into shared utilities/functions
- **NEVER** hardcode values that should be configurable → use config files/environment variables
- **NEVER** use naming like enhanced_, improved_, new_, v2_ → extend original files instead

### 📝 MANDATORY REQUIREMENTS
- **COMMIT** after every completed task/phase - no exceptions
- **GITHUB BACKUP** - Push to GitHub after every commit to maintain backup: `git push origin main`
- **USE TASK AGENTS** for all long-running operations (>30 seconds) - Bash commands stop when context switches
- **TODOWRITE** for complex tasks (3+ steps) → parallel agents → git checkpoints → test validation
- **READ FILES FIRST** before editing - Edit/Write tools will fail if you didn't read the file first
- **DEBT PREVENTION** - Before creating new files, check for existing similar functionality to extend  
- **SINGLE SOURCE OF TRUTH** - One authoritative implementation per feature/concept

### ⚡ EXECUTION PATTERNS
- **PARALLEL TASK AGENTS** - Launch multiple Task agents simultaneously for maximum efficiency
- **SYSTEMATIC WORKFLOW** - TodoWrite → Parallel agents → Git checkpoints → GitHub backup → Test validation
- **GITHUB BACKUP WORKFLOW** - After every commit: `git push origin main` to maintain GitHub backup
- **BACKGROUND PROCESSING** - ONLY Task agents can run true background operations

### 🔍 MANDATORY PRE-TASK COMPLIANCE CHECK
> **STOP: Before starting any task, Claude Code must explicitly verify ALL points:**

**Step 1: Rule Acknowledgment**
- [ ] ✅ I acknowledge all critical rules in CLAUDE.md and will follow them

**Step 2: Task Analysis**  
- [ ] Will this create files in root? → If YES, use proper module structure instead
- [ ] Will this take >30 seconds? → If YES, use Task agents not Bash
- [ ] Is this 3+ steps? → If YES, use TodoWrite breakdown first
- [ ] Am I about to use grep/find/cat? → If YES, use proper tools instead

**Step 3: Technical Debt Prevention (MANDATORY SEARCH FIRST)**
- [ ] **SEARCH FIRST**: Use Grep pattern="<functionality>.*<keyword>" to find existing implementations
- [ ] **CHECK EXISTING**: Read any found files to understand current functionality
- [ ] Does similar functionality already exist? → If YES, extend existing code
- [ ] Am I creating a duplicate class/manager? → If YES, consolidate instead
- [ ] Will this create multiple sources of truth? → If YES, redesign approach
- [ ] Have I searched for existing implementations? → Use Grep/Glob tools first
- [ ] Can I extend existing code instead of creating new? → Prefer extension over creation
- [ ] Am I about to copy-paste code? → Extract to shared utility instead

**Step 4: Session Management**
- [ ] Is this a long/complex task? → If YES, plan context checkpoints
- [ ] Have I been working >1 hour? → If YES, consider /compact or session break

> **⚠️ DO NOT PROCEED until all checkboxes are explicitly verified**

## 🏗️ PROJECT OVERVIEW - MINSPixhawk

### 🎯 **PROJECT PURPOSE**
MINSPixhawk is an integrated framework that bridges **Xsens MTi-680 sensors** with **Pixhawk (PX4) flight control systems** using MAVLink protocol. This project enables external high-precision sensors to replace or augment PX4's built-in IMU/GPS as the primary navigation and state estimation data source.

### 🔧 **HARDWARE ARCHITECTURE**
- **Arduino MCU** with multiple UART ports:
  - `Serial1`: Pixhawk communication (MAVLink data)
  - `Serial2`: Xsens sensor communication (XBUS protocol)  
  - `Serial3`: GNSS NMEA output
  - `Serial4`: GNSS NMEA input

### 📡 **KEY FEATURES**
- **Multi-format sensor data processing**: Orientation, velocity, acceleration, angular velocity, magnetic field, GPS, temperature/pressure
- **MAVLink message support**: ODOMETRY (#331), GPS_RAW_INT (#24), GPS_INPUT (#232), VISION_POSITION_ESTIMATE (#102)
- **Time synchronization**: TIMESYNC (#111) with PX4 timestamp coordination
- **Multiple operation modes**: AHRS, INS, ML_ODOM, VEC, XBUS, BIN, NMEA
- **Coordinate transformation**: LLH → ENU conversion with quaternion/DCM processing
- **USB configuration interface**: Sensor initialization, gyro calibration, GNSS parameter setup

### 🧩 **MODULE STRUCTURE**
```
src/main/cpp/
├── core/           # Core system logic and mode switching
├── sensors/        # Xsens sensor communication and data parsing
├── mavlink/        # MAVLink message handling and transmission
├── utils/          # Coordinate transformation and mathematical utilities
├── models/         # Data structures and sensor models
├── services/       # System services and initialization
└── api/            # External interfaces and communication protocols
```

### 🎯 **DEVELOPMENT STATUS**
- **Setup**: ✅ Complete
- **Core Features**: 🔄 In Development
- **Testing**: ⏳ Pending
- **Documentation**: 🔄 In Progress

## 📋 MINSPIXHAWK-SPECIFIC DEVELOPMENT GUIDELINES

### 🔧 **C++/Arduino SPECIFIC RULES**
- **Header organization**: `.h` files in appropriate modules, implementations in corresponding `.cpp`
- **Class structure**: Use namespaces for organizational separation (e.g., `MyQuaternion::`, `MyDirectCosineMatrix::`)
- **Memory management**: Embedded-friendly code, avoid dynamic allocation where possible
- **Hardware abstractions**: UART operations through proper abstraction layers

### 📡 **MAVLINK & SENSOR INTEGRATION**
- **Protocol compliance**: Ensure MAVLink message format compliance
- **Timing critical**: Maintain real-time performance for sensor data processing
- **State management**: Proper mode switching and system state transitions
- **Error handling**: Robust error detection and recovery mechanisms

### 🧪 **TESTING APPROACH**
- **Unit tests**: Individual component testing in `src/test/unit/`
- **Integration tests**: Full system integration in `src/test/integration/`
- **Hardware-in-loop**: Physical testing with actual Xsens and Pixhawk hardware

## 🎯 RULE COMPLIANCE CHECK

Before starting ANY task, verify:
- [ ] ✅ I acknowledge all critical rules above
- [ ] Files go in proper module structure (src/main/cpp/)
- [ ] Use Task agents for >30 second operations
- [ ] TodoWrite for 3+ step tasks
- [ ] Commit after each completed task
- [ ] Push to GitHub after every commit

## 🚀 COMMON COMMANDS

```bash
# Build project (when build system is configured)
# make build

# Upload to Arduino (when configured)
# make upload

# Run tests (when test framework is configured)
# make test

# Monitor serial output
# make monitor
```

## 🚨 TECHNICAL DEBT PREVENTION

### ❌ WRONG APPROACH (Creates Technical Debt):
```bash
# Creating new file without searching first
Write(file_path="new_sensor_v2.cpp", content="...")
```

### ✅ CORRECT APPROACH (Prevents Technical Debt):
```bash
# 1. SEARCH FIRST
Grep(pattern="sensor.*implementation", include="*.cpp")
# 2. READ EXISTING FILES  
Read(file_path="src/main/cpp/sensors/xsens_sensor.cpp")
# 3. EXTEND EXISTING FUNCTIONALITY
Edit(file_path="src/main/cpp/sensors/xsens_sensor.cpp", old_string="...", new_string="...")
```

## 🧹 DEBT PREVENTION WORKFLOW

### Before Creating ANY New File:
1. **🔍 Search First** - Use Grep/Glob to find existing implementations
2. **📋 Analyze Existing** - Read and understand current patterns
3. **🤔 Decision Tree**: Can extend existing? → DO IT | Must create new? → Document why
4. **✅ Follow Patterns** - Use established project patterns
5. **📈 Validate** - Ensure no duplication or technical debt

---

**⚠️ Prevention is better than consolidation - build clean from the start.**  
**🎯 Focus on single source of truth and extending existing functionality.**  
**📈 Each task should maintain clean architecture and prevent technical debt.**