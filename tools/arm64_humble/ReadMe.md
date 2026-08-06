# Windows ARM64 Humble build environment

This environment runs an actual `linux/arm64` Ubuntu 22.04 ROS 2 Humble
container through Docker Buildx and QEMU. It catches architecture-specific
compile/link errors without changing the Jetson workspace.

## Prerequisites

First inspect the current machine from PowerShell:

```powershell
.\tools\arm64_humble\CHECK_WINDOWS_PREREQUISITES.ps1
```

If WSL2 or Docker is missing, follow the commands printed by that script and
run the following from an **Administrator PowerShell**:

```powershell
.\tools\arm64_humble\INSTALL_WINDOWS_PREREQUISITES.ps1
```

Restart Windows, start Docker Desktop, wait until its Linux engine reports
that it is running, and then execute a build. No Ubuntu desktop VM or separate
ROS installation is required on Windows.

## Builds

Fast project-package validation:

```powershell
.\tools\arm64_humble\BUILD_ARM64_HUMBLE.ps1 -Mode quick
```

Full ARM64 validation including Cartographer availability and the bundled
Orbbec wrapper/SDK dependency graph:

```powershell
.\tools\arm64_humble\BUILD_ARM64_HUMBLE.ps1 -Mode full
```

Force a clean image build with `-NoCache`. The first emulated ARM64 build can
take tens of minutes; later builds reuse Docker layers.

`full` also verifies that the bundled Orbbec SDK ELF is AArch64 and that the
`orbbec_camera` package is present in the resulting ROS overlay. A successful
x86-only build therefore cannot produce a false pass.

## Boundary

Passing this build proves Ubuntu/ROS/API/ABI and package compilation only.
Jetson CUDA, USB camera access, serial ports, clock behavior, GPU load, memory
pressure and real-time navigation still require a hardware run.
