# STM32F103RB Port Scaffold

This folder is a scaffold for the planned `STM32F103RB` migration. It is not a buildable project by itself.

The intended workflow is:

1. Create a fresh `NUCLEO-F103RB` STM32CubeIDE project.
2. Copy the portable application files listed in `portable_files.txt`.
3. Follow the pin/peripheral recipe in `../../docs/stm32f103rb_migration.md`.
4. Build, flash, and validate on hardware.

The reason this folder is not a standalone firmware repo is that the correct F103 startup files, HAL, CMSIS package, linker script, and Cube-generated init code should come directly from a fresh CubeMX/CubeIDE generation step.
