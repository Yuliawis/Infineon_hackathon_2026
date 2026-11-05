# Installing ModusToolbox and Generating the VS Code Project

This guide shows how to install ModusToolbox on Windows and open the
**Hackathon Self-Driving Car** project (`student-challenge-main`) in Visual Studio Code.

The project targets the **PSoC 6 BLE Prototyping Kit (CY8CPROTO-063-BLE)**. It is a
dual-core ModusToolbox application:

| Folder | Content |
|---|---|
| `proj_cm4` | Main application (CM4 core). Write your car logic here. |
| `proj_cm0p` | BLE stack (CM0+ core). |
| `cores_shared` | Code shared by both cores (IPC definitions). |
| `bsps/TARGET_APP_CY8CPROTO-063-BLE` | Board support package. |
| `doc` | Documentation (see `API_UserManual.md`). |

---

## 1. Requirements

- Windows 10 or 11, 64-bit. MacOS or Linux.
- About 10 GB of free disk space.
- Internet access (libraries are downloaded from GitHub).
- A USB Micro-B cable for the kit.

> **Important:** The project configuration was saved with **ModusToolbox 3.9**.
> Older versions (for example 3.5) cannot open it and the build fails with
> *"last saved with an incompatible version of the tools"*.
> Install **ModusToolbox Tools Package 3.9 or newer**.

---

## 2. Install Visual Studio Code

1. Download VS Code from <https://code.visualstudio.com/> and install it.
2. Start VS Code. Open the Extensions view (`Ctrl+Shift+X`) and install:

| Extension | ID | Purpose |
|---|---|---|
| C/C++ | `ms-vscode.cpptools` | Code navigation and IntelliSense |
| Cortex-Debug | `marus25.cortex-debug` | Debugging the PSoC 6 |
| ModusToolbox Assistant (optional) | `Infineon.modustoolbox-assistant` | ModusToolbox commands inside VS Code |
---

## 3. Install ModusToolbox

1. Open <https://www.infineon.com/modustoolbox> and sign in with your Infineon account.
2. Download **ModusToolbox Setup** for Windows and run it.
3. In ModusToolbox Setup, select these packages (choose the newest versions):
   - **ModusToolbox Tools Package** (3.9)
   - **ModusToolbox Programming Tools** (OpenOCD, fw-loader and drivers for the on-board KitProg3 programmer)
   - **Arm GCC toolchain 14** (GCC_ARM compiler used by this project)
4. Accept the license agreements and keep the **default installation folders**.
5. When OS asks to install the USB drivers, accept.
6. Restart the computer when the installation is finished.

The Tools Package is installed to the following folder. The project finds it
automatically:

```text
C:\Users\<your-user>\ModusToolbox\tools_3.x
or
/opt/Tools/ModusToolbox/...
```

### Custom installation folder

If you install ModusToolbox in another folder, set a user environment variable
so the project can find the tools (use forward slashes):

```text
Windows Only:
Name:  CY_TOOLS_PATHS
Value: D:/MyTools/ModusToolbox/tools_3.9
```

Alternatively, add the path to `CY_TOOLS_PATHS+=` in `common_app.mk`.

### Check the installation

Open **modus-shell** (Start menu → *ModusToolbox* → *modus-shell*, or run
`C:\Users\<your-user>\ModusToolbox\tools_3.x\modus-shell\Cygwin.bat`) and type:

```bash
make --version
```

GNU Make prints its version if the shell works.

---

## 4. Copy the project

Copy or unzip the project folder `student-challenge-main` to a short path
**without spaces or non-English characters**, for example:

```text
C:\Projects\student-challenge-main
```

The project downloads its shared libraries into a folder named `mtb_shared`.

```text
C:\Projects\
|-- mtb_shared\              <- created by "make getlibs"
`-- student-challenge-main\  <- the project
```

---

## 5. Download the libraries

The repository contains only the library references (`*.mtb` files and
`deps/assetlocks.json`). The library sources must be downloaded only once.

Open **modus-shell**, go to the project folder and download the libraries.
In modus-shell, drive `C:` is `/cygdrive/c`:

```bash
cd /cygdrive/c/Projects/student-challenge-main
make getlibs
```

The command finishes with `mtbgetlibs: completed in ... milliseconds` and
no `ERROR` lines.

Alternative: start **ModusToolbox Dashboard**, select the project folder and
click **Library Manager** → **Update**.

---

## 6. Generate the VS Code project

In modus-shell, in the project folder, run:

```bash
make vscode
```

This command:

- runs the configurators and generates the configuration sources (`GeneratedSource`),
- creates the `.vscode` folders with `tasks.json`, `launch.json` and `settings.json`
  for the application and for both cores,
- creates the VS Code workspace file `student-challenge-main.code-workspace`
  in the project folder.

---

## 7. Open the project in VS Code

1. In VS Code select **File → Open Workspace from File...**
2. Select `student-challenge-main.code-workspace`.
3. If VS Code asks whether you trust the authors, select **Yes, I trust the authors**.
4. Install the recommended extensions if VS Code asks for them.

Always open the **workspace file**, not the folder. Otherwise the build tasks
and debug configurations are not available.

---

## 8. Build, program and debug

### Build

- Press `Ctrl+Shift+B` and select the **Build** task, or
- **Terminal → Run Task...** → **Build**, or
- in modus-shell or UNIX terminal: `make build -j8`

A successful build ends without errors and creates the files in
`proj_cm4/build` and `proj_cm0p/build`.

### Program the kit

1. Connect the kit to the PC with the USB cable (KitProg3 USB connector).
2. Select **Terminal → Run Task...** → **Program**, or run `make program` in modus-shell. Use `make qprogram` to flash latest built version.

Both cores are programmed together.


### Debug

1. Open the **Run and Debug** view (`Ctrl+Shift+D`).
2. Select the *Launch* configuration for the **CM4** core (KitProg3) and press `F5`.
3. The firmware is built, programmed and stopped at `main()` in `proj_cm4/main.c`.

### Debug output (UART)

Debug messages (`DBG_PRINTF`) are sent over the KitProg3 USB-UART.

1. Select the core for the debug output with `RETARGET_CORE` in `common.mk`
   (`CM4` by default or use `CM0P` for CM0+ debug) and rebuild.
2. Open a serial terminal (for example PuTTY or the VS Code *Serial Monitor*)
   on the KitProg3 COM port: **115200 baud, 8 data bits, no parity, 1 stop bit**.

---

## 9. Useful commands

Run these commands in modus-shell from the project folder.

| Command | Purpose |
|---|---|
| `make getlibs` | Download or update the libraries |
| `make vscode` | Generate the VS Code workspace |
| `make build -j8` | Build both cores |
| `make clean` | Delete the build output |
| `make program` | Build and program the kit |
| `make config` | Open the Device Configurator (pins and peripherals) |
| `make config_bt` | Open the Bluetooth Configurator (from proj_cm0p folder) |
| `make library-manager` | Open the Library Manager |

---

## 10. Troubleshooting

| Problem | Solution |
|---|---|
| `Unable to find any of the available CY_TOOLS_PATHS` | ModusToolbox is not installed in the default folder. Set `CY_TOOLS_PATHS` (see section 3). |
| `make getlibs` fails with `Unsupported SSL backend 'schannel'` | Your global Git settings select the Windows SSL backend, which modus-shell Git does not support. Run once in modus-shell: `git config --global http.sslBackend openssl`. To restore it later: `git config --global http.sslBackend schannel`. |
| `make getlibs` fails with timeouts or certificate errors | Check the internet connection and proxy. On a university or company network, ask for the proxy settings and run the **proxy-helper** tool from the ModusToolbox tools folder. |
| `design.modus ... last saved with an incompatible version of the tools` | Install ModusToolbox Tools Package 3.7 or newer. |
| `make: command not found` in Windows PowerShell or CMD | Use modus-shell, or the VS Code tasks of the generated workspace. |
| VS Code has no **Build** task or debug configuration | Open the `.code-workspace` file, not the folder. Run `make vscode` again if the file is missing. |
| Many red IntelliSense errors after opening | Run the **Build** task once, then reload the window (**Developer: Reload Window**). |
| Programming fails / kit not found | Reconnect the USB cable, use another USB port, update the kit to KitProg3 (see section 8). |
| Car does not move after programming | Press the power button on the car chassis and check that only one jumper is set on the adapter board. |

---

## 11. Next steps

Read [API_UserManual.md](API_UserManual.md) for the description of the car
hardware and the APIs (motors, ultrasonic sensor, accelerometer, BLE, LEDs,
sound and timing).
