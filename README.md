# cmsis-rp2040
Pico/RP2040 project template. Allows use of the Open-CMSIS-Pack tools (csolutions, software packs, etc...)  with the Pico SDK with little additional work.

# Dependencies

- Powershell
- [pico-sdk](https://github.com/raspberrypi/pico-sdk) (submodule)

Powershell (5.1 or so should be fine, works with the version installed on Windows 11). Everything else (compilers, etc...) will be installed by a bootstrapped `vcpkg`. This removes the need to install the ARM GCC compiler, Open-CMSIS-Pack ctools/[devtools](https://github.com/Open-CMSIS-Pack/devtools), cmake or ninja - at least within the build environment. If you need to access them quickly a second script is provided, `bootstrap.ps1` which will install all the same build tools. The Open-CMSIS-Pack suite is licensed under Apache 2.0 and is suitable for any FOSS application!

[Powershell](https://github.com/PowerShell/PowerShell) is now a crossplatform tool and this should work just about fine on linux - though this has not been properly tested yet. Powershell is licensed under the MIT license.

## Usage

This template initially contains a simple blink program that you can run on your Pico to ensure
everything works as expected. Requires powershell and git.

- Pull all submodules - this will provide the latest [pico-sdk](https://github.com/raspberrypi/pico-sdk) and its submodules.

```bash
git submodule update --init --recursive
```

- Build the example program using the buildgen powershell script

```bash
powershell .\buildgen.ps1 -BuildTarget Debug -TargetProject picopico 
```
You can link additional parts of the pico sdk using the argument `-link_libs a,b,c,d`
The resulting uf2 file will be availalbe in ./build/debug/{project name}.uf2 !

### Additional demos / templates 

[demo-lvgl](https://github.com/cinnamondev/cmsis-rp2040/tree/demo-lvgl) provides a template for a LVGL project, with an included driver for the ILI9341.
## WIP

- don't believe the script is actually yet capable of pulling the packs by itself- they should be preinstalled first (for now)
- debug stuff


## Licenses

This code is licensed under the [Apache 2.0 License](./LICENSE).
