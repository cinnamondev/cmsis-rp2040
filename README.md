# cmsis-rp2040

CMSIS Pico (RP2040) project temp.

Self-enclosed build system - only requires powershell, which can be installed on linux these days. This is used as its particularly useful for bootstrapping build dependencies (vcpkg for compilers and build tools)

## Usage

This template initially contains a simple blink program that you can run on your Pico to ensure
everything works as expected. Requires powershell and git.

- Pull all submodules - this will provide a local instance of the latest [pico-sdk](https://github.com/raspberrypi/pico-sdk).

```bash
git submodule update --init --recursive
```

- Build the example program using the buildgen powershell script

```bash
powershell .\buildgen.ps1
```

Your UF2 output file is now avialable in ./build/Debug/picopico.uf2 (fix later)

## CMSIS-Toolbox

This project makes use of the CMSIS-Toolbox (ctools). After running the build script, they will be available within that local environment (ie: the terminal it was executed in). If you want easy access to the build tools (ctools, arm gcc, etc...), you can execute `powershell .\bootstrap.ps1` to bootstrap them (they are cached by vcpkg).

## WIP

- output is not fully adaptable - project name is hardcoded in parts and you cannot choose between build and release.
- more testing is required with the CMSIS core parts. I would assume they work fine?

- vscode: clangd linking automatic - `.clangd` can be generated in the project root by CMSIS extension, but it will target the wrong location for this setup. this can be resolved via symlinks, or manually editing `.clangd` to point to `./build/{project}/{release}/out`. buildgen could have a flag to generate this for us so we dont have to manually edit it.

## Licenses

This code is licensed under the [Apache 2.0 License](./LICENSE).
