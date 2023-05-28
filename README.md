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

## WIP

- output is not fully adaptable - project name is hardcoded in parts and you cannot choose between build and release.
- more testing is required with the CMSIS core parts. I would assume they work fine?

## Licenses

This code is licensed under the [Apache 2.0 License](./LICENSE).
