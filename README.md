# picopico

CMSIS Pico environment template

## Setup environment

bootstrap vcpkg into environment and pull dependencies

```bash
.\vcpkg\bootstrap-vcpkg.sh
.\vcpkg\vcpkg activate
```

### vscode environment

in a vscode environment, install ARM CMSIS csolution. ninja is also used as a dependency as it is used by the extension.

## deps

### vcpkg

vcpkg is included as a submodule, and is used to pull in various arm build dependencies (arm gcc & arm target for pico).

vcpkg is licensed under the [MIT License](./vcpkg/LICENSE.txt).

### packs

### pico sdk

pico sdk is included as a submodule, and provides interface for pico.

pico sdk [License](./pico-sdk/LICENSE.TXT)

## License

This is licensed under the [Apache 2.0 License](./LICENSE).
