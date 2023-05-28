# bootstrap vcpkg & activate build environment
Invoke-Expression (Invoke-WebRequest -useb https://aka.ms/vcpkg-init.ps1)
#.\vcpkg\bootstrap-vcpkg.bat
#.\vcpkg\vcpkg activate
vcpkg activate
$_T = Get-Location

$SOLUTION = "./picopico.csolution.yaml"
$MODE = "Debug"

$BDIR = "./build"
$BTMP = "./build/.tmp"
$BTARGETS = "./build/targets"

mkdir $BDIR
mkdir $BTMP
mkdir $BTARGETS

# Get packages
csolution -s $SOLUTION list packs -m > "${BTMP}/packs.txt"

# Create *.CPRJ targets
csolution convert -s $SOLUTION -o $BTARGETS
cbuildgen cmake "${BTARGETS}/picopico.Debug+Device.cprj" --intdir "${BDIR}/${MODE}/" --outdir "${BDIR}/${MODE}/out/"
Set-Location "${BDIR}/${MODE}"

# Hijack CMake and insert Pico SDK
(Get-Content "CMakeLists.txt") | 
    Foreach-Object {
        $_
        if ($_ -match 'cmake_minimum_required\(.+\)') {
            Write-Output "
include(${_T}/pico-sdk/external/pico_sdk_import.cmake)
pico_sdk_init()".Replace('\','/')
        }
    } | Set-Content "CMakeLists.txt"

$Env:PICO_SDK_PATH = "${_T}/pico-sdk"
cmake -GNinja -B . 
ninja
Set-Location $_T