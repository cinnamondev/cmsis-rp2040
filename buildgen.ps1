# buildgen.ps1
<# Copyright 2023 Cinnamondev

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
#>

param (
    [string[]]
    $link_libs = "",
    [string]
    $BuildTarget = "Debug",
    [string]
    $TargetProject = "",
    [switch]
    $target_usb,
    [switch]
    $NoNinja                    # Ninja seems to cause problems with the toolchain path on linux.

)

# bootstrap vcpkg & activate build environment
Invoke-Expression (Invoke-WebRequest -useb https://aka.ms/vcpkg-init.ps1)
vcpkg activate

# Find solution file
$SOLUTION = @(Get-ChildItem -Filter "*.csolution.yaml")[0] # TODO: detect many csolutions?
if(!$SOLUTION) {$SOLUTION = @(Get-ChildItem -Filter "*.csolution.yml")[0]}

# All projects matching name in csolution
$int_ttypes = $false
$Type = "Device" # use project default as a fallback
$_Projects = # Support for file path as target project (vscode cmsis integ.)
if($TargetProject -match "((.*[\\\/])(.+).cproject.yaml)") {
    [System.Tuple]::Create($Matches.3,$Matches.2,$Matches.1)
} else {
    (Get-Content $SOLUTION) | ForEach-Object {
        if ($_ -match "- project: ((.*[\\\/])(.+).cproject.yaml)") {
            if($Matches.3 -eq $TargetProject) {
                [System.Tuple]::Create($Matches.3,$Matches.2,$Matches.1)
            }
        }
        if ($_ -match "target-types:") {$int_ttypes = $true}
        if ($_ -match "- type: (.+)" -and $int_ttypes) {
            $int_ttypes = $false
            $Type =  $Matches.1
        }
    }
}

if ($_Projects.Count -ne 1) {
    if($_Projects.Count -eq 0) {Write-Error "Could not find the target project in the csolution."} 
    else {Write-Error "You have duplicate projects of the same name in your csolution."}
    exit
}

#$Project = $_Projects[0]
$Project_Name = $_Projects.Item1
$Project_Path = $_Projects.Item2
$Project_Full_Path = $_Projects.Item3

$_T = Get-Location
$BUILDDIR = "${_T}/build"
$TEMPDIR = "${_T}/build/.tmp"
$PROJECTDIR ="${_T}/build/${Project_Name}"
$TARGETDIR = "${_T}/build/${Project_Name}/${BuildTarget}"
$OUTPUTDIR = "${_T}/build/${Project_Name}/${BuildTarget}/out"


# ensure build directories exist
if (!((Test-Path $OUTPUTDIR) -and (Test-Path $TEMPDIR))) {
    New-Item -Path $BUILDDIR -ItemType Directory
    New-Item -Path $TEMPDIR -ItemType Directory
    New-Item -Path $PROJECTDIR -ItemType Directory
    New-Item -Path $TARGETDIR -ItemType Directory
    New-Item -Path $OUTPUTDIR -ItemType Directory
}


# Get missing packages
foreach($l in (csolution -s $SOLUTION list packs -m)) {
    Write-Host "[BUILDGEN] Installing CMSIS-Pack: "$l
    cpackget pack add $l
}
# Create *.CPRJ targets
csolution convert -s $SOLUTION -o $TEMPDIR
# Create CMakeList for target project
cbuildgen cmake "${TEMPDIR}/${Project_Name}.${BuildTarget}+${Type}.cprj" --intdir $OUTPUTDIR --outdir $OUTPUTDIR

# Build with CMake + Ninja
Set-Location $OUTPUTDIR

$env:PICO_SDK_PATH = "${_T}/pico-sdk"       # set pico sdk path


(Get-Content "CMakeLists.txt") | 
    Foreach-Object {
        $_
        if ($_ -match 'cmake_minimum_required\(.+\)') {
            # Insert SDK after version
            Write-Output "
include(${_T}/pico_sdk_import.cmake)".Replace('\','/')
        }
        if ($_ -match 'project\(.+\)') {
            # Load SDK after project (+ additional languages for SDK)
            Write-Output "project(`${TARGET} CXX ASM)
pico_sdk_init()"
        }
    } | Set-Content "CMakeLists.txt"

# stdio should redirect to UART by default for use by debugger probe.
# Setting parameter -target_usb will redirect stdio to usb instead.
$stdio_usb = If ($target_usb.IsPresent) {1} else {0}
$stdio_uart = $stdio_usb -bxor 1 

$append = "#Pico SDK Configuration
pico_enable_stdio_usb(`${TARGET} ${stdio_usb})
pico_enable_stdio_uart(`${TARGET} ${stdio_uart}) 
target_link_libraries(`${TARGET} pico_stdlib ${link_libs})
pico_add_extra_outputs(`${TARGET})
".Replace('\','/')
Add-Content "CMakeLists.txt" $append 

$GeneratorArg = if ($NoNinja) {""} else {"-GNinja"}
cmake $GeneratorArg -B . 
ninja 
Set-Location $_T
