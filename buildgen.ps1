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
    $TargetProject = ""
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
$_Projects =
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
csolution -s $SOLUTION list packs -m > "${TEMPDIR}/packs.txt"

# Create *.CPRJ targets
csolution convert -s $SOLUTION -o $TEMPDIR
# Create CMakeList for target project
cbuildgen cmake "${TEMPDIR}/${Project_Name}.${BuildTarget}+${Type}.cprj" --intdir $OUTPUTDIR --outdir $OUTPUTDIR

# Build with CMake + Ninja
Set-Location $OUTPUTDIR
(Get-Content "CMakeLists.txt") -replace 'project\(.+\)', 'project(${TARGET} LANGUAGES C CXX ASM)' | Set-Content "CMakeLists.txt"

$append = 
"include(${_T}/pico-sdk/external/pico_sdk_import.cmake)
pico_sdk_init()
target_link_libraries(`${TARGET} pico_stdlib ${link_libs})
pico_add_extra_outputs(`${TARGET})
".Replace('\','/')
Add-Content "CMakeLists.txt" $append 

$env:PICO_SDK_PATH = "${_T}/pico-sdk"

cmake -GNinja -B . 
ninja
Set-Location $_T
