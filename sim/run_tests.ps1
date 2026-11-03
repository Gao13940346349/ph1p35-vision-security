param(
    [string]$ModelSimBin = $env:MODELSIM_BIN,
    [string]$TDHome = $env:ANLOGIC_TD_HOME,
    [switch]$CompileOnly
)
$ErrorActionPreference='Stop'
if (-not $ModelSimBin -or -not $TDHome) { throw 'Specify -ModelSimBin and -TDHome, or set MODELSIM_BIN and ANLOGIC_TD_HOME.' }
foreach($name in @('vlib.exe','vlog.exe','vsim.exe')) {
    if (-not (Test-Path -LiteralPath (Join-Path $ModelSimBin $name))) { throw "Missing ModelSim tool: $name" }
}
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$vendor=Join-Path $TDHome 'sim_release'
if (-not (Test-Path -LiteralPath "$vendor/ph1p")) { throw 'TD PH1P simulation library not found.' }
$run=Join-Path $PSScriptRoot '_work'
New-Item -ItemType Directory -Force -Path $run,"$run/assets" | Out-Null
Copy-Item -LiteralPath "$root/user_source/hdl_source/assets/logo_index.dath","$root/user_source/hdl_source/assets/logo_palette.dath" -Destination "$run/assets" -Force
Copy-Item -LiteralPath "$root/user_source/hdl_source/ph1p35_ddr/ddr2/src/rtl/phy/ddrphy_cfg_rom.txt" -Destination $run -Force
$project="$root/td_project/camera_to_dsi_display.al"
$files=[Collections.Generic.List[string]]::new()
$includes=[Collections.Generic.HashSet[string]]::new()
foreach($entry in [regex]::Matches([IO.File]::ReadAllText($project),'<File Path="([^"]+)"')) {
    $path=[IO.Path]::GetFullPath("$root/td_project/$($entry.Groups[1].Value)")
    if (-not (Test-Path -LiteralPath $path)) { throw "Missing input: $path" }
    if ($path -match '\.(v|sv)$') { $files.Add($path) }
    elseif ($path -match '\.xml$') {
        [xml]$xml=[IO.File]::ReadAllText($path)
        foreach($node in $xml.SelectNodes('//*[local-name()="fileSets"]/*[local-name()="fileSet"]/*[local-name()="file"]/*[local-name()="name"]')) {
            if($node.InnerText -match '\.(v|sv)$') { $files.Add([IO.Path]::GetFullPath("$([IO.Path]::GetDirectoryName($path))/$($node.InnerText)")) }
        }
    }
}
$files=@($files | Select-Object -Unique)
foreach($file in $files) { [void]$includes.Add([IO.Path]::GetDirectoryName($file)) }
$ddr="$root/user_source/hdl_source/ph1p35_ddr/ddr2/src/rtl"
foreach($dir in @("$ddr/include","$ddr/phy/include","$ddr/timing/ddr2")) { [void]$includes.Add($dir) }
$inc=@($includes | ForEach-Object { '+incdir+'+$_ })
$primitives=@((Get-ChildItem "$vendor/common" -Filter '*.v').FullName)+@((Get-ChildItem "$vendor/ph1p" -Filter '*.v').FullName)
$testFiles=@(Get-ChildItem -LiteralPath $PSScriptRoot -Recurse -File | Where-Object { $_.DirectoryName -ne $run -and $_.Name -like 'tb_*' -and $_.Extension -in @('.v','.sv') })
$testNames=@($testFiles | ForEach-Object { $_.BaseName })
Push-Location $run
try {
    if (-not (Test-Path work)) { & "$ModelSimBin/vlib.exe" work; if($LASTEXITCODE){throw 'vlib failed'} }
    & "$ModelSimBin/vlog.exe" -work work @primitives *> compile_primitives.log
    if($LASTEXITCODE) { Get-Content compile_primitives.log -Tail 35; throw 'Vendor primitive compilation failed.' }
    & "$ModelSimBin/vlog.exe" -mfcu -work work @inc "$ddr/include/ph1p_mis_define.vh" @files *> compile_rtl.log
    if($LASTEXITCODE) { Get-Content compile_rtl.log -Tail 35; throw 'Production RTL compilation failed.' }
    foreach($file in $testFiles) {
        $options=@();if($file.Extension -eq '.sv'){$options=@('-sv')}
        & "$ModelSimBin/vlog.exe" -work work @options $file.FullName *> "$($file.BaseName)_compile.log"
        if($LASTEXITCODE) { Get-Content "$($file.BaseName)_compile.log" -Tail 30; throw "Test compile failed: $($file.Name)" }
    }
    Write-Output "PASS: $($files.Count) production RTL/IP files and $($testNames.Count) testbenches compiled."
    if ($CompileOnly) { return }
    & "$ModelSimBin/vsim.exe" -c -l top_elaboration.log work.design_top_wrapper work.glbl work.PH1P_PHY_GSR -do 'onerror {quit -code 1 -f}; quit -code 0 -f' *> top_console.log
    if($LASTEXITCODE) { Get-Content top_console.log -Tail 35; throw 'Top-level elaboration failed.' }
    Write-Output 'PASS: full production hierarchy elaborated.'
    foreach($test in $testNames) {
        & "$ModelSimBin/vsim.exe" -c -onfinish exit -l "$test.log" "work.$test" work.glbl work.PH1P_PHY_GSR -do 'onbreak {quit -code 1 -f}; onerror {quit -code 1 -f}; run -all; quit -code 0 -f' *> "$test.console.log"
        if($LASTEXITCODE -or -not (Select-String -LiteralPath "$test.log" -Pattern 'PASS' -Quiet) -or (Select-String -LiteralPath "$test.log" -Pattern '\*\* (Fatal|Error)' -Quiet)) { Get-Content "$test.log" -Tail 35; throw "Simulation failed: $test" }
        Write-Output "PASS: $test"
    }
    Write-Output "PASS: $($testNames.Count) functional testbenches; no synthesis or programming performed."
} finally { Pop-Location }
