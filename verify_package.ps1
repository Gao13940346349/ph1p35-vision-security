param()
$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath($PSScriptRoot)
$project = Join-Path $root 'td_project/camera_to_dsi_display.al'
foreach ($line in [IO.File]::ReadAllLines((Join-Path $root 'SHA256SUMS.txt'))) {
    if ($line -notmatch '^([0-9a-f]{64})  (.+)$') { throw 'Malformed SHA256SUMS entry.' }
    $digest=$Matches[1]; $relative=$Matches[2]
    $path=[IO.Path]::GetFullPath((Join-Path $root $relative))
    if (-not $path.StartsWith($root+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { throw 'Manifest path escapes project.' }
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing: $relative" }
    if ($relative -eq 'td_project/camera_to_dsi_display.al') {
        $text=[IO.File]::ReadAllText($path)
        $text=[regex]::Replace($text,'(<Project\b[^>]*\bPath=")[^"]*(")','$1.$2')
        $algorithm=[Security.Cryptography.SHA256]::Create()
        try { $actual=[BitConverter]::ToString($algorithm.ComputeHash([Text.Encoding]::UTF8.GetBytes($text))).Replace('-','').ToLowerInvariant() }
        finally { $algorithm.Dispose() }
    } else { $actual=(Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant() }
    if ($actual -ne $digest) { throw "Checksum mismatch: $relative" }
}
$count=0; $ipCount=0
foreach ($match in [regex]::Matches([IO.File]::ReadAllText($project),'<File Path="([^"]+)"')) {
    $path=[IO.Path]::GetFullPath((Join-Path (Split-Path $project) $match.Groups[1].Value))
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing project input: $path" }
    $count++
    if ([IO.Path]::GetExtension($path) -eq '.xml') {
        [xml]$xml=[IO.File]::ReadAllText($path)
        foreach($entry in $xml.SelectNodes('//*[local-name()="fileSets"]/*[local-name()="fileSet"]/*[local-name()="file"]/*[local-name()="name"]')) {
            $child=Join-Path (Split-Path $path) $entry.InnerText
            if (-not (Test-Path -LiteralPath $child -PathType Leaf)) { throw "Missing IP input: $child" }
            $ipCount++
        }
    }
}
Write-Output "PASS: file checksums, $count project references and $ipCount IP file-set references."
