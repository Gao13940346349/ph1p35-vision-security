param()
$ErrorActionPreference = 'Stop'
$projectDirectory = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot 'td_project')).Replace('\','/')
$projectFile = Join-Path $projectDirectory 'camera_to_dsi_display.al'
$text = [IO.File]::ReadAllText($projectFile)
$pattern = '(<Project\b[^>]*\bPath=")[^"]*(")'
if (-not [regex]::IsMatch($text,$pattern)) { throw 'Project location field not found.' }
$replacement = [Text.RegularExpressions.MatchEvaluator]{ param($match) $match.Groups[1].Value + $projectDirectory + $match.Groups[2].Value }
$updated = [regex]::Replace($text,$pattern,$replacement)
[IO.File]::WriteAllText($projectFile,$updated,[Text.UTF8Encoding]::new($false))
Write-Output "Project prepared: $projectFile"
