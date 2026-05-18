param(
  [Parameter(Mandatory = $true)]
  [ValidateSet('m7_0', 'm7_1')]
  [string]$Core,

  [Parameter(Mandatory = $true)]
  [string]$WorkspaceFolder,

  [string]$CSpyBatPath = ''
)

$ErrorActionPreference = 'Stop'

$workspace = (Resolve-Path -LiteralPath $WorkspaceFolder).Path
$settingsDir = Join-Path $workspace 'project\iar\project_config\settings'
if (!(Test-Path -LiteralPath $settingsDir)) {
  throw "IAR settings dir not found: $settingsDir"
}

$generalXcl = Get-ChildItem -LiteralPath $settingsDir -Filter '*.Debug.general.xcl' |
  Sort-Object LastWriteTime -Descending |
  Select-Object -First 1
if (!$generalXcl) {
  throw "No *.Debug.general.xcl found in $settingsDir. Start a debug session in IAR once to generate it."
}

function Select-DriverXcl([string]$coreId) {
  $needleA = if ($coreId -eq 'm7_0') { '7_0' } else { '7_1' }
  $needleB = if ($coreId -eq 'm7_0') { 'cm_7_0' } else { 'cm_7_1' }

  $match = Get-ChildItem -LiteralPath $settingsDir -Filter '*.Debug.driver.xcl' |
    Where-Object { $_.Name -match $needleA -or $_.Name -match $needleB } |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1

  if ($match) { return $match.FullName }

  if ($coreId -eq 'm7_1') {
    $base0 = Get-ChildItem -LiteralPath $settingsDir -Filter '*.Debug.driver.xcl' |
      Where-Object { $_.Name -match '7_0' -or $_.Name -match 'cm_7_0' } |
      Sort-Object LastWriteTime -Descending |
      Select-Object -First 1
    if (!$base0) {
      $base0 = Get-ChildItem -LiteralPath $settingsDir -Filter '*.Debug.driver.xcl' |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    }
    if ($base0) {
      $tmp = Join-Path $env:TEMP ("iar_${coreId}_" + [guid]::NewGuid().ToString('N') + '.xcl')
      (Get-Content -LiteralPath $base0.FullName -Raw) -replace '--jet_board_did=M7_0', '--jet_board_did=M7_1' |
        Set-Content -LiteralPath $tmp -Encoding ASCII
      return $tmp
    }
  }

  $fallback = Get-ChildItem -LiteralPath $settingsDir -Filter '*.Debug.driver.xcl' |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1
  if (!$fallback) {
    throw "No *.Debug.driver.xcl found in $settingsDir"
  }
  return $fallback.FullName
}

$driverXclPath = Select-DriverXcl -coreId $Core

$exeDir = Join-Path $workspace ("project\iar\project_config\Debug_{0}\Exe" -f $Core)
if (!(Test-Path -LiteralPath $exeDir)) {
  throw "IAR output dir not found: $exeDir"
}

$debugOut = Get-ChildItem -LiteralPath $exeDir -Filter '*.out' -ErrorAction SilentlyContinue |
  Sort-Object LastWriteTime -Descending |
  Select-Object -First 1
if (!$debugOut) {
  throw "No *.out found in $exeDir. Build the IAR project first."
}

function Find-CSpyBat([string]$hint) {
  if (![string]::IsNullOrWhiteSpace($hint) -and (Test-Path -LiteralPath $hint)) {
    return (Resolve-Path -LiteralPath $hint).Path
  }

  $candidates = @(
    'D:\IAR Systems\Embedded Workbench*\common\bin\CSpyBat.exe',
    'C:\IAR Systems\Embedded Workbench*\common\bin\CSpyBat.exe',
    "$env:ProgramFiles\IAR Systems\Embedded Workbench*\common\bin\CSpyBat.exe",
    "$env:ProgramFiles(x86)\IAR Systems\Embedded Workbench*\common\bin\CSpyBat.exe"
  )

  $found = @()
  foreach ($p in $candidates) {
    $found += Get-ChildItem -Path $p -ErrorAction SilentlyContinue
  }

  $best = $found | Sort-Object LastWriteTime -Descending | Select-Object -First 1
  if ($best) { return $best.FullName }

  throw 'CSpyBat.exe not found. Set zf.iar.cspybatPath in .vscode/settings.json or pass -CSpyBatPath.'
}

$cspy = Find-CSpyBat -hint $CSpyBatPath

Write-Host "CSpyBat: $cspy"
Write-Host "GeneralXcl: $($generalXcl.FullName)"
Write-Host "DriverXcl: $driverXclPath"
Write-Host "DebugOut: $($debugOut.FullName)"

& $cspy -f $generalXcl.FullName "--debug_file=$($debugOut.FullName)" --download_only --backend -f $driverXclPath
