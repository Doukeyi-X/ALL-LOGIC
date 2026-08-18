#Requires -Version 5.1
<#
.SYNOPSIS
  Package ALL LOGIC as a portable folder + ZIP + Windows NSIS installer.

.DESCRIPTION
  Layout matches Windows path resolution in AppConfig:
    <install>/AllLogic.exe
    <install>/res, demo, lang, decoders
    <install>/*.dll + Qt plugins
    <install>/lib/python3.14  (MinGW Python stdlib for protocol decoders)
#>
param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [string]$OutDir = "",
    [switch]$SkipBuild,
    [switch]$SkipInstaller
)

$ErrorActionPreference = "Stop"

$Version = "1.3.2"
$PkgName = "ALLLOGIC-$Version-win64"
$ExeName = "AllLogic.exe"
$MingwBin = "C:\msys64\mingw64\bin"
$MingwLib = "C:\msys64\mingw64\lib"
$Objdump = Join-Path $MingwBin "objdump.exe"
$Windeploy = Join-Path $MingwBin "windeployqt-qt5.exe"
$Ninja = Join-Path $MingwBin "ninja.exe"
$Makensis = Join-Path $MingwBin "makensis.exe"
$BuildDir = Join-Path $Root "build"
$BinDir = Join-Path $Root "build.dir"
$StageRoot = if ($OutDir) { $OutDir } else { Join-Path $Root "dist" }
$Stage = Join-Path $StageRoot $PkgName
$ZipPath = Join-Path $StageRoot "$PkgName.zip"
$SetupPath = Join-Path $StageRoot "$PkgName-setup.exe"

function Write-Step($msg) { Write-Host "`n=== $msg ===" -ForegroundColor Cyan }

function Get-DllDeps([string]$path) {
    & $Objdump -p $path 2>$null |
        Select-String "DLL Name:\s+(\S+)" |
        ForEach-Object { $_.Matches.Groups[1].Value }
}

function Test-SystemDll([string]$name) {
    $n = $name.ToUpperInvariant()
    $sys = @(
        'KERNEL32.DLL','USER32.DLL','GDI32.DLL','ADVAPI32.DLL','SHELL32.DLL',
        'OLE32.DLL','OLEAUT32.DLL','COMDLG32.DLL','COMCTL32.DLL','WS2_32.DLL',
        'SETUPAPI.DLL','WINMM.DLL','IMM32.DLL','VERSION.DLL','CRYPT32.DLL',
        'SECUR32.DLL','BCRYPT.DLL','NTDLL.DLL','MSVCRT.DLL','RPCRT4.DLL',
        'SHLWAPI.DLL','UXTHEME.DLL','DWMAPI.DLL','D3D11.DLL','DXGI.DLL',
        'DWRITE.DLL','GDIPLUS.DLL','IPHLPAPI.DLL','DNSAPI.DLL','WLDAP32.DLL',
        'NORMALIZ.DLL','NSI.DLL','PROPSYS.DLL','CFGMGR32.DLL','POWRPROF.DLL',
        'WINSPOOL.DRV','USERENV.DLL','NETAPI32.DLL','WTSAPI32.DLL','HID.DLL',
        'WINTRUST.DLL','DBGHELP.DLL','PSAPI.DLL','MSIMG32.DLL','AUTHZ.DLL',
        'CREdui.DLL','SECAPI.DLL','MPR.DLL','NCORPT.DLL','DPAPI.DLL',
        'CRYPTBASE.DLL','SSPICLI.DLL','PROFAPI.DLL','KERNELBASE.DLL',
        'BCRYPTPRIMITIVES.DLL','NTMARTA.DLL','SAMCLI.DLL','LOGONCLI.DLL',
        'WLDPSRV.DLL','MSWSOCK.DLL','WINHTTP.DLL','URLMON.DLL','IERTUTIL.DLL',
        'WINDOWSCODECS.DLL','D2D1.DLL','D3D9.DLL','OPENGL32.DLL','GLU32.DLL',
        'DXCORE.DLL','DIRECTML.DLL','MFPLAT.DLL','MF.DLL','MFREADWRITE.DLL',
        'API-MS-WIN-*.DLL'
    )
    foreach ($p in $sys) {
        if ($p -like '*`**') {
            $pat = $p -replace '\*','.*'
            if ($n -match "^$pat$") { return $true }
        } elseif ($n -eq $p) { return $true }
    }
    if ($n.StartsWith('API-MS-WIN-')) { return $true }
    if ($n.StartsWith('EXT-MS-')) { return $true }
    return $false
}

function Collect-DllTree([string]$seedPath, [string]$destDir) {
    $needed = New-Object 'System.Collections.Generic.HashSet[string]'
    $queue = New-Object 'System.Collections.Generic.Queue[string]'
    $visited = New-Object 'System.Collections.Generic.HashSet[string]'
    $queue.Enqueue((Resolve-Path $seedPath).Path)

    # Also seed from already-copied DLLs and Qt plugins later
    while ($queue.Count -gt 0) {
        $cur = $queue.Dequeue()
        if (-not $visited.Add($cur.ToLowerInvariant())) { continue }
        if (-not (Test-Path $cur)) { continue }
        foreach ($d in (Get-DllDeps $cur)) {
            if (Test-SystemDll $d) { continue }
            $src = Join-Path $MingwBin $d
            if (-not (Test-Path $src)) { continue }
            if ($needed.Add($d)) {
                $queue.Enqueue($src)
            }
        }
    }

    New-Item -ItemType Directory -Force -Path $destDir | Out-Null
    foreach ($d in ($needed | Sort-Object)) {
        $src = Join-Path $MingwBin $d
        Copy-Item -Force $src (Join-Path $destDir $d)
    }
    return $needed.Count
}

function Copy-PythonRuntime([string]$destRoot) {
    $pySrc = Join-Path $MingwLib "python3.14"
    if (-not (Test-Path $pySrc)) {
        throw "Python stdlib not found: $pySrc"
    }
    $pyDst = Join-Path $destRoot "lib\python3.14"
    Write-Host "Copying Python 3.14 stdlib (slim) -> $pyDst"
    New-Item -ItemType Directory -Force -Path $pyDst | Out-Null

    $excludeDirs = @('test','tests','idlelib','ensurepip','turtledemo','tkinter','__pycache__','site-packages')
    $robocopyArgs = @(
        $pySrc, $pyDst, '/E', '/NFL', '/NDL', '/NJH', '/NJS', '/nc', '/ns', '/np',
        '/XD'
    ) + $excludeDirs + @('__pycache__')
    & robocopy @robocopyArgs | Out-Null
    # robocopy exit 0-7 is success
    if ($LASTEXITCODE -ge 8) { throw "robocopy python failed: $LASTEXITCODE" }

    # Drop .pyc leftover and huge cache if any
    Get-ChildItem $pyDst -Recurse -Directory -Filter '__pycache__' -ErrorAction SilentlyContinue |
        Remove-Item -Recurse -Force -ErrorAction SilentlyContinue
}

function Ensure-Resources {
    # Ensure runtime data next to exe (Windows layout)
    $pairs = @(
        @{ Src = Join-Path $Root "DSView\res";          Dst = Join-Path $BinDir "res" },
        @{ Src = Join-Path $Root "DSView\demo";         Dst = Join-Path $BinDir "demo" },
        @{ Src = Join-Path $Root "lang";                Dst = Join-Path $BinDir "lang" },
        @{ Src = Join-Path $Root "libsigrokdecode4DSL\decoders"; Dst = Join-Path $BinDir "decoders" }
    )
    foreach ($p in $pairs) {
        if (-not (Test-Path $p.Src)) { throw "Missing: $($p.Src)" }
        if (-not (Test-Path $p.Dst)) {
            Write-Host "Sync $($p.Src) -> $($p.Dst)"
            New-Item -ItemType Directory -Force -Path (Split-Path $p.Dst) | Out-Null
            Copy-Item -Recurse -Force $p.Src $p.Dst
        }
    }
    foreach ($f in @('NEWS25','NEWS31','ug25.pdf','ug31.pdf')) {
        $src = Join-Path $Root $f
        $dst = Join-Path $BinDir $f
        if ((Test-Path $src) -and -not (Test-Path $dst)) {
            Copy-Item -Force $src $dst
        }
    }
}

# -------------------- main --------------------
Write-Step "ALL LOGIC Windows package $Version"
Write-Host "Root:  $Root"
Write-Host "Stage: $Stage"

$builtExe = Join-Path $BinDir $ExeName
if (-not (Test-Path $builtExe)) {
    $builtExe = Join-Path $BinDir "DSView.exe"
}
if (-not (Test-Path $builtExe)) {
    throw "AllLogic.exe not found in $BinDir — build first."
}
if (-not (Test-Path $Windeploy)) { throw "windeployqt not found: $Windeploy" }
if (-not (Test-Path $Objdump)) { throw "objdump not found: $Objdump" }

if (-not $SkipBuild -and (Test-Path $Ninja) -and (Test-Path (Join-Path $BuildDir "build.ninja"))) {
    Write-Step "Rebuild ALL LOGIC"
    & $Ninja -C $BuildDir DSView
    if ($LASTEXITCODE -ne 0) { throw "ninja build failed: $LASTEXITCODE" }
}

Write-Step "Ensure resources beside exe"
Ensure-Resources

Write-Step "Clean stage"
if (Test-Path $StageRoot) {
    # only remove our package folder / outputs
}
if (Test-Path $Stage) { Remove-Item -Recurse -Force $Stage }
New-Item -ItemType Directory -Force -Path $Stage | Out-Null
New-Item -ItemType Directory -Force -Path $StageRoot | Out-Null

Write-Step "Copy application files"
Copy-Item -Force $builtExe (Join-Path $Stage $ExeName)
foreach ($d in @('res','demo','lang','decoders')) {
    $src = Join-Path $BinDir $d
    if (-not (Test-Path $src)) { throw "Missing runtime dir: $src" }
    Copy-Item -Recurse -Force $src (Join-Path $Stage $d)
}
foreach ($f in @('NEWS25','NEWS31','ug25.pdf','ug31.pdf')) {
    $src = Join-Path $BinDir $f
    if (Test-Path $src) { Copy-Item -Force $src (Join-Path $Stage $f) }
}

# App icon for installer/shortcuts if present
$icon = Join-Path $Root "logo-win.ico"
if (Test-Path $icon) { Copy-Item -Force $icon (Join-Path $Stage "AllLogic.ico") }
$notice = Join-Path $Root "NOTICE.txt"
if (Test-Path $notice) { Copy-Item -Force $notice (Join-Path $Stage "NOTICE.txt") }

Write-Step "Deploy Qt plugins (windeployqt)"
& $Windeploy --release --no-translations --no-system-d3d-compiler --no-opengl-sw `
    --compiler-runtime (Join-Path $Stage $ExeName)
if ($LASTEXITCODE -ne 0) {
    Write-Warning "windeployqt exit $LASTEXITCODE — continuing with manual plugin/DLL collect"
}

# windeployqt may fail on missing libGLESv2; always ensure core plugins exist
Write-Step "Ensure essential Qt plugins"
$qtPlug = "C:\msys64\mingw64\share\qt5\plugins"
$plugMap = @{
    platforms     = @('qwindows.dll', 'qminimal.dll', 'qoffscreen.dll')
    styles        = @('qwindowsvistastyle.dll')
    imageformats  = @('qgif.dll', 'qico.dll', 'qjpeg.dll', 'qsvg.dll')
    iconengines   = @('qsvgicon.dll')
}
foreach ($dir in $plugMap.Keys) {
    $srcDir = Join-Path $qtPlug $dir
    $dstDir = Join-Path $Stage $dir
    if (-not (Test-Path $srcDir)) { continue }
    New-Item -ItemType Directory -Force -Path $dstDir | Out-Null
    foreach ($f in $plugMap[$dir]) {
        $src = Join-Path $srcDir $f
        if (Test-Path $src) {
            Copy-Item -Force $src (Join-Path $dstDir $f)
        }
    }
}
# drop optional heavy platform plugins if windeployqt copied them
foreach ($extra in @('qwebgl.dll', 'qdirect2d.dll')) {
    $p = Join-Path $Stage "platforms\$extra"
    if (Test-Path $p) { Remove-Item -Force $p }
}
if (-not (Test-Path (Join-Path $Stage "platforms\qwindows.dll"))) {
    throw "platforms\qwindows.dll missing — GUI cannot start"
}

Write-Step "Collect MinGW / app DLL dependencies"
$n = Collect-DllTree (Join-Path $Stage $ExeName) $Stage
# Also resolve deps of every plugin DLL
Get-ChildItem $Stage -Recurse -Filter *.dll | ForEach-Object {
    Collect-DllTree $_.FullName $Stage | Out-Null
}
Write-Host "Primary DLL set ready ($n seeds from exe)"

Write-Step "Bundle Python runtime for protocol decoders"
Copy-PythonRuntime $Stage

Write-Step "Write launcher and readme"
$bat = @"
@echo off
setlocal
cd /d "%~dp0"
rem MinGW Python looks for lib\python3.14 next to the DLL when prefix == install dir
set "PATH=%~dp0;%PATH%"
start "" "%~dp0AllLogic.exe" %*
"@
Set-Content -Path (Join-Path $Stage "AllLogic.bat") -Value $bat -Encoding ASCII

$readme = @"
ALL LOGIC $Version (Windows x64)
================================

Unofficial multi-vendor logic analyzer host, based on DSView (GPLv3+).
NOT affiliated with DreamSourceLab or any hardware vendor. See NOTICE.txt.

【运行】
  双击 AllLogic.exe 或 AllLogic.bat
  标题栏：ALL LOGIC v$Version
  MCP 设置在工具栏「MCP」菜单

【MCP】
  http://127.0.0.1:10110
    claude mcp add --transport http alllogic http://127.0.0.1:10110

【安装版】
  使用 $PkgName-setup.exe，安装名为 ALL LOGIC。

【许可证】
  GNU GPLv3+（COPYING）。分发二进制时须提供对应源码。

【USB】
  设备无法识别时可用 Zadig 安装 WinUSB（VID:PID 以设备管理器为准）。

版本: $Version
"@
Set-Content -Path (Join-Path $Stage "README.txt") -Value $readme -Encoding UTF8

# License
$lic = Join-Path $Root "DSView\COPYING"
if (Test-Path $lic) { Copy-Item -Force $lic (Join-Path $Stage "COPYING") }

Write-Step "Create portable ZIP"
if (Test-Path $ZipPath) { Remove-Item -Force $ZipPath }
Compress-Archive -Path $Stage -DestinationPath $ZipPath -CompressionLevel Optimal
$zipMb = [math]::Round((Get-Item $ZipPath).Length / 1MB, 1)
Write-Host "ZIP: $ZipPath ($zipMb MB)"

if (-not $SkipInstaller) {
    if (-not (Test-Path $Makensis)) {
        Write-Warning "makensis not found ($Makensis); skip installer. Portable ZIP is ready."
    } else {
        Write-Step "Build NSIS installer"
        $nsi = Join-Path $StageRoot "AllLogic.nsi"
        $stageUnix = $Stage -replace '\\','/'
        $setupUnix = $SetupPath -replace '\\','/'
        $icoLine = ""
        $icoUnix = ""
        $icoPath = Join-Path $Stage "AllLogic.ico"
        if (Test-Path $icoPath) {
            $icoUnix = $icoPath -replace '\\','/'
            $icoLine = "Icon `"$icoUnix`"`nUninstallIcon `"$icoUnix`""
        }

        $nsiText = @"
Unicode true
Name "ALL LOGIC $Version"
OutFile "$setupUnix"
InstallDir "`$PROGRAMFILES64\ALL LOGIC"
InstallDirRegKey HKLM "Software\AllLogic" "Install_Dir"
RequestExecutionLevel admin
SetCompressor /SOLID lzma
$icoLine

!include "MUI2.nsh"

!define MUI_ABORTWARNING
!define MUI_ICON "$icoUnix"
!define MUI_UNICON "$icoUnix"
!define MUI_WELCOMEPAGE_TITLE "ALL LOGIC $Version"
!define MUI_WELCOMEPAGE_TEXT "Unofficial multi-vendor logic analyzer host based on DSView (GPLv3+).$\r$\n$\r$\nNot affiliated with DreamSourceLab or any hardware vendor. See NOTICE.txt."
!insertmacro MUI_PAGE_WELCOME
!insertmacro MUI_PAGE_LICENSE "$stageUnix/COPYING"
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_PAGE_FINISH
!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_LANGUAGE "English"
!insertmacro MUI_LANGUAGE "SimpChinese"

Section "ALL LOGIC" SecMain
  SectionIn RO
  SetOutPath "`$INSTDIR"
  File /r "$stageUnix\*.*"

  WriteRegStr HKLM "Software\AllLogic" "Install_Dir" "`$INSTDIR"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\AllLogic" "DisplayName" "ALL LOGIC $Version"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\AllLogic" "UninstallString" '"`$INSTDIR\Uninstall.exe"'
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\AllLogic" "DisplayVersion" "$Version"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\AllLogic" "Publisher" "ALL LOGIC (unofficial DSView fork)"
  WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\AllLogic" "NoModify" 1
  WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\AllLogic" "NoRepair" 1

  WriteUninstaller "`$INSTDIR\Uninstall.exe"

  CreateDirectory "`$SMPROGRAMS\ALL LOGIC"
  CreateShortCut "`$SMPROGRAMS\ALL LOGIC\ALL LOGIC.lnk" "`$INSTDIR\AllLogic.exe" "" "`$INSTDIR\AllLogic.exe" 0
  CreateShortCut "`$SMPROGRAMS\ALL LOGIC\Uninstall.lnk" "`$INSTDIR\Uninstall.exe"
  CreateShortCut "`$DESKTOP\ALL LOGIC.lnk" "`$INSTDIR\AllLogic.exe" "" "`$INSTDIR\AllLogic.exe" 0
SectionEnd

Section "Uninstall"
  Delete "`$DESKTOP\ALL LOGIC.lnk"
  RMDir /r "`$SMPROGRAMS\ALL LOGIC"
  RMDir /r "`$INSTDIR"
  DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\AllLogic"
  DeleteRegKey HKLM "Software\AllLogic"
SectionEnd
"@
        # Fix File /r path for NSIS (backslash ok on Windows)
        $nsiText = $nsiText -replace 'File /r "(.+?)\\\*\.\*"', 'File /r "$1\*.*"'
        Set-Content -Path $nsi -Value $nsiText -Encoding UTF8

        & $Makensis $nsi
        if ($LASTEXITCODE -ne 0) { throw "makensis failed: $LASTEXITCODE" }
        if (-not (Test-Path $SetupPath)) { throw "Installer not produced: $SetupPath" }
        $setupMb = [math]::Round((Get-Item $SetupPath).Length / 1MB, 1)
        Write-Host "SETUP: $SetupPath ($setupMb MB)"
    }
}

Write-Step "Done"
Write-Host "Portable folder : $Stage"
Write-Host "Portable ZIP    : $ZipPath"
if (Test-Path $SetupPath) { Write-Host "Installer       : $SetupPath" }
Write-Host ""
Write-Host "Install: run the *-setup.exe as Administrator, or unzip the portable package."
