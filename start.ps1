# Windows entry point. Compatible with Windows PowerShell 5.1 and PowerShell 7.
# Keep UTF-8 BOM so Chinese messages display correctly in Windows PowerShell 5.1.
$ErrorActionPreference = 'Stop'
$OutputEncoding = New-Object System.Text.UTF8Encoding($false)
[Console]::OutputEncoding = $OutputEncoding
$runnerArguments = @($args | ForEach-Object { [string]$_ })
$distributionName = 'Ubuntu-22.04'
$launcherDirectory = [System.IO.Path]::GetFullPath($PSScriptRoot)
$wslExecutable = Join-Path $env:SystemRoot 'System32\wsl.exe'

function ConvertTo-NativeArgument {
    param([AllowEmptyString()][string]$Value)
    # Quote one Windows argv element. Embedded quotes and trailing backslashes
    # must survive Windows PowerShell 5.1's older native-command serialization.
    # WSL parses its own switches before argv conversion, so leave simple
    # switches unquoted instead of quoting every element indiscriminately.
    if ($Value.Length -gt 0 -and $Value -notmatch '[\s"]') {
        return $Value
    }
    $builder = New-Object System.Text.StringBuilder
    [void]$builder.Append('"')
    $backslashes = 0
    foreach ($character in $Value.ToCharArray()) {
        if ($character -eq '\') {
            $backslashes++
        } elseif ($character -eq '"') {
            [void]$builder.Append(('\' * (2 * $backslashes + 1)))
            [void]$builder.Append('"')
            $backslashes = 0
        } else {
            [void]$builder.Append(('\' * $backslashes))
            [void]$builder.Append($character)
            $backslashes = 0
        }
    }
    [void]$builder.Append(('\' * (2 * $backslashes)))
    [void]$builder.Append('"')
    return $builder.ToString()
}

function Invoke-WslProcess {
    param([string[]]$Arguments, [switch]$CaptureOutput)
    $startInfo = New-Object System.Diagnostics.ProcessStartInfo
    $startInfo.FileName = $wslExecutable
    $startInfo.Arguments = (($Arguments | ForEach-Object { ConvertTo-NativeArgument $_ }) -join ' ')
    $startInfo.UseShellExecute = $false
    $pipeOutput = (-not $CaptureOutput) -and ([Console]::IsOutputRedirected -or [Console]::IsErrorRedirected)
    # Reuse the existing interactive console. For redirected callers, pipe
    # bytes through explicitly so output is retained without opening a window.
    $startInfo.CreateNoWindow = $CaptureOutput -or $pipeOutput
    $startInfo.WorkingDirectory = $launcherDirectory
    if ($CaptureOutput -or $pipeOutput) {
        $startInfo.RedirectStandardOutput = $true
        $startInfo.RedirectStandardError = $true
    }
    if ($CaptureOutput) {
        $startInfo.StandardOutputEncoding = New-Object System.Text.UTF8Encoding($false)
        $startInfo.StandardErrorEncoding = New-Object System.Text.UTF8Encoding($false)
    }
    $process = New-Object System.Diagnostics.Process
    $process.StartInfo = $startInfo
    try {
        [void]$process.Start()
        if ($CaptureOutput) {
            $stdoutTask = $process.StandardOutput.ReadToEndAsync()
            $stderrTask = $process.StandardError.ReadToEndAsync()
        } elseif ($pipeOutput) {
            $stdoutTask = $process.StandardOutput.BaseStream.CopyToAsync([Console]::OpenStandardOutput())
            $stderrTask = $process.StandardError.BaseStream.CopyToAsync([Console]::OpenStandardError())
        }
        $process.WaitForExit()
        if ($CaptureOutput) {
            return [PSCustomObject]@{
                ExitCode = $process.ExitCode
                Stdout = $stdoutTask.Result
                Stderr = $stderrTask.Result
            }
        }
        if ($pipeOutput) {
            [void]$stdoutTask.GetAwaiter().GetResult()
            [void]$stderrTask.GetAwaiter().GetResult()
        }
        return $process.ExitCode
    } finally {
        $process.Dispose()
    }
}

function ConvertTo-WslPath {
    param([string]$WindowsPath)
    $conversion = Invoke-WslProcess -Arguments @('--distribution', $distributionName, '--exec', 'wslpath', '-a', '-u', $WindowsPath) -CaptureOutput
    if ($conversion.ExitCode -ne 0) {
        $script:launcherExitCode = $conversion.ExitCode
        throw ('WSL 路径转换失败。请确认 Ubuntu-22.04 已安装且能正常启动。' + [Environment]::NewLine + $conversion.Stderr.Trim())
    }
    $linuxPath = $conversion.Stdout.TrimEnd("`r", "`n")
    if (-not $linuxPath.StartsWith('/')) {
        throw ('WSL 未返回有效的绝对路径：' + $linuxPath)
    }
    return $linuxPath
}

$launcherExitCode = 1
try {
    if (-not (Test-Path -LiteralPath $wslExecutable -PathType Leaf)) {
        throw '未找到 Windows Subsystem for Linux。请先安装 WSL 和 Ubuntu-22.04。'
    }
    if (-not (Test-Path -LiteralPath (Join-Path $launcherDirectory 'start.sh') -PathType Leaf)) {
        throw '未找到同目录的 start.sh。请使用完整的定位仓库目录。'
    }
    $linuxDirectory = ConvertTo-WslPath $launcherDirectory
    $forwarded = New-Object 'System.Collections.Generic.List[string]'
    $expectPathValue = $false
    foreach ($argument in $runnerArguments) {
        if ($argument -match '^[A-Za-z]:[\\/]' -or $argument.StartsWith('\\')) {
            $forwarded.Add((ConvertTo-WslPath $argument))
        } elseif ($argument -match '^--[^=]+=([A-Za-z]:[\\/]|\\\\)') {
            $equalsIndex = $argument.IndexOf('=')
            $forwarded.Add($argument.Substring(0, $equalsIndex + 1) + (ConvertTo-WslPath $argument.Substring($equalsIndex + 1)))
        } elseif ($expectPathValue) {
            $forwarded.Add($argument.Replace('\', '/'))
        } elseif ($argument -match '^--(config|output)=') {
            $equalsIndex = $argument.IndexOf('=')
            $pathValue = $argument.Substring($equalsIndex + 1)
            if ($pathValue -match '^[A-Za-z]:[\\/]' -or $pathValue.StartsWith('\\')) {
                $pathValue = ConvertTo-WslPath $pathValue
            } else {
                $pathValue = $pathValue.Replace('\', '/')
            }
            $forwarded.Add($argument.Substring(0, $equalsIndex + 1) + $pathValue)
        } else {
            $forwarded.Add($argument)
        }
        $expectPathValue = $argument -in @('--config', '--output')
    }
    Write-Host ('正在启动定位工具（WSL：' + $distributionName + '）')
    Write-Host ('仓库目录：' + $launcherDirectory)
    if ($runnerArguments -contains '--check') {
        Write-Host '本次只检查配置与环境，不回放数据。'
    }
    $launchArguments = @('--distribution', $distributionName, '--exec', 'bash', '--', ($linuxDirectory + '/start.sh')) + $forwarded.ToArray()
    $launcherExitCode = Invoke-WslProcess -Arguments $launchArguments
    if ($launcherExitCode -ne 0) {
        Write-Host ('定位工具未完成，退出码：' + $launcherExitCode) -ForegroundColor Red
        Write-Host '请查看上方错误；首次安装依赖时可使用 start.cmd --install-deps。'
    }
} catch {
    Write-Host ('启动失败：' + $_.Exception.Message) -ForegroundColor Red
}
exit $launcherExitCode
