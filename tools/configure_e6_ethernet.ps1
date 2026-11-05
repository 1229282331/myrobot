$ErrorActionPreference = 'Stop'
$stateDirectory = Join-Path $PSScriptRoot '..\local-network'
New-Item -ItemType Directory -Path $stateDirectory -Force | Out-Null
$resultPath = Join-Path $stateDirectory 'configure-result.json'
try {
    $adapter = Get-NetAdapter -InterfaceIndex 6
    if ($adapter.InterfaceDescription -ne 'Killer E5000B 5 Gigabit Ethernet Controller' -or $adapter.Status -ne 'Up') {
        throw 'Expected connected physical Ethernet adapter was not found. No network settings changed.'
    }
    $addresses = @(Get-NetIPAddress -InterfaceIndex 6 -AddressFamily IPv4)
    if (@($addresses | Where-Object { $_.IPAddress -notlike '169.254.*' -and $_.IPAddress -ne '192.168.5.10' }).Count -gt 0) {
        throw 'Ethernet address differs from the inspected configuration. No network settings changed.'
    }
    $routes = @(Get-NetRoute -InterfaceIndex 6 -AddressFamily IPv4)
    if (@($routes | Where-Object { $_.DestinationPrefix -eq '0.0.0.0/0' }).Count -gt 0) {
        throw 'Ethernet has a default gateway. No network settings changed.'
    }
    $backupPath = Join-Path $stateDirectory 'ethernet-before.json'
    if (-not (Test-Path -LiteralPath $backupPath)) {
        [pscustomobject]@{
            capturedAt = [DateTime]::UtcNow.ToString('o')
            adapter = $adapter | Select-Object Name,InterfaceIndex,InterfaceDescription,InterfaceGuid
            ipInterface = Get-NetIPInterface -InterfaceIndex 6 -AddressFamily IPv4 | Select-Object Dhcp,InterfaceMetric
            addresses = $addresses | Select-Object IPAddress,PrefixLength,PrefixOrigin
            dns = @( (Get-DnsClientServerAddress -InterfaceIndex 6 -AddressFamily IPv4).ServerAddresses )
            routes = $routes | Select-Object DestinationPrefix,NextHop,RouteMetric
        } | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $backupPath -Encoding UTF8
    }
    if (-not ($addresses | Where-Object { $_.IPAddress -eq '192.168.5.10' })) {
        New-NetIPAddress -InterfaceIndex 6 -IPAddress '192.168.5.10' -PrefixLength 24 -AddressFamily IPv4 | Out-Null
    }
    [pscustomobject]@{ success=$true; interfaceIndex=6; address='192.168.5.10/24'; defaultGateway='none'; wifiChanged=$false; time=[DateTime]::UtcNow.ToString('o') } |
        ConvertTo-Json | Set-Content -LiteralPath $resultPath -Encoding UTF8
} catch {
    [pscustomobject]@{success=$false; error=$_.Exception.Message; time=[DateTime]::UtcNow.ToString('o')} |
        ConvertTo-Json | Set-Content -LiteralPath $resultPath -Encoding UTF8
    exit 1
}
