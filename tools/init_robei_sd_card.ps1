[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [int]$DiskNumber,

    [double]$ExpectedMinGiB = 100.0,
    [double]$ExpectedMaxGiB = 140.0,

    [switch]$Force
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$sectorSize = 512
$partitionStartLba = 65536
$rawStartLba = 2048
$rawSectorCount = $partitionStartLba - $rawStartLba
$superPrimaryLba = 2048
$superBackupLba = 2049
$mapPayloadBytes = 8192
$mapPayloadSectors = 16
$mapAHeaderLba = 4096
$mapAPayloadLba = 4097
$mapACommitLba = 4113
$mapBHeaderLba = 8192
$mapBPayloadLba = 8193
$mapBCommitLba = 8209
$calibrationALba = 12288
$calibrationBLba = 12304

if (-not $Force) {
    throw 'This operation destroys all partitions on the target disk. Re-run with -Force after checking the disk number.'
}

$principal = [Security.Principal.WindowsPrincipal]::new(
    [Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'Administrator privileges are required.'
}

$disk = Get-Disk -Number $DiskNumber
$sizeGiB = $disk.Size / 1GB
if ($disk.IsBoot -or $disk.IsSystem) {
    throw "Refusing to initialize system or boot disk $DiskNumber."
}
if ($disk.BusType -ne 'USB') {
    throw "Refusing disk $DiskNumber because BusType is $($disk.BusType), not USB."
}
if ($disk.IsReadOnly) {
    throw "Disk $DiskNumber is read-only."
}
if ($sizeGiB -lt $ExpectedMinGiB -or $sizeGiB -gt $ExpectedMaxGiB) {
    throw ("Disk {0} size {1:N3} GiB is outside the expected range {2:N1}-{3:N1} GiB." -f
        $DiskNumber, $sizeGiB, $ExpectedMinGiB, $ExpectedMaxGiB)
}

$source = @'
using System;
using System.IO;

public static class RobeiSdRaw
{
    public static uint Crc32(byte[] data, int count)
    {
        uint crc = 0xffffffffu;
        for (int i = 0; i < count; i++)
        {
            crc ^= data[i];
            for (int bit = 0; bit < 8; bit++)
                crc = (crc & 1u) != 0 ? (crc >> 1) ^ 0xedb88320u : crc >> 1;
        }
        return crc ^ 0xffffffffu;
    }

    public static void WriteSector(int diskNumber, long lba, byte[] data)
    {
        if (data == null || data.Length != 512)
            throw new ArgumentException("A raw sector must contain exactly 512 bytes.");

        string path = @"\\.\PhysicalDrive" + diskNumber;
        using (FileStream stream = new FileStream(
            path, FileMode.Open, FileAccess.ReadWrite, FileShare.ReadWrite, 4096,
            FileOptions.WriteThrough))
        {
            stream.Seek(lba * 512L, SeekOrigin.Begin);
            stream.Write(data, 0, data.Length);
            stream.Flush(true);
        }
    }

    public static byte[] ReadSector(int diskNumber, long lba)
    {
        byte[] data = new byte[512];
        string path = @"\\.\PhysicalDrive" + diskNumber;
        using (FileStream stream = new FileStream(
            path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite))
        {
            stream.Seek(lba * 512L, SeekOrigin.Begin);
            int offset = 0;
            while (offset < data.Length)
            {
                int count = stream.Read(data, offset, data.Length - offset);
                if (count == 0)
                    throw new EndOfStreamException();
                offset += count;
            }
        }
        return data;
    }
}
'@

if (-not ('RobeiSdRaw' -as [type])) {
    Add-Type -TypeDefinition $source -Language CSharp
}

function Set-U32Le {
    param([byte[]]$Buffer, [int]$Offset, [uint32]$Value)
    [BitConverter]::GetBytes($Value).CopyTo($Buffer, $Offset)
}

function Set-U64Le {
    param([byte[]]$Buffer, [int]$Offset, [uint64]$Value)
    [BitConverter]::GetBytes($Value).CopyTo($Buffer, $Offset)
}

Write-Output ("TARGET disk={0} model={1} size_gib={2:N3}" -f
    $disk.Number, $disk.FriendlyName, $sizeGiB)

Clear-Disk -Number $DiskNumber -RemoveData -RemoveOEM -Confirm:$false
Update-HostStorageCache
$clearedDisk = Get-Disk -Number $DiskNumber
if ($clearedDisk.PartitionStyle -eq 'RAW') {
    Initialize-Disk -Number $DiskNumber -PartitionStyle MBR | Out-Null
} elseif ($clearedDisk.PartitionStyle -ne 'MBR') {
    throw "Disk $DiskNumber retained partition style $($clearedDisk.PartitionStyle); MBR is required."
}

$totalSectors = [uint64]([math]::Floor($disk.Size / $sectorSize))
$superblock = [byte[]]::new($sectorSize)
[Text.Encoding]::ASCII.GetBytes('ROBEISD1').CopyTo($superblock, 0x000)
Set-U32Le $superblock 0x008 1
Set-U32Le $superblock 0x00c $sectorSize
Set-U32Le $superblock 0x010 $rawStartLba
Set-U32Le $superblock 0x014 $rawSectorCount
Set-U32Le $superblock 0x018 $partitionStartLba
Set-U32Le $superblock 0x01c $mapPayloadBytes
Set-U32Le $superblock 0x020 $mapPayloadSectors
Set-U32Le $superblock 0x024 $mapAHeaderLba
Set-U32Le $superblock 0x028 $mapAPayloadLba
Set-U32Le $superblock 0x02c $mapACommitLba
Set-U32Le $superblock 0x030 $mapBHeaderLba
Set-U32Le $superblock 0x034 $mapBPayloadLba
Set-U32Le $superblock 0x038 $mapBCommitLba
Set-U32Le $superblock 0x03c $calibrationALba
Set-U32Le $superblock 0x040 $calibrationBLba
Set-U32Le $superblock 0x044 1
Set-U64Le $superblock 0x048 ([DateTimeOffset]::UtcNow.ToUnixTimeSeconds())
Set-U64Le $superblock 0x050 $totalSectors
$description = [byte[]]::new(16)
[Text.Encoding]::ASCII.GetBytes('ROBEI CAR MAP').CopyTo($description, 0)
$description.CopyTo($superblock, 0x058)
$superCrc = [RobeiSdRaw]::Crc32($superblock, 508)
Set-U32Le $superblock 0x1fc $superCrc

$emptySector = [byte[]]::new($sectorSize)
[RobeiSdRaw]::WriteSector($DiskNumber, $superPrimaryLba, $superblock)
[RobeiSdRaw]::WriteSector($DiskNumber, $superBackupLba, $superblock)
foreach ($lba in @(
    $mapAHeaderLba, $mapACommitLba,
    $mapBHeaderLba, $mapBCommitLba,
    $calibrationALba, $calibrationBLba)) {
    [RobeiSdRaw]::WriteSector($DiskNumber, $lba, $emptySector)
}

$partitionParameters = @{
    DiskNumber       = $DiskNumber
    Offset           = [uint64]$partitionStartLba * $sectorSize
    UseMaximumSize   = $true
    AssignDriveLetter = $true
}
$partition = New-Partition @partitionParameters

$formatParameters = @{
    Partition          = $partition
    FileSystem         = 'exFAT'
    NewFileSystemLabel = 'ROBEI_SD'
    Confirm            = $false
    Force              = $true
}
$volume = Format-Volume @formatParameters

$infoPath = "{0}:\ROBEI_SD_INFO.txt" -f $partition.DriveLetter
@"
ROBEI car SD card

The first 32 MiB contains FPGA raw storage. Do not repartition or reformat this
card. Normal file copies to this exFAT volume are safe.

Format: ROBEI_SD_RAW_V1
FPGA raw start LBA: $rawStartLba
PC partition start LBA: $partitionStartLba
Map payload: $mapPayloadBytes bytes
"@ | Set-Content -LiteralPath $infoPath -Encoding ascii

$primary = [RobeiSdRaw]::ReadSector($DiskNumber, $superPrimaryLba)
$backup = [RobeiSdRaw]::ReadSector($DiskNumber, $superBackupLba)
$primaryMagic = [Text.Encoding]::ASCII.GetString($primary, 0, 8)
$backupMagic = [Text.Encoding]::ASCII.GetString($backup, 0, 8)
$storedCrc = [BitConverter]::ToUInt32($primary, 508)
$calculatedCrc = [RobeiSdRaw]::Crc32($primary, 508)
$actualPartition = Get-Partition -DiskNumber $DiskNumber |
    Where-Object DriveLetter -eq $partition.DriveLetter
$actualVolume = Get-Volume -DriveLetter $partition.DriveLetter

if ($primaryMagic -ne 'ROBEISD1' -or $backupMagic -ne 'ROBEISD1') {
    throw 'Superblock readback magic verification failed.'
}
if ($storedCrc -ne $calculatedCrc) {
    throw 'Superblock readback CRC verification failed.'
}
if ($actualPartition.Offset -ne ([uint64]$partitionStartLba * $sectorSize)) {
    throw "Partition offset verification failed: $($actualPartition.Offset)."
}
if ($actualVolume.FileSystem -ne 'exFAT' -or $actualVolume.FileSystemLabel -ne 'ROBEI_SD') {
    throw 'Computer-visible volume verification failed.'
}

Write-Output ("RESULT superblock_magic={0} crc32=0x{1:x8}" -f $primaryMagic, $storedCrc)
Write-Output ("RESULT raw_lba={0}-{1} map_a={2} map_b={3}" -f
    $rawStartLba, ($partitionStartLba - 1), $mapAHeaderLba, $mapBHeaderLba)
Write-Output ("RESULT volume={0}: filesystem={1} label={2} partition_start_lba={3}" -f
    $partition.DriveLetter, $actualVolume.FileSystem, $actualVolume.FileSystemLabel,
    ($actualPartition.Offset / $sectorSize))
Write-Output 'RESULT TEST_PASS robei_sd_initialization'
