# Legacy Kexts

* [FAT kexts](#fat-kexts)
* [32 Bit Kexts](#32-bit-kexts)

Goal of this repo is to help preserve legacy OSX kexts as more and more sites remove old content. If there's any source code, original thread, prebuilt binaries, etc you'd like to add, feel free to make a PR.

Good chunk fo these kexts were also provided by [this Archive](https://code.google.com/archive/p/leohazard/downloads)

## FAT Kexts

Includes both 32 and 64 bit slices allowing to be used with 10.4-10.7

| Kext | OS Support | Comments |
| :--- | :--- | :--- |
| [FakeSMC](/FAT/Zip/fakesmc.kext.zip) | 10.5+ | SMC Emulator |
| [VoodooHDA](/FAT/Zip/VoodooHDA.kext.zip) | 10.4+ | Audio |
| [VoodooPS2](/FAT/Zip/VoodooPS2Controller.kext.zip) | 10.4+ | PS2 Support |
| [NullCPUPowerManagement](/FAT/Zip/NullCPUPowerManagement.kext.zip) | 10.4+ | Disables AppleIntelCPUPowerManagement |
| [AppleACPIPS2Nub](/FAT/Zip/AppleACPIPS2Nub.kext.zip) | 10.4+ | PS2 Nub |
| [CPUi](/FAT/Zip/CPUi.kext.zip) | 10.5+ | Unknown |
| [VoodooTSCsync](/FAT/Zip/VoodooTSCSync.kext.zip) | 10.4+ | Syncs TSC |

## 32 Bit kexts

| Kext | OS Support | Comments |
| :--- | :--- | :--- |
| [FakeSMC-32](/32Bit-only/Zip/FakeSMC-32.kext.zip) | 10.4+ | SMC Emulator |
| [VoodooBattery](/32Bit-only/Zip/VoodooBattery.kext.zip) | 10.4 | Battery Reporting |
| [VoodooPowerMini](/32Bit-only/Zip/VoodooPowerMini.kext.zip) | 10.4 | Intel Speed Step support |
| [IOSDHCIBlockDevice](/32Bit-only/Zip/IOSDHCIBlockDevice.kext.zip) | 10.4+ | SD Reader support |
| [RealtekR1000](/32Bit-only/Zip/RealtekR1000.kext.zip) | 10.4+ | Ethernet |
| [ACPIBatteryManager](/32Bit-only/Zip/AppleACPIBatteryManager.kext.zip) | 10.4+ | Battery Reporting, [Source code](/32Bit-only/AppleACPIBatteryManager-Source-Code/), [Original Tread](https://www.insanelymac.com/forum/topic/114105-appleacpibatterymanager-for-tiger-and-leopard/) |
| [AppleIntelE1000e](/32Bit-only/Zip/AppleIntelE1000e.kext.zip) | 10.5+ | Ethernet, [Source code](/32Bit-only/AppleIntelE1000e-Source-Code/), [Original Repo](https://sourceforge.net/p/osx86drivers/), [Original Thread](https://www.insanelymac.com/forum/topic/205771-appleintele1000ekext-for-108107106105/) |
