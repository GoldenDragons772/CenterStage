# CenterStage
**Welcome! to GoldenDragon FTC #772 from SCGSSM 🐉.**
<p>This repo consists of TeamCode and other information regarding the Robot used for the Year 2024 CenterStage Game.</p>

---
### Getting Started:
1. [Setup Android Studio.](#how-to-set-up-android-studio)
2. [Install Dependendies](#dependencies)
3. [Build Project](#build-project)

## How to set up Android Studio
**Two ways to install Android Studio:**
- [Using the Installer](#1-using-the-installer-android-studio). (For Noobs)
- [Using the Commandline.](#2-using-the-commadline-winget-and-windows-only) (For Pros and Windows Only)

---

#### 1. Using the Installer ([Android Studio](https://developer.android.com/studio)).
This is the normal way of installing Android Studio (Pretty Boring).

**Steps to setup:**
1. Download **Android Studio** from [here](https://developer.android.com/studio).
2. Run the installer that you just Downloaded.
3. The installer should guide you through the process.
4. Done!, Android Studio should have been installed Successfully.

#### 2. Using the Commadline ([Winget](https://winget.run) and Windows Only) 
Winget essentially downloads and installs it for you automatically, saving you some clicks.

Steps to setup:
1. Open `PowerShell.exe` (Windows 10/11 should come with it by default.)

2. Next, run the command 
```pwsh
> winget install -e --id Google.AndroidStudio
```
3. You should have downloaded Android Studio if you followed the steps correctly.

## Dependencies

- [git](https://git-scm.com/downloads) or [GitHub Desktop](https://desktop.github.com/)
- Add any other dependencies as you need. By Default Gradle should be able to manage these automatically.
## Build Project

1. First Clone the Repo by running
```
> mkdir GoldenDragon && cd GoldenDragon
> git clone https://github.com/GoldenDragons772/CenterStage.git CenterStage
```
or use [GitHub Desktop](https://desktop.github.com/) to clone the Repo, if you don't prefer the commandline.

2. Open the Project in [Android Studio](#how-to-set-up-android-studio), and wait for graddle to sync the project. This may take anywhere between 2-3 minutes.
3. Then Proceed to Build the Project, and deploy it if needed.

---