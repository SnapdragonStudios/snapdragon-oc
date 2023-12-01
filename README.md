# Snapdragon™ Occlusion Culling (SDOC)
SDOC is a mobile-optimized version of the Open Source project https://github.com/rawrunprotected/rasterizer
The related optimizations include
1. Same frame skipping
2. Interleave rendering
3. Fast block traversal
4. Mix use of triangle and quad rendering
5. Checkerboard memory optimization
6. Mesh rectangle simplification during baking
7. Memory compression during baking
8. Potential visibility for occluder occludee object

## Guidence on building SDOC for Windows, Mac, Android and iOS.


### Windows

1. Install CMake 3.0 or higher.
2. Install Visual Studio 2017 or higher.
3. Open your source folder in Explorer and enter the `Source/app/FuzzyCulling/`. Run `GenerateProjectFiles.bat` to create project files for SOC. The Visual Studio project file will be generated in  `build` folder.
4. Unzip app\FuzzyCulling\GOLDEN_DATA\all.7z file
5. Set the `SDOCReplay` as **StartUp Project** in Visual Studio, 
6. Visual studio opens Source\app\FuzzyCulling\build\SDOC-Lib.sln
7. Set SDOCReplay project as start up project to test, compile and run
8. For debug testing, set SDOC_DevelopMode to 1, set it as 0 when generating dynamic library libSDOC.quic.dll

### Android

Build the shared library
1. Latest Android studio to open the `Source` folder to load the project
2. Sync Project with Gradle Files, then setup the key for building the releasing application. 
3. Build->Generate Signed Bundle/APK, select **release** as the **Build Variants**.
4. Unzip the APK for libSDOC.quic.so, the libSDOC.quic.so could be integrated with game engines.

Android demo to verify libSDOC.quic.so
1. Put built android libSDOC.quic.so in Demo\Android\SDOCDemo\app\src\main\jnilibs
2. Android Studio open Demo\Android\SDOCDemo and build the ready to run apk. 



### OSX
1. Install CMake 3.0 or higher.
2. Install the latest version of Xcode.
3. Open the **Terminal** at the source folder and enter `Source/app/FuzzyCuling`, execute the following command to generate Xcode project to `build` folder.
   ```   
   chmod 755 GenerateProjectFiles_Mac.command
   GenerateProjectFiles_Mac.command
   ```
5. Enter the `build` folder and load the project into Xcode by double-clicking on the `SDOC-Lib.xcodeproj`.  Select the **ALL_BUILD** as the target and then build the project.
6. To change the build configuration to **Release**, **Edit Scheme** in **Main Menu->Product->Scheme**, set the Run Executable to SDOCReplay and other
7. Compile and execute. The libSDOC.quic.dylib is shared library could be integrated with game engines.



### iOS
Build the static library
1. Sync the submodule to get [ios-cmake](https://github.com/leetal/ios-cmake.git). 
   git submodule init
   git submodule update   
   You might need to call "git submodule add https://github.com/leetal/ios-cmake.git" in the directory of sdoc\Source\app\submodules if submodule init/update fail
2. Open the **Terminal** at the source folder and enter `Source/app/FuzzyCuling`, execute the following command to generate Xcode project to `build` folder.
   ```
   chmod 755 GenerateProjectFiles_ios.command
   GenerateProjectFiles_ios.command
   ```
3. Enter the `build` folder and load the project into Xcode by double-clicking on the `SDOC-Lib.xcodeproj`.  Select the **ALL_BUILD** as the target and then build the project.
4. To change the build configuration to **Release**, **Edit Scheme** in **Main Menu->Product->Scheme**.

iOS demo to verify 
1. Connect iphone
2. iOS: open `Demo\iOS\iQOC.xcodeproj` to build iQOC (QOC is the previous name of SDOC)
3. "Fail to load the file" message would show for the init run
4. Cmd+shift+2 to summon iphone device UI
5. from iphone device UI, select iQOC and click "App Container Action" and choose "Download Container"
5. Right click the "Download Container" and choose "Show Package Contents"
6. Copy files from Source\app\FuzzyCulling\GOLDEN_DATA\all to container's AppData/Documents
7. From iphone device UI, click "App Container Action" and choose "Replace Container"
8. Restart iQOC on the iphone

# License
Snapdragon™ Occlusion Culling (SDOC) is licensed under the BSD 3-clause “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.
