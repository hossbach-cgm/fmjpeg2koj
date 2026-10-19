IF "%1"=="Release" (
SET TYPE=Release
) ELSE (
SET TYPE=Debug
)

IF "%2"=="localgit" (
SET GITHUBURL=http://gitcache:8080/github.com
) ELSE (
SET GITHUBURL=https://github.com
)

REM a top level directory for all PACS related code
SET DEVSPACE="%CD%"
SET GENERATOR="Visual Studio 17 2022" -A x64

cd %DEVSPACE%
git clone --branch=DCMTK-3.6.8 --single-branch --depth 1 https://github.com/DCMTK/dcmtk.git
cd dcmtk
mkdir build-%TYPE%
cd build-%TYPE%
cmake .. -G %GENERATOR% -DDCMTK_WIDE_CHAR_FILE_IO_FUNCTIONS=1 -DCMAKE_INSTALL_PREFIX=%DEVSPACE%\dcmtk\%TYPE%
msbuild /P:Configuration=%TYPE% INSTALL.vcxproj 

cd %DEVSPACE%
git clone --branch=v2.4.0 --single-branch --depth 1 https://github.com/uclouvain/openjpeg.git
cd openjpeg
mkdir build-%TYPE%
cd build-%TYPE%
cmake .. -G %GENERATOR% -DBUILD_SHARED_LIBS=OFF -DBUILD_THIRDPARTY=1 -DCMAKE_C_FLAGS_RELEASE="/MT /O2 /D NDEBUG" -DCMAKE_C_FLAGS_DEBUG="/D_DEBUG /MTd /Od" -DCMAKE_INSTALL_PREFIX=%DEVSPACE%\openjpeg\%TYPE%
msbuild /P:Configuration=%TYPE% INSTALL.vcxproj

cd %DEVSPACE%
mkdir build-%TYPE%
cd build-%TYPE%
cmake .. -G %GENERATOR% -DBUILD_SHARED_LIBS=OFF -DCMAKE_CXX_FLAGS_RELEASE="/MT /O2 /D NDEBUG" -DCMAKE_CXX_FLAGS_DEBUG="/D_DEBUG /MTd /Od" -DOpenJPEG_ROOT=%DEVSPACE%\openjpeg\%TYPE% -DDCMTK_ROOT=%DEVSPACE%\dcmtk\build-%TYPE% -DCMAKE_INSTALL_PREFIX=%DEVSPACE%\%TYPE%
msbuild /P:Configuration=%TYPE% INSTALL.vcxproj

cd %DEVSPACE%
