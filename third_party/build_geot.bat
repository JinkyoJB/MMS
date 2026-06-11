@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Professional\VC\Auxiliary\Build\vcvars64.bat"
set DISTUTILS_USE_SDK=1
set INCLUDE=C:\Users\user\miniconda3\envs\reg-dl\Lib\site-packages\nvidia\cuda_runtime\include;C:\Users\user\miniconda3\envs\reg-dl\Lib\site-packages\nvidia\cuda_cccl\include;%INCLUDE%
cd /d "C:\Users\user\workspace\MMS\third_party\GeoTransformer"
"C:\Users\user\miniconda3\envs\reg-dl\python.exe" setup.py build_ext --inplace
