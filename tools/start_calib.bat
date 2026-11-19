@echo off
rem Start a local HTTP server for the focal calibration page.
rem Browsers block camera access on file:// pages, so the page must be
rem served over http://localhost.
cd /d "%~dp0.."
echo Serving %CD%
echo Open: http://localhost:8000/tools/focal_calib.html
start "calib-server" /min cmd /c "python -m http.server 8000"
timeout /t 2 >nul
start "" "http://localhost:8000/tools/focal_calib.html"
