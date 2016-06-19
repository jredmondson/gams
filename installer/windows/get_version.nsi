!define File "MyFile.exe"
 
OutFile "get_version.exe"
SilentInstall silent
 
Section
 
 ## Get file version

 ; Get version number
 FileOpen $4 "$%GAMS_ROOT%\VERSION.txt" r
 FileRead $4 $1
 FileClose $4

 ## Write it to a !define for use in main script
 FileOpen $4 "$EXEDIR\VERSION.txt" w
  FileWrite $4 '!define PRODUCT_VERSION "$1"'
 FileClose $4

SectionEnd