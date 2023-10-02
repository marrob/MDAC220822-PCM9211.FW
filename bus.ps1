$argsArray = 
    "/BAUD=230400", #Speed
    "/C=24" ,        #Serial port COM16
    "/DS"           #Disable displaying the "New connection" dialog on startup
Start-Process -FilePath "C:\Program Files (x86)\teraterm\ttermpro.exe" -ArgumentList $argsArray
