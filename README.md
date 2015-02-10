# PLX_Pressure
Alternative firmware for PLX Pressure module

Это прошивка для [**PLX Fluid Pressure Sensor Module**](http://plxdevices.com/product_info.php?id=SEMOSMFP) основанном на PIC18F2410.
Я полностью написал код с ноля, т.к. PLX не пожелал поделиться оригиналом, ну оно и понятно. Не так велика проблема. Я постарался повторить функционал на сколько мне хватило терпения. Так же работает JUMPер перевода MASTER/SLAVE и устройство корректно работает в паре с заводскими. За исключением нескольких моментов:
* один вход рачитан на заводской датчик давления маса Nissan с линейной характеристикой от 0 до 5В**[1]*
* нет режима дампа параеметров**[2]*
* не откалиброван DAC**[3]*

[1] я как мог калибровал датчик, уж не обессудьте  
[2] в описании PLX на [протокол](http://www.plxdevices.com/AppNotes/PLXApp018.pdf) есть некий режим *Fast Download (150,000 Baud 8E1)*, при подключении логгера, модули общаются по нему. Но у меня нет логгера и проверить как это работает нет возможности.  
[3] у меня нет надобности в нём. Я описал заглушку в коде для вывода информации на DAC. Но не углублялся.  
