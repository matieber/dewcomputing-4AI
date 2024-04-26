Para usar el dispositivo: 

1- Conectar el cable USB terminal verde a la máquina donde se correrá el comando de medición
2- Enfuchar el hardware de medición a la corriente eléctrica
3- Enchufar el aparato cuyo consumo se quiere medir al enchufe hembra del hardware de medición

Para compilar la aplicación, desde línea de comando "gcc pmRead.cpp". Generará un archivo a.out que se corre sin parámetros durante el paso 2)
El comando abre el device Linux /dev/ttyUSB0. Puede ser necesario agregar al grupo "dialout" el usuario Linux actual.

La salida del comando es por pantalla, similar a: 

Bienvenido a Power Meter Software
Abriendo puerto
OK
Estableciendo comunicacion con el medidor... 
VEF,IEF,Pact,Pap,FP,KWh
219,9,7,19,37,0
221,9,8,19,43,0
221,9,8,19,43,0
221,9,7,19,37,0
221,9,7,19,37,0
^C

Las columnas son: 

- Voltage[V]
- Current[A]
- Active power[W]
- Apparent power[VA]
- Power factor
- Energy[KWh]
