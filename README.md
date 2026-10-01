# Secadora-de-filamento
Resumo do sistema
Você está montando um sistema de aquecimento/estufa com:
- ESP32 WROOM-32
- LCD 16x2 com I2C, endereço 0x27
- 5 botões ligados entre GPIO e GND, usando INPUT_PULLUP
- Relé real para controlar a grelha/aquecedor
- BTS7960 43A para controlar um cooler
- Temperatura virtual, temporariamente, porque o DHT22 estava apresentando problemas
- Temperatura inicial virtual: 30 °C
- Temperatura alvo configurável pelo menu
- Relé controlado automaticamente por temperatura com histerese
- Velocidade da fan controlada manualmente de 0 a 100%
- UP/DOWN têm repetição após 0,5 s e vão acelerando enquanto o botão permanece pressionado
- LCD mostra temperatura, estado do relé, setpoint e velocidade da fan
- O PWM usa a API ESP32 Core 3.x, que você confirmou funcionar:
  - ledcAttach()
  - ledcWrite(pino, valor)
Pinagem atual
Função	ESP32
LCD SDA	GPIO 21
LCD SCL	GPIO 22
Relé	GPIO 18
BTS7960 RPWM	GPIO 25
BTS7960 LPWM	GPIO 26
BTS7960 R_EN	GPIO 27
BTS7960 L_EN	GPIO 19
Botão UP	GPIO 32
Botão DOWN	GPIO 33
Botão LEFT	GPIO 13
Botão RIGHT	GPIO 14
Botão OK	GPIO 15
https://chatgpt.com/share/6abe9a1b-7aac-83ea-bf22-7d291df16d04
