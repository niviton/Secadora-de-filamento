// Garra Inteligente - Controle dos eixos X (horizontal) e Z (vertical)
// Placa: Anet v1.5 (Sanguino / ATmega1284P) - drivers A4988 em 1/16 de passo
//
// Sistema de coordenadas:
//   ZERO (0,0) = garra no ponto MAIS ALTO e MAIS À DIREITA (visto de frente, como na foto)
//   X positivo = garra andando para a ESQUERDA (afastando do zero)
//   Z positivo = garra DESCENDO
//
// Comandos pelo Monitor Serial (115200 baud, final de linha "Nova linha"):
//   ?            -> mostra posição e parâmetros
//   x 120        -> vai para X = 120 mm (respeita os limites)
//   z 50         -> vai para Z = 50 mm  (respeita os limites)
//   m 120 50     -> move X e Z AO MESMO TEMPO, em linha reta, até X=120 Z=50
//   xr 10        -> anda +10 mm em X a partir da posição atual (zr para Z)
//   !            -> freia e para o movimento em andamento
//   (toda resposta termina com "ok"; a linha "POS x z cx cz" traz a posição)
//   h            -> volta ao zero (primeiro sobe Z, depois recolhe X)
//   xs 800       -> move X 800 passos relativos, IGNORANDO limites (calibração)
//   zs -800      -> idem para Z
//   zero         -> define a posição atual como zero (X=0, Z=0)
//   fimx / fimz  -> define a posição atual como fim de curso daquele eixo
//   vx 1500      -> velocidade máxima de X em passos/s   (vz para Z)
//   ax 2000      -> aceleração de X em passos/s²         (az para Z)
//   teste x      -> vai do início ao fim do eixo e volta, 3 vezes (teste z para Z)
//   rampa x      -> aumenta a velocidade aos poucos até você mandar parar (rampa z)
//   acel x       -> aumenta a aceleração aos poucos até você mandar parar (acel z)
//   soltar       -> desliga os drivers (CUIDADO: o eixo Z cai por gravidade!)
//   travar       -> liga os drivers

// =========================================================================
// PARÂMETROS DE CALIBRAÇÃO (AJUSTE AQUI)
// =========================================================================
// Passos por mm = 3200 / (mm que a garra anda em 1 volta do motor).
// Para medir: use "xs 3200" (1 volta), meça com régua quanto andou e faça a conta.
// (Pela fórmula: 3200 / (nº de dentes do pinhão × módulo × π))
// ESTIMATIVA: 6720 passos andaram ~1/2 de 400 mm (~200 mm) -> 6720 / 200 ≈ 33,6.
// Troque pelo valor medido com régua.
const float PASSOS_POR_MM_X = 33.6;
const float PASSOS_POR_MM_Z = 33.6;

// Curso útil de cada eixo em mm (medido com "fimx"/"fimz")
// A estrutura tem 400 x 400 mm, mas o curso útil é MENOR: desconta o carro,
// o suporte do motor e a própria garra. Valores iniciais conservadores até medir.
// Para achar o valor: aumente de 20 em 20 mm até a garra parar ~5 mm antes do fim.
// Cremalheira do X = 400 mm. Margem de 10 mm para o pinhão não sair do fim dos dentes.
float CURSO_X_MM = 390.0;   // 13104 passos
// Cremalheira do Z = 400 mm (mesma do X). Mesma margem de 10 mm.
// ATENÇÃO: a garra pode encostar na mesa antes do fim da cremalheira.
float CURSO_Z_MM = 390.0;   // 13104 passos

// Se o motor andar para o lado errado com "xs 800" (deveria ir para a ESQUERDA)
// ou "zs 800" (deveria DESCER), troque para true.
const bool INVERTER_X = true;   // testado: estava andando ao contrário
const bool INVERTER_Z = true;   // testado: estava andando ao contrário

// Velocidades e acelerações
// Teste de rampa: melhor velocidade sem falha = 2800 passos/s (com aceleração 2000).
// Usamos ~70% disso como margem de segurança.
float VMAX_X = 2000;   // passos/s (≈ 60 mm/s)
float ACEL_X = 2000;   // passos/s²
float VMAX_Z = 2000;
float ACEL_Z = 2000;

// Velocidade de partida (sem rampa). Abaixo disso o motor nunca perde passo.
const float V_INICIAL = 250;   // passos/s

// O pino de ENABLE do X (14) liga juntos os drivers X, Y e E0.
// Como o Z está na porta Y, desligar o X também soltaria o Z (que cairia).
// Por isso isso é ignorado enquanto X e Z dividirem o mesmo ENABLE.
const bool DESLIGAR_X_PARADO = false;

// Liga as saídas de cooler da placa para resfriar os drivers (se houver ventoinha ligada nelas).
const bool COOLER_NOS_DRIVERS = true;
const float V_LIMITE  = 6000;  // teto de segurança do software

// Fim de curso (opcional). A placa Anet tem conectores: X_MIN = 18, Z_MIN = 20.
// Sem fim de curso: ao ligar, a garra PRECISA estar no zero (alto + direita).
const bool USAR_FIM_DE_CURSO = false;
#define X_MIN_PIN 18
#define Z_MIN_PIN 19   // conector Y_MIN, já que o Z usa a porta Y
const int FIM_ACIONADO = HIGH;   // chaves da Anet: HIGH quando pressionada
const float V_HOMING = 400;      // passos/s na busca do fim de curso

// Teste automático ao ligar (não precisa do Monitor Serial):
// X vai do zero até CURSO_X_MM e volta; depois Z desce até CURSO_Z_MM e sobe.
// Desligado: abrir o Monitor Serial reinicia a placa e dispararia o teste toda vez.
const bool TESTE_AO_LIGAR = false;

// Teste de velocidade máxima (comando "rampa x" / "rampa z")
const float DIST_RAMPA_MM = 200;   // trecho de ida e volta em cada velocidade
const float ACEL_RAMPA    = 2000;  // passos/s² usada só durante o teste
// true = roda a rampa sozinha ao ligar (X e depois Z), sem precisar digitar nada.
// Cada ida-e-volta é uma velocidade: 1ª = 600, 2ª = 800, 3ª = 1000 ... (+200 por vez).
// Conte as idas-e-voltas até ouvir o "tec-tec".
const bool RAMPA_AO_LIGAR = false;   // já medido: 2800 passos/s

// Teste de aceleração máxima (comando "acel x" / "acel z")
const float DIST_ACEL_MM  = 60;    // movimentos curtos: quase só arrancada e frenagem
const bool TESTAR_X = true;
const bool TESTAR_Z = true;
// =========================================================================

// Pinos da placa (mesmos do teste original)
#define X_STEP   15
#define X_DIR    21
#define X_ENABLE 14   // compartilhado com Y e E0
// O motor do eixo Z da garra está ligado no conector Y da placa
#define Z_STEP   22
#define Z_DIR    23
#define Z_ENABLE 14   // mesmo ENABLE do X (a placa liga X, Y e E0 juntos)
#define PORTA_Z_ENABLE 26   // conector Z da placa, sem uso -> fica desligado

#define FAN1_PIN 4
#define FAN2_PIN 13
#define BED_PIN  12

struct Eixo {
  char nome;
  uint8_t pinStep, pinDir, pinEnable, pinFim;
  bool inverter;
  float passosPorMm;
  float *cursoMm;
  float *vmax;
  float *acel;
  long pos;   // posição atual em passos
};

Eixo eixoX = {'X', X_STEP, X_DIR, X_ENABLE, X_MIN_PIN, INVERTER_X, PASSOS_POR_MM_X, &CURSO_X_MM, &VMAX_X, &ACEL_X, 0};
Eixo eixoZ = {'Z', Z_STEP, Z_DIR, Z_ENABLE, Z_MIN_PIN, INVERTER_Z, PASSOS_POR_MM_Z, &CURSO_Z_MM, &VMAX_Z, &ACEL_Z, 0};

long cursoPassos(Eixo &e) { return (long)(*e.cursoMm * e.passosPorMm); }
float posMm(Eixo &e) { return e.pos / e.passosPorMm; }

void driversLigados(bool ligar) {
  digitalWrite(X_ENABLE, ligar ? LOW : HIGH);
  digitalWrite(Z_ENABLE, ligar ? LOW : HIGH);
}

// Movimento com rampa trapezoidal: acelera, anda na velocidade máxima, desacelera.
// É a rampa que evita o "tec-tec" (perda de passo) na partida e na parada.
// Retorna false se foi interrompido por um caractere recebido na Serial.
bool moverPassos(Eixo &e, long alvo, bool interrompivel) {
  long delta = alvo - e.pos;
  if (delta == 0) return true;

  if (digitalRead(e.pinEnable) == HIGH) {   // driver estava desligado
    digitalWrite(e.pinEnable, LOW);
    delay(5);
  }

  bool frente = delta > 0;
  digitalWrite(e.pinDir, (frente != e.inverter) ? HIGH : LOW);
  delayMicroseconds(5);

  long n = labs(delta);
  float vmax = constrain(*e.vmax, V_INICIAL, V_LIMITE);
  float v0q = V_INICIAL * V_INICIAL;
  float dois_a = 2.0 * *e.acel;
  long passo = frente ? 1 : -1;

  unsigned long t = micros();
  for (long i = 0; i < n; i++) {
    long k = min(i, n - 1 - i);               // distância até a ponta mais próxima
    float v = sqrt(v0q + dois_a * k);         // v² = v0² + 2·a·s
    if (v > vmax) v = vmax;
    unsigned long periodo = (unsigned long)(1000000.0 / v);

    // Onda quadrada igual à do código original: metade do período em HIGH, metade em LOW
    digitalWrite(e.pinStep, HIGH);
    while (micros() - t < periodo / 2) {}
    digitalWrite(e.pinStep, LOW);
    e.pos += passo;

    while (micros() - t < periodo) {}
    t += periodo;

    if (interrompivel && (i & 63) == 0 && Serial.available()) {
      // Desacelera antes de parar para não perder passo
      long restante = min((long)((v * v - v0q) / dois_a), n - 1 - i);
      moverPassos(e, e.pos + passo * restante, false);
      return false;
    }
  }
  if (DESLIGAR_X_PARADO && X_ENABLE != Z_ENABLE && e.nome == 'X') {
    delay(100);                          // deixa o carro assentar antes de soltar
    digitalWrite(e.pinEnable, HIGH);
  }
  return true;
}

// Move X e Z AO MESMO TEMPO, em linha reta (algoritmo de Bresenham).
// O eixo que anda mais passos ("dominante") dita o ritmo com a rampa de aceleração;
// o outro dá um passo só quando o erro acumulado passa da metade, assim os dois
// chegam juntos no destino.
bool moverLinear(long alvoX, long alvoZ, bool interrompivel) {
  long dx = alvoX - eixoX.pos, dz = alvoZ - eixoZ.pos;
  long ax = labs(dx), az = labs(dz);
  long n = max(ax, az);
  if (n == 0) return true;

  if (digitalRead(X_ENABLE) == HIGH || digitalRead(Z_ENABLE) == HIGH) {
    digitalWrite(X_ENABLE, LOW); digitalWrite(Z_ENABLE, LOW);
    delay(5);
  }
  digitalWrite(X_DIR, ((dx > 0) != INVERTER_X) ? HIGH : LOW);
  digitalWrite(Z_DIR, ((dz > 0) != INVERTER_Z) ? HIGH : LOW);
  delayMicroseconds(5);

  // Usa o mais lento dos dois eixos para nenhum deles perder passo
  float vmax = constrain(min(VMAX_X, VMAX_Z), V_INICIAL, V_LIMITE);
  float v0q = V_INICIAL * V_INICIAL;
  float dois_a = 2.0 * min(ACEL_X, ACEL_Z);
  long passoX = dx > 0 ? 1 : -1, passoZ = dz > 0 ? 1 : -1;
  long erroX = n / 2, erroZ = n / 2;
  long fim = n;
  bool interrompido = false;

  unsigned long t = micros();
  for (long i = 0; i < fim; i++) {
    long k = min(i, fim - 1 - i);
    float v = sqrt(v0q + dois_a * k);
    if (v > vmax) v = vmax;
    unsigned long periodo = (unsigned long)(1000000.0 / v);

    bool darX = false, darZ = false;
    erroX += ax; if (erroX >= n) { erroX -= n; darX = true; }
    erroZ += az; if (erroZ >= n) { erroZ -= n; darZ = true; }

    if (darX) digitalWrite(X_STEP, HIGH);
    if (darZ) digitalWrite(Z_STEP, HIGH);
    while (micros() - t < periodo / 2) {}
    digitalWrite(X_STEP, LOW);
    digitalWrite(Z_STEP, LOW);
    if (darX) eixoX.pos += passoX;
    if (darZ) eixoZ.pos += passoZ;

    while (micros() - t < periodo) {}
    t += periodo;

    if (interrompivel && !interrompido && (i & 63) == 0 && Serial.available()) {
      // Freia seguindo a mesma linha: encurta o fim para caber só a desaceleração
      long restante = (long)((v * v - v0q) / dois_a);
      fim = min(fim, i + 1 + restante);
      interrompido = true;
    }
  }
  return !interrompido;
}

void moverLinearMm(float xmm, float zmm) {
  long alvoX = constrain((long)(xmm * PASSOS_POR_MM_X), 0, cursoPassos(eixoX));
  long alvoZ = constrain((long)(zmm * PASSOS_POR_MM_Z), 0, cursoPassos(eixoZ));
  if (!moverLinear(alvoX, alvoZ, true)) Serial.println(F("Movimento interrompido."));
}

// Movimento em mm, sempre dentro dos limites [0, curso]
void moverMm(Eixo &e, float mm) {
  long alvo = (long)(mm * e.passosPorMm);
  long lim = cursoPassos(e);
  if (alvo < 0 || alvo > lim) {
    Serial.print(F("! Fora do limite, limitado a 0..")); Serial.println(*e.cursoMm);
    alvo = constrain(alvo, 0, lim);
  }
  // Interrompível: qualquer comando recebido no meio (ex.: "!" do botão PARAR) freia o eixo
  if (!moverPassos(e, alvo, true)) Serial.println(F("Movimento interrompido."));
}

void homingFimDeCurso(Eixo &e) {
  digitalWrite(e.pinDir, e.inverter ? HIGH : LOW);   // direção negativa
  unsigned long periodo = 1000000.0 / V_HOMING;
  long maxPassos = cursoPassos(e) + 20 * e.passosPorMm;
  for (long i = 0; i < maxPassos && digitalRead(e.pinFim) != FIM_ACIONADO; i++) {
    digitalWrite(e.pinStep, HIGH); delayMicroseconds(2); digitalWrite(e.pinStep, LOW);
    delayMicroseconds(periodo);
  }
  if (digitalRead(e.pinFim) != FIM_ACIONADO) {
    Serial.print(F("! Fim de curso de ")); Serial.print(e.nome); Serial.println(F(" nao encontrado"));
  }
  e.pos = 0;
  moverPassos(e, (long)(2 * e.passosPorMm), false);   // afasta 2 mm da chave
  e.pos = 0;
}

void irParaZero() {
  if (USAR_FIM_DE_CURSO) {
    homingFimDeCurso(eixoZ);   // sempre sobe primeiro para não bater em nada
    homingFimDeCurso(eixoX);
  } else {
    moverPassos(eixoZ, 0, false);
    moverPassos(eixoX, 0, false);
  }
  Serial.println(F("Em zero."));
}

void testeCurso(Eixo &e) {
  Serial.print(F("Teste de curso completo em ")); Serial.println(e.nome);
  if (e.nome == 'X') moverPassos(eixoZ, 0, false);   // Z em cima antes de varrer X
  for (int c = 1; c <= 3; c++) {
    moverPassos(e, cursoPassos(e), false);
    delay(300);
    moverPassos(e, 0, false);
    delay(300);
    Serial.print(F("  ciclo ")); Serial.print(c); Serial.println(F(" ok"));
  }
  Serial.println(F("Confira se a garra voltou EXATAMENTE para a marca do zero."));
}

// Sobe a velocidade de 200 em 200 passos/s, indo e voltando DIST_RAMPA_MM.
// Quando ouvir "tec-tec" ou ver o motor travar, mande qualquer tecla.
void testeRampa(Eixo &e) {
  float original = *e.vmax;
  float acelOriginal = *e.acel;
  *e.acel = max(acelOriginal, ACEL_RAMPA);   // precisa acelerar rápido para atingir a velocidade no trecho
  long fim = min(cursoPassos(e), (long)(DIST_RAMPA_MM * e.passosPorMm));
  if (e.nome == 'X') moverPassos(eixoZ, 0, false);
  moverPassos(e, 0, false);
  while (Serial.available()) Serial.read();

  float vPico = sqrt(sq(V_INICIAL) + *e.acel * fim);   // maior velocidade que cabe no trecho
  Serial.println(F("Rampa: envie qualquer tecla quando o motor falhar."));
  float ultimaOk = 0;
  for (float v = 600; v <= V_LIMITE && v <= vPico; v += 200) {
    *e.vmax = v;
    Serial.print(F("  testando ")); Serial.print(v, 0); Serial.print(F(" passos/s = "));
    Serial.print(v / e.passosPorMm, 0); Serial.println(F(" mm/s"));
    bool ok = moverPassos(e, fim, true) && moverPassos(e, 0, true);
    if (!ok) break;
    ultimaOk = v;
    delay(1500);   // pausa visível entre uma velocidade e outra (para contar)
  }
  while (Serial.available()) Serial.read();
  *e.vmax = original;
  *e.acel = acelOriginal;

  Serial.print(F("Ultima velocidade sem falha: ")); Serial.println(ultimaOk, 0);
  Serial.print(F("Recomendado (margem de 30%): v")); Serial.print((char)tolower(e.nome));
  Serial.print(' '); Serial.println(ultimaOk * 0.7, 0);
  Serial.println(F("ATENCAO: se falhou, a posicao pode estar errada. Recoloque no zero e use 'zero'."));
}

// Com a velocidade máxima já definida (vx/vz), sobe a aceleração de 250 em 250
// fazendo movimentos curtos (arrancar e frear é onde a aceleração pesa).
// Quando ouvir "tec-tec" na partida ou na parada, mande qualquer tecla.
void testeAceleracao(Eixo &e) {
  float original = *e.acel;
  long fim = min(cursoPassos(e), (long)(DIST_ACEL_MM * e.passosPorMm));
  if (e.nome == 'X') moverPassos(eixoZ, 0, false);
  moverPassos(e, 0, false);
  while (Serial.available()) Serial.read();

  Serial.print(F("Aceleracao com vmax = ")); Serial.print(*e.vmax, 0);
  Serial.println(F(" passos/s. Envie qualquer tecla quando o motor falhar."));
  float ultimaOk = 0;
  for (float a = 500; a <= 20000; a += 250) {
    *e.acel = a;
    Serial.print(F("  testando ")); Serial.print(a, 0); Serial.println(F(" passos/s2"));
    bool ok = true;
    for (int c = 0; c < 2 && ok; c++)
      ok = moverPassos(e, fim, true) && moverPassos(e, 0, true);
    if (!ok) break;
    ultimaOk = a;
    delay(300);
  }
  while (Serial.available()) Serial.read();
  *e.acel = original;

  Serial.print(F("Ultima aceleracao sem falha: ")); Serial.println(ultimaOk, 0);
  Serial.print(F("Recomendado (margem de 30%): a")); Serial.print((char)tolower(e.nome));
  Serial.print(' '); Serial.println(ultimaOk * 0.7, 0);
  Serial.println(F("ATENCAO: se falhou, a posicao pode estar errada. Recoloque no zero e use 'zero'."));
}

void mostrarStatus() {
  Serial.print(F("X = ")); Serial.print(posMm(eixoX)); Serial.print(F(" mm (")); Serial.print(eixoX.pos);
  Serial.print(F(" passos)  curso ")); Serial.print(CURSO_X_MM);
  Serial.print(F(" mm  vmax ")); Serial.print(VMAX_X, 0); Serial.print(F("  acel ")); Serial.println(ACEL_X, 0);
  Serial.print(F("Z = ")); Serial.print(posMm(eixoZ)); Serial.print(F(" mm (")); Serial.print(eixoZ.pos);
  Serial.print(F(" passos)  curso ")); Serial.print(CURSO_Z_MM);
  Serial.print(F(" mm  vmax ")); Serial.print(VMAX_Z, 0); Serial.print(F("  acel ")); Serial.println(ACEL_Z, 0);
  // Linha para o programa do computador ler: POS <x_mm> <z_mm> <curso_x> <curso_z>
  Serial.print(F("POS ")); Serial.print(posMm(eixoX)); Serial.print(' '); Serial.print(posMm(eixoZ));
  Serial.print(' '); Serial.print(CURSO_X_MM); Serial.print(' '); Serial.println(CURSO_Z_MM);
}

void processarComando(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  int esp = cmd.indexOf(' ');
  String nome = esp < 0 ? cmd : cmd.substring(0, esp);
  String arg  = esp < 0 ? "" : cmd.substring(esp + 1);
  float valor = arg.toFloat();

  if      (nome == "?")      mostrarStatus();
  else if (nome == "!")      Serial.println(F("Parado."));   // chega aqui depois de frear o movimento
  else if (nome == "x")      moverMm(eixoX, valor);
  else if (nome == "z")      moverMm(eixoZ, valor);
  else if (nome == "m") {    // "m 120 80" -> X e Z juntos, em linha reta
    int esp2 = arg.indexOf(' ');
    float zmm = esp2 < 0 ? posMm(eixoZ) : arg.substring(esp2 + 1).toFloat();
    moverLinearMm(valor, zmm);
  }
  else if (nome == "xr")     moverMm(eixoX, posMm(eixoX) + valor);   // relativo em mm
  else if (nome == "zr")     moverMm(eixoZ, posMm(eixoZ) + valor);
  else if (nome == "h")      irParaZero();
  else if (nome == "xs")     moverPassos(eixoX, eixoX.pos + (long)valor, false);
  else if (nome == "zs")     moverPassos(eixoZ, eixoZ.pos + (long)valor, false);
  else if (nome == "zero")   { eixoX.pos = 0; eixoZ.pos = 0; Serial.println(F("Zero definido.")); }
  else if (nome == "fimx")   { CURSO_X_MM = posMm(eixoX); Serial.print(F("CURSO_X_MM = ")); Serial.println(CURSO_X_MM); }
  else if (nome == "fimz")   { CURSO_Z_MM = posMm(eixoZ); Serial.print(F("CURSO_Z_MM = ")); Serial.println(CURSO_Z_MM); }
  else if (nome == "vx")     VMAX_X = valor;
  else if (nome == "vz")     VMAX_Z = valor;
  else if (nome == "ax")     ACEL_X = valor;
  else if (nome == "az")     ACEL_Z = valor;
  else if (nome == "teste")  testeCurso(arg == "z" ? eixoZ : eixoX);
  else if (nome == "rampa")  testeRampa(arg == "z" ? eixoZ : eixoX);
  else if (nome == "acel")   testeAceleracao(arg == "z" ? eixoZ : eixoX);
  else if (nome == "soltar") { driversLigados(false); Serial.println(F("Drivers desligados.")); }
  else if (nome == "travar") { driversLigados(true);  Serial.println(F("Drivers ligados.")); }
  else { Serial.print(F("Comando desconhecido: ")); Serial.println(cmd); Serial.println(F("ok")); return; }

  if (nome != "?") mostrarStatus();
  Serial.println(F("ok"));   // avisa o computador que terminou e pode mandar o próximo
}

void setup() {
  // Saídas de potência desligadas por segurança
  pinMode(FAN1_PIN, OUTPUT); digitalWrite(FAN1_PIN, COOLER_NOS_DRIVERS ? HIGH : LOW);
  pinMode(FAN2_PIN, OUTPUT); digitalWrite(FAN2_PIN, LOW);   // pino 13 = aquecedor do bico na Anet: nunca ligar
  pinMode(BED_PIN, OUTPUT);  digitalWrite(BED_PIN, LOW);

  pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT); pinMode(X_ENABLE, OUTPUT);
  pinMode(Z_STEP, OUTPUT); pinMode(Z_DIR, OUTPUT); pinMode(Z_ENABLE, OUTPUT);
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(Z_MIN_PIN, INPUT_PULLUP);

  // Serial liberada: só usamos X e Z (E0 usava os pinos 0/1 da Serial).
  // Essa Serial também será o canal de comando vindo da câmera/IA.
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(PORTA_Z_ENABLE, OUTPUT);
  digitalWrite(PORTA_Z_ENABLE, HIGH);   // driver do conector Z (sem motor) desligado

  // Z fica SEMPRE ligado para segurar contra a gravidade.
  driversLigados(true);
  if (DESLIGAR_X_PARADO && X_ENABLE != Z_ENABLE) digitalWrite(X_ENABLE, HIGH);

  if (USAR_FIM_DE_CURSO) irParaZero();

  if (TESTE_AO_LIGAR) {
    delay(1000);
    if (TESTAR_X) {
      Serial.println(F("Teste: X vai ate o fim do curso e volta..."));
      moverPassos(eixoX, cursoPassos(eixoX), false);
      delay(500);
      moverPassos(eixoX, 0, false);
      delay(500);
    }
    if (TESTAR_Z) {
      Serial.println(F("Teste: Z desce ate o fim do curso e sobe..."));
      moverPassos(eixoZ, cursoPassos(eixoZ), false);
      delay(500);
      moverPassos(eixoZ, 0, false);
    }
  }

  if (RAMPA_AO_LIGAR) {
    delay(1000);
    testeRampa(eixoX);
    delay(3000);   // pausa longa separando o teste do X do teste do Z
    testeRampa(eixoZ);
  }

  Serial.println(F("Garra pronta. Posicao atual = zero. Digite ? para ajuda."));
  mostrarStatus();
}

void loop() {
  if (Serial.available()) processarComando(Serial.readStringUntil('\n'));
}
