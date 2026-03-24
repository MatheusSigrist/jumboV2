
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <EEPROM.h>
#include <stdint.h>
#include <HMC5883L.h>  // Biblioteca da bussola
//------------------------------COMUNICACAo-SERIAL------------------------------
#define BYTE_INICIA 0xAA
#define BYTE_PARA 0x55
bool sozinho = false;

#define ID_PLACA_OLHO 0x01
#define ID_PLACA_PE 0x02

#define RX_OLHO 30
#define TX_OLHO 40
#define RX_PE 20
#define TX_PE 10

#define RX_MUSCULO 44
#define TX_MUSCULO 43

HardwareSerial SerialMusculo(0);
#define SERIAL_MUSCULO SerialMusculo

// I2C pins (SDA/SCL) usados para módulos como compass HMC5883L
#define SDA_PIN 8  // RTC_GPIO8
#define SCL_PIN 9  // RTC_GPIO9

// Bussola HMC5883L
HMC5883L compass;
float headingAtual = 0.0;  // ângulo do robô (0-360 graus)
bool bussolaOK = false;    // se a bussola está funcionando

// Botões
#define BOTAO_1 3  // RTC_GPIO3
#define BOTAO_2 37 // GPIO37
#define BOTAO_3 46 // GPIO46

// Máquina de Estados
enum Estado { MENU, CALIBRACAO, INICIAR };
Estado estadoAtual = MENU;
int itemSelecionado = 0;  // 0 = Calibração, 1 = Iniciar
unsigned long ultimaBotao = 0;
const int DEBOUNCE_TIME = 200;  // 200ms de debounce

// Submenu de Calibração
enum SubMenuCalibracao { SUBMENU_PRINCIPAL, SUBMENU_GOL, SUBMENU_BUSSOLA };
SubMenuCalibracao subMenuCalibracao = SUBMENU_PRINCIPAL;
int itemSubMenu = 0;  // 0 = Gol, 1 = Bussola

// Variáveis EEPROM para calibração
#define EEPROM_SIZE 512
#define EEPROM_ADDR_BUSSOLA_OFFSET 0  // endereço para guardar offset da bussola (float = 4 bytes)
float bussolaOffsetSalvo = 0.0;  // valor calibrado e salvo
float bussola_temp = 0.0;        // valor temporário durante calibração


struct PacoteCima {
  int16_t uD;
  int16_t uE;
  int16_t uF;
  int16_t uT;
  int16_t angulo;
  int16_t intensidade;
  int16_t erroGol;
  uint16_t pixelsGol;
  uint8_t golDetectado;
};

struct PacoteBaixo {
  int16_t angulo;
};

struct PacoteEstado {
  bool sozinho;
  bool atacante;  // role do robô: true = atacante, false = defensor
  bool corGolAzul; // true = azul, false = amarelo
};

struct MenuData {
  int itemSelecionado;      // 0 = Calibração, 1 = Iniciar
  int estadoMenu;           // 0 = MENU, 1 = CALIBRACAO, 2 = INICIAR
  int16_t anguloBola;       // ângulo do IR (escalado *10)
  int16_t anguloLinha;      // ângulo da linha (escalado *10)
  bool linhaDetectada;      // se linha foi detectada
  bool IRdetectado;         // se IR foi detectado
  int subMenuCal;           // 0 = SUBMENU_PRINCIPAL, 1 = SUBMENU_GOL, 2 = SUBMENU_BUSSOLA
  int itemSubMenuCal;       // 0 = Gol, 1 = Bussola
  int16_t headingBussola;   // heading atual da bussola para calibração (escalado *1)
};

struct StatusComunicacao {
  bool olhoOK;
  bool peOK;
  bool allOK;  // tudo funcionando
};

enum CasoComandoMusculo : uint8_t {
  CMD_NENHUM = 0,
  CMD_IR = 1,
  CMD_LINHA = 2
};

struct ComandoMusculo {
  uint8_t caso;
  int16_t angulo;  // escalado em x10
};

PacoteCima placaCima;
PacoteBaixo placaBaixo;

bool linhaDetectada = false;  // true se linha foi detectada, false caso contrário
bool IRdetectado = false; // true se ângulo IR válido recebido
bool golDetectado = false; // true se a camera detectou o gol selecionado
float uD, uE, uF, uT;
float intensidade;
float anguloBola;
float anguloLinha;  // armazena o valor do ângulo da linha
float erroGol = 0.0;
uint16_t pixelsGol = 0;

// Flags de comunicação
bool olhoRespondeu = false;
bool peRespondeu = false;
bool atacante = true;  // true = atacante, false = defensor
float ultraT_parceiro = 0;  // valor de ultraT recebido do outro robô
bool corGolAzul = false;    // false = amarelo (padrao), true = azul

String bufferHandshakeMusculo = "";
unsigned long ultimoByteHandshakeMusculo = 0;
unsigned long ultimoEnvioMenuMusculo = 0;
const unsigned long INTERVALO_ENVIO_MENU_MUSCULO_MS = 150;

void ProcessarHandshakeMusculo() {
  while (SERIAL_MUSCULO.available() > 0) {
    char c = (char)SERIAL_MUSCULO.read();
    ultimoByteHandshakeMusculo = millis();

    if (c == '\n' || c == '\r') {
      bufferHandshakeMusculo.trim();
      bufferHandshakeMusculo.toLowerCase();
      if (bufferHandshakeMusculo == "oi") {
        SERIAL_MUSCULO.println("oi");
      }
      bufferHandshakeMusculo = "";
      continue;
    }

    if (bufferHandshakeMusculo.length() < 32) {
      bufferHandshakeMusculo += c;
    }
  }

  if (bufferHandshakeMusculo.length() > 0 && (millis() - ultimoByteHandshakeMusculo) > 80) {
    bufferHandshakeMusculo.trim();
    bufferHandshakeMusculo.toLowerCase();
    if (bufferHandshakeMusculo == "oi") {
      SERIAL_MUSCULO.println("oi");
    }
    bufferHandshakeMusculo = "";
  }
}


void LeituraSerial() {
    // leitura do OLHO via Serial1
    while (Serial1.available() >= 5) {
        if (Serial1.read() != BYTE_INICIA) continue;
        byte id = Serial1.read();
        if (id == ID_PLACA_OLHO) {
            if (Serial1.available() >= sizeof(PacoteCima) + 1) {
                PacoteCima temp;
                Serial1.readBytes((uint8_t*)&temp, sizeof(PacoteCima));
                byte stop = Serial1.read();
                if (stop == BYTE_PARA) {
                    placaCima = temp;
                    uD = placaCima.uD / 10.0;
                    uE = placaCima.uE / 10.0;
                    uF = placaCima.uF / 10.0;
                    uT = placaCima.uT / 10.0;
                    anguloBola = placaCima.angulo / 10.0;
                    // ângulo -10 corresponde a -1.0 (não detectado)
                    IRdetectado = (placaCima.angulo != -10);
                    intensidade = placaCima.intensidade / 10.0;
                  erroGol = placaCima.erroGol / 10.0;
                  pixelsGol = placaCima.pixelsGol;
                  golDetectado = (placaCima.golDetectado != 0);
                    olhoRespondeu = true;
                }
            }
        } else {
            continue;
        }
    }

    // leitura do PE via Serial2
    while (Serial2.available() >= 4) {
        if (Serial2.read() != BYTE_INICIA) continue;
        byte id = Serial2.read();
        if (id == ID_PLACA_PE) {
            if (Serial2.available() >= sizeof(PacoteBaixo) + 1) {
                PacoteBaixo temp;
                Serial2.readBytes((uint8_t*)&temp, sizeof(PacoteBaixo));
                byte stop = Serial2.read();
                if (stop == BYTE_PARA) {
                    placaBaixo = temp;
                    if (placaBaixo.angulo == -1) {
                        linhaDetectada = false;
                        anguloLinha = 0.0;
                    } else {
                        linhaDetectada = true;
                        anguloLinha = placaBaixo.angulo / 10.0;
                    }
                    peRespondeu = true;
                }
            }
        } else {
            continue;
        }
    }
}

// Função para ler o heading da bussola
void LerBussola() {
  if (!bussolaOK) return;  // se a bussola não inicializou, não tenta ler

  Vector norm = compass.readNormalize();
  float heading = atan2(norm.YAxis, norm.XAxis) * 180.0 / PI;

  // Garantir que está entre 0-360
  if (heading < 0) {
    heading += 360;
  } else if (heading >= 360) {
    heading -= 360;
  }

  headingAtual = heading;
}

// Função para gravar o valor calibrado da bussola na EEPROM
void GravarBussolaEEPROM() {
  // Gravar headingAtual na EEPROM usando EEPROM.put() para preservar precisão
  bussolaOffsetSalvo = headingAtual;
  EEPROM.put(EEPROM_ADDR_BUSSOLA_OFFSET, bussolaOffsetSalvo);
  EEPROM.commit();
}

// Função para ler o valor calibrado da bussola da EEPROM
float LerBussolaEEPROM() {
  float valor;
  EEPROM.get(EEPROM_ADDR_BUSSOLA_OFFSET, valor);
  return (valor > 360 || valor < 0) ? 0.0 : valor;  // retorna 0 se inválido
}

void EnviarSerial() {
    PacoteEstado estado;
    estado.sozinho = sozinho;
    estado.atacante = atacante;
  estado.corGolAzul = corGolAzul;

    // Enviar para placa OLHO (Serial1)
    Serial1.write(BYTE_INICIA);
    Serial1.write(ID_PLACA_OLHO);
    Serial1.write((uint8_t*)&estado, sizeof(PacoteEstado));
    Serial1.write(BYTE_PARA);

    // Enviar para placa PE (Serial2)
    Serial2.write(BYTE_INICIA);
    Serial2.write(ID_PLACA_PE);
    Serial2.write((uint8_t*)&estado, sizeof(PacoteEstado));
    Serial2.write(BYTE_PARA);
}

// Enviar dados do menu e sensores para musculo via Serial
void EnviarMenuParaMusculo() {
  MenuData menuData;
  menuData.itemSelecionado = itemSelecionado;
  menuData.estadoMenu = (int)estadoAtual;  // 0=MENU, 1=CALIBRACAO, 2=INICIAR
  menuData.anguloBola = (int16_t)round(anguloBola * 10.0);
  menuData.anguloLinha = (int16_t)round(anguloLinha * 10.0);
  menuData.linhaDetectada = linhaDetectada;
  menuData.IRdetectado = IRdetectado;
  menuData.subMenuCal = (int)subMenuCalibracao;  // 0=PRINCIPAL, 1=GOL, 2=BUSSOLA
  menuData.itemSubMenuCal = itemSubMenu;  // 0=Gol, 1=Bussola
  menuData.headingBussola = (int16_t)round(headingAtual);  // ângulo da bussola
  
  SERIAL_MUSCULO.write(BYTE_INICIA);
  SERIAL_MUSCULO.write((uint8_t*)&menuData, sizeof(MenuData));
  SERIAL_MUSCULO.write(BYTE_PARA);
  ultimoEnvioMenuMusculo = millis();
}

void AtualizarMenuMusculoPeriodicamente() {
  if ((millis() - ultimoEnvioMenuMusculo) >= INTERVALO_ENVIO_MENU_MUSCULO_MS) {
    EnviarMenuParaMusculo();
  }
}

// Enviar status de comunicacao para musculo via Serial
void EnviarStatusComunicacao() {
  StatusComunicacao status;
  status.olhoOK = olhoRespondeu;
  status.peOK = peRespondeu;
  status.allOK = (olhoRespondeu && peRespondeu);
  
  SERIAL_MUSCULO.write(0xFF);  // marcador especial para status
  SERIAL_MUSCULO.write((uint8_t*)&status, sizeof(StatusComunicacao));
  SERIAL_MUSCULO.write(0xFE);  // fim do status
}

void EnviarComandoParaMusculo(uint8_t caso, float anguloGraus) {
  ComandoMusculo cmd;
  cmd.caso = caso;
  cmd.angulo = (int16_t)round(anguloGraus * 10.0);

  SERIAL_MUSCULO.write(0xAB);
  SERIAL_MUSCULO.write((uint8_t*)&cmd, sizeof(ComandoMusculo));
  SERIAL_MUSCULO.write(0xBA);
}

// Testar comunicacao com todas as placas
void TestarComunicacao() {
  olhoRespondeu = false;
  peRespondeu = false;
  
  // Aguardar respostas do OLHO e PE
  unsigned long tempoInicio = millis();
  while ((millis() - tempoInicio) < 2000) {
    ProcessarHandshakeMusculo();

    if (Serial1.available()) {
      String msg = "";
      while (Serial1.available()) {
        msg += (char)Serial1.read();
      }
      if (msg.indexOf("OK") >= 0) {
        olhoRespondeu = true;
      }
    }
    if (Serial2.available()) {
      String msg = "";
      while (Serial2.available()) {
        msg += (char)Serial2.read();
      }
      if (msg.indexOf("OK") >= 0) {
        peRespondeu = true;
      }
    }
    if (olhoRespondeu && peRespondeu) break;
    delay(100);
  }
  
  // Enviar resultado para musculo
  delay(500);
  EnviarStatusComunicacao();
}

// Função para verificar pressionamento dos botões
void verificarBotoes() {
  static bool botao1_anterior = HIGH;
  static bool botao2_anterior = HIGH;
  static bool botao3_anterior = HIGH;
  
  bool botao1_atual = digitalRead(BOTAO_1);
  bool botao2_atual = digitalRead(BOTAO_2);
  bool botao3_atual = digitalRead(BOTAO_3);
  
  unsigned long agora = millis();
  
  // Debounce: ignora presses se passaram menos de 200ms
  if (agora - ultimaBotao < DEBOUNCE_TIME) {
    return;
  }
  
  // Detecta borda de descida (botão pressionado)
  if (botao1_anterior == HIGH && botao1_atual == LOW) {
    ultimaBotao = agora;
    
    if (estadoAtual == MENU) {
      // Botão 1 = Scroll UP (item anterior)
      itemSelecionado--;
      if (itemSelecionado < 0) itemSelecionado = 1;  // wrap around
      EnviarMenuParaMusculo();  // envia atualização para musculo
    } else if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_PRINCIPAL) {
      // Botão 1 = Scroll UP no submenu de calibração
      itemSubMenu--;
      if (itemSubMenu < 0) itemSubMenu = 1;  // wrap around
    } else if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_GOL) {
      // Botão 1 = alternar seleção de cor do gol (0=AMARELO, 1=AZUL)
      itemSubMenu = (itemSubMenu == 0) ? 1 : 0;
    }
  }
  
  if (botao2_anterior == HIGH && botao2_atual == LOW) {
    ultimaBotao = agora;
    
    if (estadoAtual == MENU) {
      // Botão 2 = Scroll DOWN (item próximo)
      itemSelecionado++;
      if (itemSelecionado > 1) itemSelecionado = 0;  // wrap around
      EnviarMenuParaMusculo();  // envia atualização para musculo
    } else if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_PRINCIPAL) {
      // Botão 2 = Scroll DOWN no submenu de calibração
      itemSubMenu++;
      if (itemSubMenu > 1) itemSubMenu = 0;  // wrap around
    } else if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_GOL) {
      // Botão 2 = alternar seleção de cor do gol (0=AMARELO, 1=AZUL)
      itemSubMenu = (itemSubMenu == 0) ? 1 : 0;
    }
  }
  
  if (botao3_anterior == HIGH && botao3_atual == LOW) {
    ultimaBotao = agora;
    
    if (estadoAtual == MENU) {
      // Botão 3 = Select (muda estado)
      if (itemSelecionado == 0) {
        estadoAtual = CALIBRACAO;
        subMenuCalibracao = SUBMENU_PRINCIPAL;  // inicia no submenu principal
        itemSubMenu = 0;
      } else if (itemSelecionado == 1) {
        estadoAtual = INICIAR;
      }
    } else if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_PRINCIPAL) {
      // Select no submenu principal = entra no submenu escolhido
      if (itemSubMenu == 0) {
        subMenuCalibracao = SUBMENU_GOL;
        itemSubMenu = corGolAzul ? 1 : 0;  // ao entrar no GOL, mostra cor atual
      } else if (itemSubMenu == 1) {
        subMenuCalibracao = SUBMENU_BUSSOLA;
      }
    } else if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_BUSSOLA) {
      // Select durante calibração da bussola = salva o valor
      GravarBussolaEEPROM();
      subMenuCalibracao = SUBMENU_PRINCIPAL;  // volta ao submenu principal
    } else if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_GOL) {
      // Select durante calibração do gol = salva cor e envia para OLHO
      corGolAzul = (itemSubMenu == 1);  // 0=AMARELO, 1=AZUL
      EnviarSerial();
      subMenuCalibracao = SUBMENU_PRINCIPAL;
      itemSubMenu = 0;
    }
  }
  
  botao1_anterior = botao1_atual;
  botao2_anterior = botao2_atual;
  botao3_anterior = botao3_atual;
}






//-----------------------------------------------------------------------









//----------------------------ESP-NOW----------------------------

struct DadosRobo {
  int16_t uD;
  int16_t uE;
  int16_t uF;
  int16_t uT;
};

uint8_t peerMac[] = {0xC8,0xF0,0x9E,0xA8,0x22,0xD0}; // TROQUE

DadosRobo dadosRecebidos = {0, 0, 0, 0};
unsigned long lastSend = 0;
unsigned long lastResponse = 0;
bool awaitingAck = false;

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(DadosRobo)) {
    memcpy(&dadosRecebidos, data, sizeof(DadosRobo));
    // Armazenar ultraT do parceiro para comparação de papéis
    ultraT_parceiro = dadosRecebidos.uT / 10.0;
    awaitingAck = false;
    sozinho = false;  // Parceiro encontrado! Resetar flag sozinho
    lastResponse = millis();
  }
}
//-----------------------------------------------------------------------






void setup(){
  Serial.begin(115200);
  SERIAL_MUSCULO.begin(9600, SERIAL_8N1, RX_MUSCULO, TX_MUSCULO);
  Serial1.begin(9600, SERIAL_8N1, RX_OLHO, TX_OLHO);
  Serial2.begin(9600, SERIAL_8N1, RX_PE, TX_PE);
  
  // Inicializar EEPROM para armazenar valores calibrados
  EEPROM.begin(EEPROM_SIZE);
  bussolaOffsetSalvo = LerBussolaEEPROM();  // ler valor salvo
  
  // Inicializar I2C para compass e outros dispositivos
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Inicializar e configurar bussola HMC5883L
  if (compass.begin()) {
    compass.setRange(HMC5883L_RANGE_1_3GA);  // escala 1.3 Gauss (mais sensível)
    compass.setMeasurementMode(HMC5883L_CONTINOUS);  // modo contínuo
    compass.setDataRate(HMC5883L_DATARATE_15HZ);  // 15 Hz
    compass.setSamples(HMC5883L_SAMPLES_8);  // 8 amostras
    bussolaOK = true;
  } else {
    bussolaOK = false;
  }
  
  // Configura pinos dos botões como INPUT_PULLUP
  pinMode(BOTAO_1, INPUT_PULLUP);
  pinMode(BOTAO_2, INPUT_PULLUP);
  pinMode(BOTAO_3, INPUT_PULLUP);
  
  // Testar comunicação com as placas
  delay(1000);  // aguardar outras placas iniciarem
  TestarComunicacao();
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if(esp_now_init()!=ESP_OK){
    return;
  }

  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  
  // Enviar menu inicial para músculo
  EnviarMenuParaMusculo();

  // Enviar estado inicial (inclui cor do gol) para OLHO e PE
  EnviarSerial();
  
  // Inicializar lastSend para evitar timeout na primeira iteração
  lastSend = millis();
}

// Função que executa a lógica normal de operação
void iniciarOperacao() {

  // envia dados de ultrassonicos a cada 1 segundo
  if(millis() - lastSend > 1000){
    DadosRobo dados;
    dados.uD = (int16_t)round(uD * 10.0);
    dados.uE = (int16_t)round(uE * 10.0);
    dados.uF = (int16_t)round(uF * 10.0);
    dados.uT = (int16_t)round(uT * 10.0);
    
    esp_now_send(peerMac, (uint8_t*)&dados, sizeof(DadosRobo));
    lastSend = millis();
    awaitingAck = true;
  }

  // verifica timeout e define papéis (atacante/defensor)
  if(awaitingAck && millis() - lastSend > 500){
    // Timeout: nenhuma resposta do parceiro após 500ms
    awaitingAck = false;
    sozinho = true;
    atacante = true;  // se sozinho, vira atacante
  } 
  
  // SEMPRE tentar comparação se temos dados do parceiro (pode se reconectar)
  if(!sozinho) {
    // SE TEM PARCEIRO: comparar ultraT para definir papéis
    // Quem tiver MENOR ultraT traseiro vira defensor
    // Quem tiver MAIOR ultraT traseiro vira atacante
    // A bussola (headingAtual) está sendo lida continuamente e pode ser usada para navegação
    if(uT < ultraT_parceiro) {
      atacante = false;  // eu tenho menos espaço atrás, viro defensor
    } else {
      atacante = true;   // eu tenho mais espaço atrás, viro atacante
    }
  }

  // Enviar estado (sozinho/com parceiro) via serial para olho e pé
  EnviarSerial();
  LeituraSerial();

  // determinar estado geral a partir das flags
  enum EstadoSensor { NENHUM, LINHA, IR };
  EstadoSensor estado = NENHUM;
  if (linhaDetectada) {
    estado = LINHA;
  } else if (IRdetectado) {
    estado = IR;
  }
  

  if (atacante) {
    switch (estado) {
      case LINHA:
        // ATACANTE + LINHA:
        // Prioridade máxima é fugir da linha usando anguloLinha.
        EnviarComandoParaMusculo(CMD_LINHA, anguloLinha);
        break;

      case IR:
        // ATACANTE + IR:
        // Seguir/alinhar a bola usando anguloBola e intensidade.
        EnviarComandoParaMusculo(CMD_IR, anguloBola);
        break;

      case NENHUM:
      default:
        // ATACANTE + NENHUM:
        // Procurar a bola ou reposicionar para ataque.
        EnviarComandoParaMusculo(CMD_NENHUM, 0.0);
        break;
    }
  } else {
    switch (estado) {
      case LINHA:
        // DEFENSOR + LINHA:
        // Prioridade máxima é fugir da linha usando anguloLinha.
        EnviarComandoParaMusculo(CMD_LINHA, anguloLinha);
        break;

      case IR:
        // DEFENSOR + IR:
        // Interceptar a bola e manter comportamento defensivo.
        EnviarComandoParaMusculo(CMD_IR, anguloBola);
        break;

      case NENHUM:
      default:
        // DEFENSOR + NENHUM:
        // Reposicionar e proteger a área defensiva.
        EnviarComandoParaMusculo(CMD_NENHUM, 0.0);
        break;
    }
  }
}

void loop(){

  ProcessarHandshakeMusculo();

  LerBussola();
  

  verificarBotoes();

  // Máquina de Estados
  switch (estadoAtual) {
    case MENU:
      AtualizarMenuMusculoPeriodicamente();
      delay(100);
      break;
    case CALIBRACAO:
      // Submenu de calibração
      if (subMenuCalibracao == SUBMENU_PRINCIPAL) {
        // Exibir submenu (Gol / Bussola)
        // MenuData será usado para passagem de dados ao Musculo
        AtualizarMenuMusculoPeriodicamente();
      } else if (subMenuCalibracao == SUBMENU_GOL) {
        // Exibir opções de cor do gol (AMARELO / AZUL)
        AtualizarMenuMusculoPeriodicamente();
      } else if (subMenuCalibracao == SUBMENU_BUSSOLA) {
        // Monitorar bussola em tempo real e enviar valor para Musculo exibir
        // A bussola já está sendo lida no loop principal
        // Aqui apenas enviamos o valor para o display mostrar "Aperte para gravar: XXX°"
        AtualizarMenuMusculoPeriodicamente();
      }
      delay(100);
      break;
    case INICIAR:
      iniciarOperacao();
      // Enviar dados atualizados periodicamente para musculo exibir
      AtualizarMenuMusculoPeriodicamente();
      break;
  }
  delay(50);
}
