// =============================================================================
// SECAO 19 — ESTRATEGIA DO ATACANTE (COM 4 CAMADAS DE DATABASE)
// =============================================================================
// Inclua no topo do musculo.cpp:
// #include "DATABASES_LAYER.hpp"
//
// Integração das 4 camadas de dados:
// Layer 1: IR Bruto (sensor raw)
// Layer 2: IR Previsto (com Kalman)
// Layer 3: Comando de Movimento (decisão)
// Layer 4: Saída PWM (execução real)
// =============================================================================

void atacante() {
  int16_t anguloGolCamera = -999;
  bool golVisivelCamera = cameraTemGolSelecionadoValido(anguloGolCamera);
  erroAlinhamentoGraus = golVisivelCamera ? calcularErroGolPorPapel((float)anguloGolCamera) : 0.0f;
  fugindoLinhaAgora    = false;
  anguloFugaLinhaCmd   = 0.0f;

  static uint8_t confirmacoesLinhaParede  = 0;
  static bool    forcarIrDiretoAposLinha  = false;

  int  cmdPidAssinado         = 0;
  bool cameraBolaBrutaVisivel = cameraTemBolaValida();
  float anguloCameraBolaFiltrado  = -1.0f;
  bool cameraBolaFiltradaVisivel  = obterAnguloCameraBolaFiltrado(anguloCameraBolaFiltrado);
  float anguloIrBufferizado        = -1.0f;
  bool irDisponivel               = obterAnguloIrComBuffer(anguloIrBufferizado);

  // =========================================================================
  // CAMADA 1: Adicionar IR BRUTO ao banco de dados
  // =========================================================================
  if (irDetectado && anguloIr >= 0.0f) {
    adicionarIrBruto((int16_t)anguloIr, cameraBallDist, 95);
  } else if (ultimoAnguloIrValido >= 0.0f && irDisponivel) {
    adicionarIrBruto((int16_t)ultimoAnguloIrValido, cameraBallDist, 60);
  }

  // =========================================================================
  // CAMADA 2: Filtro Kalman + Adicionar IR PREVISTO
  // =========================================================================
  int16_t angloPrevisto = filtroKalmanIr(irDisponivel ? anguloIrBufferizado : -1.0f);
  if (irDisponivel) {
    adicionarIrPrevisto((int16_t)anguloIrBufferizado, angloPrevisto, 0);
  }

  // =========================================================================
  // Fuga de linha com prioridade máxima
  // =========================================================================
  if (linhaDetectada && anguloLinhaPe >= 0.0f) {
    fugindoLinhaAgora  = true;
    anguloFugaLinhaCmd = normalizarAngulo360(anguloLinhaPe);
    float anguloFugaSuavizado = suavizarAnguloMovimentoAtacante(anguloFugaLinhaCmd);
    int vel = aplicarFreioUltrassonicoAtacante(VELOCIDADE_FUGA_LINHA);
    
    // CAMADA 3: Registrar comando de fuga de linha
    adicionarComandoMovimento(anguloFugaSuavizado, vel, 1, true, false, 0);
    
    // CAMADA 4: Registrar PWM executado
    int v1 = ledcRead(PWM_CH1), v2 = ledcRead(PWM_CH2);
    int v3 = ledcRead(PWM_CH3), v4 = ledcRead(PWM_CH4);
    float vel_media = (v1 + v2 + v3 + v4) / 4.0f;
    adicionarSaidaPwm(v1, v2, v3, v4, anguloFugaSuavizado, vel_media, 0);
    
    seguirDirecaoPorAngulo(anguloFugaSuavizado, vel);
    return;
  }

  // =========================================================================
  // Decide se precisa girar para alinhar ao gol
  // =========================================================================
  bool precisaAlinhar = golVisivelCamera && (fabsf(erroAlinhamentoGraus) > TOLERANCIA_ALINHAMENTO_GRAUS);
  bool erroGrande     = golVisivelCamera && (fabsf(erroAlinhamentoGraus) > 40.0f);
  
  if (precisaAlinhar) {
    alinhandoAgora = true;
    int cmdPid    = calcularSaidaPidBussola(erroAlinhamentoGraus);
    cmdPidAssinado = -SINAL_GIRO_PID * cmdPid;
  } else {
    alinhandoAgora = false; 
    resetPidBussola();
  }

  // =========================================================================
  // Lógica existente de câmera / IR
  // =========================================================================
  if (!irDisponivel && cameraBolaFiltradaVisivel) {
    if (inicioCameraSemIrMs == 0) inicioCameraSemIrMs = millis();
  } else {
    inicioCameraSemIrMs = 0;
  }
  
  bool ignorarLinhaPorCamera = (inicioCameraSemIrMs != 0) &&
                               ((millis() - inicioCameraSemIrMs) >= TEMPO_CAMERA_SEM_IR_PARA_IGNORAR_LINHA_MS);

  bool linhaRecenteForcada = (ultimoComandoLinhaMs > 0) &&
                             ((millis() - ultimoComandoLinhaMs) <= RETENCAO_FUGA_LINHA_MS);
  float anguloLinhaParaFuga = (linhaDetectada && anguloLinhaPe >= 0.0f) ? anguloLinhaPe : ultimoAnguloLinhaValido;

  bool linhaValida = ((linhaDetectada && (anguloLinhaPe >= 0.0f) && !ignorarLinhaPorCamera) ||
                      (linhaRecenteForcada && (anguloLinhaParaFuga >= 0.0f)));

  bool linhaComParedeCritica = linhaValida && ultraLateralCriticoAtacante();
  if (linhaComParedeCritica) {
    if (confirmacoesLinhaParede < ATACANTE_LINHA_PAREDE_CONFIRMACAO) confirmacoesLinhaParede++;
    if (confirmacoesLinhaParede >= ATACANTE_LINHA_PAREDE_CONFIRMACAO) forcarIrDiretoAposLinha = true;
  } else if (!linhaValida) {
    confirmacoesLinhaParede = 0; 
    forcarIrDiretoAposLinha = false;
  }

  bool irDiretoAtivo = forcarIrDiretoAposLinha && irDisponivel;

  // =========================================================================
  // PRIORIDADE 1: linha + parede = IR direto
  // =========================================================================
  if (linhaValida && irDiretoAtivo) {
    if (irNaFaixaFrontal(anguloIrBufferizado)) {
      int vel = aplicarFreioUltrassonicoAtacanteFrente(VELOCIDADE_IR_FRONTAL_PWM);
      adicionarComandoMovimento(0.0f, vel, 0, false, false, 0);
      int v1 = ledcRead(PWM_CH1), v2 = ledcRead(PWM_CH2);
      int v3 = ledcRead(PWM_CH3), v4 = ledcRead(PWM_CH4);
      adicionarSaidaPwm(v1, v2, v3, v4, 0.0f, (v1+v2+v3+v4)/4.0f, 0);
      
      if (ultraTcm > 150) moverFrenteComGiro(vel, cmdPidAssinado);
      else                 moverFrenteComGiro(vel, cmdPidAssinado);
    } else {
      float anguloIrAlvo = mapearAnguloBolaParaMovimento(anguloIrBufferizado);
      float anguloIrComRampa = obterAnguloIrSuavizado(anguloIrAlvo);
      int velocidadeIr = aplicarFreioUltrassonicoAtacante(calcularVelocidadeIrPorAngulo(anguloIrBufferizado));
      float anguloMovimentoComControle = suavizarAnguloMovimentoAtacante(anguloIrComRampa);
      
      adicionarComandoMovimento(anguloMovimentoComControle, velocidadeIr, 0, false, false, 0);
      int v1 = ledcRead(PWM_CH1), v2 = ledcRead(PWM_CH2);
      int v3 = ledcRead(PWM_CH3), v4 = ledcRead(PWM_CH4);
      adicionarSaidaPwm(v1, v2, v3, v4, anguloMovimentoComControle, (v1+v2+v3+v4)/4.0f, 0);
      
      seguirDirecaoPorAngulo(anguloMovimentoComControle, velocidadeIr);
    }
    return;
  }

  // =========================================================================
  // PRIORIDADE 2: fuga de linha
  // =========================================================================
  if (linhaValida) {
    fugindoLinhaAgora  = true;
    anguloFugaLinhaCmd = normalizarAngulo360(anguloLinhaParaFuga);
    float anguloFugaSuavizado = suavizarAnguloMovimentoAtacante(anguloFugaLinhaCmd);
    int vel = aplicarFreioUltrassonicoAtacante(VELOCIDADE_FUGA_LINHA);
    
    adicionarComandoMovimento(anguloFugaSuavizado, vel, 1, true, false, 0);
    int v1 = ledcRead(PWM_CH1), v2 = ledcRead(PWM_CH2);
    int v3 = ledcRead(PWM_CH3), v4 = ledcRead(PWM_CH4);
    adicionarSaidaPwm(v1, v2, v3, v4, anguloFugaSuavizado, (v1+v2+v3+v4)/4.0f, 0);
    
    seguirDirecaoPorAngulo(anguloFugaSuavizado, vel);
    return;
  }

  // =========================================================================
  // PRIORIDADE 3: erro grande = giro puro
  // =========================================================================
  if (erroGrande) {
    adicionarComandoMovimento(0.0f, 0, 2, false, true, (int16_t)erroAlinhamentoGraus);
    int v1 = ledcRead(PWM_CH1), v2 = ledcRead(PWM_CH2);
    int v3 = ledcRead(PWM_CH3), v4 = ledcRead(PWM_CH4);
    adicionarSaidaPwm(v1, v2, v3, v4, 0.0f, 0, (int16_t)erroAlinhamentoGraus);
    
    girarNoEixo(cmdPidAssinado);
    return;
  }

  // =========================================================================
  // PRIORIDADE 4: IR disponível
  // =========================================================================
  if (irDisponivel) {
    if (linhaDetectada && anguloLinhaPe >= 0.0f) {
      fugindoLinhaAgora  = true;
      anguloFugaLinhaCmd = normalizarAngulo360(anguloLinhaPe);
      float anguloFugaSuavizado = suavizarAnguloMovimentoAtacante(anguloFugaLinhaCmd);
      int vel = aplicarFreioUltrassonicoAtacante(VELOCIDADE_FUGA_LINHA);
      
      adicionarComandoMovimento(anguloFugaSuavizado, vel, 1, true, false, 0);
      int v1 = ledcRead(PWM_CH1), v2 = ledcRead(PWM_CH2);
      int v3 = ledcRead(PWM_CH3), v4 = ledcRead(PWM_CH4);
      adicionarSaidaPwm(v1, v2, v3, v4, anguloFugaSuavizado, (v1+v2+v3+v4)/4.0f, 0);
      
      seguirDirecaoPorAngulo(anguloFugaSuavizado, vel);
      return;
    }
    
    if (irNaFaixaFrontal(anguloIrBufferizado)) {
      int vel = aplicarFreioUltrassonicoAtacanteFrente(VELOCIDADE_IR_FRONTAL_PWM);
      adicionarComandoMovimento(0.0f, vel, 0, false, false, 0);
      int v1 = ledcRead(PWM_CH1), v2 = ledcRead(PWM_CH2);
      int v3 = ledcRead(PWM_CH3), v4 = ledcRead(PWM_CH4);
      adicionarSaidaPwm(v1, v2, v3, v4, 0.0f, (v1+v2+v3+v4)/4.0f, 0);
      
      if (ultraTcm > 150) moverFrenteComGiro(vel, cmdPidAssinado);
      else                 moverFrenteComGiro(vel, cmdPidAssinado);
    } else {
      float anguloIrAlvo = normalizarAngulo360(mapearAnguloBolaParaMovimento(anguloIrBufferizado) + (erroAlinhamentoGraus * 2));
      float anguloIrComRampa = obterAnguloIrSuavizado(anguloIrAlvo);
      int velocidadeIr = aplicarFreioUltrassonicoAtacante(calcularVelocidadeIrPorAngulo(anguloIrBufferizado));
      float anguloMovimentoComControle = suavizarAnguloMovimentoAtacante(anguloIrComRampa);
      
      adicionarComandoMovimento(anguloMovimentoComControle, velocidadeIr, 0, false, false, 0);
      int v1 = ledcRead(PWM_CH1), v2 = ledcRead(PWM_CH2);
      int v3 = ledcRead(PWM_CH3), v4 = ledcRead(PWM_CH4);
      adicionarSaidaPwm(v1, v2, v3, v4, anguloMovimentoComControle, (v1+v2+v3+v4)/4.0f, 0);
      
      seguirDirecaoPorAngulo(anguloMovimentoComControle, velocidadeIr);
    }
    return;
  }

  // =========================================================================
  // PRIORIDADE 5: câmera como fallback de bola
  // =========================================================================
  if (cameraBolaFiltradaVisivel) {
    bool semBolaTempoSuficiente = !cameraBolaBrutaVisivel &&
                                  (cameraSemBolaBrutaInicioMs > 0) &&
                                  ((millis() - cameraSemBolaBrutaInicioMs) >= ATACANTE_ESPERA_SEM_BOLA_CAMERA_MS);
    float anguloCameraVetorial = semBolaTempoSuficiente ? calcularAnguloBuscaSemBolaCameraAtacante() : anguloCameraBolaFiltrado;
    float anguloCameraComRampa = obterAnguloIrSuavizado(anguloCameraVetorial);
    float anguloMovimentoComControle = suavizarAnguloMovimentoAtacante(anguloCameraComRampa);
    int vel = aplicarFreioUltrassonicoAtacante(velocidade_maxima);
    
    adicionarComandoMovimento(anguloMovimentoComControle, vel, 0, false, false, 0);
    int v1 = ledcRead(PWM_CH1), v2 = ledcRead(PWM_CH2);
    int v3 = ledcRead(PWM_CH3), v4 = ledcRead(PWM_CH4);
    adicionarSaidaPwm(v1, v2, v3, v4, anguloMovimentoComControle, (v1+v2+v3+v4)/4.0f, 0);
    
    seguirDirecaoComGiro(anguloMovimentoComControle, vel, cmdPidAssinado);
    return;
  }

  // =========================================================================
  // PRIORIDADE 6: apenas alinhamento (sem bola)
  // =========================================================================
  if (precisaAlinhar) {
    adicionarComandoMovimento(0.0f, 0, 2, false, true, (int16_t)erroAlinhamentoGraus);
    int v1 = ledcRead(PWM_CH1), v2 = ledcRead(PWM_CH2);
    int v3 = ledcRead(PWM_CH3), v4 = ledcRead(PWM_CH4);
    adicionarSaidaPwm(v1, v2, v3, v4, 0.0f, 0, (int16_t)erroAlinhamentoGraus);
    
    girarNoEixo(cmdPidAssinado);
    return;
  }

  // =========================================================================
  // Sem informações: para
  // =========================================================================
  adicionarComandoMovimento(0.0f, 0, 3, false, false, 0);
  pararMotores();
  adicionarSaidaPwm(0, 0, 0, 0, 0.0f, 0.0f, 0);
  delay(10);
}

// ============================================================================
// PARA USAR EM SEU LOOP:
// ============================================================================
// No seu loop() principal, adicione periodicamente (a cada 5 segundos):
//
// static unsigned long ultimaExportacao = 0;
// if ((millis() - ultimaExportacao) > 5000) {
//   Serial.println("\n=== DADOS COLETADOS (50 amostras) ===");
//   exportarDadosJson();
//   ultimaExportacao = millis();
// }
